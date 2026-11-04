/*
 *	safety_monitor.c
 *	Subsystem 5 -- Safety Monitor task (priority S5_PRI_SAFETY, above Motion
 *	Control and every other application task).
 *
 *	Owns the latched motor inhibit (IF-11, flg_inhibit) and reads the fault
 *	flags (IF-12, flg_fault_i14). The decisions are in safety_logic.c (tested
 *	on a PC); this file only reads the flags, calls sm_step() and does what
 *	it answers.
 *
 *	Wakes at once when any fault bit is set, and every S5_SM_PERIOD_MS
 *	otherwise (to follow the Obstacle Detection state).
 */

#include "subsystem5.h"
#include "safety_logic.h"

#define S5_SM_PERIOD_MS		20

EXPORT ID		flg_inhibit;
EXPORT volatile UB	s5_fault_mask;

LOCAL UW now_ms(void)
{
	SYSTIM t;

	tk_get_otm(&t);
	return t.lo;
}

EXPORT void safety_monitor_task(INT stacd, void *exinf)
{
	SM_CFG		cfg;
	SM_STATE	st;
	SM_OUT		o;
	UINT		got;
	UINT		faults;
	ER		er;

	(void)stacd;
	(void)exinf;
	sm_cfg_default(&cfg);
	cfg.obstacle_bit = FLT_OBSTACLE_BIT;
	cfg.fault_mask   = FLT_ALL_MASK;
	sm_init(&st);
	tm_printf((UB*)"safety: monitor running (hold %d ms after a fault, %d ms after a scan)\n",
		  (INT)cfg.hold_fault_ms, (INT)cfg.hold_scan_ms);

	for (;;) {
		/* wait for any fault bit; do NOT clear here, sm_step says which to clear */
		got = 0;
		er = tk_wai_flg(flg_fault_i14, FLT_ALL_MASK, TWF_ORW, &got, S5_SM_PERIOD_MS);
		faults = (er == E_OK) ? (got & FLT_ALL_MASK) : 0;

		o = sm_step(&st, &cfg, faults, s5_obs_state, now_ms());
		s5_fault_mask |= (UB)faults;		/* for the status to subsystem 1 */

		if (o.set_inhibit) {
			/* set the latch first, so Motion Control stops on its next cycle */
			tk_set_flg(flg_inhibit, S5_INHIBIT_BIT);
			tm_printf((UB*)"safety: INHIBIT set (faults 0x%x, obstacle state %d)\n",
				  (UINT)faults, (INT)s5_obs_state);
		}
		if (o.clear_bits != 0) {
			/* clear exactly the bits we handled; a bit set by another task
			   a moment ago is left for the next round */
			tk_clr_flg(flg_fault_i14, ~(UINT)o.clear_bits);
		}
		if (o.clear_inhibit) {
			tk_clr_flg(flg_inhibit, ~(UINT)S5_INHIBIT_BIT);
			s5_fault_mask = 0;
			tm_printf((UB*)"safety: inhibit released (a newer epoch is needed to move)\n");
		}
		if (o.reset_obstacle) {
			tk_set_flg(flg_s5_ctrl, S5_CTRL_RESET);
		}
	}
}
