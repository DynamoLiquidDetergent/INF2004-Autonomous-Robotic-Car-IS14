/*
 *	subsystem5.c
 *	Subsystem 5 -- Spatial Optimizer: creates its kernel objects and
 *	starts its tasks.
 *
 *	Objects created here (TEAM.md section 5/6):
 *	  flg_fault_i14  event flag group  IF-12  fault bits, one per owner
 *	  flg_inhibit    event flag group  IF-11  latched motor inhibit, to Motion Control
 *	  flg_s5_ctrl    event flag group         control into Obstacle Detection
 *	  mbf_obstacle   message buffer    i8     obstacle profile -> Vehicle Controller
 *
 *	Tasks: s5_bringup_task (phases 1-4) or obstacle_detect_task and
 *	safety_monitor_task (phases 5-6). vehicle_controller_task comes next.
 */

#include "subsystem5.h"
#include "../subsystem2/subsystem2.h"	/* subsystem2_bind_safety() */

EXPORT ID		flg_fault_i14;
EXPORT ID		flg_s5_ctrl;
EXPORT ID		mbf_obstacle;

EXPORT ER subsystem5_init(void)
{
	T_CFLG	cflg = {0};
	T_CMBF	cmbf = {0};

	/* Fault flags: starts with every bit clear. Several tasks set bits,
	   one task (the Safety Monitor) waits and clears. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_fault_i14 = tk_cre_flg(&cflg);
	if (flg_fault_i14 < E_OK) {
		return flg_fault_i14;
	}

	/* Control flags into Obstacle Detection. In stand-alone bench mode the
	   obstacle watch starts switched on. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = S5_STANDALONE ? S5_CTRL_MONITOR_EN : 0;
	flg_s5_ctrl = tk_cre_flg(&cflg);
	if (flg_s5_ctrl < E_OK) {
		return flg_s5_ctrl;
	}

	/* IF-11: the motor inhibit latch. Clear at power-up: the robot may move. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_inhibit = tk_cre_flg(&cflg);
	if (flg_inhibit < E_OK) {
		return flg_inhibit;
	}
#if S5_PHASE >= 5
	/* Motion Control now obeys the latch and reports a stall on bit 0 of the
	   fault flags (FLT_MOTION_BIT). Phases 1-4 leave Motion Control free for
	   bench work. */
	subsystem2_bind_safety(flg_inhibit, S5_INHIBIT_BIT, flg_fault_i14, FLT_MOTION_BIT);
#endif

	/* i8: room for 4 profiles (8 bytes + 4 bytes kernel overhead each) */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 4 * (OBS_MSG_BYTES + 4);
	cmbf.maxmsz = OBS_MSG_BYTES;
	mbf_obstacle = tk_cre_mbf(&cmbf);
	return (mbf_obstacle < E_OK) ? mbf_obstacle : E_OK;
}

EXPORT ER subsystem5_start(void)
{
	ID tid;

	/* Phases 1-4 (s5_phase.h) run the small bring-up task that touches
	   only the pins wired so far; phases 5-6 run the real task.
	   No core affinity: the scheduler may run it on either core. */
#if S5_PHASE <= 4
	tid = app_start_task(s5_bringup_task, S5_PRI_OBSTACLE, 0, "s5_bringup");
#else
	tid = app_start_task(obstacle_detect_task, S5_PRI_OBSTACLE, 0, "obstacle_task");
#endif
	if (tid < E_OK) {
		return tid;
	}
#if S5_PHASE >= 5
	tid = app_start_task(safety_monitor_task, S5_PRI_SAFETY, 0, "safety_monitor");
	if (tid < E_OK) {
		return tid;
	}
#endif
	return E_OK;
}
