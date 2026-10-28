/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01, ...
 *
 *	These need NO sensor attached. Tests that need the HC-SR04 or the
 *	servo moving are manual (see TEST_LOG.md).
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem5_test(void)
{
	TEST_INIT();
	OBS_PROFILE p, q;
	uint32_t w[OBS_MSG_WORDS];
	uint8_t seq = 0;
	UINT got = 0;

	/* let obstacle_detect_task run its driver init first */
	tk_dly_tsk(500);

	TEST_CHECK("S5-U01", tk_get_tid() > 0, "running in a task context");

	TEST_CHECK("S5-U02", drv_pwm_reset_done() != 0,
		   "PWM block is out of reset (kernel leaves it in reset)");

#if S5_PHASE >= 2
	TEST_CHECK("S5-U03", in_w(PWM_BASE + 0x14 * 6 + PWM_CHx_TOP) == 19999u &&
		   (in_w(PWM_BASE + 0x14 * 6 + PWM_CHx_CSR) & 1u) == 1u,
		   "servo slice 6 runs with a 20 ms period (TOP 19999)");

	TEST_CHECK("S5-U04", drv_servo_pulse_us() == 1500u,
		   "servo starts centred at 90 deg (1500 us pulse)");
#endif

	TEST_CHECK("S5-U05", s5_obs_state <= OBS_ST_FAULT, "Obstacle state fits in 3 bits");

	/* packed i8 message round trip */
	p.detected = 1; p.closest_idx = 6; p.closest_cm = 30; p.width_cm = 12;
	p.action = OBS_ACT_TURN_LEFT; p.partial = 0; p.left_cm = 14; p.right_cm = 22;
	obs_pack(&p, 5, w);
	obs_unpack(w, &q, &seq);
	TEST_CHECK("S5-U06", q.closest_cm == 30 && q.width_cm == 12 && q.left_cm == 14 &&
		   q.right_cm == 22 && q.action == OBS_ACT_TURN_LEFT && seq == 5,
		   "i8 profile message packs and unpacks");
	TEST_CHECK("S5-U07", OBS_MSG_BYTES == 8, "i8 message is 8 bytes (two packed words)");

	/* a task sets its own fault bit and it is seen, then cleared */
	tk_clr_flg(flg_fault_i14, 0);	/* clear everything for the test */
	tk_set_flg(flg_fault_i14, FLT_LINE_BIT);
	tk_set_flg(flg_fault_i14, FLT_IMU_BIT);
	TEST_CHECK("S5-U08", tk_wai_flg(flg_fault_i14, FLT_LINE_BIT | FLT_IMU_BIT, TWF_ANDW,
		   &got, TMO_POL) == E_OK &&
		   (got & (FLT_LINE_BIT | FLT_IMU_BIT)) == (FLT_LINE_BIT | FLT_IMU_BIT),
		   "two simultaneous fault bits are both kept");
	tk_wai_flg(flg_fault_i14, FLT_LINE_BIT, TWF_ORW | TWF_BITCLR, &got, TMO_POL);
	TEST_CHECK("S5-U09", tk_wai_flg(flg_fault_i14, FLT_IMU_BIT, TWF_ORW, &got, TMO_POL) == E_OK &&
		   tk_wai_flg(flg_fault_i14, FLT_LINE_BIT, TWF_ORW, &got, TMO_POL) == E_TMOUT,
		   "clearing one fault bit leaves the other set (TWF_BITCLR)");
	tk_clr_flg(flg_fault_i14, 0);

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
