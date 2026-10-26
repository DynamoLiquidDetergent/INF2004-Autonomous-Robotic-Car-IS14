/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 *
 *	S2-U01, S2-U02  template example producer (kept, see subsystem2.h B)
 *	S2-U03..S2-U11  Motion Control.  None of them turns the motors:
 *	                every command sent has speed 0 or is blocked.
 *	S2-M05          optional wheels-up drive test, set S2_TEST_DRIVE to 1.
 *
 *	PC unit tests of the same maths: testing/subsystem2/host/.
 */

#include "subsystem2.h"
#include "drv_motor.h"
#include "motion_math.h"
#include "../test_report.h"

#if APP_TEST

/* 1 = also run S2-M05: drives BOTH wheels for 2 s.  LIFT THE ROBOT FIRST. */
#ifndef S2_TEST_DRIVE
#define S2_TEST_DRIVE		0
#endif

#define T_SETTLE_MS		(3 * S2_CYCLE_MS)	/* long enough for 2+ cycles */
#define T_TIMING_MS		1000			/* run time before timing check */
#define T_MAX_LATE_US		2000U			/* NFR1: <= 2 ms extra jitter */
#define T_MAX_EXEC_US		1000U			/* WCET budget 0.3 ms, test 1 ms */
#define T_DRIVE_MMPS		200
#define T_DRIVE_MS		2000

LOCAL UINT next_epoch(void)
{
	return (s2_status_epoch(s2_get_status()) + 1U) & S2_MASK_4;
}

LOCAL ER send_cmd(UW word)
{
	return tk_snd_mbf(q_motion_cmd, &word, (INT)sizeof(word), TMO_POL);
}

EXPORT void subsystem2_test(void)
{
	T_RSEM		rsem;
	T_RMPF		rmpf;
	T_CFLG		cflg = {0};
	S2_TIMING	t0;
	S2_TIMING	t1;
	UW		seq;
	UW		w;
	INT		v;
	INT		bad = 0;
	UINT		e;
	UINT		m;
	ID		flg;
	BOOL		ok;

	TEST_INIT();

	/* ---------------- template example (unchanged) ---------------- */

	/* S2-U01: every sample value is inside the documented 0..100 range */
	for (seq = 0; seq < 40; seq++) {
		v = s2_make_value(seq);
		if (v < 0 || v > 100) bad++;
	}
	TEST_CHECK("S2-U01", bad == 0, "sample values stay within 0..100");

	/* S2-U02: flow control never over- or under-counts */
	TEST_CHECK("S2-U02",
		   tk_ref_sem(sem_slots, &rsem) == E_OK &&
		   rsem.semcnt >= 0 && rsem.semcnt <= S2_POOL_COUNT &&
		   tk_ref_mpf(mpf_samples, &rmpf) == E_OK &&
		   rmpf.frbcnt <= S2_POOL_COUNT,
		   "free slots and free blocks within pool size");

	/* ---------------- Motion Control ---------------- */

	/* S2-U03: packed command round trip at the field limits */
	ok = TRUE;
	for (m = 0U; m < 8U; m++) {
		for (e = 0U; e < S2_EPOCH_MOD; e++) {
			w = s2_cmd_pack(m, -S2_SPEED_LIMIT, -512, e);
			ok = ok && (s2_cmd_mode(w) == m) && (s2_cmd_epoch(w) == e)
			        && (s2_cmd_speed(w) == -S2_SPEED_LIMIT) && (s2_cmd_param_s(w) == -512);
			w = s2_cmd_pack(m, S2_SPEED_LIMIT, 1023, e);
			ok = ok && (s2_cmd_speed(w) == S2_SPEED_LIMIT) && (s2_cmd_param_u(w) == 1023U)
			        && ((w >> 27) == 0U);
		}
	}
	TEST_CHECK("S2-U03", ok, "IF-10 command pack/unpack at field limits (27 bits)");

	/* S2-U04: epoch staleness, including the 15 -> 0 wrap */
	TEST_CHECK("S2-U04",
		   motion_epoch_is_stale(4U, 5U) && !motion_epoch_is_stale(5U, 5U) &&
		   !motion_epoch_is_stale(6U, 5U) && motion_epoch_is_stale(15U, 0U) &&
		   !motion_epoch_is_stale(0U, 15U),
		   "stale epochs detected, wrap 15 -> 0 handled");

	/* S2-U05: the 50 Hz task is running */
	s2_get_timing(&t0);
	tk_dly_tsk(10 * S2_CYCLE_MS);
	s2_get_timing(&t1);
	TEST_CHECK("S2-U05", (t1.cycles - t0.cycles) >= 8U,
		   "motion_task runs about every 20 ms (8+ cycles in 200 ms)");

	/* S2-U06: timing evidence for NFR9 / NFR1 */
	tk_dly_tsk(T_TIMING_MS);
	s2_get_timing(&t1);
	tm_printf((UB*)"[TEST] S2 timing: cycles=%u max_exec_us=%u max_late_us=%u\n",
		  t1.cycles, t1.max_exec_us, t1.max_late_us);
	TEST_CHECK("S2-U06", (t1.max_exec_us < T_MAX_EXEC_US) && (t1.max_late_us < T_MAX_LATE_US),
		   "cycle WCET < 1 ms and release jitter < 2 ms");

	/* S2-U07: a STOP command is accepted and its epoch reported */
	e = next_epoch();
	(void)send_cmd(s2_cmd_pack(S2_MODE_STOP, 0, 0, e));
	tk_dly_tsk(T_SETTLE_MS);
	w = s2_get_status();
	TEST_CHECK("S2-U07", (s2_status_epoch(w) == e) && (s2_status_state(w) == S2_STATE_IDLE)
		   && (motor_get_duty(0U) == 0) && (motor_get_duty(1U) == 0),
		   "STOP accepted: epoch reported, state IDLE, PWM 0");

	/* S2-U08: an older epoch is dropped */
	s2_get_timing(&t0);
	(void)send_cmd(s2_cmd_pack(S2_MODE_STOP, 0, 0, (e - 1U) & S2_MASK_4));
	tk_dly_tsk(T_SETTLE_MS);
	s2_get_timing(&t1);
	TEST_CHECK("S2-U08", (s2_status_epoch(s2_get_status()) == e) && (t1.cmds_stale > t0.cmds_stale),
		   "command with an older epoch is discarded");

	/* S2-U09: command watchdog.  DRIVE at speed 0, then silence. */
	e = next_epoch();
	(void)send_cmd(s2_cmd_pack(S2_MODE_DRIVE, 0, 0, e));
	tk_dly_tsk(T_SETTLE_MS);
	ok = (s2_status_state(s2_get_status()) == S2_STATE_DRIVING);
	tk_dly_tsk(S2_CMD_TIMEOUT_MS + T_SETTLE_MS);
	TEST_CHECK("S2-U09", ok && (s2_status_state(s2_get_status()) == S2_STATE_TIMEOUT),
		   "no command for 100 ms -> state TIMEOUT (motors stopped)");

	/* S2-U10: inhibit beats commands, and needs a NEWER epoch to release.
	   Uses a temporary event flag, so it runs only while the Safety
	   Monitor (subsystem 5) has not bound its own. */
	if (!s2_safety_is_bound()) {
		cflg.flgatr  = TA_TFIFO | TA_WMUL;
		cflg.iflgptn = 1U;			/* inhibit already set */
		flg = tk_cre_flg(&cflg);
		subsystem2_bind_safety(flg, 1U, 0, 0U);
		e = s2_status_epoch(s2_get_status());	/* epoch when the latch is set */
		(void)send_cmd(s2_cmd_pack(S2_MODE_DRIVE, 0, 0, (e + 1U) & S2_MASK_4));
		tk_dly_tsk(T_SETTLE_MS);
		ok = (s2_status_state(s2_get_status()) == S2_STATE_INHIBITED)
		  && (s2_status_epoch(s2_get_status()) == e)	/* command was discarded */
		  && (motor_get_duty(0U) == 0) && (motor_get_duty(1U) == 0);
		(void)tk_clr_flg(flg, 0U);		/* release the latch */
		(void)send_cmd(s2_cmd_pack(S2_MODE_STOP, 0, 0, e));	/* same epoch: refused */
		tk_dly_tsk(T_SETTLE_MS);
		ok = ok && (s2_status_state(s2_get_status()) == S2_STATE_INHIBITED);
		(void)send_cmd(s2_cmd_pack(S2_MODE_STOP, 0, 0, (e + 1U) & S2_MASK_4));
		tk_dly_tsk(T_SETTLE_MS);
		ok = ok && (s2_status_state(s2_get_status()) == S2_STATE_IDLE);
		subsystem2_bind_safety(0, 0U, 0, 0U);
		(void)tk_del_flg(flg);
		TEST_CHECK("S2-U10", ok, "inhibit: commands discarded, PWM 0, released only by a newer epoch");
	} else {
		tm_printf((UB*)"[TEST] S2-U10 SKIP  Safety Monitor bound; covered by INT tests\n");
	}

	/* S2-U11: the shared GPIO hook protects the encoder pins */
	TEST_CHECK("S2-U11",
		   (s2_gpio_hook_register(S2_PIN_ENC_R_A, S2_GPIO_EDGE_RISE, (S2_GPIO_HOOK)1) == E_OBJ) &&
		   (s2_gpio_hook_register(40U, S2_GPIO_EDGE_RISE, (S2_GPIO_HOOK)1) == E_PAR),
		   "GPIO hook rejects encoder pins and bad pin numbers");

#if S2_TEST_DRIVE
	/* S2-M05: wheels-up drive.  Both wheels must turn forward at about
	   200 mm/s; watch them, then read the printed speeds and distance. */
	{
		UINT	i;

		tm_printf((UB*)"[TEST] S2-M05 driving both wheels for 2 s...\n");
		e = next_epoch();
		for (i = 0U; i < (UINT)(T_DRIVE_MS / (S2_CMD_TIMEOUT_MS / 2)); i++) {
			(void)send_cmd(s2_cmd_pack(S2_MODE_DRIVE, T_DRIVE_MMPS, 0, e));
			tk_dly_tsk(S2_CMD_TIMEOUT_MS / 2);
		}
		w = s2_get_status();
		tm_printf((UB*)"[TEST] S2-M05 speed L=%d R=%d mm/s, distance=%u mm\n",
			  s2_status_speed_l(w), s2_status_speed_r(w),
			  s2_odo_dist_mm(s2_get_odometry()));
		(void)send_cmd(s2_cmd_pack(S2_MODE_STOP, 0, 0, e));
	}
#endif

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
