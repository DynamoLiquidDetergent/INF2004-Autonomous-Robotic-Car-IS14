/*
 *	vehicle_controller.c
 *	Subsystem 5 -- Vehicle Controller task (priority S5_PRI_VEHICLE).
 *
 *	Version 1 (S5_PHASE 6): follows the line (IF-05), stands still while
 *	Obstacle Detection is scanning or the Safety Monitor's stop latch is set,
 *	then drives on with a newer epoch. It is the only sender to Motion
 *	Control (IF-10): subsystem 3's drive demo MUST be off (S3_BC_DRIVE_DEMO 0
 *	in s3_private.h).
 *
 *	The decisions are in vc_logic.c (tested on a PC); this file reads the
 *	inputs, calls vc_step() and sends the command.
 */

#include "subsystem5.h"
#include "vc_logic.h"
#include "../subsystem2/subsystem2.h"	/* IF-10, IF-16, IF-17 */
#include "../subsystem3/subsystem3.h"	/* IF-05, calibration */
#include "../subsystem1/subsystem1.h"	/* IF-13 */

#if S5_PHASE >= 6

#define VC_SEND_MS		50	/* < S2_CMD_TIMEOUT_MS (100 ms) */
#define VC_NODATA_MS		200	/* no line message this long -> stop */
#define VC_PRINT_MS		500
#define VC_STATUS_MS		500	/* IF-13 period */
#define VC_PLACE_MS		3000	/* time to put the robot on the line */

EXPORT volatile UB	s5_mission_state = VC_MS_INIT;

/* Bench only: without subsystem 1 nobody sends IF-14. With VC_BENCH_SCRIPT 1 this task
   sends the commands itself, through the same buffer, at these times after the run
   starts (ms): START (RESUME), PAUSE, RESUME, ABORT. */
#ifndef VC_BENCH_SCRIPT
#define VC_BENCH_SCRIPT		0
#endif
#if VC_BENCH_SCRIPT
LOCAL const struct { UW t_ms; UB cmd; } bench_script[] = {
	{  5000u, SUP_CMD_RESUME }, { 15000u, SUP_CMD_PAUSE },
	{ 19000u, SUP_CMD_RESUME }, { 40000u, SUP_CMD_ABORT },
};
#endif

LOCAL const char *ms_name(UINT ms)
{
	static const char *const n[] = { "INIT", "READY", "RUNNING", "PAUSED", "AVOIDING",
					 "FAULT", "ABORTED", "TURNING" };

	return (ms < (sizeof(n) / sizeof(n[0]))) ? n[ms] : "?";
}

/* IF-13: build the status and hand it to subsystem 1 (non-blocking; S1 keeps the newest) */
LOCAL void send_status(UINT ms, UW run_ms, UINT barcodes, UINT last_action)
{
	VC_STATUS_IN	si;
	VC_STATUS	vs;
	s1_status_t	st;
	UW		sum = s5_obs_summary;

	si.obst_present  = obs_summary_present(sum);
	si.obst_dist_cm  = obs_summary_dist(sum);
	si.obst_width_cm = obs_summary_width(sum);
	si.obst_side     = obs_summary_side(sum);
	si.mission_state = ms;
	si.fault_mask    = s5_fault_mask;
	si.run_ms        = run_ms;
	si.letter        = -1;			/* subsystem 3 sends the movement, not the letter */
	si.last_action   = last_action;
	si.barcodes      = barcodes;
	si.hump_mm       = -1;			/* subsystem 4 has no hump interface yet */
	vc_status_fill(&si, &vs);

	st.mission_time_ds = (uint16_t)vs.mission_time_ds;
	st.obst_dist_cm    = (uint16_t)vs.obst_dist_cm;
	st.mission_state   = (uint8_t)vs.mission_state;
	st.fault_mask      = (uint8_t)vs.fault_mask;
	st.letter          = (uint8_t)vs.letter;
	st.last_action     = (uint8_t)vs.last_action;
	st.barcodes        = (uint8_t)vs.barcodes;
	st.b_obstacle      = (uint8_t)vs.b_obstacle;
	st.obst_side       = (uint8_t)vs.obst_side;
	st.obst_width_cm   = (uint8_t)vs.obst_width_cm;
	st.hump_peak_mm    = (uint8_t)vs.hump_peak_mm;
	(void)s1_status_send(&st);
}

/* IF-14: take every waiting supervisor message. ABORT wins; otherwise the last
   PAUSE / RESUME counts. Returns a VC_SUP_* value; *link gets the newest link bit. */
LOCAL UINT take_sup_cmd(int *link)
{
	UB	b;
	UINT	cmd = VC_SUP_NONE, c;

	while (tk_rcv_mbf(mbf_sup_cmd, &b, TMO_POL) == (INT)sizeof(b)) {
		c = (UINT)b & SUP_CMD_MASK;
		*link = ((b & SUP_LINK_UP) != 0) ? 1 : 0;
		if (c != SUP_CMD_NONE && cmd != VC_SUP_ABORT) {
			cmd = c;
		}
	}
	return cmd;
}

LOCAL UW now_ms(void)
{
	SYSTIM t;

	tk_get_otm(&t);
	return t.lo;
}

LOCAL void send_cmd(UW word)
{
	(void)tk_snd_mbf(q_motion_cmd, &word, (INT)sizeof(word), TMO_POL);
}

EXPORT void vehicle_controller_task(INT stacd, void *exinf)
{
	VC_STATE	v;
	VC_IN		in = {0, VC_ZONE_CENTRE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	VC_OUT		o = {VC_MODE_STOP, 0, 0, 0, 0, 0, VC_MS_INIT};
	UW		last_status = 0, run_start = 0, run_frozen = 0;
	int		run_on = 0;
	UINT		barcodes = 0, last_action = 0;
	int		link = -1;
	int		new_link = -1;
	UINT		last_ms = 0xFFu;
	LINE_DATA_MSG	m, newer;
	UW		start, now, last_data, last_step, last_print, st;
	UINT		last_state = 0xFFFFu;
	INT		rc;
	int		last_hold = 0;
	int		last_creep = 0;
	int		done_sent = 0;

	(void)stacd;
	(void)exinf;

	s5_mission_state = VC_MS_INIT;
	tm_printf((UB*)"vc: calibrating the line sensor for %d s -- sweep the robot slowly "
		  "side to side across the black line (or leave it: defaults kept)\n",
		  (INT)(S3_CALIB_TIMEOUT_MS / 1000));
	if (subsystem3_run_calibration()) {
		tm_printf((UB*)"vc: calibration done\n");
	} else {
		tm_printf((UB*)"vc: calibration not done, default range kept\n");
	}
	tm_printf((UB*)"vc: put the robot on the line, centre sensor over it -- starting in %d s\n",
		  (INT)(VC_PLACE_MS / 1000));
	tk_dly_tsk(VC_PLACE_MS);

	/* a newer epoch than anything Motion Control has seen */
	vc_init(&v, (s2_status_epoch(s2_get_status()) + 1U) & S2_MASK_4);

	/* old messages from the countdown are stale */
	while (tk_rcv_mbf(q_line_data, &newer, TMO_POL) == (INT)sizeof(newer)) { }

	start = last_data = last_step = last_print = now_ms();
	in.ready      = 1;
	in.auto_start = VC_AUTO_START;
#if VC_AUTO_START
	tm_printf((UB*)"vc: following the line at up to %d mm/s (starts by itself; VC_AUTO_START 0 waits for RESUME)\n",
		  VC_LF_SPEED);
#else
	tm_printf((UB*)"vc: ready, waiting for START (RESUME from the dashboard)\n");
#endif

	for (;;) {
		/* the next line state (every 20 ms), but never wait longer than
		   VC_SEND_MS so the motion command keeps being sent */
		rc  = tk_rcv_mbf(q_line_data, &m, VC_SEND_MS);
		now = now_ms();

		in.dt_ms = 0;
		if (rc == (INT)sizeof(m)) {
			while (tk_rcv_mbf(q_line_data, &newer, TMO_POL) == (INT)sizeof(newer)) {
				m = newer;		/* fell behind: use the newest */
			}
			in.zone       = line_data_zone(m);
			in.lost       = line_data_lost(m) ? 1 : 0;
			in.line_fresh = 1;
			in.dt_ms      = now - last_step;
			last_step     = now;
			last_data     = now;
		} else if ((now - last_data) >= VC_NODATA_MS) {
			in.line_fresh = 0;		/* line sensor silent: do not drive blind */
		}
		st = s2_get_status();
		in.obs_state    = s5_obs_state;
		in.motion_state = s2_status_state(st);
		in.approach_mm  = s5_approach_mm;
		in.odo_mm       = s2_odo_dist_mm(s2_get_odometry());
		in.fault_mask   = s5_fault_mask;
#if VC_BENCH_SCRIPT
		{
			static UINT next_step;

			if (next_step < (sizeof(bench_script) / sizeof(bench_script[0])) &&
			    (now - start) >= bench_script[next_step].t_ms) {
				UB b = (UB)(bench_script[next_step].cmd | SUP_LINK_UP);

				(void)tk_snd_mbf(mbf_sup_cmd, &b, 1, TMO_POL);
				next_step++;
			}
		}
#endif
		in.sup_cmd      = take_sup_cmd(&new_link);
		if (new_link != link) {
			link = new_link;
			tm_printf((UB*)"vc: dashboard link %s\n", link ? "up" : "down");
		}
		if (in.sup_cmd != VC_SUP_NONE) {
			tm_printf((UB*)"vc: command from the dashboard: %s\n",
				  (in.sup_cmd == VC_SUP_PAUSE) ? "PAUSE" :
				  (in.sup_cmd == VC_SUP_RESUME) ? "RESUME" : "ABORT");
		}

		/* the line controller only advances on a new line message (dt_ms > 0);
		   in between the previous answer stays valid */
		if (in.dt_ms > 0 || !in.line_fresh || in.obs_state != 0 ||
		    in.motion_state == VC_STATE_INHIBITED || in.approach_mm != 0 || last_creep ||
		    in.sup_cmd != VC_SUP_NONE || o.mission_state != VC_MS_RUNNING) {
			o = vc_step(&v, &in);
		}
		s5_mission_state = (UB)o.mission_state;
		if (!run_on && v.started) {
			run_on = 1;
			run_start = now;			/* the time in the status counts from START */
		}
		if (run_on && !v.aborted) {
			run_frozen = now - run_start;		/* stops counting at ABORT */
		}
		if ((now - last_status) >= VC_STATUS_MS) {
			send_status(o.mission_state, run_frozen, barcodes, last_action);
			last_status = now;
		}
		if (o.mission_state != last_ms) {
			tm_printf((UB*)"vc: mission state %s\n", ms_name(o.mission_state));
			last_ms = o.mission_state;
		}

		if (o.mode == VC_MODE_DRIVE) {
			send_cmd(s2_cmd_pack(S2_MODE_DRIVE, o.speed, o.steer, o.epoch));
		} else {
			send_cmd(s2_cmd_pack(S2_MODE_STOP, 0, 0, o.epoch));
		}

		if (o.holding != last_hold) {
			tm_printf((UB*)"vc: %s (obstacle state %d, motion state %d, epoch %d)\n",
				  o.holding ? "HOLD" : "driving on", (INT)in.obs_state,
				  (INT)in.motion_state, (INT)o.epoch);
			last_hold = o.holding;
		}
		/* a creep / back-away order is open (s5_approach_mm != 0) */
		if ((in.approach_mm != 0) != last_creep) {
			if (in.approach_mm != 0 && !o.holding) {
				tm_printf((UB*)"vc: %s %d mm (epoch %d)\n",
					  (in.approach_mm < 0) ? "backing away" : "creeping forward along the line",
					  (INT)((in.approach_mm < 0) ? -in.approach_mm : in.approach_mm), (INT)o.epoch);
				last_creep = 1;
			} else if (in.approach_mm == 0) {
				last_creep = 0;
			}
		}
		if (o.creep_done && !done_sent) {
			tk_set_flg(flg_s5_ctrl, S5_CTRL_APPROACH_DONE);
			done_sent = 1;
			tm_printf((UB*)"vc: creep done, odometry %d mm\n", (INT)in.odo_mm);
		} else if (!o.creep_done && in.approach_mm == 0) {
			done_sent = 0;
		}
		if (in.motion_state != last_state) {
			tm_printf((UB*)"vc: motion state %d\n", (INT)in.motion_state);
			last_state = in.motion_state;
		}
		if ((now - last_print) >= VC_PRINT_MS) {
			tm_printf((UB*)"vc: t=%5d zone=%d lost=%d cmd %4d mm/s steer %4d  wheels L=%4d R=%4d  dist=%5d%s\n",
				  (INT)(now - start), (INT)in.zone, (INT)in.lost,
				  (INT)((o.mode == VC_MODE_DRIVE) ? o.speed : 0), (INT)o.steer,
				  (INT)s2_status_speed_l(st), (INT)s2_status_speed_r(st),
				  (INT)s2_odo_dist_mm(s2_get_odometry()),
				  v.line_gave_up ? "  LINE LOST FOR GOOD" : "");
			last_print = now;
		}
	}
}

#else  /* below phase 6: no Vehicle Controller */

EXPORT void vehicle_controller_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
}

#endif
