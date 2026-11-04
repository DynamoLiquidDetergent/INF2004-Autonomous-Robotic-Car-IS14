/*
 *	vehicle_controller.c
 *	Subsystem 5 -- Vehicle Controller task (priority S5_PRI_VEHICLE).
 *
 *	Version 3 (S5_PHASE 6): follows the line (IF-05), carries out the barcode
 *	commands (IF-07) at junctions, stands still while
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

/* Bench only: no barcode strip needed. With VC_BENCH_INJECT 1 this task puts a barcode
   command into q_barcode_cmd by itself, in turn LEFT, RIGHT, U-TURN, FORWARD: the first
   3 s after START, the next 3 s after each junction is done (or the command was dropped).
   The car then takes the next junction of the track with it. */
#ifndef VC_BENCH_INJECT
#define VC_BENCH_INJECT		0
#endif
#if VC_BENCH_INJECT
LOCAL const UB bench_inject[] = { VC_BC_LEFT, VC_BC_RIGHT, VC_BC_UTURN, VC_BC_FORWARD };
#endif

LOCAL const char *ms_name(UINT ms)
{
	static const char *const n[] = { "INIT", "READY", "RUNNING", "PAUSED", "AVOIDING",
					 "FAULT", "ABORTED", "TURNING" };

	return (ms < (sizeof(n) / sizeof(n[0]))) ? n[ms] : "?";
}

/* IF-13: build the status and hand it to subsystem 1 (non-blocking; S1 keeps the newest) */
LOCAL void send_status(UINT ms, UW run_ms, UINT barcodes, UINT last_action, INT letter)
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
	si.letter        = letter;		/* 0 = A .. 25 = Z, -1 none yet */
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

/* IF-07: take every waiting barcode command; the newest counts, all are counted.
   Also the newest decoded letter (ASCII 'A'..'Z') for the status. */
LOCAL UINT take_barcode_cmd(UINT *n)
{
	UB	b;
	UINT	cmd = VC_BC_NONE;

	*n = 0;
	while (tk_rcv_mbf(q_barcode_cmd, &b, TMO_POL) == (INT)sizeof(b)) {
		if (b >= VC_BC_FORWARD && b <= VC_BC_RIGHT) {
			cmd = b;
			(*n)++;
		}
	}
	return cmd;
}

LOCAL void take_letter(INT *letter)
{
	UB	c;

	while (tk_rcv_mbf(q_decoded_char, &c, TMO_POL) == (INT)sizeof(c)) {
		*letter = (c >= 'A' && c <= 'Z') ? (INT)(c - 'A') : -1;
	}
}

LOCAL const char *bc_name(UINT c)
{
	static const char *const n[] = { "NONE", "FORWARD", "U-TURN", "LEFT", "RIGHT" };

	return (c < (sizeof(n) / sizeof(n[0]))) ? n[c] : "?";
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
	VC_IN		in = {0};
	VC_OUT		o = {0};
	UW		last_status = 0, run_start = 0, run_frozen = 0, last_vc = 0;
	INT		letter = -1;
	UINT		bc_n;
	int		run_on = 0;
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

	in.zone = VC_ZONE_CENTRE;
	o.mode = VC_MODE_STOP;
	o.mission_state = VC_MS_INIT;
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

	start = last_data = last_step = last_print = last_vc = now_ms();
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
			in.junction   = line_data_junction(m) ? 1 : 0;
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
		in.obst_present = obs_summary_present(s5_obs_summary) ? 1 : 0;
		in.wheel_l      = s2_status_speed_l(st);
		in.wheel_r      = s2_status_speed_r(st);
		in.motion_done  = s2_status_done(st) ? 1 : 0;
#if VC_BENCH_INJECT
		{
			static UINT inj_i;
			static UW inj_at;
			static int inj_wait = 1;

			if (v.started && inj_at == 0) {
				inj_at = now + 3000;
			}
			if (inj_at != 0 && !inj_wait && (o.event & (VC_EV_JN_DONE | VC_EV_BC_EXPIRED))) {
				inj_at = now + 3000;
				inj_wait = 1;
			}
			if (inj_at != 0 && inj_wait && (INT)(now - inj_at) >= 0 && !o.busy &&
			    v.bc_pending == VC_BC_NONE) {
				UB c = bench_inject[inj_i % (sizeof(bench_inject) / sizeof(bench_inject[0]))];

				(void)tk_snd_mbf(q_barcode_cmd, &c, 1, TMO_POL);
				inj_i++;
				inj_wait = 0;
				tm_printf((UB*)"vc: bench: command %s put into the barcode queue\n", bc_name(c));
			}
		}
#endif
		in.bc_cmd       = take_barcode_cmd(&bc_n);
		in.bc_n         = bc_n;
		take_letter(&letter);
		if (in.bc_n > 0) {
			tm_printf((UB*)"vc: barcode command %s (letter %c)\n", bc_name(in.bc_cmd),
				  (letter >= 0) ? (char)('A' + letter) : '?');
		}
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
		    in.sup_cmd != VC_SUP_NONE || o.mission_state != VC_MS_RUNNING ||
		    o.busy || in.bc_n > 0) {
			in.elapsed_ms = now - last_vc;
			if (in.elapsed_ms > 100) {
				in.elapsed_ms = 100;		/* a long gap must not count as a long spin */
			}
			last_vc = now;
			o = vc_step(&v, &in);
			if (o.event != 0) {
				if (o.event & VC_EV_JN_START) {
					tm_printf((UB*)"vc: junction -> %s (epoch %d)\n", bc_name(v.jn_cmd), (INT)o.epoch);
				}
				if (o.event & VC_EV_TURN_TIMEOUT) {
					tm_printf((UB*)"vc: WARNING turn did not finish in %d ms, carrying on\n",
						  (INT)VC_TURN_TIMEOUT_MS);
				}
				if ((o.event & VC_EV_JN_DONE) && v.jn_cmd != VC_BC_FORWARD) {
					tm_printf((UB*)"vc: junction done, turned about %d degrees\n",
						  (INT)(((v.turn_arc_um * v.turn_dir * 573) / (VC_TRACK_MM * 1000)) / 10));
				}
				if (o.event & VC_EV_SEARCH) {
					tm_printf((UB*)"vc: line lost, spinning on the spot to look for it (%s first)\n",
						  (v.search_dir > 0) ? "right" : "left");
				}
				if (o.event & VC_EV_FOUND) {
					tm_printf((UB*)"vc: line found again by the search\n");
				}
				if (o.event & VC_EV_GAVE_UP) {
					tm_printf((UB*)"vc: line not found, stopped (line fault reported)\n");
					(void)tk_set_flg(flg_fault_i14, FLT_LINE_BIT);
				}
				if (o.event & VC_EV_BC_EXPIRED) {
					tm_printf((UB*)"vc: barcode command dropped, no junction within %d mm\n",
						  (INT)VC_BC_EXPIRE_MM);
				}
			}
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
			send_status(o.mission_state, run_frozen, o.barcodes, o.last_action, letter);
			last_status = now;
		}
		if (o.mission_state != last_ms) {
			tm_printf((UB*)"vc: mission state %s\n", ms_name(o.mission_state));
			last_ms = o.mission_state;
		}

		if (o.mode == VC_MODE_DRIVE) {
			send_cmd(s2_cmd_pack(S2_MODE_DRIVE, o.speed, o.steer, o.epoch));
		} else if (o.mode == VC_MODE_TURN) {
			send_cmd(s2_cmd_pack(S2_MODE_TURN, o.speed, o.turn_deg, o.epoch));
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
			tm_printf((UB*)"vc: t=%5d zone=%d lost=%d jn=%d cmd %4d mm/s steer %4d  wheels L=%4d R=%4d  dist=%5d%s\n",
				  (INT)(now - start), (INT)in.zone, (INT)in.lost, (INT)in.junction,
				  (INT)((o.mode == VC_MODE_STOP) ? 0 : o.speed),
				  (INT)((o.mode == VC_MODE_TURN) ? o.turn_deg : o.steer),
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
