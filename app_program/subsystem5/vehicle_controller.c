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

#if S5_PHASE >= 6

#define VC_SEND_MS		50	/* < S2_CMD_TIMEOUT_MS (100 ms) */
#define VC_NODATA_MS		200	/* no line message this long -> stop */
#define VC_PRINT_MS		500
#define VC_PLACE_MS		3000	/* time to put the robot on the line */

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
	VC_IN		in = {0, VC_ZONE_CENTRE, 0, 0, 0, 0, 0};
	VC_OUT		o = {VC_MODE_STOP, 0, 0, 0, 0, 0};
	LINE_DATA_MSG	m, newer;
	UW		start, now, last_data, last_step, last_print, st;
	UINT		last_state = 0xFFFFu;
	INT		rc;
	int		last_hold = 0;
	int		last_creep = 0;

	(void)stacd;
	(void)exinf;

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
	tm_printf((UB*)"vc: following the line at up to %d mm/s\n", VC_LF_SPEED);

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

		/* the line controller only advances on a new line message (dt_ms > 0);
		   in between the previous answer stays valid */
		if (in.dt_ms > 0 || !in.line_fresh || in.obs_state != 0 ||
		    in.motion_state == VC_STATE_INHIBITED || in.approach_mm != 0 || last_creep) {
			o = vc_step(&v, &in);
		}

		if (o.mode == VC_MODE_DRIVE) {
			send_cmd(s2_cmd_pack(S2_MODE_DRIVE, o.speed, o.steer, o.epoch));
		} else if (o.mode == VC_MODE_DRIVE_DIST) {
			send_cmd(s2_cmd_pack(S2_MODE_DRIVE_DIST, o.speed, (INT)o.param, o.epoch));
		} else {
			send_cmd(s2_cmd_pack(S2_MODE_STOP, 0, 0, o.epoch));
		}

		if (o.holding != last_hold) {
			tm_printf((UB*)"vc: %s (obstacle state %d, motion state %d, epoch %d)\n",
				  o.holding ? "HOLD" : "driving on", (INT)in.obs_state,
				  (INT)in.motion_state, (INT)o.epoch);
			last_hold = o.holding;
		}
		if ((o.mode == VC_MODE_DRIVE_DIST) != last_creep) {
			if (o.mode == VC_MODE_DRIVE_DIST) {
				tm_printf((UB*)"vc: %s %d mm at %d mm/s (epoch %d)\n",
					  (o.speed < 0) ? "backing away" : "creeping forward",
					  (INT)o.param, (INT)((o.speed < 0) ? -o.speed : o.speed), (INT)o.epoch);
			}
			last_creep = (o.mode == VC_MODE_DRIVE_DIST);
		}
		if (in.motion_state != last_state) {
			tm_printf((UB*)"vc: motion state %d\n", (INT)in.motion_state);
			last_state = in.motion_state;
		}
		if ((now - last_print) >= VC_PRINT_MS) {
			tm_printf((UB*)"vc: t=%5d zone=%d lost=%d cmd %4d mm/s steer %4d  wheels L=%4d R=%4d  dist=%5d%s\n",
				  (INT)(now - start), (INT)in.zone, (INT)in.lost,
				  (INT)((o.mode == VC_MODE_DRIVE || o.mode == VC_MODE_DRIVE_DIST) ? o.speed : 0),
				  (INT)o.steer,
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
