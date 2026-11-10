/*
 *	obstacle_detect.c
 *	Subsystem 5 -- Obstacle Detection Task (statechart 6.3).
 *
 *	IDLE_MONITORING -> COARSE_SCANNING -> FINE_SCANNING ->
 *	PROFILING_AND_PLANNING -> BYPASSING -> REACQUIRING_LINE -> IDLE_MONITORING
 *	with REVERSE_RETRY (up to 2 attempts) and FAULT.
 *
 *	This task NEVER drives the motors. It publishes the obstacle profile (i8)
 *	to the Vehicle Controller and raises its own bit in the fault flags
 *	(i14). The Vehicle Controller does the bypass and tells this task how it
 *	is going through flg_s5_ctrl.
 *
 *	It owns GP12 (servo), GP16/GP17 (HC-SR04) and PWM slices 6 and 0.
 *	It never busy-waits except the ~20 us trigger pulse; waits are
 *	tk_dly_tsk() or event flags.
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"
#if S5_PHASE >= 6
#include "../subsystem2/subsystem2.h"	/* s2_get_status(): has the creep finished? */
#endif

/* Creep before the second scan (phase 6 and up, needs the Vehicle Controller).
   The first scan starts when something is within start_cm (30 cm) straight ahead; the
   car stops about 2 cm later. If the profile chooses a bypass, the car creeps up to about 17 cm from
   the obstacle (following the line), or backs away if it is closer than 15 cm,
   and scans again from there: a closer scan has a narrower uncertainty, and it
   is a second scan position. The one-ray watch is not used to stop at 17 cm: at
   that range it covers only +/-2.6 cm of the 16 cm wide car, so a box off to one
   side would be missed. The first scan covers the whole width, the move is a
   fixed distance measured by the Vehicle Controller with the odometry, and the
   watch ahead only cuts it short. */
#define S5_APPROACH		(S5_PHASE >= 6)
#define S5_FINE_EDGES		(S5_PHASE >= 6)	/* before a bypass: look at both edges of the box in 3 degree steps */
#define S5_MAX_APPROACHES	2	/* moves per obstacle (forward or back), then the car just holds */
#define S5_APPROACH_START_MS	1500	/* Safety Monitor release (500 ms) and first command */
#define S5_APPROACH_SLOWEST	30	/* mm/s, for the time limit only (the car drives 40) */
#define S5_APPROACH_EXTRA_MS	1500

/* ---- Tunables. PLACEHOLDERS until measured on the robot. ---- */
#define S5_DETECT_CONFIRM	2	/* consecutive close readings before a scan starts */
#define S5_NOECHO_FAULT_N	10	/* consecutive "no pulse at all" readings = sensor fault.
					   0 = never fault (use if the kit's sensor stays silent
					   when nothing is in range: check on the bench) */
#define S5_SCAN_TIMEOUT_MS	6000	/* a whole scan must finish within this, else FAULT
					   (13 angles x ~250 ms with the median = about 3.3 s worst case) */
#define S5_PINGS		3	/* readings per angle; the median is used (1..5).
					   Each ping costs about 60 ms (HC-SR04 minimum cycle). */
#define S5_USE_BACKGROUND	1	/* take an empty-track scan at start-up and subtract it.
					   Needs the track in front of the robot to be CLEAR at power-up. */
#define S5_BYPASS_LIMIT_MS	25000	/* the detour (turns, sideways, past the obstacle, back to the line) or
					   the back-away of a retry must finish within this; the clock restarts
					   when it begins, so the scans before it do not count (NFR31's 5 s is
					   detection to profile, S5-M09) */
#define S5_MAX_RETRIES		2	/* REVERSE_RETRY attempts (statechart 6.3) */
#define S5_SERVO_SETTLE_BASE_MS	40	/* assumption A-servo: no SG90 datasheet supplied */
#define S5_SERVO_SETTLE_PER_DEG	2	/* ms per degree moved (SG90 is about 100 ms / 60 deg) */
#define S5_STANDALONE_REST_MS	1500	/* bench mode: pause after a scan before watching again */
#define S5_IDX_MIN		2	/* the servo is only ever sent between these scan indices:  */
#define S5_IDX_MAX		10	/* 2..10 = 30..150 degrees (index x 15). Never 0/180: at those
					   the sensor was thrown off its mount. The sensor was moved
					   forward on its bracket so it clears the car body at 30/150;
					   if it touches anything again, narrow these (4 and 8 = 60..120).
					   Keep MIN <= 5 and MAX >= 7 (the lane ahead is 5..7). */
/* coarse scan: every second index, starting so that index 6 (straight ahead) is included */
#define S5_COARSE_FIRST		(OBS_IDX_AHEAD - 2 * ((OBS_IDX_AHEAD - S5_IDX_MIN) / 2))
#define S5_SERVO_STEP_DEG	15	/* the servo is moved at most this far per command, so it
					   never snaps across the whole range */

EXPORT volatile UB	s5_obs_state = OBS_ST_IDLE_MONITORING;
EXPORT volatile H	s5_approach_mm = 0;
EXPORT volatile UW	s5_obs_summary = 0;

LOCAL OBS_CFG		cfg;
LOCAL OBS_SCAN		scan;
LOCAL OBS_PROFILE	profile;
LOCAL UINT		servo_deg = 90;
LOCAL BOOL		fine_done;		/* the edges of this obstacle have been looked at closely */
LOCAL UW		noecho_run;
LOCAL UB		seq;
LOCAL UW		dropped_msgs;
LOCAL UW		fault_ms;
#if S5_APPROACH
LOCAL UW		approaches;		/* creeps done for this obstacle; 0 again when the way is clear */
LOCAL UW		approach_t0;
LOCAL UW		approach_limit_ms;
LOCAL BOOL		approach_moving;	/* Motion Control has reported DRIVING_DIST */
LOCAL UW		guard_hits;
LOCAL UH		approach_odo0;		/* odometry when the move was ordered */
LOCAL BOOL		bg_shift_pending;	/* a move happened: re-project the background before the next profile */
#endif

LOCAL UW now_ms(void)
{
	SYSTIM t;

	tk_get_otm(&t);
	return t.lo;
}

LOCAL void set_state(OBS_STATE st)
{
	s5_obs_state = (UB)st;
}

LOCAL void raise_fault(const char *why)
{
	/* tk_set_flg is one kernel call: an OR on the shared flag word, so a
	   simultaneous fault from another task is never lost */
	tk_set_flg(flg_fault_i14, FLT_OBSTACLE_BIT);
	s5_approach_mm = 0;
	fault_ms = now_ms();
	set_state(OBS_ST_FAULT);
	tm_printf((UB*)"obstacle: FAULT (%s)\n", why);
}

/* Wait up to tmo ms for any of the bits in ptn; clears the bits it returns */
LOCAL UINT wait_ctrl(UINT ptn, TMO tmo)
{
	UINT got = 0;
	ER   er = tk_wai_flg(flg_s5_ctrl, ptn, TWF_ORW | TWF_BITCLR, &got, tmo);

	return (er == E_OK) ? got : 0;
}

LOCAL BOOL monitor_enabled(void)
{
	UINT got = 0;

	/* level, not event: look at the bit but leave it set */
	return tk_wai_flg(flg_s5_ctrl, S5_CTRL_MONITOR_EN, TWF_ORW, &got, TMO_POL) == E_OK;
}

/* Turn the servo to an angle and wait for it to get there */
LOCAL void servo_goto(UINT deg)
{
	/* never outside the allowed scan window */
	if (deg < S5_IDX_MIN * OBS_STEP_DEG) {
		deg = S5_IDX_MIN * OBS_STEP_DEG;
	}
	if (deg > S5_IDX_MAX * OBS_STEP_DEG) {
		deg = S5_IDX_MAX * OBS_STEP_DEG;
	}
	/* small steps: a servo told to jump far accelerates hard and stops hard */
	while (servo_deg != deg) {
		UINT next, diff;

		if (deg > servo_deg) {
			next = (deg - servo_deg > S5_SERVO_STEP_DEG) ? servo_deg + S5_SERVO_STEP_DEG : deg;
		} else {
			next = (servo_deg - deg > S5_SERVO_STEP_DEG) ? servo_deg - S5_SERVO_STEP_DEG : deg;
		}
		diff = (next > servo_deg) ? next - servo_deg : servo_deg - next;
		drv_servo_set_deg(next);
		servo_deg = next;
		tk_dly_tsk(S5_SERVO_SETTLE_BASE_MS + diff * S5_SERVO_SETTLE_PER_DEG);
	}
}

/* One reading. Updates the no-echo counter and may raise the sensor fault.
   Returns the distance in cm, or OBS_DIST_NONE for out of range / no echo. */
LOCAL UINT read_distance(void)
{
	US_RESULT r = drv_us_ping();
	UINT cm = OBS_DIST_NONE;

	if (r.status == US_NO_ECHO) {
		noecho_run++;
		if (S5_NOECHO_FAULT_N != 0 && noecho_run >= S5_NOECHO_FAULT_N &&
		    s5_obs_state != OBS_ST_FAULT) {
			raise_fault("no echo pulse: sensor missing or dead");
		}
	} else {
		noecho_run = 0;
		if (r.status == US_OK) {
			cm = r.dist_cm;
		}
	}
#if S5_VERBOSE
	tm_printf((UB*)"obstacle: %3d deg  echo %5d us  status %d  -> %d cm\n",
		  servo_deg, (INT)r.echo_us, (INT)r.status, (INT)cm);
#endif
	return cm;
}

/* S5_PINGS readings at the current angle, median of them (the servo has
   already settled). One stray echo cannot change the result. */
LOCAL UINT read_median(void)
{
	uint16_t v[5];
	unsigned n;

	for (n = 0; n < S5_PINGS && n < 5; n++) {
		v[n] = (uint16_t)read_distance();
		if (s5_obs_state == OBS_ST_FAULT) {
			n++;
			break;
		}
	}
	return obs_median(v, n);
}

/* reader for obs_fine_edges: servo to the angle, median distance there */
LOCAL uint16_t fine_read(void *ctx, unsigned deg)
{
	(void)ctx;
	servo_goto(deg);
	return (uint16_t)read_median();
}

LOCAL UINT measure_index(unsigned idx)
{
	UINT cm;

	servo_goto(idx * OBS_STEP_DEG);
	cm = read_median();
	obs_scan_set(&scan, idx, (uint16_t)cm);
	return cm;
}

/* Empty-track scan: remember what each angle sees when no obstacle is there,
   so a wall or the track edge is not mistaken for an obstacle later. */
LOCAL void take_background(void)
{
	unsigned i, ahead;

	tm_printf((UB*)"obstacle: taking background scan, keep the path ahead CLEAR\n");
	for (i = S5_IDX_MIN; i <= S5_IDX_MAX; i++) {
		servo_goto(i * OBS_STEP_DEG);
		obs_scan_set_bg(&scan, i, (uint16_t)read_median());
		if (s5_obs_state == OBS_ST_FAULT) {
			return;
		}
	}
	servo_goto(90);
	ahead = obs_bg_clear_ahead(&scan, &cfg);
	if (ahead != 0) {
		tm_printf((UB*)"obstacle: WARNING %d background value(s) ahead were close and are ignored. "
			  "Is something in front of the robot?\n", (INT)ahead);
	}
	for (i = S5_IDX_MIN; i <= S5_IDX_MAX; i++) {
		tm_printf((UB*)"obstacle: background %3d deg = %d cm%s\n", (INT)(i * OBS_STEP_DEG),
			  (INT)scan.bg_cm[i], (scan.bg_cm[i] == OBS_DIST_NONE) ? " (none)" : "");
	}
}

#if S5_APPROACH
/* A creep is in progress. Watch straight ahead with the tight guard distance and
   see whether Motion Control has finished the distance. TRUE = the creep is over,
   scan again. */
LOCAL BOOL approach_over(UINT cm)
{
	UINT st = s2_status_state(s2_get_status());
	UINT got = 0;
	const char *why = (const char*)0;
	OBS_CFG g = cfg;

	g.detect_cm = cfg.approach_guard_cm;
	/* the guard only applies going forward: when backing away the car starts
	   inside it */
	if (s5_approach_mm > 0 && obs_dist_is_hit(&scan, OBS_IDX_AHEAD, (uint16_t)cm, &g)) {
		guard_hits++;
	} else {
		guard_hits = 0;
	}
	if (st == S2_STATE_DRIVING) {
		approach_moving = TRUE;
	}
	if (guard_hits >= S5_DETECT_CONFIRM) {
		why = "obstacle at the guard distance";
	} else if (tk_wai_flg(flg_s5_ctrl, S5_CTRL_APPROACH_DONE, TWF_ORW | TWF_BITCLR, &got, TMO_POL) == E_OK) {
		why = "distance done";
	} else if (approach_moving && (st == S2_STATE_INHIBITED || st == S2_STATE_STALLED ||
				       st == S2_STATE_TIMEOUT)) {
		why = "car was stopped";
	} else if ((now_ms() - approach_t0) > approach_limit_ms) {
		why = "no movement in time";
	}
	if (why == (const char*)0) {
		return FALSE;
	}
	tm_printf((UB*)"obstacle: creep over (%s), %d cm ahead, motion state %d, scanning again\n",
		  why, (INT)cm, (INT)st);
	return TRUE;
}
#endif

LOCAL BOOL scan_timed_out(UW start_ms)
{
	if ((now_ms() - start_ms) > S5_SCAN_TIMEOUT_MS) {
		raise_fault("scan timed out");
		return TRUE;
	}
	return FALSE;
}

EXPORT void obstacle_detect_task(INT stacd, void *exinf)
{
	UW	detect_ms = 0;		/* when the obstacle was first seen (NFR31 clock) */
	UW	scan_ms = 0;
	UW	hits = 0;
	UW	retries = 0;
	UINT	cm, got;
	int	i, n;
#if S5_APPROACH
	int	mm;
#endif
	uint32_t w[OBS_MSG_WORDS];
	ER	er;

	obs_cfg_default(&cfg);

	er = drv_s5_init(DRV_PART_ALL);
	if (er != E_OK) {
		tm_printf((UB*)"obstacle: driver init failed (%d)\n", (INT)er);
		raise_fault("driver init failed");
	} else {
		drv_servo_set_deg(90);
		servo_deg = 90;
		tk_dly_tsk(300);	/* let the servo reach centre */
		set_state(OBS_ST_IDLE_MONITORING);
		tm_printf((UB*)"obstacle: ready (servo GP%d, trig GP%d, echo GP%d)\n",
			  S5_PIN_SERVO, S5_PIN_TRIG, S5_PIN_ECHO);
		obs_scan_init(&scan);
#if S5_USE_BACKGROUND
		take_background();
#endif
	}

	for (;;) {
		switch (s5_obs_state) {

		case OBS_ST_IDLE_MONITORING:
			if (!monitor_enabled()) {
				hits = 0;
				tk_dly_tsk(20);
				break;
			}
			servo_goto(90);
			cm = read_distance();
			/* summary for the status to subsystem 1: live distance while an obstacle is
			   known, gone once the way is clear */
			if (cm == OBS_DIST_NONE || cm > cfg.detect_cm + cfg.detect_hyst_cm) {
				s5_obs_summary = 0;
			} else if (obs_summary_present(s5_obs_summary)) {
				s5_obs_summary = obs_summary_with_dist(s5_obs_summary, (unsigned)cm);
			}
#if S5_APPROACH
			if (s5_approach_mm != 0) {
				if (approach_over(cm)) {
					hits = 0;
					detect_ms = now_ms();
					retries = 0;
					set_state(OBS_ST_COARSE_SCANNING);	/* first: the car holds again */
					s5_approach_mm = 0;
				}
				break;
			}
#endif
			if (cm <= cfg.start_cm && obs_dist_is_hit(&scan, OBS_IDX_AHEAD, (uint16_t)cm, &cfg)) {
				hits++;
			} else {
				hits = 0;
#if S5_APPROACH
				if (cm == OBS_DIST_NONE || cm > cfg.detect_cm + cfg.detect_hyst_cm) {
					approaches = 0;		/* the way is clear: a new obstacle may be approached */
				}
#endif
			}
			if (hits >= S5_DETECT_CONFIRM && s5_obs_state == OBS_ST_IDLE_MONITORING) {
				hits = 0;
				detect_ms = now_ms();
				retries = 0;
				set_state(OBS_ST_COARSE_SCANNING);
			}
			break;

		case OBS_ST_COARSE_SCANNING:
			/* every second angle inside the servo window (FR38: 30..150 when it is allowed) */
			scan_ms = now_ms();
			fine_done = FALSE;
			obs_scan_clear(&scan);		/* new readings, keep the background */
			for (i = S5_COARSE_FIRST; i <= S5_IDX_MAX; i += 2) {
				measure_index((unsigned)i);
				if (s5_obs_state == OBS_ST_FAULT || scan_timed_out(scan_ms)) {
					break;
				}
			}
			if (s5_obs_state == OBS_ST_FAULT) {
				break;
			}
			/* object confirmed if any coarse angle sees one */
			n = 0;
			for (i = S5_COARSE_FIRST; i <= S5_IDX_MAX; i += 2) {
				n += obs_scan_is_hit(&scan, (unsigned)i, &cfg);
			}
			set_state(n ? OBS_ST_FINE_SCANNING : OBS_ST_IDLE_MONITORING);
			break;

		case OBS_ST_FINE_SCANNING:
			/* Measure the unmeasured neighbours of EVERY obstacle (FR39), nearest to
			   the servo first so it travels as little as possible. Only inside the safe
			   servo window: an edge outside it stays unmeasured and the profile then
			   reports it as partial. At most 9 readings in all. */
			for (n = 0; n < OBS_NUM_ANGLES; n++) {
				int a = obs_next_fine_idx(&scan, &cfg, S5_IDX_MIN, S5_IDX_MAX,
							  (int)(servo_deg / OBS_STEP_DEG));

				if (a < 0) {
					break;
				}
				measure_index((unsigned)a);
				if (s5_obs_state == OBS_ST_FAULT || scan_timed_out(scan_ms)) {
					break;
				}
			}
			if (s5_obs_state != OBS_ST_FAULT) {
				set_state(OBS_ST_PROFILING_AND_PLANNING);
			}
			break;

		case OBS_ST_PROFILING_AND_PLANNING:
#if S5_APPROACH
			if (bg_shift_pending) {
				/* the car has moved since the background was taken (the signed
				   16 bit odometry difference, so the wrap is harmless) */
				INT moved = (INT)(H)(UH)(s2_odo_dist_mm(s2_get_odometry()) - approach_odo0);

				bg_shift_pending = FALSE;
				obs_bg_forget(&scan);
				tm_printf((UB*)"obstacle: the car moved %d mm, background forgotten (everything within %d cm counts)\n",
					  moved, (INT)cfg.detect_cm);
			}
#endif
			obs_profile(&scan, &cfg, &profile);
#if S5_FINE_EDGES
			/* About to bypass (no creep first): look at both edges of the box in 3 degree
			   steps, so the swing is planned for the real edges and not for the outer end of
			   a 15 degree step. */
			if (profile.detected && !fine_done &&
			    (profile.action == OBS_ACT_TURN_RIGHT || profile.action == OBS_ACT_TURN_LEFT ||
			     profile.action == OBS_ACT_REVERSE_RETRY) &&
			    !(approaches < S5_MAX_APPROACHES && obs_approach_mm(&profile, &cfg) != 0)) {
				UW t0 = now_ms();

				n = (int)obs_fine_edges(&scan, &cfg, S5_IDX_MIN * OBS_STEP_DEG,
							S5_IDX_MAX * OBS_STEP_DEG, fine_read, (void*)0);
				fine_done = TRUE;
				if (s5_obs_state == OBS_ST_FAULT) {
					break;
				}
				tm_printf((UB*)"obstacle: fine edges: %d readings in %d ms, right edge %d deg (%d cm), "
					  "left edge %d deg (%d cm) (0 = not refined)\n", n, (INT)(now_ms() - t0),
					  (INT)scan.fine_deg[0], (INT)scan.fine_cm[0],
					  (INT)scan.fine_deg[1], (INT)scan.fine_cm[1]);
				if (n != 0) {
					obs_profile(&scan, &cfg, &profile);
				}
			}
#endif
			servo_goto(90);			/* face forward again */
			s5_obs_summary = obs_summary_pack(&profile);
			seq = (UB)((seq + 1) & 0xF);
			obs_pack(&profile, seq, w);
			er = tk_snd_mbf(mbf_obstacle, w, OBS_MSG_BYTES, TMO_POL);
			if (er != E_OK) {
				dropped_msgs++;	/* buffer full: newest dropped, counted */
			}
			tm_printf((UB*)"obstacle: PROFILE seq %d detected %d closest %d cm @%d deg, "
				  "width %d cm (+/-%d), planned %d cm, left %d cm, right %d cm, action %d%s, objects %d, scan %d ms\n",
				  (INT)seq, (INT)profile.detected, (INT)profile.closest_cm,
				  (INT)profile.closest_idx * OBS_STEP_DEG, (INT)profile.width_cm,
				  (INT)profile.unc_cm, (INT)profile.plan_width_cm,
				  (INT)profile.left_cm, (INT)profile.right_cm, (INT)profile.action,
				  profile.partial ? " (partial)" : "", (INT)profile.clusters, (INT)(now_ms() - scan_ms));
			if (!profile.detected) {
				set_state(OBS_ST_IDLE_MONITORING);
#if S5_APPROACH
			/* before STOP / REVERSE_RETRY: STOP (closer than stop_cm) backs straight
			   away instead of faulting, when the reading can be trusted */
			} else if (approaches < S5_MAX_APPROACHES &&
				   (mm = obs_approach_mm(&profile, &cfg)) != 0) {
				approaches++;
				approach_moving = FALSE;
				guard_hits = 0;
				tk_clr_flg(flg_s5_ctrl, ~S5_CTRL_APPROACH_DONE);	/* no stale 'done' */
				approach_t0 = now_ms();
				approach_odo0 = s2_odo_dist_mm(s2_get_odometry());
				bg_shift_pending = TRUE;
				approach_limit_ms = S5_APPROACH_START_MS +
						    ((UW)((mm < 0) ? -mm : mm) * 1000u) / S5_APPROACH_SLOWEST +
						    S5_APPROACH_EXTRA_MS;
				tm_printf((UB*)"obstacle: %s %d mm to a gap of about %d cm "
					  "(move %d of %d)\n", (mm < 0) ? "back away" : "creep forward",
					  (INT)((mm < 0) ? -mm : mm), (INT)cfg.approach_cm,
					  (INT)approaches, (INT)S5_MAX_APPROACHES);
				s5_approach_mm = (H)mm;
				set_state(OBS_ST_IDLE_MONITORING);
#endif
			} else if (profile.action == OBS_ACT_STOP) {
				raise_fault("obstacle too close, no safe move");
			} else if (profile.action == OBS_ACT_REVERSE_RETRY) {
				detect_ms = now_ms();
				set_state(OBS_ST_REVERSE_RETRY);
			} else if (S5_STANDALONE) {
				tk_dly_tsk(S5_STANDALONE_REST_MS);
				set_state(OBS_ST_IDLE_MONITORING);
			} else {
				detect_ms = now_ms();		/* the bypass clock starts here */
				set_state(OBS_ST_BYPASSING);
			}
			break;

		case OBS_ST_BYPASSING:
			/* Vehicle Controller drives round the obstacle, then tells us */
			got = wait_ctrl(S5_CTRL_BYPASS_DONE, 50);
			if (got & S5_CTRL_BYPASS_DONE) {
				set_state(OBS_ST_REACQUIRING_LINE);
			} else if ((now_ms() - detect_ms) > S5_BYPASS_LIMIT_MS) {
				raise_fault("bypass over 8 s");
			}
			break;

		case OBS_ST_REACQUIRING_LINE:
			got = wait_ctrl(S5_CTRL_LINE_FOUND, 50);
			if (got & S5_CTRL_LINE_FOUND) {
				tm_printf((UB*)"obstacle: bypass done in %d ms\n", (INT)(now_ms() - detect_ms));
				set_state(OBS_ST_IDLE_MONITORING);
			} else if ((now_ms() - detect_ms) > S5_BYPASS_LIMIT_MS) {
				raise_fault("line not found in time");
			}
			break;

		case OBS_ST_REVERSE_RETRY:
			/* clearance too tight: Vehicle Controller backs away, we scan again */
			if (retries >= S5_MAX_RETRIES) {
				raise_fault("retries used up");
				break;
			}
			got = wait_ctrl(S5_CTRL_REVERSE_DONE, S5_STANDALONE ? 500 : 50);
			if ((got & S5_CTRL_REVERSE_DONE) || S5_STANDALONE) {
				retries++;
				set_state(OBS_ST_COARSE_SCANNING);
			} else if ((now_ms() - detect_ms) > S5_BYPASS_LIMIT_MS) {
				raise_fault("reverse retry timed out");
			}
			break;

		case OBS_ST_FAULT:
		default:
			/* stay here until the Safety Monitor has re-initialised us */
			got = wait_ctrl(S5_CTRL_RESET, 100);
#if S5_STANDALONE
			/* bench mode only: there is no Safety Monitor yet, so reset
			   ourselves after 5 s and clear our own fault bit */
			if ((now_ms() - fault_ms) > 5000) {
				tk_clr_flg(flg_fault_i14, ~FLT_OBSTACLE_BIT);
				got |= S5_CTRL_RESET;
			}
#endif
			if (got & S5_CTRL_RESET) {
				noecho_run = 0;
				hits = 0;
				set_state(OBS_ST_IDLE_MONITORING);
				tm_printf((UB*)"obstacle: reset by Safety Monitor\n");
			}
			break;
		}
	}
}
