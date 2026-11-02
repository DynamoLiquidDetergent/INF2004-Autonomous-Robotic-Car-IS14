/*
 * Host-side unit tests for app_program/subsystem5/obstacle_profile.c
 * Build and run on a PC:   sh testing/subsystem5/host/run.sh
 * Output lines: [HOST] S5-H01 PASS ...
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../../../app_program/subsystem5/obstacle_profile.h"

static int pass_n, fail_n;

#define CHECK(id, cond, desc) do { \
	if (cond) { pass_n++; printf("[HOST] %s PASS  %s\n", id, desc); } \
	else { fail_n++; printf("[HOST] %s FAIL  %s (line %d)\n", id, desc, __LINE__); } \
} while (0)

static OBS_CFG cfg;

static void scan_with(OBS_SCAN *s, const int *idx, const int *cm, int n)
{
	int i;
	obs_scan_init(s);
	for (i = 0; i < n; i++) obs_scan_set(s, (unsigned)idx[i], (uint16_t)cm[i]);
}

int main(void)
{
	OBS_SCAN s;
	OBS_PROFILE p, q;
	uint32_t w[OBS_MSG_WORDS];
	uint8_t seq;
	int i;

	obs_cfg_default(&cfg);

	/* S5-H01 conversions */
	CHECK("S5-H01", obs_echo_us_to_cm(580) == 10 && obs_echo_us_to_cm(23200) == 400 &&
	      obs_echo_us_to_cm(116) == 2, "echo time to cm (58 us/cm)");
	CHECK("S5-H02", obs_angle_to_pulse_us(0) == 600 && obs_angle_to_pulse_us(90) == 1500 &&
	      obs_angle_to_pulse_us(180) == 2400 && obs_angle_to_pulse_us(999) == 2400,
	      "servo pulse 600/1500/2400 us and clamped");
	CHECK("S5-H03", obs_isqrt(0) == 0 && obs_isqrt(1) == 1 && obs_isqrt(144) == 12 &&
	      obs_isqrt(145) == 12 && obs_isqrt(260100) == 510, "integer square root");

	/* S5-H04 nothing in range */
	obs_scan_init(&s);
	for (i = 2; i <= 10; i += 2) obs_scan_set(&s, (unsigned)i, OBS_DIST_NONE);
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H04", !p.detected && p.action == OBS_ACT_CONTINUE, "clear scan: not detected, CONTINUE");

	/* S5-H05 narrow obstacle dead ahead, 30 cm, hits at 5,6,7 */
	{ int ix[] = {5,6,7}, cm[] = {30,30,30}; scan_with(&s, ix, cm, 3); }
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H05", p.detected && p.closest_idx == 5 && p.closest_cm == 30 &&
	      p.idx_right == 5 && p.idx_left == 7, "cluster 5..7 found, closest 30 cm (tie: lowest index)");
	/* true width of beams edge to edge: 2*30*sin(7.5 deg) = 7.8 cm */
	CHECK("S5-H06", p.width_cm >= 7 && p.width_cm <= 9, "width about 8 cm (expected 7.8)");
	CHECK("S5-H07", p.right_cm == p.left_cm && p.action == OBS_ACT_TURN_RIGHT,
	      "symmetric obstacle: equal clearances, tie goes right");

	/* S5-H08 obstacle on the right side (hits 1..3 at 40 cm): pass on the LEFT */
	{ int ix[] = {1,2,3,4}, cm[] = {40,40,40,OBS_DIST_NONE}; scan_with(&s, ix, cm, 4); }
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H08", p.detected && p.left_cm < p.right_cm && p.action == OBS_ACT_TURN_LEFT,
	      "obstacle on the right: left shift smaller, TURN_LEFT");

	/* S5-H09 obstacle on the left (mirror): pass on the RIGHT */
	{ int ix[] = {9,10,11,8}, cm[] = {40,40,40,OBS_DIST_NONE}; scan_with(&s, ix, cm, 4); }
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H09", p.detected && p.right_cm < p.left_cm && p.action == OBS_ACT_TURN_RIGHT,
	      "obstacle on the left: right shift smaller, TURN_RIGHT");

	/* S5-H10 wide wall across the whole scan: too tight, reverse and retry */
	{ int ix[13], cm[13]; for (i = 0; i < 13; i++) { ix[i] = i; cm[i] = 35; } scan_with(&s, ix, cm, 13); }
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H10", p.detected && p.partial && p.action == OBS_ACT_REVERSE_RETRY,
	      "wall across all angles: partial, REVERSE_RETRY");

	/* S5-H11 too close: STOP */
	{ int ix[] = {6}, cm[] = {5}; scan_with(&s, ix, cm, 1); }
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H11", p.detected && p.action == OBS_ACT_STOP, "closer than stop distance: STOP");

	/* S5-H12 single sample is flagged partial with a beam-footprint width */
	{ int ix[] = {6}, cm[] = {30}; scan_with(&s, ix, cm, 1); }
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H12", p.partial && p.width_cm >= 7 && p.width_cm <= 8, "single sample: partial, width = beam footprint");

	/* S5-H13 reading beyond detect distance is not an obstacle */
	{ int ix[] = {6}, cm[] = {120}; scan_with(&s, ix, cm, 1); }
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H13", !p.detected, "reading above detect_cm ignored");

	/* S5-H14 fine scan walks outwards until an edge is found */
	{ int ix[] = {2,4,6,8,10}, cm[] = {OBS_DIST_NONE,OBS_DIST_NONE,30,OBS_DIST_NONE,OBS_DIST_NONE}; scan_with(&s, ix, cm, 5); }
	CHECK("S5-H14a", obs_next_edge_idx(&s, &cfg, -1) == 5 && obs_next_edge_idx(&s, &cfg, +1) == 7,
	      "coarse hit at 6: next to measure are 5 and 7");
	obs_scan_set(&s, 5, 30);	/* hit: cluster grows to 5..6 */
	CHECK("S5-H14b", obs_next_edge_idx(&s, &cfg, -1) == -1, "index 4 already measured (coarse): right edge known");
	obs_scan_set(&s, 7, OBS_DIST_NONE);
	CHECK("S5-H14c", obs_next_edge_idx(&s, &cfg, +1) == -1, "index 7 measured, not a hit: left edge known");

	/* S5-H15 scan limit stops the fine scan */
	{ int ix[] = {0}, cm[] = {30}; scan_with(&s, ix, cm, 1); }
	CHECK("S5-H15", obs_next_edge_idx(&s, &cfg, -1) == -1 && obs_next_edge_idx(&s, &cfg, +1) == 1,
	      "obstacle at 0 degrees: no index below 0, next is 1");

	/* S5-H16 packed message round trip, all field extremes */
	memset(&p, 0, sizeof p);
	p.detected = 1; p.closest_idx = 12; p.closest_cm = 400; p.width_cm = 510;
	p.action = OBS_ACT_STOP; p.partial = 1; p.left_cm = 510; p.right_cm = 7;
	obs_pack(&p, 15, w);
	memset(&q, 0, sizeof q); seq = 0;
	obs_unpack(w, &q, &seq);
	CHECK("S5-H16", q.detected == 1 && q.closest_idx == 12 && q.closest_cm == 400 &&
	      q.width_cm == 510 && q.action == OBS_ACT_STOP && q.partial == 1 &&
	      q.left_cm == 510 && q.right_cm == 7 && seq == 15, "pack/unpack round trip at field limits");
	CHECK("S5-H17", (w[0] >> 31) == 0 && (w[1] >> 18) == 0, "unused bits stay zero (31 + 18 bits used)");

	/* S5-H18 sequence wraps in 4 bits */
	obs_pack(&p, 16, w); obs_unpack(w, &q, &seq);
	CHECK("S5-H18", seq == 0, "seq 16 wraps to 0 in 4 bits");


	/* ---- Added after the professor's guidance: median, background,
	   hysteresis, planning for the widest obstacle, uncertainty ---- */

	/* S5-H19 median of repeated readings; "nothing" counts as largest */
	{
		uint16_t a[] = {30, 31, 90}, b[] = {30, OBS_DIST_NONE, OBS_DIST_NONE},
			 c[] = {OBS_DIST_NONE, 30, 32}, d[] = {10, 200, 12, 11, 13}, e[] = {44};
		CHECK("S5-H19", obs_median(a, 3) == 31 && obs_median(b, 3) == OBS_DIST_NONE &&
		      obs_median(c, 3) == 32 && obs_median(d, 5) == 12 && obs_median(e, 1) == 44 &&
		      obs_median(a, 0) == OBS_DIST_NONE,
		      "median: outlier rejected, majority 'nothing' wins, n=1 and n=0 safe");
	}

	/* S5-H20 a wall that is in the background is not an obstacle */
	obs_scan_init(&s);
	for (i = 0; i < 13; i++) obs_scan_set_bg(&s, (unsigned)i, 30);
	for (i = 0; i < 13; i++) obs_scan_set(&s, (unsigned)i, 30);
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H20", !p.detected && p.action == OBS_ACT_CONTINUE,
	      "reading equal to its background: not an obstacle");

	/* S5-H21 box in front of a wall: only the angles clearly closer than the wall hit */
	obs_scan_init(&s);
	for (i = 0; i < 13; i++) obs_scan_set_bg(&s, (unsigned)i, 40);
	for (i = 0; i < 13; i++) obs_scan_set(&s, (unsigned)i, (i >= 5 && i <= 7) ? 25 : 40);
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H21", p.detected && p.idx_right == 5 && p.idx_left == 7 && !p.partial,
	      "box in front of a wall: cluster 5..7, edges found against the wall");

	/* S5-H22 hysteresis with a background: enter 6 cm, stay 3 cm */
	obs_scan_init(&s);
	for (i = 0; i < 13; i++) obs_scan_set_bg(&s, (unsigned)i, 40);
	obs_scan_set(&s, 6, 30);	/* 10 cm closer: starts the obstacle */
	obs_scan_set(&s, 5, 36);	/*  4 cm closer: stays in (>= 3) */
	obs_scan_set(&s, 4, 38);	/*  2 cm closer: out (< 3) */
	obs_scan_set(&s, 7, 38);	/*  2 cm closer: out */
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H22a", p.detected && p.idx_right == 5 && p.idx_left == 6,
	      "hysteresis: 4 cm closer extends the cluster, 2 cm closer does not");
	obs_scan_init(&s);
	for (i = 0; i < 13; i++) obs_scan_set_bg(&s, (unsigned)i, 40);
	obs_scan_set(&s, 6, 36);	/* only 4 cm closer: below the 6 cm start rule */
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H22b", !p.detected, "hysteresis: 4 cm closer alone cannot start an obstacle");

	/* S5-H23 hysteresis without a background: enter 50, stay up to 55 */
	{ int ix[] = {6,7}, cm[] = {40,53}; scan_with(&s, ix, cm, 2); }
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H23a", p.detected && p.idx_left == 7, "no background: 53 cm (<= 50+5) stays in the cluster");
	{ int ix[] = {6,7}, cm[] = {40,57}; scan_with(&s, ix, cm, 2); }
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H23b", p.detected && p.idx_left == 6, "no background: 57 cm is out of the cluster");

	/* S5-H24 a background right in the path ahead is never trusted */
	obs_scan_init(&s);
	obs_scan_set_bg(&s, 3, 30);
	obs_scan_set_bg(&s, 5, 30);
	obs_scan_set_bg(&s, 6, 30);
	obs_scan_set_bg(&s, 7, 30);
	CHECK("S5-H24a", obs_bg_clear_ahead(&s, &cfg) == 3, "3 background values ahead (5..7) forgotten");
	obs_scan_set(&s, 6, 30);
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H24b", p.detected, "something 30 cm dead ahead at start-up is still an obstacle");

	/* S5-H25 bypass is planned for the widest allowed obstacle (20 cm) */
	{ int ix[] = {5,6,7}, cm[] = {30,30,30}; scan_with(&s, ix, cm, 3); }
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H25a", p.width_cm >= 7 && p.width_cm <= 9 && p.plan_width_cm >= 19 && p.plan_width_cm <= 21 &&
	      p.right_cm >= 20 && p.right_cm <= 22 && p.left_cm >= 20 && p.left_cm <= 22,
	      "measured width ~8 cm, planned 20 cm: shift = 10 + 8 half robot + 3 margin = 21");
	{ OBS_CFG c2 = cfg; c2.plan_width_cm = 0; obs_profile(&s, &c2, &q); }
	CHECK("S5-H25b", q.plan_width_cm == q.width_cm || q.plan_width_cm + 1 >= q.width_cm,
	      "plan_width_cm = 0 switches planning back to the measured width");
	CHECK("S5-H25c", q.right_cm < p.right_cm, "planning for 20 cm needs a bigger shift than the measured 8 cm");
	{ int ix[13], cm[13]; for (i = 0; i < 13; i++) { ix[i] = i; cm[i] = 35; } scan_with(&s, ix, cm, 13); }
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H25d", p.plan_width_cm + 1 >= p.width_cm, "a wide obstacle is never planned narrower than measured");

	/* S5-H26 uncertainty from the 15 degree step is reported */
	{ int ix[] = {5,6,7}, cm[] = {30,30,30}; scan_with(&s, ix, cm, 3); }
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H26", p.unc_cm >= 6 && p.unc_cm <= 9, "width uncertainty at 30 cm is about +/-7 cm");

	/* S5-H27 SIMULATION: flat-faced box, 15 degree beam, 13 angles, 1 cm rounding.
	   For every box width 10..20 cm, offset -30..+30 cm and range 20..40 cm, the
	   chosen bypass side must keep the robot's half width clear of the TRUE edge.
	   "Detected nothing" while the box is inside the robot's lane also counts as unsafe. */
	{
		int W, X, D, a, k, cases = 0, unsafe = 0, missed = 0, retry = 0, stop = 0, worst = 1000;

		for (W = 10; W <= 20; W += 2) {
		for (X = -30; X <= 30; X += 5) {
		for (D = 20; D <= 40; D += 5) {
			int true_r = X + W / 2, true_l = X - W / 2;	/* + = right of the centre line */
			OBS_SCAN sc;

			/* the nearer the box, the less of it matters beyond the lane */
			obs_scan_init(&sc);
			for (a = 0; a < OBS_NUM_ANGLES; a++) {
				double best = 1e9;
				for (k = -5; k <= 5; k++) {		/* rays across the +/-7.5 deg beam */
					double th = ((a * 15.0) + k * 1.5) * 3.14159265358979 / 180.0;
					double sn = sin(th), cs = cos(th), x, t;
					if (sn <= 0.05) continue;
					t = D / sn;
					x = D * cs / sn;
					if (x >= true_l && x <= true_r && t < best) best = t;
				}
				obs_scan_set(&sc, (unsigned)a, best > 1e8 ? OBS_DIST_NONE : (uint16_t)(best + 0.5));
			}
			obs_profile(&sc, &cfg, &p);
			cases++;
			if (!p.detected) {
				if (true_r > -(int)cfg.robot_half_width_cm && true_l < (int)cfg.robot_half_width_cm) {
					missed++;	/* box overlaps the lane but the scan saw nothing */
				}
				continue;
			}
			if (p.action == OBS_ACT_TURN_RIGHT) {
				/* moving right by right_cm must put the robot's LEFT side beyond the right edge */
				int clr = (int)p.right_cm - (true_r + (int)cfg.robot_half_width_cm);
				if (clr < worst) worst = clr;
				if (clr < 0) unsafe++;
			} else if (p.action == OBS_ACT_TURN_LEFT) {
				int clr = (int)p.left_cm - (-true_l + (int)cfg.robot_half_width_cm);
				if (clr < worst) worst = clr;
				if (clr < 0) unsafe++;
			} else if (p.action == OBS_ACT_REVERSE_RETRY) {
				retry++;
			} else if (p.action == OBS_ACT_STOP) {
				stop++;
			}
		}}}
		printf("[HOST] S5-H27 sim: %d cases, %d unsafe, %d missed, %d reverse-retry, %d stop, worst clearance %d cm\n",
		       cases, unsafe, missed, retry, stop, worst);
		CHECK("S5-H27", unsafe == 0 && missed == 0,
		      "simulated boxes 10..20 cm: every chosen bypass clears the true edge, none missed");
	}

	printf("[HOST] subsystem5 SUMMARY %d passed, %d failed\n", pass_n, fail_n);
	return fail_n ? 1 : 0;
}
