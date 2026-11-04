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

/* Simulation: flat-faced box, 15 degree beam, 1 cm rounding. Only indices lo..hi are
   measured (the servo window). The chosen bypass must keep the robot's half width
   clear of the TRUE edge; a box inside the robot's lane that is not detected counts
   as missed. Returns the number of unsafe + missed cases. */
static int sim_boxes(int lo, int hi, int *cases_o, int *retry_o, int *worst_o)
{
	int W, X, D, a, k, cases = 0, unsafe = 0, missed = 0, retry = 0, stop = 0, worst = 1000;
	OBS_PROFILE p;

	for (W = 10; W <= 20; W += 2) {
	for (X = -30; X <= 30; X += 5) {
	for (D = 20; D <= 40; D += 5) {
		int true_r = X + W / 2, true_l = X - W / 2;	/* + = right of the centre line */
		OBS_SCAN sc;

		obs_scan_init(&sc);
		for (a = lo; a <= hi; a++) {
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
				missed++;
			}
			continue;
		}
		if (p.action == OBS_ACT_TURN_RIGHT) {
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
	*cases_o = cases; *retry_o = retry + stop; *worst_o = worst;
	return unsafe + missed;
}

/* Simulation with TWO flat-faced boxes. Every chosen bypass must keep the robot's
   half width clear of BOTH true boxes. A box in the lane that is not detected counts
   as missed. REVERSE_RETRY and STOP are safe outcomes (the car does not move sideways). */
static int sim_two_boxes(int lo, int hi, int *cases_o, int *retry_o, int *go_o)
{
	int W1, W2, X1, X2, D1, D2, a, k, cases = 0, unsafe = 0, missed = 0, retry = 0, go = 0;
	OBS_PROFILE p;
	int hw = (int)cfg.robot_half_width_cm;

	for (W1 = 10; W1 <= 20; W1 += 5) {
	for (W2 = 10; W2 <= 20; W2 += 5) {
	for (X1 = -40; X1 <= 40; X1 += 10) {
	for (X2 = -40; X2 <= 40; X2 += 10) {
	for (D1 = 25; D1 <= 35; D1 += 10) {
	for (D2 = 25; D2 <= 35; D2 += 10) {
		int r1 = X1 + W1 / 2, l1 = X1 - W1 / 2, r2 = X2 + W2 / 2, l2 = X2 - W2 / 2;
		OBS_SCAN sc;

		if (X2 <= X1 || X2 - X1 < (W1 + W2) / 2 + 2) continue;	/* boxes must not touch */
		obs_scan_init(&sc);
		for (a = lo; a <= hi; a++) {
			double best = 1e9;
			for (k = -5; k <= 5; k++) {
				double th = ((a * 15.0) + k * 1.5) * 3.14159265358979 / 180.0;
				double sn = sin(th), cs = cos(th), x, t;
				if (sn <= 0.05) continue;
				t = D1 / sn; x = D1 * cs / sn;
				if (x >= l1 && x <= r1 && t < best) best = t;
				t = D2 / sn; x = D2 * cs / sn;
				if (x >= l2 && x <= r2 && t < best) best = t;
			}
			obs_scan_set(&sc, (unsigned)a, best > 1e8 ? OBS_DIST_NONE : (uint16_t)(best + 0.5));
		}
		obs_profile(&sc, &cfg, &p);
		cases++;
		if (!p.detected) {
			if ((r1 > -hw && l1 < hw) || (r2 > -hw && l2 < hw)) missed++;
			continue;
		}
		if (p.action == OBS_ACT_TURN_RIGHT || p.action == OBS_ACT_TURN_LEFT) {
			int c = (p.action == OBS_ACT_TURN_RIGHT) ? (int)p.right_cm : -(int)p.left_cm;
			int hit1 = (r1 > c - hw && l1 < c + hw), hit2 = (r2 > c - hw && l2 < c + hw);

			go++;
			if (hit1 || hit2) unsafe++;
		} else {
			retry++;
		}
	}}}}}}
	*cases_o = cases; *retry_o = retry; *go_o = go;
	return unsafe + missed;
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
	CHECK("S5-H07", (p.right_cm == p.left_cm || p.right_cm + 1 == p.left_cm || p.left_cm + 1 == p.right_cm) &&
	      p.action == OBS_ACT_TURN_RIGHT,
	      "symmetric obstacle: clearances equal within 1 cm (integer rounding), tie goes right");

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
	CHECK("S5-H25a", p.width_cm >= 7 && p.width_cm <= 9 && p.plan_width_cm >= 23 && p.plan_width_cm <= 27 &&
	      p.right_cm >= 22 && p.right_cm <= 26 && p.left_cm >= 22 && p.left_cm <= 26,
	      "measured width ~8 cm, planned ~25 cm (outer edge estimate, wider than the 20 cm floor): shift about 12 + 8 + 3");
	{ OBS_CFG c2 = cfg; c2.plan_width_cm = 0; obs_profile(&s, &c2, &q); }
	CHECK("S5-H25b", q.plan_width_cm + 1 >= q.width_cm,
	      "plan_width_cm = 0: the plan is still never narrower than the measured width");
	/* the 20 cm floor binds when the scan is a single sample (footprint 8 cm only) */
	{ int ix[] = {6}, cm[] = {30}; OBS_CFG c2 = cfg; c2.plan_width_cm = 0;
	  scan_with(&s, ix, cm, 1); obs_profile(&s, &cfg, &p); obs_profile(&s, &c2, &q); }
	CHECK("S5-H25c", p.right_cm >= 20 && p.right_cm <= 22 && q.right_cm < p.right_cm,
	      "single sample at 30 cm: the 20 cm floor gives shift 10 + 8 + 3 = 21, without it about 15");
	{ int ix[13], cm[13]; for (i = 0; i < 13; i++) { ix[i] = i; cm[i] = 35; } scan_with(&s, ix, cm, 13); }
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H25d", p.plan_width_cm + 1 >= p.width_cm, "a wide obstacle is never planned narrower than measured");

	/* S5-H26 uncertainty from the 15 degree step is reported */
	{ int ix[] = {5,6,7}, cm[] = {30,30,30}; scan_with(&s, ix, cm, 3); }
	obs_profile(&s, &cfg, &p);
	CHECK("S5-H26", p.unc_cm >= 6 && p.unc_cm <= 9, "width uncertainty at 30 cm is about +/-7 cm");

	/* S5-H27 SIMULATION: flat-faced boxes 10..20 cm wide, offset -30..+30 cm, 20..40 cm
	   away. Full 0..180 degree scan: every chosen bypass must clear the true edge and no
	   box in the lane may be missed. */
	{
		int cases, retry, worst, bad = sim_boxes(0, 12, &cases, &retry, &worst);

		printf("[HOST] S5-H27 sim 0..180 deg: %d cases, %d unsafe or missed, %d reverse-retry/stop, worst clearance %d cm\n",
		       cases, bad, retry, worst);
		CHECK("S5-H27", bad == 0, "simulated boxes, full scan: every bypass clears the true edge, none missed");
	}

	/* S5-H28 the same simulation with the servo limited to the window the car body allows */
	{
		int cases, retry, worst, bad = sim_boxes(4, 8, &cases, &retry, &worst);

		printf("[HOST] S5-H28 sim 60..120 deg: %d cases, %d unsafe or missed, %d reverse-retry/stop, worst clearance %d cm\n",
		       cases, bad, retry, worst);
		CHECK("S5-H28", bad == 0, "simulated boxes, servo limited to 60..120 deg: every bypass clears the true edge, none missed");
	}

	/* S5-H30 and the full 30..150 degree window */
	{
		int cases, retry, worst, bad = sim_boxes(2, 10, &cases, &retry, &worst);

		printf("[HOST] S5-H30 sim 30..150 deg: %d cases, %d unsafe or missed, %d reverse-retry/stop, worst clearance %d cm\n",
		       cases, bad, retry, worst);
		CHECK("S5-H30", bad == 0, "simulated boxes, servo limited to 30..150 deg: every bypass clears the true edge, none missed");
	}

	/* S5-H29 and a window of 45..135 */
	{
		int cases, retry, worst, bad = sim_boxes(3, 9, &cases, &retry, &worst);

		printf("[HOST] S5-H29 sim 45..135 deg: %d cases, %d unsafe or missed, %d reverse-retry/stop, worst clearance %d cm\n",
		       cases, bad, retry, worst);
		CHECK("S5-H29", bad == 0, "simulated boxes, servo limited to 45..135 deg: every bypass clears the true edge, none missed");
	}

	/* ---- Hardware regression: the first full scans on the real car (2026-11-04).
	   Background (cm) at index 2..10: 76 NONE NONE 289 183 NONE NONE NONE 80.
	   One box dead ahead and one to the right of the sensor. The old code looked only
	   at the closest obstacle and said "turn right 17 cm", towards the box on the right. */
	{
		static const int bgi[] = {2,3,4,5,6,7,8,9,10};
		static const int bgv[] = {76,OBS_DIST_NONE,OBS_DIST_NONE,289,183,OBS_DIST_NONE,OBS_DIST_NONE,OBS_DIST_NONE,80};

		/* scan 2 of the run: 30 deg 30, 60 deg 32, 90 deg 29, 120 deg none, 150 deg 80, 75 deg 72, 105 deg 31 */
		obs_scan_init(&s);
		for (i = 0; i < 9; i++) obs_scan_set_bg(&s, (unsigned)bgi[i], (uint16_t)bgv[i]);
		obs_scan_set(&s, 2, 30); obs_scan_set(&s, 4, 32); obs_scan_set(&s, 6, 29);
		obs_scan_set(&s, 8, OBS_DIST_NONE); obs_scan_set(&s, 10, 80);
		obs_scan_set(&s, 5, 72); obs_scan_set(&s, 7, 31);
		obs_profile(&s, &cfg, &p);
		printf("[HOST] S5-H31 scan 2: clusters %d closest %d cm @%d deg, left %d cm, right %d cm, action %d\n",
		       p.clusters, p.closest_cm, p.closest_idx * 15, p.left_cm, p.right_cm, p.action);
		CHECK("S5-H31", p.detected && p.clusters >= 2 && p.action == OBS_ACT_TURN_LEFT &&
		      p.right_cm == OBS_DIST_CAP && p.left_cm >= 20 && p.left_cm <= 30,
		      "centre box + box on the right: turn LEFT (right is blocked by the second box)");

		/* scan 1 of the run: boxes on the right (30..60 deg) and the left (120 deg) */
		obs_scan_init(&s);
		for (i = 0; i < 9; i++) obs_scan_set_bg(&s, (unsigned)bgi[i], (uint16_t)bgv[i]);
		obs_scan_set(&s, 2, 31); obs_scan_set(&s, 4, 32); obs_scan_set(&s, 6, 74);
		obs_scan_set(&s, 8, 31); obs_scan_set(&s, 10, 79);
		obs_scan_set(&s, 3, 32); obs_scan_set(&s, 5, 69);
		obs_profile(&s, &cfg, &p);
		printf("[HOST] S5-H32 scan 1: clusters %d closest %d cm @%d deg, left %d cm, right %d cm, action %d\n",
		       p.clusters, p.closest_cm, p.closest_idx * 15, p.left_cm, p.right_cm, p.action);
		CHECK("S5-H32", p.detected && p.clusters >= 2 && p.action == OBS_ACT_REVERSE_RETRY,
		      "boxes on both sides: neither side is free, REVERSE_RETRY (the old code said turn left 0 cm)");

		/* scan 3 of the run: everything from 30 to 120 deg at 30 to 35 cm */
		obs_scan_init(&s);
		for (i = 0; i < 9; i++) obs_scan_set_bg(&s, (unsigned)bgi[i], (uint16_t)bgv[i]);
		obs_scan_set(&s, 2, 31); obs_scan_set(&s, 3, 30); obs_scan_set(&s, 4, 33); obs_scan_set(&s, 5, 35);
		obs_scan_set(&s, 6, 31); obs_scan_set(&s, 7, 31); obs_scan_set(&s, 8, 32);
		obs_scan_set(&s, 9, 82); obs_scan_set(&s, 10, 80);
		obs_profile(&s, &cfg, &p);
		printf("[HOST] S5-H33 scan 3: clusters %d width %d cm, left %d cm, right %d cm, action %d\n",
		       p.clusters, p.width_cm, p.left_cm, p.right_cm, p.action);
		CHECK("S5-H33", p.detected && p.clusters == 1 && p.width_cm >= 30 && p.action != OBS_ACT_CONTINUE,
		      "one wide obstacle from 30 to 120 deg: a single cluster, a bypass is chosen");
	}

	/* S5-H34 fine scan: next index to measure for several obstacles, inside the servo window, nearest first */
	obs_scan_init(&s);
	obs_scan_set(&s, 2, 30); obs_scan_set(&s, 4, 32); obs_scan_set(&s, 6, 29);
	obs_scan_set(&s, 8, OBS_DIST_NONE); obs_scan_set(&s, 10, OBS_DIST_NONE);
	CHECK("S5-H34a", obs_next_fine_idx(&s, &cfg, 2, 10, 10) == 7, "from index 10: nearest unmeasured neighbour is 7");
	obs_scan_set(&s, 7, OBS_DIST_NONE);
	CHECK("S5-H34b", obs_next_fine_idx(&s, &cfg, 2, 10, 7) == 5, "then 5");
	obs_scan_set(&s, 5, OBS_DIST_NONE);
	CHECK("S5-H34c", obs_next_fine_idx(&s, &cfg, 2, 10, 5) == 3, "then 3 (between the two right-hand obstacles)");
	obs_scan_set(&s, 3, OBS_DIST_NONE);
	CHECK("S5-H34d", obs_next_fine_idx(&s, &cfg, 2, 10, 3) == -1, "index 1 is outside the servo window: nothing left to measure");

	/* S5-H35 SIMULATION with two boxes, servo window 30..150 */
	{
		int cases, retry, go, bad = sim_two_boxes(2, 10, &cases, &retry, &go);

		printf("[HOST] S5-H35 sim two boxes 30..150 deg: %d cases, %d unsafe or missed, %d bypass chosen, %d reverse-retry/stop\n",
		       cases, bad, go, retry);
		CHECK("S5-H35", bad == 0, "simulated TWO boxes: every chosen bypass clears both boxes, none missed");
	}

	printf("[HOST] subsystem5 SUMMARY %d passed, %d failed\n", pass_n, fail_n);
	return fail_n ? 1 : 0;
}
