/*
 * Host-side unit tests for app_program/subsystem5/obstacle_profile.c
 * Build and run on a PC:   sh testing/subsystem5/host/run.sh
 * Output lines: [HOST] S5-H01 PASS ...
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

	printf("[HOST] subsystem5 SUMMARY %d passed, %d failed\n", pass_n, fail_n);
	return fail_n ? 1 : 0;
}
