/*
 *	test_motion_math.c
 *	PC unit tests for subsystem 2's packing helpers (subsystem2.h) and
 *	control maths (motion_math.c).  No Pico needed.
 *
 *	Run:  sh testing/subsystem2/host/run_host_tests.sh
 *	Output lines use the same format as the on-target tests:
 *	    [TEST] S2-H01 PASS  ...
 */

#include <stdio.h>
#include "host_shim.h"
#include "../../../app_program/subsystem2/motion_math.h"

static int n_pass = 0;
static int n_fail = 0;

static void check(const char *id, int cond, const char *desc)
{
	if (cond) {
		n_pass++;
		printf("[TEST] %s PASS  %s\n", id, desc);
	} else {
		n_fail++;
		printf("[TEST] %s FAIL  %s\n", id, desc);
	}
}

int main(void)
{
	int	ok;
	int	mode, speed, param, epoch;
	UW	w;
	PI_STATE pi = { 0 };
	INT	d;

	/* H01: every mode/speed/epoch and every signed param round-trips */
	ok = 1;
	for (mode = 0; mode < 8; mode++) {
		for (speed = -S2_SPEED_LIMIT; speed <= S2_SPEED_LIMIT; speed += 7) {
			for (epoch = 0; epoch < 16; epoch++) {
				for (param = -512; param <= 511; param += 31) {
					w = s2_cmd_pack((UINT)mode, speed, param, (UINT)epoch);
					if ((s2_cmd_mode(w) != (UINT)mode) || (s2_cmd_speed(w) != speed)
					    || (s2_cmd_param_s(w) != param) || (s2_cmd_epoch(w) != (UINT)epoch)
					    || ((w >> 27) != 0U)) {
						ok = 0;
					}
				}
			}
		}
	}
	check("S2-H01", ok, "IF-10 command pack/unpack round trip, bits 27-31 unused");

	/* H02: boundary values, including unsigned distance 0..1023 */
	w = s2_cmd_pack(S2_MODE_DRIVE_DIST, -511, 1023, 15U);
	ok = (s2_cmd_speed(w) == -511) && (s2_cmd_param_u(w) == 1023U) && (s2_cmd_epoch(w) == 15U);
	w = s2_cmd_pack(S2_MODE_TURN, 0, -180, 0U);
	ok = ok && (s2_cmd_param_s(w) == -180) && (s2_cmd_speed(w) == 0);
	w = s2_cmd_pack(S2_MODE_DRIVE, 900, 0, 1U);		/* speed clamps to 511 */
	ok = ok && (s2_cmd_speed(w) == S2_SPEED_LIMIT);
	check("S2-H02", ok, "command boundaries: -511, 1023 mm, -180 deg, clamp 900 -> 511");

	/* H03: status word round trip */
	w = s2_status_pack(-300, 450, S2_STATE_STALLED, TRUE, 9U);
	ok = (s2_status_speed_l(w) == -300) && (s2_status_speed_r(w) == 450)
	  && (s2_status_state(w) == S2_STATE_STALLED) && (s2_status_done(w) == TRUE)
	  && (s2_status_epoch(w) == 9U) && ((w >> 28) == 0U);
	check("S2-H03", ok, "IF-16 status pack/unpack round trip, 28 bits used");

	/* H04: odometry word and wrap-safe difference */
	w = s2_odo_pack(65530U, 65535U);
	ok = (s2_odo_dist_mm(w) == 65530U) && (s2_odo_time_ms(w) == 65535U)
	  && ((UH)(10U - s2_odo_dist_mm(w)) == 16U);
	check("S2-H04", ok, "IF-17 odometry pack and wrap-safe difference");

	/* H05: epoch staleness with wrap-around */
	ok = (motion_epoch_is_stale(4U, 5U) == TRUE)		/* one behind  */
	  && (motion_epoch_is_stale(5U, 5U) == FALSE)		/* same        */
	  && (motion_epoch_is_stale(6U, 5U) == FALSE)		/* newer       */
	  && (motion_epoch_is_stale(15U, 0U) == TRUE)		/* 15 -> 0 wrap: 15 is older */
	  && (motion_epoch_is_stale(0U, 15U) == FALSE)		/* 0 after 15 is newer */
	  && (motion_epoch_is_stale(14U, 5U) == TRUE)		/* 7 behind (wrapped): stale */
	  && (motion_epoch_is_stale(13U, 5U) == FALSE);		/* 8 behind = 8 ahead: newer */
	check("S2-H05", ok, "epoch staleness, including wrap 15 -> 0");

	/* H06: distance per pulse from the geometry (65 mm, 20 pulses) */
	check("S2-H06", S2_UM_PER_PULSE == 10210, "um per pulse = 10210 for 65 mm / 20 pulses");

	/* H07: speed from edge period */
	ok = (motion_speed_mmps(51050U, 1000U, 10U, 1) == 200)		/* 20 Hz edges */
	  && (motion_speed_mmps(51050U, 1000U, 10U, -1) == -200)
	  && (motion_speed_mmps(51050U, 102100U, 10U, 1) == 100)	/* slowing: since > period */
	  && (motion_speed_mmps(51050U, 300000U, 10U, 1) == 0)	/* timed out */
	  && (motion_speed_mmps(0U, 1000U, 1U, 1) == 0)		/* one edge only */
	  && (motion_speed_mmps(1000U, 100U, 10U, 1) == S2_SPEED_LIMIT);	/* clamps */
	check("S2-H07", ok, "speed from edge period: 200, -200, decay, timeout, clamp");

	/* H08: feed-forward */
	ok = (motion_ff_duty(0) == 0) && (motion_ff_duty(200) == 500)
	  && (motion_ff_duty(-200) == -500) && (motion_ff_duty(2000) == 1000);
	check("S2-H08", ok, "feed-forward duty: 0, +/-500 at 200 mm/s, clamp 1000");

	/* H09: PI never drives against the target direction, saturates at 1000 */
	pi.integ = 0;
	d = motion_pi_step(&pi, 100, 400);		/* far too fast */
	ok = (d >= 0);
	pi.integ = 0;
	d = motion_pi_step(&pi, 500, 0);		/* stalled at full demand */
	ok = ok && (d == 1000) && (pi.integ == 0);	/* saturated: integrator held */
	pi.integ = 123;
	d = motion_pi_step(&pi, 0, 50);
	ok = ok && (d == 0) && (pi.integ == 0);	/* target 0: off and reset */
	check("S2-H09", ok, "PI: no reversal, anti-windup, target 0 resets");

	/* H10: ramp */
	ok = (motion_ramp(0, 200, 25) == 25) && (motion_ramp(190, 200, 25) == 200)
	  && (motion_ramp(0, -200, 25) == -25);
	check("S2-H10", ok, "setpoint ramp limited to 25 mm/s per cycle");

	/* H11: turn arc for 90 deg with 120 mm track = 94.26 mm */
	ok = (motion_turn_arc_um(90) == 94260) && (motion_turn_arc_um(-90) == 94260);
	check("S2-H11", ok, "turn arc 90 deg = 94.26 mm per wheel");

	/* H12: straight-line hold steers against the wheel that is ahead */
	ok = (motion_heading_correction(10000, 0) < 0) && (motion_heading_correction(0, 10000) > 0)
	  && (motion_heading_correction(0, 0) == 0)
	  && (motion_heading_correction(1000000, 0) == -S2_HEADING_MAX_MMPS);
	check("S2-H12", ok, "heading correction sign and clamp");

	printf("[TEST] subsystem2-host SUMMARY %d passed, %d failed\n", n_pass, n_fail);
	return (n_fail == 0) ? 0 : 1;
}
