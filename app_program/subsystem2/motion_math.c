/*
 *	motion_math.c
 *	Subsystem 2 -- pure calculations for Motion Control (see motion_math.h).
 */

#include "motion_math.h"

#define MM_PER_S_FROM_UM_PER_US	1000L	/* um/us = m/s; x1000 -> mm/s */
#define MILLI			1000L
#define DEG_FULL_TURN		360L
#define PI_X1000		3142L	/* pi to 3 decimals, for the turn arc */
#define EPOCH_HALF		(S2_EPOCH_MOD / 2U)

EXPORT INT motion_clamp(INT v, INT lo, INT hi)
{
	if (v < lo) {
		return lo;
	}
	if (v > hi) {
		return hi;
	}
	return v;
}

/*
 * Wheel speed from the time between encoder edges.  Counting pulses per
 * 20 ms would give steps of ~500 mm/s (one 10 mm pulse per cycle), far
 * too coarse, so the period between edges is used instead.  If the wheel
 * slows down, the time since the last edge grows past the last period and
 * is used instead, so the estimate falls smoothly to 0.
 */
EXPORT INT motion_speed_mmps(UW period_us, UW since_last_us, UW edges, INT dir)
{
	UW	eff;
	W	mmps;

	if ((edges < 2U) || (period_us == 0U) || (since_last_us > S2_SPEED_TIMEOUT_US)) {
		return 0;
	}
	eff = (since_last_us > period_us) ? since_last_us : period_us;
	mmps = (S2_UM_PER_PULSE * MM_PER_S_FROM_UM_PER_US) / (W)eff;
	if (mmps > S2_SPEED_LIMIT) {
		mmps = S2_SPEED_LIMIT;
	}
	return (dir < 0) ? -(INT)mmps : (INT)mmps;
}

/*
 * Epochs are 4-bit and wrap.  A command is stale when it is 1..7 steps
 * behind the newest epoch seen; 8..15 behind is treated as "ahead" (newer).
 */
EXPORT BOOL motion_epoch_is_stale(UINT cmd_epoch, UINT newest_epoch)
{
	UINT	behind = (newest_epoch - cmd_epoch) & S2_MASK_4;

	return ((behind >= 1U) && (behind < EPOCH_HALF)) ? TRUE : FALSE;
}

/* Feed-forward: the duty that roughly gives this speed on a flat floor */
EXPORT INT motion_ff_duty(INT target_mmps)
{
	INT	mag;
	INT	duty;

	if (target_mmps == 0) {
		return 0;
	}
	mag  = (target_mmps < 0) ? -target_mmps : target_mmps;
	duty = S2_FF_DEADBAND_PERMILLE + (INT)(((W)mag * S2_FF_GAIN_MILLI) / MILLI);
	duty = motion_clamp(duty, 0, 1000);
	return (target_mmps < 0) ? -duty : duty;
}

/*
 * One PI step for one wheel.  Returns the duty in permille.
 * Anti-windup: the integrator is not advanced while the output is
 * saturated.  The output never reverses the wheel against the target
 * direction; to stop, the task sets target 0 and the wheel coasts.
 * The integrator stays full-width W (32-bit) on purpose: only stored,
 * shared and transmitted data is packed (Tribunal 1 curveball).
 */
EXPORT INT motion_pi_step(PI_STATE *pi, INT target_mmps, INT measured_mmps)
{
	W	err;
	W	integ_new;
	W	out;

	if (target_mmps == 0) {
		pi->integ = 0;
		return 0;
	}
	err = (W)target_mmps - (W)measured_mmps;
	integ_new = pi->integ + err;
	if (integ_new > S2_INTEG_MAX) {
		integ_new = S2_INTEG_MAX;
	} else if (integ_new < -S2_INTEG_MAX) {
		integ_new = -S2_INTEG_MAX;
	}

	out = (W)motion_ff_duty(target_mmps)
	    + ((S2_KP_MILLI * err) + (S2_KI_MILLI * integ_new)) / MILLI;

	if ((out > 1000L) || (out < -1000L)) {
		out = (out > 0) ? 1000L : -1000L;	/* saturated: hold integrator */
	} else {
		pi->integ = integ_new;
	}

	if ((target_mmps > 0) && (out < 0)) {
		out = 0;
	} else if ((target_mmps < 0) && (out > 0)) {
		out = 0;
	}
	return (INT)out;
}

/* Move current toward target by at most step (soft start / soft stop) */
EXPORT INT motion_ramp(INT current, INT target, INT step)
{
	if (target > (current + step)) {
		return current + step;
	}
	if (target < (current - step)) {
		return current - step;
	}
	return target;
}

/* Arc each wheel travels for a turn on the spot: |deg| * pi * track / 360 */
EXPORT W motion_turn_arc_um(INT degrees)
{
	W	deg = (degrees < 0) ? -(W)degrees : (W)degrees;

	return (deg * (W)S2_TRACK_WIDTH_MM * PI_X1000) / DEG_FULL_TURN;
}

/*
 * Straight-line hold (NFR10): if the left wheel has gone further than the
 * right, slow the left / speed up the right.  Returns a steering value in
 * the same sense as the DRIVE command (+ = more speed on the left).
 */
EXPORT INT motion_heading_correction(W left_um, W right_um)
{
	W	diff_mm = (left_um - right_um) / MILLI;
	W	steer   = -(diff_mm * S2_HEADING_GAIN_MILLI) / MILLI;

	return motion_clamp((INT)steer, -S2_HEADING_MAX_MMPS, S2_HEADING_MAX_MMPS);
}
