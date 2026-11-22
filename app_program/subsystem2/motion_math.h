/*
 *	motion_math.h
 *	Subsystem 2 -- pure calculations used by the Motion Control task.
 *
 *	No kernel calls and no hardware access, so every function here can be
 *	unit-tested on the target (subsystem2_test.c) or on a PC.
 *
 *	Units: speed in mm/s, distance in micrometres (um), duty in permille
 *	(-1000..1000), time in microseconds unless the name says _ms.
 *	All integer maths: the Cortex-M0+ has no FPU.
 */

#ifndef MOTION_MATH_H
#define MOTION_MATH_H

#ifdef S2_HOST_TEST
#include "host_shim.h"		/* PC build of the unit tests only */
#else
#include "subsystem2.h"
#endif

/* ---------------------------------------------------------------------
 *  Derived geometry.  pi is 314159 / 100000.
 *  um per pulse = pi * diameter / pulses per rev  (65 mm, 20 -> 10210 um)
 * --------------------------------------------------------------------- */
#define S2_PI_NUM		314159L
#define S2_PI_DEN		100000L
#define S2_UM_PER_PULSE		((W)(((W)S2_WHEEL_DIAMETER_MM * S2_PI_NUM * 10L) \
					/ ((W)S2_ENC_PULSES_PER_REV * S2_PI_DEN / 100L)))

/* ---------------------------------------------------------------------
 *  Tuning values: START POINTS ONLY.  Tune on the bench (S2-M03, S2-M04)
 *  and record the final values in testing/subsystem2/TEST_LOG.md.
 * --------------------------------------------------------------------- */
#define S2_SPEED_TIMEOUT_US	250000U	/* no edge for this long -> speed 0 */
#define S2_FF_DEADBAND_PERMILLE	110	/* duty where the wheel just starts */
#define S2_FF_GAIN_MILLI	560	/* extra permille per (mm/s), x1000 */
#define S2_KP_MILLI		800	/* permille per (mm/s) of error, x1000 */
#define S2_KI_MILLI		60	/* permille per (mm/s * cycle), x1000 */
#define S2_INTEG_MAX		5000	/* integrator clamp, (mm/s * cycles) */
#define S2_ACCEL_STEP_MMPS	25	/* max setpoint change per cycle (soft start) */
#define S2_HEADING_GAIN_MILLI	2000	/* straight-line hold: mm/s per mm, x1000 */
#define S2_HEADING_MAX_MMPS	80	/* cap on straight-line correction */
#define S2_TURN_DEFAULT_MMPS	150	/* wheel speed for TURN when speed = 0 */
#define S2_STALL_DUTY_PERMILLE	400	/* duty at or above which a stall is possible */
#define S2_STALL_TIME_MS	300	/* driven hard but not turning this long = stall */
#define S2_STALL_MIN_MMPS	60	/* below this, edges are too far apart to judge */
#define S2_STALL_SPEED_MMPS	30	/* measured speed below this while driven hard = stalled */

typedef struct {
	W	integ;			/* integrator, (mm/s * cycles) */
} PI_STATE;

IMPORT INT	motion_speed_mmps(UW period_us, UW since_last_us, UW edges, INT dir);
IMPORT BOOL	motion_epoch_is_stale(UINT cmd_epoch, UINT newest_epoch);
IMPORT INT	motion_ff_duty(INT target_mmps);
IMPORT INT	motion_pi_step(PI_STATE *pi, INT target_mmps, INT measured_mmps);
IMPORT INT	motion_ramp(INT current, INT target, INT step);
IMPORT W	motion_turn_arc_um(INT degrees);
IMPORT INT	motion_heading_correction(W left_um, W right_um);
IMPORT INT	motion_clamp(INT v, INT lo, INT hi);

#endif /* MOTION_MATH_H */
