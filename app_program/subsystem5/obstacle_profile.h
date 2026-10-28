/*
 *	obstacle_profile.h
 *	Subsystem 5 -- pure maths for the obstacle scan (no hardware, no RTOS).
 *
 *	Kept free of kernel and register code so it can be unit-tested on a PC
 *	(testing/subsystem5/host/) as well as on the Pico.
 *
 *	Angle convention: scan index 0..12 = 0..180 degrees in 15 degree steps.
 *	0 degrees is the robot's RIGHT, 90 degrees (index 6) is straight ahead,
 *	180 degrees (index 12) is the robot's LEFT.
 *
 *	Minimum-bit data (Change Requirement 1): every field below is stored in
 *	the fewest bits that holds its range, packed with explicit masks and
 *	shifts (no C bit-fields on data that crosses a task boundary).
 */

#ifndef OBSTACLE_PROFILE_H
#define OBSTACLE_PROFILE_H

#include <stdint.h>

#define OBS_NUM_ANGLES		13	/* 0..180 degrees at 15 degree steps */
#define OBS_STEP_DEG		15
#define OBS_IDX_AHEAD		6	/* 90 degrees */

#define OBS_DIST_BITS		9	/* 0..400 cm needs 9 bits */
#define OBS_DIST_NONE		0x1FF	/* spare 9-bit code: out of range, no echo within range */
#define OBS_DIST_CAP		510	/* largest real value we ever store */

/* Recommended action (3 bits, FR41) */
typedef enum {
	OBS_ACT_CONTINUE	= 0,	/* nothing to avoid */
	OBS_ACT_TURN_RIGHT	= 1,	/* bypass on the right */
	OBS_ACT_TURN_LEFT	= 2,	/* bypass on the left */
	OBS_ACT_REVERSE_RETRY	= 3,	/* clearance too tight: back off, scan again */
	OBS_ACT_STOP		= 4	/* too close to do anything: stop */
} OBS_ACTION;

/* Obstacle Detection state (3 bits for 8 states, statechart 6.3) */
typedef enum {
	OBS_ST_IDLE_MONITORING		= 0,
	OBS_ST_COARSE_SCANNING		= 1,
	OBS_ST_FINE_SCANNING		= 2,
	OBS_ST_PROFILING_AND_PLANNING	= 3,
	OBS_ST_BYPASSING		= 4,
	OBS_ST_REACQUIRING_LINE		= 5,
	OBS_ST_REVERSE_RETRY		= 6,
	OBS_ST_FAULT			= 7
} OBS_STATE;

/* Tunable values.  ALL DEFAULTS ARE PLACEHOLDERS until measured on the
   real robot and track (see testing/subsystem5/TEST_LOG.md). */
typedef struct {
	uint16_t detect_cm;		/* a reading at or below this is an obstacle */
	uint16_t stop_cm;		/* closer than this: STOP */
	uint16_t robot_half_width_cm;	/* half the robot's width */
	uint16_t margin_cm;		/* extra safety gap beside the obstacle */
	uint16_t max_shift_cm;		/* most the robot can move sideways on this track */
	uint8_t  beam_comp_deg;		/* beam spread removed from the edges: 0, 15 or 30 */
} OBS_CFG;

/* One scan: a distance per angle index, plus which indices were measured */
typedef struct {
	uint16_t dist_cm[OBS_NUM_ANGLES];	/* OBS_DIST_NONE = nothing in range */
	uint16_t measured;			/* bit i set = index i was measured */
} OBS_SCAN;

/* Result of profiling (FR40) */
typedef struct {
	uint8_t  detected;
	uint8_t  closest_idx;	/* scan index of the closest point */
	uint16_t closest_cm;
	uint16_t width_cm;	/* estimated obstacle width */
	uint16_t left_cm;	/* sideways move needed to pass on the LEFT */
	uint16_t right_cm;	/* sideways move needed to pass on the RIGHT */
	uint8_t  action;	/* OBS_ACTION */
	uint8_t  partial;	/* 1 = an edge was not found (single sample or scan limit) */
	uint8_t  idx_right;	/* first and last index of the obstacle (right/left edge) */
	uint8_t  idx_left;
} OBS_PROFILE;

void obs_cfg_default(OBS_CFG *cfg);

void obs_scan_init(OBS_SCAN *s);
void obs_scan_set(OBS_SCAN *s, unsigned idx, uint16_t cm);
int  obs_scan_measured(const OBS_SCAN *s, unsigned idx);
int  obs_scan_is_hit(const OBS_SCAN *s, unsigned idx, const OBS_CFG *cfg);

/* Fine scan helper: the next index to measure to find an edge, or -1 when
   both edges are found / limits are reached.  dir = -1 (right) or +1 (left). */
int  obs_next_edge_idx(const OBS_SCAN *s, const OBS_CFG *cfg, int dir);

void obs_profile(const OBS_SCAN *s, const OBS_CFG *cfg, OBS_PROFILE *out);

/* Conversions */
uint16_t obs_echo_us_to_cm(uint32_t echo_us);	/* round(us / 58), capped */
uint32_t obs_angle_to_pulse_us(unsigned deg);	/* servo pulse width, 0..180 degrees */
uint32_t obs_isqrt(uint32_t v);

/* i8 message to the Vehicle Controller: two 32-bit words, packed.
   word 0: bit0 detected | bits4:1 closest_idx | bits13:5 closest_cm |
           bits22:14 width_cm | bits25:23 action | bit26 partial |
           bits30:27 seq (4 bits, rises with every new profile)
   word 1: bits8:0 left_cm | bits17:9 right_cm                           */
#define OBS_MSG_WORDS		2
#define OBS_MSG_BYTES		(OBS_MSG_WORDS * 4)

void obs_pack(const OBS_PROFILE *p, uint8_t seq, uint32_t w[OBS_MSG_WORDS]);
void obs_unpack(const uint32_t w[OBS_MSG_WORDS], OBS_PROFILE *p, uint8_t *seq);

#endif /* OBSTACLE_PROFILE_H */
