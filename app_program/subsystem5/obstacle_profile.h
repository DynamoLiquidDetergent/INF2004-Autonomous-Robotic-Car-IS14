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
	uint16_t plan_width_cm;		/* plan the bypass for AT LEAST this obstacle width
					   (spec: obstacles are 10..20 cm wide, so 20). 0 = off */
	uint16_t bg_enter_cm;		/* with a background: a reading starts an obstacle only
					   when it is this much CLOSER than the background */
	uint16_t bg_exit_cm;		/* ...and keeps an obstacle going while it is at least
					   this much closer (smaller than bg_enter_cm = hysteresis) */
	uint16_t detect_hyst_cm;	/* no background: enter at detect_cm, keep going up to
					   detect_cm + this */
	uint16_t approach_cm;		/* after the first scan the car moves to this gap
					   and scans again from there (the middle of the band) */
	uint16_t approach_near_cm;	/* gap below this: back away (band lower end) */
	uint16_t approach_far_cm;	/* gap above this: creep forward (band upper end) */
	uint16_t approach_guard_cm;	/* while creeping forward, a reading at or below this cuts
					   the creep short and starts the scan at once */
	uint16_t approach_back_max_cm;	/* most the car backs away when it stopped too close
					   (nothing watches behind it, so keep this short) */
} OBS_CFG;

/* One scan: a distance per angle index, plus which indices were measured */
typedef struct {
	uint16_t dist_cm[OBS_NUM_ANGLES];	/* OBS_DIST_NONE = nothing in range */
	uint16_t measured;			/* bit i set = index i was measured */
	uint16_t bg_cm[OBS_NUM_ANGLES];		/* background distance per angle (empty track) */
	uint16_t bg_known;			/* bit i set = background for index i was taken */
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
	uint16_t plan_width_cm;	/* width the left/right shifts were planned for (>= width_cm) */
	uint16_t unc_cm;	/* +/- uncertainty of width_cm from the 15 degree step (not sent) */
	uint8_t  clusters;	/* how many separate obstacles the scan saw (not sent) */
} OBS_PROFILE;

void obs_cfg_default(OBS_CFG *cfg);

void obs_scan_init(OBS_SCAN *s);	/* clears the readings AND the background */
void obs_scan_clear(OBS_SCAN *s);	/* clears the readings, keeps the background */
void obs_scan_set_bg(OBS_SCAN *s, unsigned idx, uint16_t cm);
/* The path straight ahead (indices 5..7) must be empty when the background is
   taken. If a background value there is within detect range, forget it (so a
   real obstacle can never be hidden as "background"). Returns how many. */
unsigned obs_bg_clear_ahead(OBS_SCAN *s, const OBS_CFG *cfg);
/* Median of n (1..5) readings. OBS_DIST_NONE counts as the largest value, so
   one stray reading never wins and a majority of "nothing" gives "nothing". */
uint16_t obs_median(const uint16_t *v, unsigned n);
/* Would a reading of cm at index idx START an obstacle? (hysteresis: enter rule) */
int  obs_dist_is_hit(const OBS_SCAN *s, unsigned idx, uint16_t cm, const OBS_CFG *cfg);
void obs_scan_set(OBS_SCAN *s, unsigned idx, uint16_t cm);
int  obs_scan_measured(const OBS_SCAN *s, unsigned idx);
int  obs_scan_is_hit(const OBS_SCAN *s, unsigned idx, const OBS_CFG *cfg);

/* Fine scan helper: the next index to measure to find an edge, or -1 when
   both edges are found / limits are reached.  dir = -1 (right) or +1 (left). */
int  obs_next_edge_idx(const OBS_SCAN *s, const OBS_CFG *cfg, int dir);

/* Fine scan helper for scans with SEVERAL obstacles: the next index to measure
   (an unmeasured neighbour of any obstacle, inside the servo window lo..hi),
   the one closest to the servo's current index 'from'; -1 when nothing is left. */
int  obs_next_fine_idx(const OBS_SCAN *s, const OBS_CFG *cfg, int lo, int hi, int from);

/* Profile of the whole scan. Width, closest and idx_* describe the CLOSEST
   obstacle. The bypass side and shifts take EVERY obstacle into account: a side
   whose corridor would touch another obstacle is reported as OBS_DIST_CAP
   (not possible), and if neither side is free the action is REVERSE_RETRY. */
void obs_profile(const OBS_SCAN *s, const OBS_CFG *cfg, OBS_PROFILE *out);

/* Creep before the second scan. How far (mm) the car should drive straight
   ahead from where it stands so that the gap to the closest point of the
   obstacle becomes cfg->approach_cm. The gap is measured straight ahead, not
   along the beam: the closest point is at scan index closest_idx, so its
   distance ahead is closest_cm x sin(angle).
   Returns 0 (no move) when nothing was detected, when the profile did not
   choose a bypass (OBS_ACT_TURN_*), or when the gap is already inside the
   band approach_near_cm .. approach_far_cm (15..20 cm). Never more than 1023 mm
   (Motion Control's DRIVE_DIST range).
   Too close (gap below approach_near_cm, but the profile still chose a
   bypass): the answer is NEGATIVE, back away that many mm to approach_cm, at
   most approach_back_max_cm. Positive = drive forward. */
int obs_approach_mm(const OBS_PROFILE *p, const OBS_CFG *cfg);

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
