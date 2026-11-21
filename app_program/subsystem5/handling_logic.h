/*
 *	handling_logic.h
 *	Subsystem 5: "the car was lifted or overturned" ends the run (demo rubric item 10,
 *	as changed by the lecturer on 11 Nov: collision detection with the IMU is removed;
 *	lifting the robot at the end of the demonstration must stop the motors).
 *
 *	Pure logic, no kernel calls, so it runs in the host tests. It is fed every IMU sample
 *	(subsystem 4's task, 50 Hz) with the acceleration in the car's frame (mg, x forward,
 *	y left, z up, about +920 at rest on this board) and the three line sensors' raw readings.
 *
 *	PICKED_UP: all three line sensors see no floor (a lifted sensor gets no light back
 *	  and reads like black) for HL_NOFLOOR_MS, AND the IMU moved (any change of at least
 *	  HL_MOVE_MG, averaged over HL_AVG samples (160 ms) so that motor vibration does not
 *	  count) from HL_MOVE_WIN_MS before that until now. A black junction bar or a
 *	  barcode also reads black on all three, but the car is not lifted there, so the IMU
 *	  stays quiet; a hump tilts the car, but the sensors still see the floor.
 *	OVERTURNED: the IMU's up axis reads below HL_OVER_Z_MG (tilted more than about 64
 *	  degrees, or upside down) for HL_OVER_MS. A hump tilts the car about 15-20 degrees.
 *
 *	The resting gravity vector follows slow changes (a hump, a sloped table): quickly while
 *	all is quiet, slowly (HL_BASE_SLOW_SHIFT) while the car moves, and not at all while the
 *	line sensors see no floor (the car may be in the air). The slow follow stops a wrong
 *	resting vector (the car handled during the first second) from sticking for good; M39
 *	run 1 showed "moved" stuck at about 105 mg with the car standing still.
 *	Nothing is reported for the first HL_SETTLE_SAMPLES.
 */

#ifndef HANDLING_LOGIC_H
#define HANDLING_LOGIC_H

#include <stdint.h>

#define HL_MOVE_MG		150	/* any change: the car is being moved */
#define HL_MOVE_WIN_MS		1000u	/* that movement counts this long before the sensors lose the floor */
#define HL_NOFLOOR_MS		250u	/* all three line sensors without floor this long */
#define HL_LC_DARK		2000u	/* left / centre raw at or above this: no light back (white ~150-300, line ~3000) */
#define HL_R_DARK		1000u	/* GP28 raw at or above this (white ~150-250, bars ~1450-2800) */
#define HL_OVER_Z_MG		400	/* up axis below this: overturned (920 mg at rest, cos 64 deg x 920) */
#define HL_OVER_MS		500u	/* ... for this long */
#define HL_SETTLE_SAMPLES	50u	/* 1 s at 50 Hz before anything is reported */
#define HL_AVG			8	/* samples averaged for the movement */
#define HL_BASE_SHIFT		5	/* resting vector, quiet: moves 1/32 of the way per sample (about 0.6 s) */
#define HL_BASE_SLOW_SHIFT	7	/* resting vector, moving: 1/128 per sample (about 2.5 s; a lift is
					   about 0.1 s before the floor is lost, so it absorbs ~4% of it) */

#define HL_NONE			0u
#define HL_PICKED_UP		2u
#define HL_OVERTURNED		3u

typedef struct {
	int32_t		base[3];	/* resting gravity vector, mg x 256 */
	int32_t		hist[HL_AVG][3];	/* the last HL_AVG changes from it (for the averaged movement) */
	uint32_t	hi;		/* next slot in hist */
	uint32_t	samples;
	int		have_move;	/* last_move_ms is valid */
	uint32_t	last_move_ms;	/* last sample that moved at least HL_MOVE_MG */
	int		nofloor;	/* the line sensors see no floor */
	uint32_t	nofloor_since;
	int		over;		/* the up axis is below HL_OVER_Z_MG */
	uint32_t	over_since;
	/* for the console: the extremes since hl_peaks_take() */
	int32_t		peak_move;
	int32_t		low_z;
	/* what the last event saw */
	int32_t		ev_move;
	int32_t		ev_z;
	uint32_t	ev_nofloor_ms;
} HL_STATE;

void     hl_init(HL_STATE *h);
/* One IMU sample. a_mg: x forward, y left, z up. raw: left, centre, GP28 (raw ADC).
   Returns HL_NONE, HL_PICKED_UP or HL_OVERTURNED (every sample while it lasts). */
unsigned hl_step(HL_STATE *h, const int32_t a_mg[3], const uint32_t raw[3], uint32_t t_ms);
/* largest change and lowest up axis since the last call (mg), then starts again */
void     hl_peaks_take(HL_STATE *h, int32_t *move, int32_t *low_z);

#endif /* HANDLING_LOGIC_H */
