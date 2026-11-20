/*
 *	handling_logic.h
 *	Subsystem 5: "the car was hit or picked up" (demo rubric: stop the motors when the car
 *	is picked up, aka collision detection; this ends the demo).
 *
 *	Pure logic, no kernel calls, so it runs in the host tests. It is fed every IMU sample
 *	(subsystem 4's task, 50 Hz) with the acceleration in the car's frame (mg, x forward,
 *	y left, z up, about +1000 at rest) and the three line sensors' raw readings.
 *
 *	COLLISION: a jolt along the floor. The sideways part (x, y) of the change from the
 *	  resting gravity vector reaches HL_JOLT_MG in one sample (a knock lasts a few ms;
 *	  driving, turning and starting are well under 0.1 g).
 *	PICKED_UP: all three line sensors see no floor (a lifted sensor gets no light back
 *	  and reads like black) for HL_NOFLOOR_MS, AND the IMU moved (any change of at least
 *	  HL_MOVE_MG, averaged over HL_AVG samples (160 ms) so that motor vibration does not
 *	  count) from HL_MOVE_WIN_MS before that until now. A black junction bar or a
 *	  barcode also reads black on all three, but the car is not lifted there, so the IMU
 *	  stays quiet; a hump tilts the car, but the sensors still see the floor.
 *
 *	The resting gravity vector follows slow changes (a hump, a sloped table) and is held
 *	while anything looks wrong. Nothing is reported for the first HL_SETTLE_SAMPLES.
 */

#ifndef HANDLING_LOGIC_H
#define HANDLING_LOGIC_H

#include <stdint.h>

#define HL_JOLT_MG		450	/* sideways jolt: collision */
#define HL_MOVE_MG		150	/* any change: the car is being moved */
#define HL_MOVE_WIN_MS		1000u	/* that movement counts this long before the sensors lose the floor */
#define HL_NOFLOOR_MS		250u	/* all three line sensors without floor this long */
#define HL_LC_DARK		2000u	/* left / centre raw at or above this: no light back (white ~150-300, line ~3000) */
#define HL_R_DARK		1000u	/* GP28 raw at or above this (white ~150-250, bars ~1450-2800) */
#define HL_SETTLE_SAMPLES	50u	/* 1 s at 50 Hz before anything is reported */
#define HL_AVG			8	/* samples averaged for the movement */
#define HL_BASE_SHIFT		5	/* resting vector: moves 1/32 of the way per sample (about 0.6 s) */

#define HL_NONE			0u
#define HL_COLLISION		1u
#define HL_PICKED_UP		2u

typedef struct {
	int32_t		base[3];	/* resting gravity vector, mg x 32 */
	int32_t		hist[HL_AVG][3];	/* the last HL_AVG changes from it (for the averaged movement) */
	uint32_t	hi;		/* next slot in hist */
	uint32_t	samples;
	int		have_move;	/* last_move_ms is valid */
	uint32_t	last_move_ms;	/* last sample that moved at least HL_MOVE_MG */
	int		nofloor;	/* the line sensors see no floor */
	uint32_t	nofloor_since;
	/* for the console: the largest values since hl_peaks_take() */
	int32_t		peak_jolt;
	int32_t		peak_move;
	/* what the last event saw */
	int32_t		ev_jolt;
	int32_t		ev_move;
	uint32_t	ev_nofloor_ms;
} HL_STATE;

void     hl_init(HL_STATE *h);
/* One IMU sample. a_mg: x forward, y left, z up. raw: left, centre, GP28 (raw ADC).
   Returns HL_NONE, HL_COLLISION or HL_PICKED_UP (every sample while it lasts). */
unsigned hl_step(HL_STATE *h, const int32_t a_mg[3], const uint32_t raw[3], uint32_t t_ms);
/* largest sideways jolt and change since the last call (mg), then starts again */
void     hl_peaks_take(HL_STATE *h, int32_t *jolt, int32_t *move);

#endif /* HANDLING_LOGIC_H */
