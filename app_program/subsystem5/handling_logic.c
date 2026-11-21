/*
 *	handling_logic.c
 *	Subsystem 5: the car was lifted or overturned. See handling_logic.h.
 */

#include "handling_logic.h"

#define HL_BASE_SCALE	256	/* base[] holds mg x 256 so that the slow follow (1/256) is exact */

static uint32_t isqrt32(uint32_t v)
{
	uint32_t r = 0, bit = 1u << 30;

	while (bit > v) {
		bit >>= 2;
	}
	while (bit != 0u) {
		if (v >= r + bit) {
			v -= r + bit;
			r = (r >> 1) + bit;
		} else {
			r >>= 1;
		}
		bit >>= 2;
	}
	return r;
}

static int32_t clamp_mg(int32_t v)
{
	return (v > 8000) ? 8000 : (v < -8000) ? -8000 : v;	/* keeps the squares and base[] in range */
}

void hl_init(HL_STATE *h)
{
	int i;

	for (i = 0; i < 3; i++) {
		int k;

		h->base[i] = 0;
		for (k = 0; k < HL_AVG; k++) {
			h->hist[k][i] = 0;
		}
	}
	h->samples = 0;
	h->hi = 0;
	h->have_move = 0;
	h->last_move_ms = 0;
	h->nofloor = 0;
	h->nofloor_since = 0;
	h->over = 0;
	h->over_since = 0;
	h->peak_move = 0;
	h->low_z = 8000;
	h->ev_move = 0;
	h->ev_z = 0;
	h->ev_nofloor_ms = 0;
}

unsigned hl_step(HL_STATE *h, const int32_t a_mg[3], const uint32_t raw[3], uint32_t t_ms)
{
	int32_t a[3], d[3], m[3], move;
	int i, k, dark, shift;

	for (i = 0; i < 3; i++) {
		a[i] = clamp_mg(a_mg[i]);
	}
	if (h->samples == 0u) {
		for (i = 0; i < 3; i++) {
			h->base[i] = a[i] * HL_BASE_SCALE;
		}
	}
	for (i = 0; i < 3; i++) {
		d[i] = a[i] - h->base[i] / HL_BASE_SCALE;
	}
	for (i = 0; i < 3; i++) {
		h->hist[h->hi][i] = d[i];
		m[i] = 0;
		for (k = 0; k < HL_AVG; k++) {
			m[i] += h->hist[k][i];
		}
		m[i] /= HL_AVG;
	}
	h->hi = (h->hi + 1u) % HL_AVG;
	move = (int32_t)isqrt32((uint32_t)(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]));
	if (move > h->peak_move) {
		h->peak_move = move;
	}
	if (a[2] < h->low_z) {
		h->low_z = a[2];
	}
	if (move >= HL_MOVE_MG) {
		h->have_move = 1;
		h->last_move_ms = t_ms;
	}

	dark = (raw[0] >= HL_LC_DARK && raw[1] >= HL_LC_DARK && raw[2] >= HL_R_DARK);
	if (dark && !h->nofloor) {
		h->nofloor = 1;
		h->nofloor_since = t_ms;
	} else if (!dark) {
		h->nofloor = 0;
	}
	if (a[2] < HL_OVER_Z_MG && !h->over) {
		h->over = 1;
		h->over_since = t_ms;
	} else if (a[2] >= HL_OVER_Z_MG) {
		h->over = 0;
	}

	/* the resting vector: quick while quiet, slow while moving, held while off the floor */
	if (h->samples < HL_SETTLE_SAMPLES || (move < HL_MOVE_MG / 2 && !h->nofloor)) {
		shift = HL_BASE_SHIFT;
	} else if (!h->nofloor) {
		shift = HL_BASE_SLOW_SHIFT;
	} else {
		shift = -1;
	}
	if (shift >= 0) {
		for (i = 0; i < 3; i++) {
			h->base[i] += d[i] * (HL_BASE_SCALE >> shift);
		}
	}
	if (h->samples < HL_SETTLE_SAMPLES) {
		h->samples++;
		h->have_move = 0;
		h->over = 0;
		return HL_NONE;
	}

	if (h->over && (uint32_t)(t_ms - h->over_since) >= HL_OVER_MS) {
		h->ev_move = move;
		h->ev_z = a[2];
		h->ev_nofloor_ms = h->nofloor ? (t_ms - h->nofloor_since) : 0u;
		return HL_OVERTURNED;
	}
	if (h->nofloor && (uint32_t)(t_ms - h->nofloor_since) >= HL_NOFLOOR_MS && h->have_move &&
	    (int32_t)(h->last_move_ms - (h->nofloor_since - HL_MOVE_WIN_MS)) >= 0) {
		h->ev_move = move;
		h->ev_z = a[2];
		h->ev_nofloor_ms = t_ms - h->nofloor_since;
		return HL_PICKED_UP;
	}
	return HL_NONE;
}

void hl_peaks_take(HL_STATE *h, int32_t *move, int32_t *low_z)
{
	*move = h->peak_move;
	*low_z = h->low_z;
	h->peak_move = 0;
	h->low_z = 8000;
}
