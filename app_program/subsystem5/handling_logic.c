/*
 *	handling_logic.c
 *	Subsystem 5: the car was hit or picked up. See handling_logic.h.
 */

#include "handling_logic.h"

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
	return (v > 8000) ? 8000 : (v < -8000) ? -8000 : v;	/* keeps the squares in range */
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
	h->peak_jolt = 0;
	h->peak_move = 0;
	h->ev_jolt = 0;
	h->ev_move = 0;
	h->ev_nofloor_ms = 0;
}

unsigned hl_step(HL_STATE *h, const int32_t a_mg[3], const uint32_t raw[3], uint32_t t_ms)
{
	int32_t d[3], m[3], jolt, move;
	int i, k, dark, quiet;

	if (h->samples == 0u) {
		for (i = 0; i < 3; i++) {
			h->base[i] = clamp_mg(a_mg[i]) * 32;
		}
	}
	for (i = 0; i < 3; i++) {
		d[i] = clamp_mg(a_mg[i]) - h->base[i] / 32;
	}
	jolt = (int32_t)isqrt32((uint32_t)(d[0] * d[0] + d[1] * d[1]));
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
	if (jolt > h->peak_jolt) {
		h->peak_jolt = jolt;
	}
	if (move > h->peak_move) {
		h->peak_move = move;
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

	/* the resting vector follows slow changes only while all looks normal */
	quiet = (move < HL_MOVE_MG / 2) && !h->nofloor;
	if (quiet || h->samples < HL_SETTLE_SAMPLES) {
		for (i = 0; i < 3; i++) {
			h->base[i] += d[i] * 32 / (1 << HL_BASE_SHIFT);
		}
	}
	if (h->samples < HL_SETTLE_SAMPLES) {
		h->samples++;
		h->have_move = 0;
		return HL_NONE;
	}

	if (jolt >= HL_JOLT_MG) {
		h->ev_jolt = jolt;
		h->ev_move = move;
		h->ev_nofloor_ms = h->nofloor ? (t_ms - h->nofloor_since) : 0u;
		return HL_COLLISION;
	}
	if (h->nofloor && (uint32_t)(t_ms - h->nofloor_since) >= HL_NOFLOOR_MS && h->have_move &&
	    (int32_t)(h->last_move_ms - (h->nofloor_since - HL_MOVE_WIN_MS)) >= 0) {
		h->ev_jolt = jolt;
		h->ev_move = move;
		h->ev_nofloor_ms = t_ms - h->nofloor_since;
		return HL_PICKED_UP;
	}
	return HL_NONE;
}

void hl_peaks_take(HL_STATE *h, int32_t *jolt, int32_t *move)
{
	*jolt = h->peak_jolt;
	*move = h->peak_move;
	h->peak_jolt = 0;
	h->peak_move = 0;
}
