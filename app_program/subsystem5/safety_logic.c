/*
 *	safety_logic.c
 *	Subsystem 5 -- Safety Monitor decision logic. See safety_logic.h.
 */

#include "safety_logic.h"

/* obstacle_profile.h OBS_STATE values, repeated so this file stays free of it */
#define ST_COARSE_SCANNING	1u
#define ST_FINE_SCANNING	2u
#define ST_PROFILING		3u

void sm_cfg_default(SM_CFG *c)
{
	c->hold_fault_ms = 3000;
	c->hold_scan_ms  = 500;
	c->obstacle_bit  = (1u << 3);	/* FLT_OBSTACLE_BIT */
	c->fault_mask    = 0xFu;	/* bits 0..3 of the fault flags */
}

void sm_init(SM_STATE *s)
{
	s->inhibited  = 0;
	s->active_now = 0;
	s->until_ms   = 0;
	s->handled    = 0;
}

int sm_scan_state_stops(unsigned obs_state)
{
	return (obs_state == ST_COARSE_SCANNING || obs_state == ST_FINE_SCANNING ||
		obs_state == ST_PROFILING) ? 1 : 0;
}

/* a is at or after b, with the 32-bit millisecond counter wrapping */
static int reached(uint32_t a, uint32_t b)
{
	return ((int32_t)(a - b) >= 0) ? 1 : 0;
}

SM_OUT sm_step(SM_STATE *s, const SM_CFG *c, uint32_t faults, unsigned obs_state, uint32_t now_ms)
{
	SM_OUT		o = {0, 0, 0, 0};
	uint32_t	f = faults & c->fault_mask;
	uint32_t	until;
	int		have_until = s->inhibited;	/* until_ms is only meaningful while held */

	s->active_now = 0;

	if (f != 0) {
		s->active_now = 1;
		s->handled |= f;
		o.clear_bits = f;	/* handled now: clear exactly these bits */
		until = now_ms + c->hold_fault_ms;
		if (!have_until || reached(until, s->until_ms)) {
			s->until_ms = until;
		}
		have_until = 1;
	}
	if (sm_scan_state_stops(obs_state)) {
		s->active_now = 1;
		until = now_ms + c->hold_scan_ms;
		if (!have_until || reached(until, s->until_ms)) {
			s->until_ms = until;
		}
		have_until = 1;
	}

	if (s->active_now && !s->inhibited) {
		s->inhibited = 1;
		o.set_inhibit = 1;
	} else if (!s->active_now && s->inhibited && reached(now_ms, s->until_ms)) {
		/* release: reset the obstacle task if it was the one that faulted */
		s->inhibited = 0;
		o.clear_inhibit = 1;
		if ((s->handled & c->obstacle_bit) != 0) {
			o.reset_obstacle = 1;
		}
		s->handled = 0;
	}
	return o;
}
