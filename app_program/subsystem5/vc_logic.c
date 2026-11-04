/*
 *	vc_logic.c
 *	Subsystem 5 -- Vehicle Controller decision logic. See vc_logic.h.
 */

#include "vc_logic.h"

static int clamp(int v, int lim)
{
	return (v > lim) ? lim : ((v < -lim) ? -lim : v);
}

static int iabs(int v)
{
	return (v < 0) ? -v : v;
}

void vc_init(VC_STATE *v, unsigned epoch)
{
	v->err_prev     = 0;
	v->have_prev    = 0;
	v->lost_ms      = 0;
	v->line_gave_up = 0;
	v->holding      = 0;
	v->epoch        = epoch & 0xFu;
}

/* The line controller: one cycle. Fills speed and steer. Returns 1 to stop. */
static int follow_line(VC_STATE *v, const VC_IN *in, VC_OUT *o)
{
	int err = in->zone - VC_ZONE_CENTRE;	/* + = line towards the LEFT sensor */
	int turn;

	if (in->zone == 0) {
		err = 0;
	}
	if (in->lost) {
		v->lost_ms += in->dt_ms;
		if (v->lost_ms >= VC_LF_LOST_STOP_MS) {
			v->line_gave_up = 1;
			return 1;
		}
		/* the last zone is repeated while lost: turn towards that side */
		turn        = (err > 0) ? VC_LF_SEARCH_STEER : ((err < 0) ? -VC_LF_SEARCH_STEER : 0);
		o->speed    = VC_LF_SEARCH_SPEED;
		o->steer    = -VC_LF_STEER_SIGN * turn;
		v->have_prev = 0;		/* no D kick when the line comes back */
		return 0;
	}
	v->lost_ms = 0;
	turn = VC_LF_KP * err;
	if (v->have_prev) {
		turn += VC_LF_KD * (err - v->err_prev);
	}
	v->err_prev  = err;
	v->have_prev = 1;
	o->steer = clamp(-VC_LF_STEER_SIGN * turn, VC_LF_STEER_MAX);
	o->speed = VC_LF_SPEED - VC_LF_SLOW_PER_ZONE * iabs(err);
	if (o->speed < VC_LF_MIN_SPEED) {
		o->speed = VC_LF_MIN_SPEED;
	}
	return 0;
}

VC_OUT vc_step(VC_STATE *v, const VC_IN *in)
{
	VC_OUT	o;
	int	hold;

	o.mode    = VC_MODE_STOP;
	o.speed   = 0;
	o.steer   = 0;
	o.holding = 0;

	/* Stand still unless Obstacle Detection is idle and the stop latch is clear.
	   A state that is not idle means an obstacle was seen and is being scanned. */
	hold = (in->obs_state != VC_OBS_IDLE) || (in->motion_state == VC_STATE_INHIBITED);
	if (hold) {
		if (!v->holding) {
			v->holding  = 1;
			v->epoch    = (v->epoch + 1u) & 0xFu;	/* Motion Control needs a newer epoch */
			v->have_prev = 0;
			v->lost_ms   = 0;
		}
		o.epoch   = v->epoch;
		o.holding = 1;
		return o;
	}
	v->holding = 0;
	o.epoch = v->epoch;

	if (v->line_gave_up || !in->line_fresh) {
		return o;		/* no line data, or the line is gone: do not drive blind */
	}
	if (!follow_line(v, in, &o)) {
		o.mode = VC_MODE_DRIVE;
	} else {
		o.speed = 0;
		o.steer = 0;
	}
	return o;
}
