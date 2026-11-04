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
	v->creep_on     = 0;
	v->creep_done   = 0;
	v->creep_start  = 0;
	v->creep_have_cmd = 0;
	v->creep_speed  = 0;
	v->creep_steer  = 0;
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

/* One cycle of a creep (forward along the line) or back-away (straight). */
static VC_OUT creep_step(VC_STATE *v, const VC_IN *in, VC_OUT o)
{
	int back = (in->approach_mm < 0);
	int want = iabs(in->approach_mm);
	int moved;

	if (!v->creep_on) {
		v->creep_on       = 1;
		v->creep_done     = 0;
		v->creep_start    = in->odo_mm & 0xFFFFu;
		v->creep_have_cmd = 0;
		v->have_prev      = 0;		/* no D kick at the start */
		v->lost_ms        = 0;
	}
	/* signed distance since the start: a wheel that rocks back by a few mm at the
	   start must not look like a whole lap */
	moved = (int)(int16_t)((in->odo_mm - v->creep_start) & 0xFFFFu);
	if (back) {
		moved = -moved;
	}
	if (moved >= want) {
		v->creep_done = 1;
	}
	if (v->creep_done) {
		o.creep_done = 1;		/* stand still until Obstacle Detection takes it */
		return o;
	}
	if (back) {
		o.mode  = VC_MODE_DRIVE;
		o.speed = -VC_APPROACH_SPEED;
		o.steer = VC_REVERSE_STEER;
		return o;
	}
	if (v->line_gave_up) {			/* line lost for good: nothing to follow, rescan */
		v->creep_done = 1;
		o.creep_done  = 1;
		return o;
	}
	if (!in->line_fresh) {
		return o;			/* no line data: do not creep blind (it times out) */
	}
	if (in->dt_ms > 0 || !v->creep_have_cmd) {
		if (follow_line(v, in, &o)) {
			v->creep_done = 1;
			o.creep_done  = 1;
			o.speed = 0;
			o.steer = 0;
			return o;
		}
		/* the line gains are for 100 mm/s: scale them down for the creep speed */
		if (o.speed > VC_APPROACH_SPEED) {
			o.speed = VC_APPROACH_SPEED;
		}
		o.steer = o.steer * VC_APPROACH_SPEED / VC_LF_SPEED;
		v->creep_speed    = o.speed;
		v->creep_steer    = o.steer;
		v->creep_have_cmd = 1;
	}
	o.mode  = VC_MODE_DRIVE;
	o.speed = v->creep_speed;
	o.steer = v->creep_steer;
	return o;
}

VC_OUT vc_step(VC_STATE *v, const VC_IN *in)
{
	VC_OUT	o;
	int	hold;

	o.mode    = VC_MODE_STOP;
	o.speed   = 0;
	o.steer   = 0;
	o.holding = 0;
	o.creep_done = 0;

	/* Stand still unless Obstacle Detection is idle and the stop latch is clear.
	   A state that is not idle means an obstacle was seen and is being scanned. */
	hold = (in->obs_state != VC_OBS_IDLE) || (in->motion_state == VC_STATE_INHIBITED);
	if (hold) {
		if (!v->holding) {
			v->holding  = 1;
			v->creep_on = 0;
			v->creep_done = 0;
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

	/* move ordered by Obstacle Detection: creep along the line, or back away */
	if (in->approach_mm != 0) {
		return creep_step(v, in, o);
	}
	if (v->creep_on) {			/* the order was taken: line following starts clean */
		v->creep_on   = 0;
		v->creep_done = 0;
		v->have_prev  = 0;
		v->lost_ms    = 0;
	}
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
