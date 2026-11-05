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

#define JN_NONE		0u
#define JN_DRIVE	1u	/* driving on (sensors are ahead of the wheels), or straight over */
#define JN_TURN		2u	/* turning on the spot */

/* signed distance since a 16-bit odometry mark (mm) */
static int odo_since(unsigned now, unsigned mark)
{
	return (int)(int16_t)((now - mark) & 0xFFFFu);
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
	v->creep_acc    = 0;
	v->started      = 0;
	v->paused       = 0;
	v->aborted      = 0;
	v->bc_pending   = VC_BC_NONE;
	v->bc_odo       = 0;
	v->barcodes     = 0;
	v->last_action  = VC_ACT_FORWARD;
	v->jn_armed     = 1;
	v->jn_conf      = 0;
	v->jn_rearm_odo = 0;
	v->jn_state     = JN_NONE;
	v->jn_cmd       = VC_BC_NONE;
	v->jn_on        = 0;
	v->jn_odo0      = 0;
	v->jn_acc       = 0;
	v->turn_dir     = 1;
	v->turn_target_d10 = 0;
	v->turn_cmd_deg = 0;
	v->turn_live    = 0;
	v->turn_ms      = 0;
	v->turn_arc_um  = 0;
	v->search_leg   = 0;
	v->search_dir   = 1;
	v->search_used  = 0;
	v->found_cnt    = 0;
	v->byp_step     = 0;
	v->byp_dir      = 1;
	v->byp_s_mm     = 0;
	v->byp_f_mm     = 0;
	v->drv_mm       = 0;
	v->drv_speed    = 0;
	v->drv_find     = 0;
	v->saw_lost     = 0;
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
			return 1;		/* the caller decides: spin search, or give up */
		}
		/* the last zone is repeated while lost: turn towards that side */
		turn        = (err > 0) ? VC_LF_SEARCH_STEER : ((err < 0) ? -VC_LF_SEARCH_STEER : 0);
		o->speed    = VC_LF_SEARCH_SPEED;
		o->steer    = -VC_LF_STEER_SIGN * turn;
		v->have_prev = 0;		/* no D kick when the line comes back */
		return 0;
	}
	v->lost_ms = 0;
	v->search_used = 0;		/* line seen: a new loss gets a new sweep */
	turn = VC_LF_KP * err;
	if (v->have_prev) {
		turn += VC_LF_KD * (err - v->err_prev);
	}
	v->err_prev  = err;
	v->have_prev = 1;
	o->steer = clamp(-VC_LF_STEER_SIGN * turn, VC_LF_STEER_MAX);
	if (in->bc_busy && iabs(err) <= 1) {
		/* GP28 is reading bars: a steady speed over them matters more than a tidy line */
		o->steer = clamp(o->steer, VC_BC_STEER_MAX);
	}
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
	int want = iabs(in->approach_mm) - (int)v->creep_acc;	/* what is left after a pause */
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
		if (follow_line(v, in, &o)) {		/* line lost: stop the creep and rescan from here */
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

/* Mission state from the flags. Order: aborted, not started, fault, paused, avoiding. */
static unsigned mission_of(const VC_STATE *v, const VC_IN *in)
{
	if (v->aborted) {
		return VC_MS_ABORTED;
	}
	if (!v->started) {
		return in->ready ? VC_MS_READY : VC_MS_INIT;
	}
	if (v->line_gave_up) {
		return VC_MS_FAULT;		/* line lost for good: stopped */
	}
	if (in->fault_mask != 0 || in->obs_state == VC_OBS_FAULT) {
		return VC_MS_FAULT;
	}
	if (v->paused) {
		return VC_MS_PAUSED;
	}
	if (in->obs_state != VC_OBS_IDLE || in->approach_mm != 0) {
		return VC_MS_AVOIDING;
	}
	if (v->jn_state != JN_NONE && v->jn_cmd != VC_BC_FORWARD) {
		return VC_MS_TURNING;
	}
	return VC_MS_RUNNING;
}

/* A creep that is stopped part-way (pause, hold): remember how far it got, so that
   after the stop only the rest is driven. */
static void creep_remember(VC_STATE *v, const VC_IN *in)
{
	int moved = (int)(int16_t)((in->odo_mm - v->creep_start) & 0xFFFFu);

	if (in->approach_mm < 0) {
		moved = -moved;
	}
	if (moved > 0) {
		v->creep_acc += (unsigned)moved;
	}
}


/* IF-07: take the barcode commands. The newest one waits for the next junction. */
static void bc_take(VC_STATE *v, const VC_IN *in, VC_OUT *o)
{
	if (in->bc_n > 0) {
		v->barcodes += in->bc_n;
		if (v->barcodes > 255u) {
			v->barcodes = 255u;
		}
	}
	if (in->bc_cmd != VC_BC_NONE) {
		v->bc_pending = in->bc_cmd;
		v->bc_odo     = in->odo_mm & 0xFFFFu;
		o->event     |= VC_EV_BC_TAKEN;
	}
	if (v->bc_pending != VC_BC_NONE && v->jn_state == JN_NONE &&
	    odo_since(in->odo_mm, v->bc_odo) > VC_BC_EXPIRE_MM) {
		v->bc_pending = VC_BC_NONE;	/* no junction came: a later one is not meant for it */
		o->event     |= VC_EV_BC_EXPIRED;
	}
}

/* Angle turned so far in the direction of the turn, tenths of a degree. The wheel
   speeds give it: left minus right wheel travel is track width times the angle. */
static int turned_d10(const VC_STATE *v)
{
	return (int)(((int64_t)v->turn_arc_um * v->turn_dir * 573) / ((int64_t)VC_TRACK_MM * 1000));
}



/* ---- primitives shared by the junction manoeuvre, the search and the bypass ---- */
static void turn_set(VC_STATE *v, int dir, int deg)
{
	v->jn_state        = JN_TURN;
	v->jn_on           = 0;
	v->turn_dir        = dir;
	v->turn_target_d10 = deg * 10;
	v->turn_arc_um     = 0;
	v->turn_live       = 0;
}

static void drive_set(VC_STATE *v, int mm, int speed, int find)
{
	v->jn_state  = JN_DRIVE;
	v->jn_on     = 0;
	v->jn_acc    = 0;
	v->drv_mm    = mm;
	v->drv_speed = speed;
	v->drv_find  = find;
	v->found_cnt = 0;
	v->saw_lost  = 0;
}

static void jn_finish(VC_STATE *v, const VC_IN *in, VC_OUT *o);

/* Next step of the bypass detour. Returns 1 when a new leg was set up, 0 when the detour is over. */
static int byp_next(VC_STATE *v, const VC_IN *in, VC_OUT *o)
{
	v->byp_step++;
	switch (v->byp_step) {
	case 2:
		drive_set(v, v->byp_s_mm, VC_BYP_SPEED, 0);		/* sideways, away from the line */
		return 1;
	case 3:
		turn_set(v, -v->byp_dir, VC_BYP_TURN_DEG);		/* face the old direction */
		return 1;
	case 4:
		drive_set(v, v->byp_f_mm, VC_BYP_SPEED, 0);		/* past the obstacle */
		return 1;
	case 5:
		o->event |= VC_EV_BYP_PASSED;
		turn_set(v, -v->byp_dir, VC_BYP_TURN_DEG);		/* face the line */
		return 1;
	case 6:
		o->event |= VC_EV_BYP_RETURN;
		drive_set(v, v->byp_s_mm + VC_BYP_FIND_EXTRA_MM, VC_JN_SPEED, 1);	/* until the line is seen */
		return 1;
	case 7:
		drive_set(v, VC_BYP_ADV_MM, VC_JN_SPEED, 0);		/* wheels over the line */
		return 1;
	case 8:
		turn_set(v, v->byp_dir, VC_BYP_TURN_DEG);		/* face along the line */
		return 1;
	default:
		v->byp_step = 0;
		jn_finish(v, in, o);
		o->event |= VC_EV_BYP_DONE;
		return 0;
	}
}

/* One leg of the spin search: the first and last go VC_SEARCH_DEG towards the side the line
   was lost on, the middle one twice as far the other way. */
static void search_leg_set(VC_STATE *v)
{
	v->turn_dir        = (v->search_leg == 2) ? -v->search_dir : v->search_dir;
	v->turn_target_d10 = ((v->search_leg == 2) ? 2 : 1) * VC_SEARCH_DEG * 10;
	v->turn_arc_um     = 0;
	v->turn_live       = 0;
}

static int search_begin(VC_STATE *v, const VC_IN *in, VC_OUT *o)
{
	int err = in->zone - VC_ZONE_CENTRE;	/* + = line towards the LEFT sensor */

	if (in->obst_present || v->search_used) {
		return 0;			/* no spinning next to an obstacle, one sweep per loss */
	}
	v->search_used = 1;
	v->search_leg  = 1;
	v->search_dir  = (err > 0) ? -VC_LF_STEER_SIGN : ((err < 0) ? VC_LF_STEER_SIGN : 1);
	v->found_cnt   = 0;
	v->jn_state    = JN_TURN;
	v->jn_cmd      = VC_BC_NONE;
	v->jn_on       = 0;
	search_leg_set(v);
	o->event      |= VC_EV_SEARCH;
	return 1;
}

static void jn_finish(VC_STATE *v, const VC_IN *in, VC_OUT *o)
{
	v->jn_state     = JN_NONE;
	v->jn_on        = 0;
	v->turn_live    = 0;
	v->jn_armed     = 0;
	v->jn_rearm_odo = in->odo_mm & 0xFFFFu;
	v->have_prev    = 0;		/* line following starts clean */
	v->lost_ms      = 0;
	o->event       |= VC_EV_JN_DONE;
}

static void jn_start(VC_STATE *v, VC_OUT *o)
{
	v->jn_cmd       = v->bc_pending;
	v->bc_pending   = VC_BC_NONE;
	drive_set(v, VC_JN_ADVANCE_MM, VC_JN_SPEED, 0);
	v->turn_live    = 0;
	v->turn_arc_um  = 0;
	switch (v->jn_cmd) {
	case VC_BC_LEFT:
		v->turn_dir = -1;
		v->turn_target_d10 = VC_TURN_DEG * 10;
		v->last_action = VC_ACT_LEFT;
		break;
	case VC_BC_RIGHT:
		v->turn_dir = 1;
		v->turn_target_d10 = VC_TURN_DEG * 10;
		v->last_action = VC_ACT_RIGHT;
		break;
	case VC_BC_UTURN:			/* turns on the spot where it stands */
		v->turn_dir = 1;
		v->turn_target_d10 = VC_UTURN_DEG * 10;
		v->last_action = VC_ACT_UTURN;
		v->jn_state = JN_TURN;
		break;
	default:				/* FORWARD: straight over */
		v->drv_mm      = VC_JN_CROSS_MM;
		v->last_action = VC_ACT_FORWARD;
		break;
	}
	o->event |= VC_EV_JN_START;
}

static int jn_step(VC_STATE *v, const VC_IN *in, VC_OUT *o);

/* A turn (junction turn or one leg of the search) is over. */
static int turn_over(VC_STATE *v, const VC_IN *in, VC_OUT *o)
{
	if (v->byp_step != 0) {
		return byp_next(v, in, o) ? jn_step(v, in, o) : 0;
	}
	if (v->search_leg == 0) {
		jn_finish(v, in, o);
		return 0;
	}
	if (v->search_leg < 3) {
		v->search_leg++;
		search_leg_set(v);
		return jn_step(v, in, o);	/* next leg */
	}
	v->search_leg   = 0;			/* swept both ways and back: the line is not there */
	jn_finish(v, in, o);
	v->line_gave_up = 1;
	o->event       |= VC_EV_GAVE_UP;
	return 0;
}

/* Junction handling in normal driving, and the spin search. Returns 1 when it has
   filled the command, 0 to go on with line following. */
static int jn_step(VC_STATE *v, const VC_IN *in, VC_OUT *o)
{
	if (v->jn_state == JN_NONE) {
		if (!v->jn_armed && !in->junction &&
		    odo_since(in->odo_mm, v->jn_rearm_odo) >= VC_JN_REARM_MM) {
			v->jn_armed = 1;
		}
		if (in->dt_ms > 0) {		/* count per new line message */
			if (in->junction && !in->lost && in->line_fresh) {
				if (v->jn_conf < 1000) {
					v->jn_conf++;
				}
			} else {
				v->jn_conf = 0;
			}
		}
		if (v->bc_pending != VC_BC_NONE && v->jn_armed && v->jn_conf >= VC_JN_CONFIRM_MSGS) {
			v->jn_armed = 0;
			jn_start(v, o);
		}
		if (v->jn_state == JN_NONE) {
			return 0;
		}
	}
	if (v->jn_state == JN_DRIVE) {
		int moved;

		if (!v->jn_on) {
			v->jn_on   = 1;
			v->jn_odo0 = in->odo_mm & 0xFFFFu;
		}
		moved = odo_since(in->odo_mm, v->jn_odo0) + (int)v->jn_acc;
		if (v->drv_find) {			/* bypass: until the line is seen again */
			if (in->lost) {
				v->saw_lost = 1;	/* the old 'seen' flag is stale until S3 has noticed */
			}
			if (in->dt_ms > 0) {
				if (v->saw_lost && in->line_fresh && !in->lost && in->zone != 0) {
					v->found_cnt++;
				} else {
					v->found_cnt = 0;
				}
			}
			if (v->found_cnt >= VC_SEARCH_FOUND_MSGS) {
				return byp_next(v, in, o) ? jn_step(v, in, o) : 0;
			}
		}
		if (moved < v->drv_mm) {
			o->mode  = VC_MODE_DRIVE;
			o->speed = v->drv_speed;
			o->steer = 0;		/* straight on the wheel encoders, not by the sensors */
			return 1;
		}
		if (v->drv_find) {		/* the line was not where it should be */
			v->byp_step = 0;
			jn_finish(v, in, o);
			v->line_gave_up = 1;
			o->event |= VC_EV_BYP_FAIL | VC_EV_GAVE_UP;
			return 0;
		}
		if (v->byp_step != 0) {
			return byp_next(v, in, o) ? jn_step(v, in, o) : 0;
		}
		if (v->jn_cmd == VC_BC_FORWARD) {
			jn_finish(v, in, o);
			return 0;
		}
		turn_set(v, v->turn_dir, v->turn_target_d10 / 10);
	}
	/* JN_TURN */
	if (v->search_leg != 0) {		/* the line is looked for while spinning */
		if (in->dt_ms > 0) {
			if (in->line_fresh && !in->lost && in->zone != 0) {
				v->found_cnt++;
			} else {
				v->found_cnt = 0;
			}
		}
		if (v->found_cnt >= VC_SEARCH_FOUND_MSGS) {
			v->search_leg = 0;
			jn_finish(v, in, o);
			o->event |= VC_EV_FOUND;
			return 0;
		}
	}
	if (!v->turn_live) {
		int left = v->turn_target_d10 - turned_d10(v);

		if (left < VC_TURN_LEFTOVER_D10) {	/* stopped just short of the end: done */
			return turn_over(v, in, o);
		}
		v->turn_cmd_deg = (left + 5) / 10;
		if (v->turn_cmd_deg > 180) {
			v->turn_cmd_deg = 180;
		}
		v->turn_live = 1;
		v->turn_ms   = 0;
	}
	v->turn_ms += in->elapsed_ms;
	if ((in->motion_done && in->motion_state == VC_MOTION_IDLE && v->turn_ms >= VC_TURN_MIN_MS) ||
	    turned_d10(v) >= v->turn_target_d10 + VC_TURN_OVERSHOOT_D10) {
		return turn_over(v, in, o);
	}
	if (v->turn_ms >= VC_TURN_TIMEOUT_MS) {
		o->event |= VC_EV_TURN_TIMEOUT;
		return turn_over(v, in, o);
	}
	o->mode     = VC_MODE_TURN;
	o->speed    = VC_TURN_SPEED;
	o->turn_deg = v->turn_dir * v->turn_cmd_deg;
	return 1;
}

/* The profile that came with BYPASSING can be used: pass on one side, a plausible shift */
static int byp_profile_ok(const VC_IN *in)
{
	return (in->prof_action == VC_ACT_OBS_RIGHT || in->prof_action == VC_ACT_OBS_LEFT) &&
	       in->prof_shift_cm >= 1u && in->prof_shift_cm <= 100u;
}

static void byp_begin(VC_STATE *v, const VC_IN *in, VC_OUT *o)
{
	int f;

	v->byp_dir  = (in->prof_action == VC_ACT_OBS_RIGHT) ? 1 : -1;
	v->byp_s_mm = (int)in->prof_shift_cm * 10;
	f = (int)in->prof_closest_cm * 10 + VC_BYP_DEPTH_MM + VC_BYP_CAR_MM;
	v->byp_f_mm = (f > VC_BYP_MAX_LEG_MM) ? VC_BYP_MAX_LEG_MM : f;
	if (v->byp_s_mm > VC_BYP_MAX_LEG_MM) {
		v->byp_s_mm = VC_BYP_MAX_LEG_MM;
	}
	v->byp_step = 1;
	turn_set(v, v->byp_dir, VC_BYP_TURN_DEG);		/* away from the line */
	o->event |= VC_EV_BYP_START;
}

static VC_OUT vc_step_inner(VC_STATE *v, const VC_IN *in0)
{
	VC_IN	retry;
	const VC_IN *in = in0;
	int	is_retry = 0;

	VC_OUT	o;
	int	hold;

	o.mode    = VC_MODE_STOP;
	o.speed   = 0;
	o.steer   = 0;
	o.holding = 0;
	o.creep_done = 0;
	o.turn_deg = 0;
	o.reverse_done = 0;
	o.event = 0;
	o.busy = 0;
	o.barcodes = 0;
	o.last_action = 0;

	/* supervisor commands (IF-14). ABORT works at any time and stays. RESUME in READY
	   is START; elsewhere it ends a PAUSE. PAUSE only counts once the run has started. */
	if (in->sup_cmd == VC_SUP_ABORT) {
		v->aborted = 1;
	} else if (in->sup_cmd == VC_SUP_PAUSE) {
		if (v->started) {
			v->paused = 1;
		}
	} else if (in->sup_cmd == VC_SUP_RESUME) {
		if (!v->started) {
			if (in->ready) {
				v->started = 1;
			}
		} else {
			v->paused = 0;
		}
	}
	if (!v->started && in->ready && in->auto_start) {
		v->started = 1;
	}
	bc_take(v, in, &o);
	if (v->byp_step != 0 && in->obs_state != VC_OBS_BYPASSING && in->obs_state != VC_OBS_REACQUIRING) {
		/* Obstacle Detection gave up (fault, timeout) in the middle of the detour: drop it.
		   The car is off the line; line following and the spin search take over. */
		v->byp_step = 0;
		v->jn_state = JN_NONE;
		v->jn_on    = 0;
		v->turn_live = 0;
		v->have_prev = 0;
		v->lost_ms   = 0;
		o.event |= VC_EV_BYP_FAIL;
	}
	if (v->jn_state == JN_TURN) {		/* count the angle turned, also while held */
		v->turn_arc_um += (int32_t)(in->wheel_l - in->wheel_r) * (int32_t)in->elapsed_ms;
	}
	o.mission_state = mission_of(v, in);

	/* Stand still unless the run is on, Obstacle Detection is idle and the Safety
	   Monitor's stop latch is clear. A state that is not idle means an obstacle was
	   seen and is being scanned. */
	hold = (!v->started || v->paused || v->aborted) ||
	       (in->obs_state != VC_OBS_IDLE) || (in->motion_state == VC_STATE_INHIBITED);
	/* phase 7: the bypass and the back-away of a REVERSE_RETRY are driven by this controller,
	   so those Obstacle Detection states do not hold the car (a bypass without a usable
	   profile still does) */
	if (hold && in->phase7 && v->started && !v->paused && !v->aborted &&
	    in->motion_state != VC_STATE_INHIBITED) {
		if ((in->obs_state == VC_OBS_BYPASSING && (v->byp_step != 0 || byp_profile_ok(in))) ||
		    in->obs_state == VC_OBS_REACQUIRING || in->obs_state == VC_OBS_REVERSE_RETRY) {
			hold = 0;
		}
	}
	if (in->phase7 && in->obs_state == VC_OBS_REVERSE_RETRY && !hold) {
		retry = *in0;				/* too tight to pass: back away straight, then scan again */
		retry.approach_mm = -VC_RETRY_BACK_MM;
		in = &retry;
		is_retry = 1;
	}
	if (hold) {
		if (!v->holding) {
			v->holding  = 1;
			if (v->creep_on && !v->creep_done) {
				creep_remember(v, in);
			}
			if (v->jn_state == JN_DRIVE && v->jn_on) {
				int moved = odo_since(in->odo_mm, v->jn_odo0);

				if (moved > 0) {
					v->jn_acc += (unsigned)moved;
				}
			}
			v->jn_on     = 0;		/* drive-on and turn go on with what is left */
			v->turn_live = 0;
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
		o = creep_step(v, in, o);
		if (is_retry) {			/* REVERSE_RETRY: the end of the back-away is REVERSE_DONE */
			o.reverse_done = o.creep_done;
			o.creep_done   = 0;
		}
		return o;
	}
	if (in->phase7 && in->obs_state == VC_OBS_BYPASSING && v->byp_step == 0) {
		byp_begin(v, in, &o);
	}
	v->creep_acc = 0;			/* no order open: nothing travelled yet */
	if (v->creep_on) {			/* the order was taken: line following starts clean */
		v->creep_on   = 0;
		v->creep_done = 0;
		v->have_prev  = 0;
		v->lost_ms    = 0;
	}
	if (jn_step(v, in, &o)) {
		return o;
	}
	if (v->line_gave_up || !in->line_fresh) {
		return o;		/* no line data, or the line is gone: do not drive blind */
	}
	if (!follow_line(v, in, &o)) {
		o.mode = VC_MODE_DRIVE;
	} else {
		o.speed = 0;
		o.steer = 0;
		if (search_begin(v, in, &o)) {		/* spin on the spot to find the line again */
			(void)jn_step(v, in, &o);
		} else {
			v->line_gave_up = 1;
			o.event |= VC_EV_GAVE_UP;
		}
	}
	return o;
}

VC_OUT vc_step(VC_STATE *v, const VC_IN *in)
{
	VC_OUT o = vc_step_inner(v, in);

	o.mission_state = mission_of(v, in);	/* after the step, so a manoeuvre that starts or ends shows at once */
	o.busy        = (v->jn_state != JN_NONE) ? 1 : 0;
	o.barcodes    = v->barcodes;
	o.last_action = v->last_action;
	return o;
}

static unsigned cap_u(unsigned v, unsigned max)
{
	return (v > max) ? max : v;
}

void vc_status_fill(const VC_STATUS_IN *in, VC_STATUS *out)
{
	unsigned t = (unsigned)(in->run_ms / 100u);

	out->mission_time_ds = (in->run_ms / 100u > 65535u) ? 65535u : t;
	out->mission_state   = cap_u(in->mission_state, 14u);
	out->fault_mask      = in->fault_mask & 0xFu;
	out->letter          = (in->letter >= 0 && in->letter <= 25) ? (unsigned)in->letter : VC_NO_LETTER;
	out->last_action     = cap_u(in->last_action, 3u);
	out->barcodes        = cap_u(in->barcodes, 15u);
	out->hump_peak_mm    = (in->hump_mm < 0) ? VC_NO_HUMP : cap_u((unsigned)in->hump_mm, 254u);
	if (in->obst_present) {
		out->b_obstacle    = 1;
		out->obst_dist_cm  = cap_u(in->obst_dist_cm, 510u);	/* never 511 = none */
		out->obst_width_cm = cap_u(in->obst_width_cm, 62u);	/* never 63 = unknown */
		out->obst_side     = (in->obst_side <= 2u) ? in->obst_side : 0u;
	} else {
		out->b_obstacle    = 0;
		out->obst_dist_cm  = VC_NO_OBST_DIST;
		out->obst_width_cm = VC_NO_OBST_WIDTH;
		out->obst_side     = 0;
	}
}
