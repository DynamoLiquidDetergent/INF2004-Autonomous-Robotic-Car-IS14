/*
 * Host-side unit tests for app_program/subsystem5/vc_logic.c
 * Build and run on a PC:   sh testing/subsystem5/host/run.sh
 * Output lines: [HOST] S5-H47 PASS ...
 */
#include <stdio.h>
#include <string.h>
#include "../../../app_program/subsystem5/vc_logic.h"

static int pass_n, fail_n;

#define CHECK(id, cond, desc) do { \
	if (cond) { pass_n++; printf("[HOST] %s PASS  %s\n", id, desc); } \
	else { fail_n++; printf("[HOST] %s FAIL  %s (line %d)\n", id, desc, __LINE__); } \
} while (0)

static VC_STATE v;

static VC_IN line_in(int zone, int lost)
{
	VC_IN in;

	in.line_fresh = 1;
	in.zone = zone;
	in.lost = lost;
	in.dt_ms = 20;
	in.obs_state = 0;
	in.motion_state = 1;
	in.approach_mm = 0;
	in.odo_mm = 0;
	in.sup_cmd = VC_SUP_NONE;
	in.ready = 1;
	in.auto_start = 1;		/* the older tests run without waiting for START */
	in.fault_mask = 0;
	in.junction = 0;
	in.obst_present = 0;
	in.phase7 = 0;
	in.prof_action = 0;
	in.prof_closest_cm = 0;
	in.prof_shift_cm = 0;
	in.bc_cmd = VC_BC_NONE;
	in.bc_n = 0;
	in.wheel_l = 0;
	in.wheel_r = 0;
	in.motion_done = 0;
	in.elapsed_ms = 20;
	return in;
}

/* angle turned so far (tenths of a degree), same formula as the controller */
static int turned_est_d10(const VC_STATE *st)
{
	return (int)(((long long)st->turn_arc_um * st->turn_dir * 573) / (120LL * 1000));
}


/* ---- a small world for the bypass tests: the car drives, turns finish 200 ms after the
   TURN command, the line is "seen" again once the car has travelled find_at_mm in the
   return leg (never if find_at_mm < 0) ---- */
typedef struct {
	unsigned odo;
	int      acc;		/* mm * 1000, odometry between whole mm */
	int      obs;
	int      find_at;
	int      find_mm;	/* travelled in the return leg */
	unsigned events;
	int      steps;
	int      n_turn, n_drive;
	int      order[40];	/* sequence of primitives: +-deg for turns, mm/s for drives */
	int      n_order;
	int      last_kind;	/* 0 none, 1 turn, 2 drive */
	int      drive_total;	/* mm driven forward */
} WORLD;

static VC_OUT world_step(WORLD *w, int action, int shift, int closest, unsigned sup)
{
	VC_IN b = line_in(4, 1);
	VC_OUT o;
	int line_seen = 0;

	b.phase7 = 1;
	b.obs_state = (unsigned)w->obs;
	b.prof_action = (unsigned)action;
	b.prof_shift_cm = (unsigned)shift;
	b.prof_closest_cm = (unsigned)closest;
	b.odo_mm = w->odo;
	b.sup_cmd = sup;
	if (v.byp_step >= 7) {
		line_seen = 1;
	} else if (v.byp_step == 6 && w->find_at >= 0 && w->find_mm >= w->find_at) {
		line_seen = 1;
	}
	b.lost = line_seen ? 0 : 1;
	if (v.jn_state == 2 && v.turn_live && v.turn_ms >= 200) {
		b.motion_state = 0; b.motion_done = 1;
	} else if (v.jn_state == 2) {
		b.motion_state = 2;
	}
	o = vc_step(&v, &b);
	w->events |= o.event;
	w->steps++;
	if (o.mode == VC_MODE_TURN) {
		if (w->last_kind != 1 || w->order[w->n_order - 1] != o.turn_deg) {
			if (w->n_order < 40) { w->order[w->n_order++] = o.turn_deg; }
		}
		w->last_kind = 1;
	} else if (o.mode == VC_MODE_DRIVE && o.busy) {
		if (w->last_kind != 2 || w->order[w->n_order - 1] != o.speed) {
			if (w->n_order < 40) { w->order[w->n_order++] = o.speed; }
		}
		w->last_kind = 2;
		w->acc += o.speed * 20;
		while (w->acc >= 1000) { w->acc -= 1000; w->odo++; w->drive_total++; if (v.byp_step == 6) w->find_mm++; }
	}
	if (o.event & VC_EV_BYP_PASSED) { w->obs = 5; }	/* the task moves to REACQUIRING */
	if (o.event & (VC_EV_BYP_DONE | VC_EV_BYP_FAIL)) { w->obs = 0; }
	return o;
}

int main(void)
{
	VC_IN in;
	VC_OUT o;
	int i, ok;
	unsigned e0;

	/* H47: line centred: straight at full speed */
	vc_init(&v, 3);
	in = line_in(4, 0);
	o = vc_step(&v, &in);
	CHECK("S5-H47", o.mode == VC_MODE_DRIVE && o.speed == 100 && o.steer == 0 && o.epoch == 3,
	      "line centred: DRIVE 100 mm/s, steer 0, epoch unchanged");

	/* H48: one zone off: 80 mm/s and 30 mm/s steering, as measured on the car; mirror image */
	vc_init(&v, 0);
	in = line_in(5, 0);
	o = vc_step(&v, &in);
	CHECK("S5-H48a", o.mode == VC_MODE_DRIVE && o.speed == 80 && o.steer == 30,
	      "zone 5: 80 mm/s steer +30 (same as the logged car run)");
	vc_init(&v, 0);
	in = line_in(3, 0);
	o = vc_step(&v, &in);
	CHECK("S5-H48b", o.speed == 80 && o.steer == -30, "zone 3: 80 mm/s steer -30");

	/* H49: the D term acts on the change since the last cycle */
	vc_init(&v, 0);
	in = line_in(4, 0);
	o = vc_step(&v, &in);
	in = line_in(5, 0);
	o = vc_step(&v, &in);
	CHECK("S5-H49", o.steer == 50, "zone 4 then 5: P 30 + D 20 = steer 50");

	/* H50: steering limit and minimum speed */
	vc_init(&v, 0);
	in = line_in(4, 0);
	o = vc_step(&v, &in);
	in = line_in(7, 0);
	o = vc_step(&v, &in);
	CHECK("S5-H50", o.steer == 120 && o.speed == 40, "zone 7 after 4: steer limited to 120, speed 40 (minimum)");

	/* H51: line lost: slow search towards the side it was last seen on */
	vc_init(&v, 0);
	in = line_in(5, 1);
	o = vc_step(&v, &in);
	CHECK("S5-H51a", o.mode == VC_MODE_DRIVE && o.speed == 40 && o.steer == 100, "lost, last zone 5: 40 mm/s, turn +100");
	vc_init(&v, 0);
	in = line_in(3, 1);
	o = vc_step(&v, &in);
	CHECK("S5-H51b", o.speed == 40 && o.steer == -100, "lost, last zone 3: 40 mm/s, turn -100");

	/* H52: lost for 1500 ms with an obstacle present (no spinning next to it): stop for good,
	   even if the line comes back. Without an obstacle the car spins to look for the line: H101 */
	vc_init(&v, 0);
	ok = 1;
	for (i = 0; i < 74; i++) {
		in = line_in(5, 1);
		in.obst_present = 1;
		o = vc_step(&v, &in);
		if (o.mode != VC_MODE_DRIVE) ok = 0;
	}
	CHECK("S5-H52a", ok, "lost for 1480 ms: still searching");
	in = line_in(5, 1);
	in.obst_present = 1;
	o = vc_step(&v, &in);
	CHECK("S5-H52b", o.mode == VC_MODE_STOP && v.line_gave_up, "lost for 1500 ms: STOP");
	in = line_in(4, 0);
	o = vc_step(&v, &in);
	CHECK("S5-H52c", o.mode == VC_MODE_STOP, "the line comes back: still stopped (RESET needed)");

	/* H53: no line message: do not drive blind; the epoch is not touched */
	vc_init(&v, 5);
	in = line_in(4, 0);
	in.line_fresh = 0;
	o = vc_step(&v, &in);
	CHECK("S5-H53", o.mode == VC_MODE_STOP && o.epoch == 5 && !o.holding, "line sensor silent: STOP, same epoch");

	/* H54: an obstacle being scanned: stand still, epoch +1 once */
	vc_init(&v, 5);
	e0 = v.epoch;
	ok = 1;
	for (i = 1; i <= 3; i++) {
		in = line_in(4, 0);
		in.obs_state = (unsigned)i;
		o = vc_step(&v, &in);
		if (o.mode != VC_MODE_STOP || !o.holding || o.epoch != e0 + 1u) ok = 0;
	}
	CHECK("S5-H54", ok, "obstacle states 1, 2, 3: STOP, held, epoch +1 once (same epoch all through the hold)");

	/* H55: obstacle gone, latch clear: drive on with the new epoch */
	in = line_in(4, 0);
	o = vc_step(&v, &in);
	CHECK("S5-H55", o.mode == VC_MODE_DRIVE && !o.holding && o.epoch == e0 + 1u,
	      "obstacle idle again: DRIVE with the newer epoch");

	/* H56: Motion Control still reports the stop latch: stay stopped until it clears */
	vc_init(&v, 5);
	in = line_in(4, 0);
	in.motion_state = VC_STATE_INHIBITED;
	o = vc_step(&v, &in);
	CHECK("S5-H56a", o.mode == VC_MODE_STOP && o.holding && o.epoch == 6, "stop latch set, obstacle idle: held, epoch +1");
	in.motion_state = 0;
	o = vc_step(&v, &in);
	CHECK("S5-H56b", o.mode == VC_MODE_DRIVE && o.epoch == 6, "latch released: drive on, epoch 6");

	/* H57: the epoch wraps 15 -> 0 */
	vc_init(&v, 15);
	in = line_in(4, 0);
	in.obs_state = 1;
	o = vc_step(&v, &in);
	CHECK("S5-H57", o.epoch == 0, "epoch 15 + 1 wraps to 0 (4 bits)");

	/* H58: every new hold raises the epoch again */
	vc_init(&v, 2);
	in = line_in(4, 0);
	in.obs_state = 1;
	o = vc_step(&v, &in);
	in.obs_state = 0;
	o = vc_step(&v, &in);
	in.obs_state = 2;
	o = vc_step(&v, &in);
	CHECK("S5-H58", o.epoch == 4, "two separate holds: epoch 2 -> 3 -> 4");

	/* H59: after a hold the old error is forgotten (no D kick) */
	vc_init(&v, 0);
	in = line_in(4, 0);
	o = vc_step(&v, &in);
	in.obs_state = 1;
	o = vc_step(&v, &in);
	in = line_in(5, 0);
	o = vc_step(&v, &in);
	CHECK("S5-H59", o.mode == VC_MODE_DRIVE && o.steer == 30, "first cycle after a hold: P only (steer 30, not 50)");

	/* H60: Obstacle Detection FAULT or any non-idle state also holds */
	ok = 1;
	{
		unsigned s;

		for (s = 1; s <= 7; s++) {
			vc_init(&v, 0);
			in = line_in(4, 0);
			in.obs_state = s;
			o = vc_step(&v, &in);
			if (o.mode != VC_MODE_STOP || !o.holding) ok = 0;
		}
	}
	CHECK("S5-H60", ok, "every Obstacle Detection state except idle holds the car");


	/* H67: creep forward: follows the line at 40 mm/s, finishes after the ordered distance */
	vc_init(&v, 4);
	in = line_in(4, 0);
	in.approach_mm = 320;
	in.odo_mm = 1000;
	o = vc_step(&v, &in);
	CHECK("S5-H67a", o.mode == VC_MODE_DRIVE && o.speed == 40 && o.steer == 0 && o.epoch == 4 &&
	      !o.holding && !o.creep_done, "creep order +320 mm, line centred: DRIVE 40 mm/s, steer 0, not done");
	in.odo_mm = 1000 + 319;
	o = vc_step(&v, &in);
	CHECK("S5-H67b", o.mode == VC_MODE_DRIVE && !o.creep_done, "319 mm travelled: still going");
	in.odo_mm = 1000 + 320;
	o = vc_step(&v, &in);
	CHECK("S5-H67c", o.mode == VC_MODE_STOP && o.creep_done && o.epoch == 4, "320 mm travelled: STOP, creep_done");
	in.odo_mm = 1000 + 100;		/* odometry wobble afterwards does not restart it */
	o = vc_step(&v, &in);
	CHECK("S5-H67d", o.mode == VC_MODE_STOP && o.creep_done, "done stays done while the order is open");

	/* H68: the creep follows the line: steering is the line steering scaled to 40 mm/s */
	vc_init(&v, 4);
	in = line_in(5, 0);
	in.approach_mm = 300;
	in.odo_mm = 0;
	o = vc_step(&v, &in);
	CHECK("S5-H68a", o.mode == VC_MODE_DRIVE && o.speed == 40 && o.steer == 12,
	      "line one zone to the left: steer +12 (30 x 40 / 100)");
	in = line_in(3, 0);
	in.approach_mm = 300;
	in.odo_mm = 10;
	o = vc_step(&v, &in);
	CHECK("S5-H68b", o.mode == VC_MODE_DRIVE && o.speed == 40 && o.steer < 0,
	      "line one zone to the right: steers the other way");
	in.dt_ms = 0;			/* no new line message: the last answer is kept */
	{
		VC_OUT o2 = vc_step(&v, &in);

		CHECK("S5-H68c", o2.mode == VC_MODE_DRIVE && o2.steer == o.steer && o2.speed == o.speed,
		      "no new line message: same speed and steer as before");
	}

	/* H69: back away: straight at -40 mm/s, done after the distance, also across the odometry wrap */
	vc_init(&v, 4);
	in = line_in(4, 0);
	in.approach_mm = -60;
	in.odo_mm = 5;
	o = vc_step(&v, &in);
	CHECK("S5-H69a", o.mode == VC_MODE_DRIVE && o.speed == -40 && o.steer == VC_REVERSE_STEER && !o.creep_done,
	      "back-away order -60 mm: DRIVE -40 mm/s, steer 1 (not 0: keeps the S2 straight-line hold off)");
	in.odo_mm = (65536u + 5u - 59u) & 0xFFFFu;
	o = vc_step(&v, &in);
	CHECK("S5-H69b", o.mode == VC_MODE_DRIVE && !o.creep_done, "59 mm backwards (across the 16 bit wrap): still going");
	in.odo_mm = (65536u + 5u - 60u) & 0xFFFFu;
	o = vc_step(&v, &in);
	CHECK("S5-H69c", o.mode == VC_MODE_STOP && o.creep_done, "60 mm backwards: STOP, creep_done");

	/* H70: holding comes first, and the distance counts from when the car really starts */
	vc_init(&v, 4);
	in = line_in(4, 0);
	in.approach_mm = 320;
	in.obs_state = 1;
	in.odo_mm = 2000;
	o = vc_step(&v, &in);
	CHECK("S5-H70a", o.mode == VC_MODE_STOP && o.holding && o.epoch == 5 && !o.creep_done,
	      "creep order but scanning: held, epoch +1, nothing counted");
	in.obs_state = 0;
	in.motion_state = VC_STATE_INHIBITED;
	in.odo_mm = 2010;
	o = vc_step(&v, &in);
	CHECK("S5-H70b", o.mode == VC_MODE_STOP && o.holding && o.epoch == 5, "stop latch set: still held, same epoch");
	in.motion_state = 0;
	in.odo_mm = 2015;
	o = vc_step(&v, &in);
	CHECK("S5-H70c", o.mode == VC_MODE_DRIVE && o.epoch == 5 && !o.holding, "latch released: the creep starts, newer epoch");
	in.odo_mm = 2015 + 319;
	o = vc_step(&v, &in);
	CHECK("S5-H70d", o.mode == VC_MODE_DRIVE && !o.creep_done, "counted from the start of the creep (2015), not from 2000");
	in.odo_mm = 2015 + 320;
	o = vc_step(&v, &in);
	CHECK("S5-H70e", o.creep_done, "320 mm after the start: done");

	/* H71: the line is needed to creep forward */
	vc_init(&v, 2);
	in = line_in(4, 0);
	in.line_fresh = 0;
	in.approach_mm = 100;
	in.odo_mm = 0;
	o = vc_step(&v, &in);
	CHECK("S5-H71a", o.mode == VC_MODE_STOP && !o.creep_done, "line sensor silent: no creep blind, not done");
	vc_init(&v, 2);
	in = line_in(4, 1);		/* lost */
	in.approach_mm = 100;
	in.odo_mm = 0;
	ok = 1;
	for (i = 0; i < 74; i++) {
		o = vc_step(&v, &in);
		if (o.mode != VC_MODE_DRIVE || o.speed != 40) ok = 0;
	}
	CHECK("S5-H71b", ok, "line lost for 1480 ms: still searching at 40 mm/s");
	o = vc_step(&v, &in);
	o = vc_step(&v, &in);
	CHECK("S5-H71c", o.mode == VC_MODE_STOP && o.creep_done, "line lost for 1500 ms: STOP and creep_done (rescan)");

	/* H72: a wheel that rocks back a few mm at the start is not a finished creep */
	vc_init(&v, 2);
	in = line_in(4, 0);
	in.approach_mm = 100;
	in.odo_mm = 3;
	o = vc_step(&v, &in);
	in.odo_mm = 0;			/* 3 mm backwards */
	o = vc_step(&v, &in);
	in.odo_mm = 65534u;		/* 5 mm backwards, across the wrap */
	o = vc_step(&v, &in);
	CHECK("S5-H72", o.mode == VC_MODE_DRIVE && !o.creep_done, "odometry 5 mm backwards at the start: not done");

	/* H73: after the creep the line controller starts clean (no derivative kick) */
	vc_init(&v, 2);
	in = line_in(4, 0);
	o = vc_step(&v, &in);
	in = line_in(4, 0);
	in.approach_mm = 100;
	in.odo_mm = 0;
	o = vc_step(&v, &in);
	in = line_in(5, 0);
	o = vc_step(&v, &in);
	CHECK("S5-H73", o.mode == VC_MODE_DRIVE && o.steer == 30 && o.speed == 80,
	      "line again after a creep: back to line speed, P only (steer 30, not 50)");

	/* H81: power-up and START. INIT while the power-up work runs, READY afterwards,
	   RUNNING only after RESUME (auto start off) */
	vc_init(&v, 2);
	in = line_in(4, 0);
	in.auto_start = 0;
	in.ready = 0;
	o = vc_step(&v, &in);
	CHECK("S5-H81a", o.mission_state == VC_MS_INIT && o.mode == VC_MODE_STOP && o.holding,
	      "power-up work not finished: INIT, standing still");
	in.sup_cmd = VC_SUP_RESUME;
	o = vc_step(&v, &in);
	CHECK("S5-H81b", o.mission_state == VC_MS_INIT && o.mode == VC_MODE_STOP,
	      "RESUME before the power-up work is done does not start the run");
	in.sup_cmd = VC_SUP_NONE;
	in.ready = 1;
	o = vc_step(&v, &in);
	CHECK("S5-H81c", o.mission_state == VC_MS_READY && o.mode == VC_MODE_STOP, "ready: READY, still standing");
	e0 = o.epoch;
	in.sup_cmd = VC_SUP_RESUME;
	o = vc_step(&v, &in);
	CHECK("S5-H81d", o.mission_state == VC_MS_RUNNING && o.mode == VC_MODE_DRIVE && o.epoch == e0,
	      "RESUME in READY is START: RUNNING and driving, same epoch (nothing was latched)");

	/* H82: auto start (bench) */
	vc_init(&v, 2);
	in = line_in(4, 0);
	in.ready = 0;
	o = vc_step(&v, &in);
	in.ready = 1;
	o = vc_step(&v, &in);
	CHECK("S5-H82", o.mission_state == VC_MS_RUNNING && o.mode == VC_MODE_DRIVE,
	      "auto start on: RUNNING as soon as ready, no command needed");

	/* H83: PAUSE and RESUME */
	vc_init(&v, 2);
	in = line_in(4, 0);
	o = vc_step(&v, &in);
	e0 = o.epoch;
	in.sup_cmd = VC_SUP_PAUSE;
	o = vc_step(&v, &in);
	CHECK("S5-H83a", o.mission_state == VC_MS_PAUSED && o.mode == VC_MODE_STOP && o.holding &&
	      o.epoch == ((e0 + 1u) & 0xFu), "PAUSE: PAUSED, STOP, epoch plus 1");
	in.sup_cmd = VC_SUP_NONE;
	o = vc_step(&v, &in);
	CHECK("S5-H83b", o.mission_state == VC_MS_PAUSED && o.mode == VC_MODE_STOP && o.epoch == ((e0 + 1u) & 0xFu),
	      "paused stays paused, epoch unchanged");
	in.sup_cmd = VC_SUP_RESUME;
	o = vc_step(&v, &in);
	CHECK("S5-H83c", o.mission_state == VC_MS_RUNNING && o.mode == VC_MODE_DRIVE && o.epoch == ((e0 + 1u) & 0xFu),
	      "RESUME: RUNNING and driving with the newer epoch");
	in.sup_cmd = VC_SUP_RESUME;
	o = vc_step(&v, &in);
	CHECK("S5-H83d", o.mission_state == VC_MS_RUNNING && o.epoch == ((e0 + 1u) & 0xFu),
	      "RESUME while running changes nothing");
	vc_init(&v, 2);
	in = line_in(4, 0);
	in.auto_start = 0;
	o = vc_step(&v, &in);
	in.sup_cmd = VC_SUP_PAUSE;
	o = vc_step(&v, &in);
	CHECK("S5-H83e", o.mission_state == VC_MS_READY, "PAUSE in READY is ignored (the run has not started)");

	/* H84: ABORT stops for good */
	vc_init(&v, 2);
	in = line_in(4, 0);
	o = vc_step(&v, &in);
	in.sup_cmd = VC_SUP_ABORT;
	o = vc_step(&v, &in);
	CHECK("S5-H84a", o.mission_state == VC_MS_ABORTED && o.mode == VC_MODE_STOP && o.holding, "ABORT: ABORTED and STOP");
	in.sup_cmd = VC_SUP_RESUME;
	o = vc_step(&v, &in);
	in.sup_cmd = VC_SUP_NONE;
	o = vc_step(&v, &in);
	CHECK("S5-H84b", o.mission_state == VC_MS_ABORTED && o.mode == VC_MODE_STOP,
	      "RESUME after ABORT does nothing: stopped until reset");
	vc_init(&v, 2);
	in = line_in(4, 0);
	in.auto_start = 0;
	in.sup_cmd = VC_SUP_ABORT;
	o = vc_step(&v, &in);
	CHECK("S5-H84c", o.mission_state == VC_MS_ABORTED, "ABORT in READY also works");

	/* H85: what is reported while an obstacle or a fault is handled */
	vc_init(&v, 2);
	in = line_in(4, 0);
	o = vc_step(&v, &in);
	in.obs_state = 2;
	o = vc_step(&v, &in);
	CHECK("S5-H85a", o.mission_state == VC_MS_AVOIDING && o.holding, "obstacle scan running: AVOIDING");
	in.obs_state = 0;
	in.approach_mm = 100;
	o = vc_step(&v, &in);
	CHECK("S5-H85b", o.mission_state == VC_MS_AVOIDING, "creep order open: AVOIDING");
	in.approach_mm = 0;
	in.obs_state = VC_OBS_FAULT;
	o = vc_step(&v, &in);
	CHECK("S5-H85c", o.mission_state == VC_MS_FAULT && o.mode == VC_MODE_STOP, "obstacle task in FAULT: FAULT");
	in.obs_state = 0;
	in.fault_mask = 0x2;
	in.motion_state = VC_STATE_INHIBITED;
	o = vc_step(&v, &in);
	CHECK("S5-H85d", o.mission_state == VC_MS_FAULT && o.mode == VC_MODE_STOP, "Safety Monitor handling a fault: FAULT");
	in.fault_mask = 0;
	in.motion_state = 1;
	o = vc_step(&v, &in);
	CHECK("S5-H85e", o.mission_state == VC_MS_RUNNING && o.mode == VC_MODE_DRIVE, "fault cleared: RUNNING again");
	in.sup_cmd = VC_SUP_PAUSE;
	in.fault_mask = 0x8;
	o = vc_step(&v, &in);
	CHECK("S5-H85f", o.mission_state == VC_MS_FAULT, "a fault is reported ahead of PAUSED");
	in.fault_mask = 0;
	in.sup_cmd = VC_SUP_NONE;
	o = vc_step(&v, &in);
	CHECK("S5-H85g", o.mission_state == VC_MS_PAUSED && o.mode == VC_MODE_STOP, "after the fault the PAUSE still holds");

	/* H86: a pause in the middle of a creep keeps what was already travelled */
	vc_init(&v, 2);
	in = line_in(4, 0);
	in.approach_mm = 320;
	in.odo_mm = 1000;
	o = vc_step(&v, &in);
	in.odo_mm = 1100;			/* 100 mm done */
	o = vc_step(&v, &in);
	in.sup_cmd = VC_SUP_PAUSE;
	o = vc_step(&v, &in);
	CHECK("S5-H86a", o.mission_state == VC_MS_PAUSED && o.mode == VC_MODE_STOP, "pause during a creep: stopped");
	in.sup_cmd = VC_SUP_RESUME;
	in.odo_mm = 1103;			/* drifted 3 mm while stopping */
	o = vc_step(&v, &in);
	in.sup_cmd = VC_SUP_NONE;
	in.odo_mm = 1103 + 219;			/* 100 + 219 = 319 of 320 */
	o = vc_step(&v, &in);
	CHECK("S5-H86b", o.mode == VC_MODE_DRIVE && !o.creep_done, "resumed: 219 mm more is still short of the order");
	in.odo_mm = 1103 + 221;
	o = vc_step(&v, &in);
	CHECK("S5-H86c", o.mode == VC_MODE_STOP && o.creep_done, "only the rest (220 mm) was driven, then done");

	/* H87: PAUSE does not cancel the obstacle hold; both must be clear to move */
	vc_init(&v, 2);
	in = line_in(4, 0);
	o = vc_step(&v, &in);
	in.sup_cmd = VC_SUP_PAUSE;
	in.obs_state = 1;
	o = vc_step(&v, &in);
	in.sup_cmd = VC_SUP_RESUME;
	o = vc_step(&v, &in);
	CHECK("S5-H87a", o.mode == VC_MODE_STOP && o.holding && o.mission_state == VC_MS_AVOIDING,
	      "RESUME while the scan is still running: stays held, AVOIDING");
	in.sup_cmd = VC_SUP_NONE;
	in.obs_state = 0;
	o = vc_step(&v, &in);
	CHECK("S5-H87b", o.mode == VC_MODE_DRIVE && o.mission_state == VC_MS_RUNNING, "scan over: driving");

	/* H88: the IF-13 status, capped the way subsystem 1 expects */
	{
		VC_STATUS_IN si;
		VC_STATUS vs;

		si.obst_present = 1; si.obst_dist_cm = 600; si.obst_width_cm = 70; si.obst_side = 1;
		si.mission_state = 20; si.fault_mask = 0x1F; si.run_ms = 7000000u; si.letter = 40;
		si.last_action = 9; si.barcodes = 20; si.hump_mm = 300;
		vc_status_fill(&si, &vs);
		CHECK("S5-H88a", vs.obst_dist_cm == 510 && vs.obst_width_cm == 62 && vs.b_obstacle == 1 && vs.obst_side == 1,
		      "obstacle present: distance 600 -> 510 (never 511 'none'), width 70 -> 62 (never 63 'unknown')");
		CHECK("S5-H88b", vs.mission_state == 14 && vs.fault_mask == 0xF && vs.letter == VC_NO_LETTER &&
		      vs.last_action == 3 && vs.barcodes == 15 && vs.hump_peak_mm == 254 && vs.mission_time_ds == 65535,
		      "state 20 -> 14, mask 0x1F -> 0xF, letter 40 -> none, action 9 -> 3, barcodes 20 -> 15, hump 300 -> 254, time capped");
		si.obst_present = 0; si.obst_dist_cm = 30; si.obst_width_cm = 10; si.obst_side = 2;
		si.mission_state = VC_MS_RUNNING; si.fault_mask = 0; si.run_ms = 12345; si.letter = 3;
		si.last_action = 2; si.barcodes = 4; si.hump_mm = -1;
		vc_status_fill(&si, &vs);
		CHECK("S5-H88c", vs.b_obstacle == 0 && vs.obst_dist_cm == VC_NO_OBST_DIST && vs.obst_width_cm == VC_NO_OBST_WIDTH &&
		      vs.obst_side == 0, "no obstacle: distance 511, width 63, side 0 whatever the old values were");
		CHECK("S5-H88d", vs.mission_state == VC_MS_RUNNING && vs.mission_time_ds == 123 && vs.letter == 3 &&
		      vs.last_action == 2 && vs.barcodes == 4 && vs.hump_peak_mm == VC_NO_HUMP,
		      "values in range pass through: time 12345 ms = 123 ds, hump -1 = none (255)");
	}


	/* H90 to H97: barcode commands (IF-07) and junction manoeuvres */
	{
		VC_IN b;
		unsigned odo = 0;
		int k, turning_seen, drive_mm;

		/* H90: a junction flag without a waiting barcode command is ignored (S3 raises it on
		   ordinary left corrections), however long it stays up */
		vc_init(&v, 0);
		ok = 1;
		for (k = 0; k < 20; k++) {
			b = line_in(5, 0);
			b.junction = 1;
			b.odo_mm = (unsigned)(k * 2);
			o = vc_step(&v, &b);
			if (o.mode != VC_MODE_DRIVE || o.busy || o.mission_state != VC_MS_RUNNING) {
				ok = 0;
			}
		}
		CHECK("S5-H90", ok && v.jn_state == 0, "junction flag with no barcode command: line following goes on, no manoeuvre");

		/* H91: the command is kept; one short flag (fewer than 4 messages) does not start it */
		vc_init(&v, 0);
		b = line_in(4, 0);
		b.bc_cmd = VC_BC_RIGHT; b.bc_n = 1;
		o = vc_step(&v, &b);
		CHECK("S5-H91a", o.barcodes == 1 && (o.event & VC_EV_BC_TAKEN) && !o.busy,
		      "barcode command taken and counted, nothing happens yet");
		for (k = 0; k < 3; k++) {
			b = line_in(5, 0);
			b.junction = 1;
			o = vc_step(&v, &b);
		}
		b = line_in(4, 0);
		o = vc_step(&v, &b);
		CHECK("S5-H91b", !o.busy && o.mode == VC_MODE_DRIVE && v.bc_pending == VC_BC_RIGHT,
		      "flag for only 3 line messages: no manoeuvre, command still waiting");

		/* H92: RIGHT. Flag for 4 messages starts it: drive on 55 mm straight (steer 0, 60 mm/s),
		   then TURN +90 at 100 mm/s, TURNING all the time; when S2 reports done, back to the line */
		vc_init(&v, 0);
		b = line_in(4, 0);
		b.bc_cmd = VC_BC_RIGHT; b.bc_n = 1;
		o = vc_step(&v, &b);
		odo = 1000;
		for (k = 0; k < 4; k++) {
			b = line_in(5, 0);
			b.junction = 1;
			b.odo_mm = odo;
			o = vc_step(&v, &b);
		}
		CHECK("S5-H92a", (o.event & VC_EV_JN_START) && o.busy && o.mode == VC_MODE_DRIVE &&
		      o.speed == VC_JN_SPEED && o.steer == 0 && o.mission_state == VC_MS_TURNING &&
		      o.last_action == VC_ACT_RIGHT,
		      "4th message with the flag: manoeuvre starts, driving on straight (steer 0, 60 mm/s), TURNING");
		b = line_in(4, 0);
		b.odo_mm = odo + 30;
		o = vc_step(&v, &b);
		CHECK("S5-H92b", o.mode == VC_MODE_DRIVE && o.steer == 0 && o.mission_state == VC_MS_TURNING,
		      "30 mm of 55 mm driven: still driving on, mission state TURNING");
		b = line_in(4, 0);
		b.odo_mm = odo + 56;
		o = vc_step(&v, &b);
		CHECK("S5-H92c", o.mode == VC_MODE_TURN && o.turn_deg == 90 && o.speed == VC_TURN_SPEED && o.busy,
		      "55 mm reached: TURN +90 degrees (clockwise) at 100 mm/s");
		/* wheels spin: left +100, right -100 for 20 ms steps. 120 mm track: 90 degrees = 188.5 mm differential */
		turning_seen = 1;
		for (k = 0; k < 40; k++) {
			b = line_in(4, 0);
			b.odo_mm = odo + 56;
			b.wheel_l = 100; b.wheel_r = -100;
			b.motion_state = 2;
			o = vc_step(&v, &b);
			if (o.mode != VC_MODE_TURN) {
				turning_seen = 0;
			}
			if (turned_est_d10(&v) >= 880) {
				break;
			}
		}
		CHECK("S5-H92d", turning_seen && o.mode == VC_MODE_TURN, "turning on the spot while S2 has not said done");
		b = line_in(4, 0);
		b.odo_mm = odo + 56; b.motion_state = 0; b.motion_done = 1;
		o = vc_step(&v, &b);
		CHECK("S5-H92e", (o.event & VC_EV_JN_DONE) && !o.busy && o.mode == VC_MODE_DRIVE &&
		      o.mission_state == VC_MS_RUNNING,
		      "S2 done: manoeuvre over, line following again, RUNNING");

		/* H93: LEFT turns anticlockwise: negative degrees */
		vc_init(&v, 0);
		b = line_in(4, 0); b.bc_cmd = VC_BC_LEFT; b.bc_n = 1; o = vc_step(&v, &b);
		for (k = 0; k < 4; k++) { b = line_in(5, 0); b.junction = 1; o = vc_step(&v, &b); }
		b = line_in(4, 0); b.odo_mm = 60; o = vc_step(&v, &b);
		CHECK("S5-H93", o.mode == VC_MODE_TURN && o.turn_deg == -90 && v.last_action == VC_ACT_LEFT,
		      "LEFT: TURN -90, last action LEFT");

		/* H94: U-TURN: turns at once (no drive-on), 180 degrees */
		vc_init(&v, 0);
		b = line_in(4, 0); b.bc_cmd = VC_BC_UTURN; b.bc_n = 1; o = vc_step(&v, &b);
		for (k = 0; k < 4; k++) { b = line_in(5, 0); b.junction = 1; o = vc_step(&v, &b); }
		CHECK("S5-H94", o.mode == VC_MODE_TURN && o.turn_deg == 180 && v.last_action == VC_ACT_UTURN &&
		      o.mission_state == VC_MS_TURNING, "U-TURN: TURN 180 straight away, TURNING");

		/* H95: FORWARD: straight over the junction (steer 0, no sensor steering) for 80 mm, no TURN,
		   RUNNING state, then back to the line */
		vc_init(&v, 0);
		b = line_in(4, 0); b.bc_cmd = VC_BC_FORWARD; b.bc_n = 1; b.odo_mm = 500; o = vc_step(&v, &b);
		for (k = 0; k < 4; k++) { b = line_in(6, 0); b.junction = 1; b.odo_mm = 500; o = vc_step(&v, &b); }
		CHECK("S5-H95a", o.mode == VC_MODE_DRIVE && o.steer == 0 && o.busy && o.mission_state == VC_MS_RUNNING &&
		      v.last_action == VC_ACT_FORWARD, "FORWARD: straight on, zone 6 pull ignored, RUNNING");
		b = line_in(6, 0); b.junction = 1; b.odo_mm = 500 + 60; o = vc_step(&v, &b);
		CHECK("S5-H95b", o.steer == 0 && o.busy, "60 mm of 80: still straight");
		b = line_in(4, 0); b.junction = 1; b.odo_mm = 500 + 82; o = vc_step(&v, &b);
		CHECK("S5-H95c", !o.busy && (o.event & VC_EV_JN_DONE) && o.mode == VC_MODE_DRIVE,
		      "80 mm reached: done, line following");

		/* H96: no second trigger on the same junction: flag still up after the manoeuvre, then low,
		   then the next junction needs a new command and 100 mm */
		vc_init(&v, 0);
		b = line_in(4, 0); b.bc_cmd = VC_BC_FORWARD; b.bc_n = 1; b.odo_mm = 0; o = vc_step(&v, &b);
		for (k = 0; k < 4; k++) { b = line_in(5, 0); b.junction = 1; b.odo_mm = 0; o = vc_step(&v, &b); }
		b = line_in(4, 0); b.junction = 1; b.odo_mm = 85; o = vc_step(&v, &b);
		b = line_in(4, 0); b.bc_cmd = VC_BC_LEFT; b.bc_n = 1; b.junction = 1; b.odo_mm = 90; o = vc_step(&v, &b);
		for (k = 0; k < 6; k++) { b = line_in(5, 0); b.junction = 1; b.odo_mm = 95; o = vc_step(&v, &b); }
		CHECK("S5-H96a", !o.busy && v.bc_pending == VC_BC_LEFT, "new command while the flag is still up from the last junction: not started");
		for (k = 0; k < 3; k++) { b = line_in(4, 0); b.junction = 0; b.odo_mm = 120; o = vc_step(&v, &b); }
		for (k = 0; k < 6; k++) { b = line_in(5, 0); b.junction = 1; b.odo_mm = 120; o = vc_step(&v, &b); }
		CHECK("S5-H96b", !o.busy, "flag low but only 35 mm since the last manoeuvre: not re-armed yet");
		for (k = 0; k < 3; k++) { b = line_in(4, 0); b.junction = 0; b.odo_mm = 200; o = vc_step(&v, &b); }
		for (k = 0; k < 4; k++) { b = line_in(5, 0); b.junction = 1; b.odo_mm = 200; o = vc_step(&v, &b); }
		CHECK("S5-H96c", o.busy && v.jn_cmd == VC_BC_LEFT, "flag low, 100 mm and more since: the next junction starts the waiting LEFT");

		/* H97: a command with no junction for 1500 mm is dropped; a newer command replaces an older one */
		vc_init(&v, 0);
		b = line_in(4, 0); b.bc_cmd = VC_BC_LEFT; b.bc_n = 1; b.odo_mm = 0; o = vc_step(&v, &b);
		b = line_in(4, 0); b.bc_cmd = VC_BC_RIGHT; b.bc_n = 1; b.odo_mm = 100; o = vc_step(&v, &b);
		CHECK("S5-H97a", v.bc_pending == VC_BC_RIGHT && v.barcodes == 2, "newer command replaces the older one; both counted");
		b = line_in(4, 0); b.odo_mm = 100 + 1400; o = vc_step(&v, &b);
		CHECK("S5-H97b", v.bc_pending == VC_BC_RIGHT && !(o.event & VC_EV_BC_EXPIRED), "1400 mm on: still waiting");
		b = line_in(4, 0); b.odo_mm = 100 + 1600; o = vc_step(&v, &b);
		CHECK("S5-H97c", v.bc_pending == VC_BC_NONE && (o.event & VC_EV_BC_EXPIRED), "1600 mm on: dropped");
		vc_init(&v, 0);
		b = line_in(4, 0); b.bc_cmd = VC_BC_LEFT; b.bc_n = 1; b.odo_mm = 1000; o = vc_step(&v, &b);
		b = line_in(4, 0); b.odo_mm = 1000 - 150; o = vc_step(&v, &b);
		CHECK("S5-H97d", v.bc_pending == VC_BC_LEFT, "backing away (odometry going down) does not use up the distance");
		(void)drive_mm;
	}

	/* H98: pause and obstacle during a manoeuvre */
	{
		VC_IN b;
		int k;

		/* PAUSE in the drive-on: after RESUME only the rest of the 55 mm is driven */
		vc_init(&v, 0);
		b = line_in(4, 0); b.bc_cmd = VC_BC_RIGHT; b.bc_n = 1; b.odo_mm = 0; o = vc_step(&v, &b);
		for (k = 0; k < 4; k++) { b = line_in(5, 0); b.junction = 1; b.odo_mm = 0; o = vc_step(&v, &b); }
		b = line_in(4, 0); b.odo_mm = 30; o = vc_step(&v, &b);
		b = line_in(4, 0); b.odo_mm = 30; b.sup_cmd = VC_SUP_PAUSE; o = vc_step(&v, &b);
		CHECK("S5-H98a", o.mode == VC_MODE_STOP && o.holding && o.busy && o.mission_state == VC_MS_PAUSED,
		      "PAUSE during the drive-on: stopped, manoeuvre kept");
		b = line_in(4, 0); b.odo_mm = 30; b.sup_cmd = VC_SUP_RESUME; o = vc_step(&v, &b);
		CHECK("S5-H98b", o.mode == VC_MODE_DRIVE && o.speed == VC_JN_SPEED, "RESUME: driving on again");
		b = line_in(4, 0); b.odo_mm = 30 + 20; o = vc_step(&v, &b);
		CHECK("S5-H98c", o.mode == VC_MODE_DRIVE, "30 + 20 = 50 mm: not yet 55");
		b = line_in(4, 0); b.odo_mm = 30 + 26; o = vc_step(&v, &b);
		CHECK("S5-H98d", o.mode == VC_MODE_TURN && o.turn_deg == 90, "30 + 26 = 56 mm: the turn starts");

		/* PAUSE half-way through the turn: after RESUME only the rest is turned */
		for (k = 0; k < 20; k++) {			/* 20 x 20 ms at +-100 mm/s = 80 mm = 38 degrees */
			b = line_in(4, 0); b.odo_mm = 56; b.wheel_l = 100; b.wheel_r = -100; b.motion_state = 2;
			o = vc_step(&v, &b);
		}
		b = line_in(4, 0); b.odo_mm = 56; b.sup_cmd = VC_SUP_PAUSE; b.motion_state = 2; o = vc_step(&v, &b);
		CHECK("S5-H98e", o.mode == VC_MODE_STOP && o.holding, "PAUSE during the turn: stopped");
		b = line_in(4, 0); b.odo_mm = 56; b.sup_cmd = VC_SUP_RESUME; o = vc_step(&v, &b);
		CHECK("S5-H98f", o.mode == VC_MODE_TURN && o.turn_deg >= 50 && o.turn_deg <= 54,
		      "RESUME: TURN with the rest, about 52 degrees (90 minus the 38 already turned), not 90");
		/* obstacle scan in the middle: held, then the same */
		b = line_in(4, 0); b.odo_mm = 56; b.obs_state = 3; b.motion_state = 2; o = vc_step(&v, &b);
		CHECK("S5-H98g", o.mode == VC_MODE_STOP && o.holding && o.mission_state == VC_MS_AVOIDING,
		      "obstacle scan during the turn: held, AVOIDING");
		b = line_in(4, 0); b.odo_mm = 56; o = vc_step(&v, &b);
		CHECK("S5-H98h", o.mode == VC_MODE_TURN && o.turn_deg >= 50 && o.turn_deg <= 54,
		      "scan over: TURN with the rest again");

		/* turned all but 6 degrees before the stop: nothing is left to turn, back to the line */
		vc_init(&v, 0);
		b = line_in(4, 0); b.bc_cmd = VC_BC_UTURN; b.bc_n = 1; o = vc_step(&v, &b);
		for (k = 0; k < 4; k++) { b = line_in(5, 0); b.junction = 1; o = vc_step(&v, &b); }
		for (k = 0; k < 90; k++) {			/* 90 x 20 ms x 200 mm/s = 360 mm = 172 degrees */
			b = line_in(4, 0); b.wheel_l = 100; b.wheel_r = -100; b.motion_state = 2;
			o = vc_step(&v, &b);
		}
		b = line_in(4, 0); b.sup_cmd = VC_SUP_PAUSE; o = vc_step(&v, &b);
		b = line_in(4, 0); b.sup_cmd = VC_SUP_RESUME; o = vc_step(&v, &b);
		CHECK("S5-H98i", (o.event & VC_EV_JN_DONE) && !o.busy && o.mode == VC_MODE_DRIVE,
		      "U-turn 172 of 180 degrees turned before a pause: less than 10 left, finished, line following");
	}

	/* H99: a turn that never reports done is given up, and one that overshoots is stopped */
	{
		VC_IN b;
		int k;

		vc_init(&v, 0);
		b = line_in(4, 0); b.bc_cmd = VC_BC_UTURN; b.bc_n = 1; o = vc_step(&v, &b);
		for (k = 0; k < 4; k++) { b = line_in(5, 0); b.junction = 1; o = vc_step(&v, &b); }
		for (k = 0; k < 250; k++) {			/* 250 x 20 ms = 5 s, wheels stuck (0 mm/s) */
			b = line_in(4, 0); b.motion_state = 2; o = vc_step(&v, &b);
			if (o.event & VC_EV_TURN_TIMEOUT) {
				break;
			}
		}
		CHECK("S5-H99a", (o.event & VC_EV_TURN_TIMEOUT) && k >= 195 && k <= 205 && !o.busy,
		      "wheels not turning: turn given up after 4 s");
		vc_init(&v, 0);
		b = line_in(4, 0); b.bc_cmd = VC_BC_LEFT; b.bc_n = 1; o = vc_step(&v, &b);
		for (k = 0; k < 4; k++) { b = line_in(5, 0); b.junction = 1; o = vc_step(&v, &b); }
		b = line_in(4, 0); b.odo_mm = 60; o = vc_step(&v, &b);
		for (k = 0; k < 90; k++) {			/* spins anticlockwise and S2 never says done */
			b = line_in(4, 0); b.odo_mm = 60; b.wheel_l = -100; b.wheel_r = 100; b.motion_state = 2;
			o = vc_step(&v, &b);
			if (o.event & VC_EV_JN_DONE) {
				break;
			}
		}
		CHECK("S5-H99b", (o.event & VC_EV_JN_DONE) && !(o.event & VC_EV_TURN_TIMEOUT) && k >= 50 && k <= 60,
		      "past 90 + 15 degrees (about 55 steps of 20 ms at 200 mm/s) without done: stopped as finished");
		/* done flag from an earlier move is not believed in the first 100 ms */
		vc_init(&v, 0);
		b = line_in(4, 0); b.bc_cmd = VC_BC_UTURN; b.bc_n = 1; o = vc_step(&v, &b);
		for (k = 0; k < 4; k++) { b = line_in(5, 0); b.junction = 1; o = vc_step(&v, &b); }
		b = line_in(4, 0); b.motion_state = 0; b.motion_done = 1; o = vc_step(&v, &b);
		CHECK("S5-H99c", o.mode == VC_MODE_TURN && o.busy, "old 'done' right after the TURN was sent: ignored");
	}

	/* H100: the status shows the barcode count and the last action */
	{
		VC_IN b;
		int k;

		vc_init(&v, 0);
		b = line_in(4, 0); b.bc_cmd = VC_BC_RIGHT; b.bc_n = 2; o = vc_step(&v, &b);
		CHECK("S5-H100a", o.barcodes == 2 && o.last_action == VC_ACT_FORWARD, "two commands in one cycle: counted 2");
		for (k = 0; k < 4; k++) { b = line_in(5, 0); b.junction = 1; o = vc_step(&v, &b); }
		CHECK("S5-H100b", o.last_action == VC_ACT_RIGHT && o.barcodes == 2, "last action RIGHT once the manoeuvre starts");
	}

	/* H101 to H107: the spin search for a lost line */
	{
		VC_IN b;
		int k, leg, sawturn[4];

		/* H101: lost 1500 ms, no obstacle: spin towards the side the line was last on */
		vc_init(&v, 0);
		for (k = 0; k < 74; k++) { b = line_in(5, 1); o = vc_step(&v, &b); }
		b = line_in(5, 1); o = vc_step(&v, &b);
		CHECK("S5-H101a", (o.event & VC_EV_SEARCH) && o.mode == VC_MODE_TURN && o.turn_deg == 60 &&
		      o.speed == VC_TURN_SPEED && o.mission_state == VC_MS_TURNING && !v.line_gave_up && o.busy,
		      "lost 1500 ms, last zone 5 (line towards the right of the robot): spin +60 degrees, TURNING, not given up");
		vc_init(&v, 0);
		for (k = 0; k < 75; k++) { b = line_in(3, 1); o = vc_step(&v, &b); }
		CHECK("S5-H101b", o.mode == VC_MODE_TURN && o.turn_deg == -60, "last zone 3: spin -60 degrees (left first)");

		/* H102: not found: legs +60, -120, +60 (each ends on the done flag), then stopped with a line fault */
		vc_init(&v, 0);
		for (k = 0; k < 75; k++) { b = line_in(5, 1); o = vc_step(&v, &b); }
		for (leg = 0; leg < 4; leg++) { sawturn[leg] = 0; }
		leg = 0;
		for (k = 0; k < 400 && !v.line_gave_up; k++) {
			b = line_in(5, 1);
			b.motion_state = 2;
			if (o.mode == VC_MODE_TURN && v.turn_ms >= 200) {	/* S2 reports done after 200 ms */
				b.motion_state = 0; b.motion_done = 1;
			}
			o = vc_step(&v, &b);
			if (o.mode == VC_MODE_TURN && leg < 3 && (o.turn_deg == 60 || o.turn_deg == -120) && sawturn[leg] == 0 &&
			    ((leg == 0 && o.turn_deg == 60) || (leg == 1 && o.turn_deg == -120) || (leg == 2 && o.turn_deg == 60))) {
				sawturn[leg] = 1;
				leg++;
			}
		}
		CHECK("S5-H102a", sawturn[0] && sawturn[1] && sawturn[2], "three legs sent in order: +60, -120, +60");
		CHECK("S5-H102b", v.line_gave_up && (o.event & VC_EV_GAVE_UP) && o.mode == VC_MODE_STOP && !o.busy &&
		      o.mission_state == VC_MS_FAULT, "legs done, line not found: STOP, FAULT, give-up event for the line fault");
		b = line_in(4, 0); o = vc_step(&v, &b);
		CHECK("S5-H102c", o.mode == VC_MODE_STOP && o.mission_state == VC_MS_FAULT, "the line comes back later: stays stopped (RESET needed)");

		/* H103: the line is seen during the second leg (2 line messages): stop searching, follow it */
		vc_init(&v, 0);
		for (k = 0; k < 75; k++) { b = line_in(5, 1); o = vc_step(&v, &b); }
		for (k = 0; k < 400 && o.turn_deg != -120; k++) {
			b = line_in(5, 1); b.motion_state = 2;
			if (o.mode == VC_MODE_TURN && v.turn_ms >= 200) { b.motion_state = 0; b.motion_done = 1; }
			o = vc_step(&v, &b);
		}
		CHECK("S5-H103a", o.turn_deg == -120, "second leg reached");
		b = line_in(4, 0); b.motion_state = 2; o = vc_step(&v, &b);
		CHECK("S5-H103b", o.mode == VC_MODE_TURN, "line seen in one message only: still turning");
		b = line_in(4, 0); b.motion_state = 2; o = vc_step(&v, &b);
		CHECK("S5-H103c", (o.event & VC_EV_FOUND) && !o.busy && o.mode == VC_MODE_DRIVE && !v.line_gave_up &&
		      o.mission_state == VC_MS_RUNNING, "second message: line found, line following again, RUNNING");
		/* a new loss gets a new sweep */
		for (k = 0; k < 75; k++) { b = line_in(4, 1); o = vc_step(&v, &b); }
		CHECK("S5-H103d", o.mode == VC_MODE_TURN && (v.search_leg == 1), "lost again later: a new sweep starts");

		/* H104: no spinning next to an obstacle, one sweep per loss */
		vc_init(&v, 0);
		for (k = 0; k < 75; k++) { b = line_in(5, 1); b.obst_present = 1; o = vc_step(&v, &b); }
		CHECK("S5-H104", o.mode == VC_MODE_STOP && v.line_gave_up && !(o.event & VC_EV_SEARCH),
		      "obstacle present: no spin search, stopped");

		/* H105: pause in the middle of the first leg: only the rest is turned */
		vc_init(&v, 0);
		for (k = 0; k < 75; k++) { b = line_in(5, 1); o = vc_step(&v, &b); }
		for (k = 0; k < 10; k++) {		/* 10 x 20 ms x 200 mm/s = 40 mm = 19 degrees */
			b = line_in(5, 1); b.wheel_l = 100; b.wheel_r = -100; b.motion_state = 2; o = vc_step(&v, &b);
		}
		b = line_in(5, 1); b.sup_cmd = VC_SUP_PAUSE; o = vc_step(&v, &b);
		b = line_in(5, 1); b.sup_cmd = VC_SUP_RESUME; o = vc_step(&v, &b);
		CHECK("S5-H105", o.mode == VC_MODE_TURN && o.turn_deg >= 40 && o.turn_deg <= 42,
		      "paused after 19 of 60 degrees: resumed with the rest, about 41");

		/* H106: line lost in a creep: the creep ends (for a rescan) but the car is not stopped for good */
		vc_init(&v, 0);
		b = line_in(4, 0); b.approach_mm = 150; b.odo_mm = 0; o = vc_step(&v, &b);
		for (k = 0; k < 80; k++) { b = line_in(5, 1); b.approach_mm = 150; b.odo_mm = 5; o = vc_step(&v, &b); }
		CHECK("S5-H106a", o.creep_done && !v.line_gave_up, "line lost 1.6 s into a creep: creep ends, not given up for good");
		b = line_in(4, 0); b.approach_mm = 0; b.odo_mm = 5; o = vc_step(&v, &b);
		CHECK("S5-H106b", o.mode == VC_MODE_DRIVE && !v.line_gave_up, "line back after the rescan: line following goes on");
	}

	/* H110 to H119: the bypass detour (phase 7) and REVERSE_RETRY */
	{
		WORLD w;
		VC_OUT bo;
		int k;

		/* H110: round the RIGHT, shift 21 cm, obstacle 17 cm ahead: +90, drive 210, -90, drive 520
		   (170 + 200 + 150), -90, look for the line, drive on 55, +90, done */
		memset(&w, 0, sizeof w);
		w.obs = 4; w.find_at = 205;
		vc_init(&v, 0);
		bo = world_step(&w, 1, 21, 17, 0);
		CHECK("S5-H110a", (bo.event & VC_EV_BYP_START) && bo.mode == VC_MODE_TURN && bo.turn_deg == 90 &&
		      !bo.holding && bo.mission_state == VC_MS_AVOIDING,
		      "BYPASSING with a usable profile: starts at once, TURN +90 (away from the line), AVOIDING, not held");
		for (k = 0; k < 3000 && !(w.events & (VC_EV_BYP_DONE | VC_EV_BYP_FAIL)); k++) {
			bo = world_step(&w, 1, 21, 17, 0);
		}
		CHECK("S5-H110b", (w.events & VC_EV_BYP_PASSED) && (w.events & VC_EV_BYP_RETURN) && (w.events & VC_EV_BYP_DONE) &&
		      !(w.events & VC_EV_BYP_FAIL) && !v.line_gave_up && v.byp_step == 0,
		      "events in the detour: passed, return, done; no failure");
		CHECK("S5-H110c", w.order[0] == 90 && w.order[1] == 150 && w.order[2] == -90 && w.order[3] == 150 &&
		      w.order[4] == -90 && w.order[5] == 60 && w.order[6] == 90 && w.n_order == 7,
		      "order of legs: turn +90, drive 150 mm/s, turn -90, drive 150 mm/s, turn -90, drive 60 mm/s (to the line, then on), turn +90");
		CHECK("S5-H110d", w.drive_total >= 205 + 520 + 210 - 15 && w.drive_total <= 210 + 520 + 205 + 55 + 15,
		      "distance driven about 210 + 520 + 205 + 55 mm");
		bo = world_step(&w, 1, 21, 17, 0);
		CHECK("S5-H110e", bo.mode == VC_MODE_DRIVE && !bo.busy && !v.line_gave_up, "afterwards: line following (REACQUIRING still exempt from the hold)");

		/* H111: round the LEFT mirrors it */
		memset(&w, 0, sizeof w);
		w.obs = 4; w.find_at = 205;
		vc_init(&v, 0);
		bo = world_step(&w, 2, 21, 17, 0);
		CHECK("S5-H111a", bo.mode == VC_MODE_TURN && bo.turn_deg == -90, "action LEFT: TURN -90 first");
		for (k = 0; k < 3000 && !(w.events & (VC_EV_BYP_DONE | VC_EV_BYP_FAIL)); k++) {
			bo = world_step(&w, 2, 21, 17, 0);
		}
		CHECK("S5-H111b", (w.events & VC_EV_BYP_DONE) && w.order[2] == 90 && w.order[4] == 90 && w.order[w.n_order - 1] == -90,
		      "turns: -90, +90, +90 (toward the line), -90 (along it)");

		/* H112: the line is not where it should be: stopped after shift + 120 mm, line fault */
		memset(&w, 0, sizeof w);
		w.obs = 4; w.find_at = -1;
		vc_init(&v, 0);
		for (k = 0; k < 3000 && !(w.events & (VC_EV_BYP_DONE | VC_EV_BYP_FAIL)); k++) {
			bo = world_step(&w, 1, 21, 17, 0);
		}
		CHECK("S5-H112a", (w.events & VC_EV_BYP_FAIL) && (w.events & VC_EV_GAVE_UP) && v.line_gave_up && bo.mode == VC_MODE_STOP,
		      "no line on the way back: BYP_FAIL, GAVE_UP, STOP");
		CHECK("S5-H112b", w.find_mm >= 328 && w.find_mm <= 336, "looked for 330 mm (shift 210 + 120) before giving up");

		/* H113: unusable profile (STOP action, or shift 0): the car stays held, no detour */
		memset(&w, 0, sizeof w);
		w.obs = 4;
		vc_init(&v, 0);
		bo = world_step(&w, 4, 0, 5, 0);
		CHECK("S5-H113a", bo.mode == VC_MODE_STOP && bo.holding && !(bo.event & VC_EV_BYP_START) && v.byp_step == 0,
		      "profile action STOP in BYPASSING: held");
		bo = world_step(&w, 1, 0, 17, 0);
		CHECK("S5-H113b", bo.mode == VC_MODE_STOP && bo.holding && v.byp_step == 0, "shift 0 cm: held");

		/* H114: not in phase 7: BYPASSING holds as before */
		memset(&w, 0, sizeof w);
		{
			VC_IN b = line_in(4, 0);
			b.obs_state = 4; b.prof_action = 1; b.prof_shift_cm = 21; b.prof_closest_cm = 17;
			vc_init(&v, 0);
			bo = vc_step(&v, &b);
			CHECK("S5-H114", bo.mode == VC_MODE_STOP && bo.holding && v.byp_step == 0, "phase7 off (phases 5, 6): BYPASSING holds, no detour");
		}

		/* H115: PAUSE in the middle of the sideways leg: after RESUME only the rest of the 210 mm is driven */
		memset(&w, 0, sizeof w);
		w.obs = 4; w.find_at = 205;
		vc_init(&v, 0);
		for (k = 0; k < 3000 && !(v.byp_step == 2 && w.drive_total >= 100); k++) {
			bo = world_step(&w, 1, 21, 17, 0);
		}
		k = w.drive_total;
		bo = world_step(&w, 1, 21, 17, VC_SUP_PAUSE);
		CHECK("S5-H115a", bo.mode == VC_MODE_STOP && bo.holding && bo.mission_state == VC_MS_PAUSED, "PAUSE during the sideways leg: stopped");
		bo = world_step(&w, 1, 21, 17, VC_SUP_RESUME);
		for (; w.steps < 6000 && v.byp_step == 2; ) { bo = world_step(&w, 1, 21, 17, 0); }
		CHECK("S5-H115b", w.drive_total - k >= 105 && w.drive_total - k <= 120, "only the rest of the leg was driven (about 110 mm more)");

		/* H116: Obstacle Detection falls into FAULT in the middle: the detour is dropped */
		memset(&w, 0, sizeof w);
		w.obs = 4; w.find_at = 205;
		vc_init(&v, 0);
		for (k = 0; k < 3000 && !(v.byp_step == 4 && w.drive_total >= 300); k++) {
			bo = world_step(&w, 1, 21, 17, 0);
		}
		w.obs = 7;
		bo = world_step(&w, 1, 21, 17, 0);
		CHECK("S5-H116", (bo.event & VC_EV_BYP_FAIL) && v.byp_step == 0 && bo.mode == VC_MODE_STOP && bo.holding,
		      "obstacle task FAULT during the detour: detour dropped, event for subsystem 3, held");

		/* H117: legs are limited (shift 100 cm and a gap of 90 cm stay within 1000 mm) */
		memset(&w, 0, sizeof w);
		w.obs = 4; w.find_at = 500;
		vc_init(&v, 0);
		bo = world_step(&w, 1, 100, 90, 0);
		CHECK("S5-H117", v.byp_s_mm == 1000 && v.byp_f_mm == 1000, "shift 100 cm and 90 cm gap: legs capped at 1000 mm");

		/* H118: REVERSE_RETRY: backs away 100 mm straight (-40 mm/s, steer 1), then REVERSE_DONE, then waits */
		memset(&w, 0, sizeof w);
		vc_init(&v, 0);
		{
			VC_IN b;
			int odo = 1000, back_total = 0;

			b = line_in(4, 0); b.phase7 = 1; b.obs_state = 6; b.odo_mm = (unsigned)odo;
			bo = vc_step(&v, &b);
			CHECK("S5-H118a", bo.mode == VC_MODE_DRIVE && bo.speed == -40 && bo.steer == VC_REVERSE_STEER && !bo.holding &&
			      !bo.reverse_done && bo.mission_state == VC_MS_AVOIDING, "REVERSE_RETRY: backing away, -40 mm/s, steer 1, AVOIDING");
			for (k = 0; k < 400 && !bo.reverse_done; k++) {
				odo -= 1; back_total++;
				b = line_in(4, 0); b.phase7 = 1; b.obs_state = 6; b.odo_mm = (unsigned)odo;
				bo = vc_step(&v, &b);
			}
			CHECK("S5-H118b", bo.reverse_done && bo.mode == VC_MODE_STOP && back_total >= 98 && back_total <= 102,
			      "after 100 mm: REVERSE_DONE and stopped");
			b = line_in(4, 0); b.phase7 = 1; b.obs_state = 1; b.odo_mm = (unsigned)odo;	/* scanning again: held */
			bo = vc_step(&v, &b);
			CHECK("S5-H118c", bo.holding && bo.mode == VC_MODE_STOP && !bo.reverse_done, "the next scan: held");
		}

		/* H119: SAFETY: the Safety Monitor's stop latch holds the detour too */
		memset(&w, 0, sizeof w);
		w.obs = 4; w.find_at = 205;
		vc_init(&v, 0);
		bo = world_step(&w, 1, 21, 17, 0);
		{
			VC_IN b = line_in(4, 0);
			b.phase7 = 1; b.obs_state = 4; b.prof_action = 1; b.prof_shift_cm = 21; b.prof_closest_cm = 17;
			b.motion_state = VC_STATE_INHIBITED;
			bo = vc_step(&v, &b);
			CHECK("S5-H119", bo.mode == VC_MODE_STOP && bo.holding && v.byp_step == 1, "stop latch set during the detour: held, detour kept");
		}
	}

	printf("[HOST] vehicle controller SUMMARY %d passed, %d failed\n", pass_n, fail_n);
	return fail_n ? 1 : 0;
}
