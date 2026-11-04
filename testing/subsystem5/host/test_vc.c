/*
 * Host-side unit tests for app_program/subsystem5/vc_logic.c
 * Build and run on a PC:   sh testing/subsystem5/host/run.sh
 * Output lines: [HOST] S5-H47 PASS ...
 */
#include <stdio.h>
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
	return in;
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

	/* H52: lost for 1500 ms: stop for good, even if the line comes back */
	vc_init(&v, 0);
	ok = 1;
	for (i = 0; i < 74; i++) {
		in = line_in(5, 1);
		o = vc_step(&v, &in);
		if (o.mode != VC_MODE_DRIVE) ok = 0;
	}
	CHECK("S5-H52a", ok, "lost for 1480 ms: still searching");
	in = line_in(5, 1);
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

	printf("[HOST] vehicle controller SUMMARY %d passed, %d failed\n", pass_n, fail_n);
	return fail_n ? 1 : 0;
}
