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

	printf("[HOST] vehicle controller SUMMARY %d passed, %d failed\n", pass_n, fail_n);
	return fail_n ? 1 : 0;
}
