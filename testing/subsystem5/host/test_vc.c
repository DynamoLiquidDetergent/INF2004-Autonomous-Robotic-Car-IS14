/*
 * Host-side unit tests for app_program/subsystem5/vc_logic.c
 * Build and run on a PC:   sh testing/subsystem5/host/run.sh
 * Output lines: [HOST] S5-H47 PASS ...
 */
#include <stdio.h>
#include <stdlib.h>
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


/* ---- kinematic simulation of the smooth bypass (S5-H122): a car (axle at the origin, 60 mm of nose in front,
   90 mm of tail behind, 80 mm half width) driven by the real vc_step, wheel speeds ramping 25 mm/s per 20 ms,
   a box of any width and depth on the line, the line sensor row 50 mm in front of the axle ---- */
#include <math.h>

static int sim_pad_mm = -10;	/* worst case: with the fine edge look the profile edge can lie up to 10 mm INSIDE the true edge (rounding to whole cm) */
static int sim_force_lost;	/* 1: report 'line lost, last zone 1' (a wrong guess) 3 cycles after the hand-over */
static int sim_corner_x;	/* > 0: the line turns toward the bypass side at this x (mm), the old line ends there */
static int sim_obs_lag;		/* cycles the obstacle task stays in BYPASSING after BYP_PASSED */
static int sim_never_find;	/* 1 = the line is never seen again after the start */

typedef struct {
	int   ok;		/* finished with VC_EV_BYP_DONE */
	int   fail_ev;		/* VC_EV_BYP_FAIL */
	int   hit;		/* the car touched the box */
	double clear_mm;	/* smallest gap between car and box (0 if hit) */
	int   ms;		/* time from BYP_START to BYP_DONE */
	double end_y_mm;	/* lateral error 3 s after the hand-over */
	int   lost_after;	/* line lost during those 3 s */
	double max_head_deg;	/* heading at the hand-over (absolute) */
	int   back_mm, lo_mm, lp_mm, d_mm;
	int   post_steer;	/* steering the controller gave in the forced-lost cycle */
	int   starts, early;	/* BYP_START events, BYP_DONE without the return curve (line met early) */
	int   turns, zero_steer, max_steer;	/* turns on the spot, steer 0 commands, largest |steer| during the detour */
} SIMRES;

static double rect_dist(double px, double py, double x0, double x1, double y0, double y1)
{
	double dx = (px < x0) ? x0 - px : (px > x1) ? px - x1 : 0.0;
	double dy = (py < y0) ? y0 - py : (py > y1) ? py - y1 : 0.0;

	return sqrt(dx * dx + dy * dy);
}

/* side: +1 round the right, -1 round the left. face_mm = true distance from the sensor to the box
   (the sensor read closest_cm). */
static SIMRES sim_bypass(int side, int width_mm, int depth_mm, int centre_mm, int closest_cm, int face_err_mm,
			 int shift_cm_override)
{
	SIMRES r;
	double x = 0, y = 0, th = 0, vl = 0, vr = 0, odo = 0;
	double box_x0 = 60.0 + closest_cm * 10.0 + face_err_mm, box_x1 = box_x0 + depth_mm;
	double box_y0 = centre_mm - width_mm / 2.0, box_y1 = centre_mm + width_mm / 2.0;
	int    shift_cm, t, started = 0, done_t = -1, start_t = 0;
	double edge;
	VC_IN b;
	VC_OUT o;
	int    obs = 4, lag = 0;

	memset(&r, 0, sizeof r);
	r.clear_mm = 1e9;
	edge = (side > 0) ? box_y1 : -box_y0;		/* edge of the box on the bypass side, from the line */
	if (edge < 0) { edge = 0; }
	shift_cm = (int)ceil((edge + sim_pad_mm + 80 + 30) / 10.0);
	if (shift_cm_override > 0) { shift_cm = shift_cm_override; }

	vc_init(&v, 0);
	v.smooth = 1;
	for (t = 0; t < 2000 * 50 / 20 + 400; t++) {
		double sy, rel;
		int    lost, zone, k;
		double cs = cos(th), sn = sin(th);

		b = line_in(4, 0);
		b.phase7 = 1;
		b.obs_state = (unsigned)obs;
		b.prof_action = (side > 0) ? 1u : 2u;
		b.prof_shift_cm = (unsigned)shift_cm;
		b.prof_closest_cm = (unsigned)closest_cm;
		b.odo_mm = ((unsigned)(long)floor(odo)) & 0xFFFFu;
		b.wheel_l = (int)lround(vl);
		b.wheel_r = (int)lround(vr);
		sy = y + 50.0 * sn;
		rel = -sy / (cs > 0.2 ? cs : 0.2);		/* line position across the sensor row, + = to the right */
		lost = (fabs(rel) > 32.0 || (sim_never_find && started)) ? 1 : 0;
		if (sim_corner_x > 0) {			/* main line ends at the corner, a side line goes off to the bypass side */
			double sxx = x + 50.0 * cs, syy = y + 50.0 * sn;
			double d_main = (sxx <= sim_corner_x) ? fabs(syy) : 1e9;
			double d_side = (syy * side >= -20.0) ? fabs(sxx - sim_corner_x) : 1e9;

			lost = (d_main > 32.0 && d_side > 32.0) ? 1 : 0;
			rel = 0.0;
		}
		zone = lost ? 0 : 4 + (int)lround(rel / 10.0);
		if (zone < 1 && !lost) { zone = 1; }
		if (zone > 7) { zone = 7; }
		b.lost = lost;
		b.zone = lost ? 0 : zone;
		if (sim_force_lost && done_t >= 0 && t == done_t + 3) {
			b.lost = 1; b.zone = 1;
		}
		o = vc_step(&v, &b);
		if (sim_force_lost && done_t >= 0 && t == done_t + 3) { r.post_steer = o.steer; }
		if (started && done_t < 0) {
			if (o.mode == VC_MODE_TURN) { r.turns++; }
			if (o.mode == VC_MODE_DRIVE && o.busy && o.steer == 0) { r.zero_steer++; }
			if (o.busy && (o.steer > r.max_steer || -o.steer > r.max_steer)) { r.max_steer = o.steer < 0 ? -o.steer : o.steer; }
		}
		if (o.event & VC_EV_BYP_START) {
			r.starts++;
			started = 1; start_t = t;
			r.back_mm = v.sw_back_mm; r.lo_mm = v.sw_lo_mm; r.lp_mm = v.sw_lp_mm; r.d_mm = v.sw_d_mm;
		}
		if (o.event & VC_EV_BYP_FAIL) { r.fail_ev = 1; break; }
		if ((o.event & VC_EV_BYP_PASSED) && v.sw_kind != VC_SW_RET) { r.early = 1; }
		if (o.event & VC_EV_BYP_PASSED) { lag = sim_obs_lag; if (lag == 0) { obs = 5; } }
		if (lag > 0 && --lag == 0) { obs = 5; }
		if (done_t >= 0 && lag == 0 && obs == 5) { obs = 0; }
		if (o.event & VC_EV_BYP_DONE) {
			if (lag == 0) { obs = 0; }
			r.ok = 1; done_t = t; r.ms = (t - start_t) * 20;
			r.max_head_deg = fabs(th) * 57.2958;
		}
		if (done_t >= 0 && t - done_t > 150) { break; }
		/* wheel targets */
		{
			double tl = 0, tr = 0;

			if (o.mode == VC_MODE_DRIVE && o.busy) {
				tl = o.speed + o.steer; tr = o.speed - o.steer;
			} else if (o.mode == VC_MODE_DRIVE) {
				/* line following: steering from the zone, 30 mm/s a zone (as the controller does) */
				tl = o.speed + o.steer; tr = o.speed - o.steer;
			}
			vl += (tl > vl + 25) ? 25 : (tl < vl - 25) ? -25 : tl - vl;
			vr += (tr > vr + 25) ? 25 : (tr < vr - 25) ? -25 : tr - vr;
		}
		/* move 20 ms */
		{
			double vm = (vl + vr) / 2.0, w = (vl - vr) / 120.0;

			x += vm * cs * 0.02; y += vm * sn * 0.02; th += w * 0.02; odo += vm * 0.02;
		}
		/* clearance: sample the car outline */
		cs = cos(th); sn = sin(th);
		for (k = 0; k < 4 * 32; k++) {
			double ax, ay, px, py, d;
			int    e = k / 32, i = k % 32;
			double f = i / 32.0;

			switch (e) {
			case 0: ax = -90 + f * 150; ay = -80; break;
			case 1: ax = 60; ay = -80 + f * 160; break;
			case 2: ax = 60 - f * 150; ay = 80; break;
			default: ax = -90; ay = 80 - f * 160; break;
			}
			py = y + ax * sn + ay * cs;
			px = x + ax * cs - ay * sn;
			d = rect_dist(px, py, box_x0, box_x1, box_y0, box_y1);
			if (started && d < r.clear_mm) { r.clear_mm = d; }
		}
		if (getenv("S5_SIM_TRACE") != NULL && (t % 10) == 0 && started && done_t < 0) {
			printf("[TR] t%4d kind%u x%7.1f y%7.1f th%6.1f vl%4.0f vr%4.0f st%d clr %.0f\n", t, v.sw_kind, x, y, th * 57.2958, vl, vr, o.steer, r.clear_mm);
		}
		if (r.clear_mm < 0.5) { r.hit = 1; }
		if (done_t >= 0 && t - done_t == 150) { r.end_y_mm = y; }
		if (done_t >= 0 && t > done_t && lost) { r.lost_after = 1; }
	}
	if (!r.ok) { r.end_y_mm = y; }
	return r;
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
	CHECK("S5-H70b", o.mode == VC_MODE_STOP && o.holding && o.epoch == 6,
	      "stop latch set while held: epoch raised again (Motion Control blocks the one it had)");
	in.motion_state = 0;
	in.odo_mm = 2015;
	o = vc_step(&v, &in);
	CHECK("S5-H70c", o.mode == VC_MODE_DRIVE && o.epoch == 6 && !o.holding, "latch released: the creep starts, newer epoch");
	in.odo_mm = 2015 + 319;
	o = vc_step(&v, &in);
	CHECK("S5-H70d", o.mode == VC_MODE_DRIVE && !o.creep_done, "counted from the start of the creep (2015), not from 2000");
	in.odo_mm = 2015 + 320;
	o = vc_step(&v, &in);
	CHECK("S5-H70e", o.creep_done, "320 mm after the start: done");

	/* H121: the stop latch comes on while the controller is already holding (first scan, rescan,
	   second scan). Model of Motion Control: when the latch goes on it blocks the newest epoch it
	   has accepted; after the latch is released it accepts any other epoch. Found on the car in
	   S5-M20: held for ever with the epoch the latch had blocked. */
	{
		VC_STATE hv;
		VC_IN hb;
		VC_OUT ho;
		unsigned newest = 0, blocked_epoch = 0;
		int blocked = 0, step, ok_all = 1, moved_all = 1;

		vc_init(&hv, 1);
		for (step = 0; step < 3; step++) {		/* three scans in a row, each with its own latch */
			hb = line_in(4, 0);
			hb.obs_state = (step == 1) ? 6 : 1;	/* scan, retry, scan */
			hb.motion_state = 0;
			ho = vc_step(&hv, &hb);			/* hold starts */
			newest = ho.epoch;
			hb.motion_state = VC_STATE_INHIBITED;	/* latch on: Motion Control blocks 'newest' */
			blocked = 1;
			blocked_epoch = newest;
			ho = vc_step(&hv, &hb);
			ok_all = ok_all && ho.mode == VC_MODE_STOP && ho.holding;
			newest = ho.epoch;
			ho = vc_step(&hv, &hb);			/* still held, many cycles */
			ho = vc_step(&hv, &hb);
			hb.obs_state = 0;			/* scan over, latch released */
			hb.motion_state = VC_STATE_INHIBITED;
			ho = vc_step(&hv, &hb);			/* the latch may still show for a cycle */
			if (blocked && ho.epoch != blocked_epoch) {
				blocked = 0;			/* Motion Control accepts the newer epoch */
				hb.motion_state = 0;
			}
			ho = vc_step(&hv, &hb);
			moved_all = moved_all && !blocked && ho.mode == VC_MODE_DRIVE && !ho.holding;
		}
		CHECK("S5-H121a", ok_all, "latch on during a hold: still held");
		CHECK("S5-H121b", moved_all,
		      "three scans each with a latch: the epoch always differs from the blocked one, the car drives on");
	}

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
			b = line_in(5, 1);		/* flag, but the line is reported lost: not a junction */
			b.junction = 1;
			o = vc_step(&v, &b);
		}
		b = line_in(4, 0);
		o = vc_step(&v, &b);
		CHECK("S5-H91b", !o.busy && o.mode == VC_MODE_DRIVE && v.bc_pending == VC_BC_RIGHT,
		      "junction flag while the line is reported lost: no manoeuvre, command still waiting");

		/* H92: RIGHT. The flag in one line message starts it: drive on 43 mm straight (steer 0, 60 mm/s),
		   then TURN +90 at 100 mm/s, TURNING all the time; when S2 reports done, back to the line */
		vc_init(&v, 0);
		b = line_in(4, 0);
		b.bc_cmd = VC_BC_RIGHT; b.bc_n = 1;
		o = vc_step(&v, &b);
		odo = 1000;
		b = line_in(5, 0);
		b.junction = 1;
		b.odo_mm = odo;
		o = vc_step(&v, &b);
		CHECK("S5-H92a", (o.event & VC_EV_JN_START) && o.busy && o.mode == VC_MODE_DRIVE &&
		      o.speed == VC_JN_SPEED && o.steer == 0 && o.mission_state == VC_MS_TURNING &&
		      o.last_action == VC_ACT_RIGHT,
		      "the message with the flag: manoeuvre starts, driving on straight (steer 0, 60 mm/s), TURNING");
		b = line_in(4, 0);
		b.odo_mm = odo + 30;
		o = vc_step(&v, &b);
		CHECK("S5-H92b", o.mode == VC_MODE_DRIVE && o.steer == 0 && o.mission_state == VC_MS_TURNING,
		      "30 mm of 43 mm driven: still driving on, mission state TURNING");
		b = line_in(4, 0);
		b.odo_mm = odo + 56;
		o = vc_step(&v, &b);
		CHECK("S5-H92c", o.mode == VC_MODE_TURN && o.turn_deg == 90 && o.speed == VC_TURN_SPEED && o.busy,
		      "56 mm driven, past 43 mm: TURN +90 degrees (clockwise) at 100 mm/s");
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

		/* H95: FORWARD: straight over the junction (steer 0, no sensor steering) for 25 mm, no TURN,
		   RUNNING state, then back to the line */
		vc_init(&v, 0);
		b = line_in(4, 0); b.bc_cmd = VC_BC_FORWARD; b.bc_n = 1; b.odo_mm = 500; o = vc_step(&v, &b);
		for (k = 0; k < 4; k++) { b = line_in(6, 0); b.junction = 1; b.odo_mm = 500; o = vc_step(&v, &b); }
		CHECK("S5-H95a", o.mode == VC_MODE_DRIVE && o.steer == 0 && o.busy && o.mission_state == VC_MS_RUNNING &&
		      v.last_action == VC_ACT_FORWARD, "FORWARD: straight on, zone 6 pull ignored, RUNNING");
		b = line_in(6, 0); b.junction = 1; b.odo_mm = 500 + 15; o = vc_step(&v, &b);
		CHECK("S5-H95b", o.steer == 0 && o.busy, "15 mm of 25: still straight");
		b = line_in(4, 0); b.junction = 1; b.odo_mm = 500 + 27; o = vc_step(&v, &b);
		CHECK("S5-H95c", !o.busy && (o.event & VC_EV_JN_DONE) && o.mode == VC_MODE_DRIVE,
		      "25 mm reached: done, line following");

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
		b = line_in(4, 0); b.odo_mm = 30 + 10; o = vc_step(&v, &b);
		CHECK("S5-H98c", o.mode == VC_MODE_DRIVE, "30 + 10 = 40 mm: not yet 43");
		b = line_in(4, 0); b.odo_mm = 30 + 14; o = vc_step(&v, &b);
		CHECK("S5-H98d", o.mode == VC_MODE_TURN && o.turn_deg == 90, "30 + 14 = 44 mm: the turn starts");

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
		b = line_in(5, 0); b.junction = 1; o = vc_step(&v, &b);
		b = line_in(4, 0); b.motion_state = 0; b.motion_done = 1; o = vc_step(&v, &b);
		CHECK("S5-H99c", o.mode == VC_MODE_TURN && o.busy, "old 'done' right after the TURN was sent: ignored");
	}

	/* H120: GP28 inside a barcode: steering limited to 8 mm/s while the line is 1 zone off or less */
	{
		VC_IN b;
		VC_OUT bo;

		vc_init(&v, 0);
		b = line_in(5, 0); b.bc_busy = 1;
		bo = vc_step(&v, &b);
		CHECK("S5-H120a", bo.mode == VC_MODE_DRIVE && bo.steer == VC_BC_STEER_MAX,
		      "zone 5 (30 mm/s steering normally) while reading a barcode: limited to 8 mm/s");
		vc_init(&v, 0);
		b = line_in(3, 0); b.bc_busy = 1;
		bo = vc_step(&v, &b);
		CHECK("S5-H120b", bo.steer == -VC_BC_STEER_MAX, "zone 3: limited to -8 mm/s");
		vc_init(&v, 0);
		b = line_in(6, 0); b.bc_busy = 1;
		bo = vc_step(&v, &b);
		CHECK("S5-H120c", bo.steer >= 60, "zone 6 (two zones off): not limited");
		vc_init(&v, 0);
		b = line_in(5, 0); b.bc_busy = 0;
		bo = vc_step(&v, &b);
		CHECK("S5-H120d", bo.steer == 30, "not in a barcode: zone 5 gives the normal 30 mm/s");
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
		vc_init(&v, 0); v.smooth = 0;	/* the old rectangle detour */
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
		CHECK("S5-H110d", w.drive_total >= 205 + 520 + 210 - 15 && w.drive_total <= 210 + 520 + 205 + 77 + 15,
		      "distance driven about 210 + 520 + 205 + 77 mm");
		bo = world_step(&w, 1, 21, 17, 0);
		CHECK("S5-H110e", bo.mode == VC_MODE_DRIVE && !bo.busy && !v.line_gave_up, "afterwards: line following (REACQUIRING still exempt from the hold)");

		/* H111: round the LEFT mirrors it */
		memset(&w, 0, sizeof w);
		w.obs = 4; w.find_at = 205;
		vc_init(&v, 0); v.smooth = 0;	/* the old rectangle detour */
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
		vc_init(&v, 0); v.smooth = 0;	/* the old rectangle detour */
		for (k = 0; k < 3000 && !(w.events & (VC_EV_BYP_DONE | VC_EV_BYP_FAIL)); k++) {
			bo = world_step(&w, 1, 21, 17, 0);
		}
		CHECK("S5-H112a", (w.events & VC_EV_BYP_FAIL) && (w.events & VC_EV_GAVE_UP) && v.line_gave_up && bo.mode == VC_MODE_STOP,
		      "no line on the way back: BYP_FAIL, GAVE_UP, STOP");
		CHECK("S5-H112b", w.find_mm >= 328 && w.find_mm <= 336, "looked for 330 mm (shift 210 + 120) before giving up");

		/* H113: unusable profile (STOP action, or shift 0): the car stays held, no detour */
		memset(&w, 0, sizeof w);
		w.obs = 4;
		vc_init(&v, 0); v.smooth = 0;	/* the old rectangle detour */
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
			vc_init(&v, 0); v.smooth = 0;	/* the old rectangle detour */
			bo = vc_step(&v, &b);
			CHECK("S5-H114", bo.mode == VC_MODE_STOP && bo.holding && v.byp_step == 0, "phase7 off (phases 5, 6): BYPASSING holds, no detour");
		}

		/* H115: PAUSE in the middle of the sideways leg: after RESUME only the rest of the 210 mm is driven */
		memset(&w, 0, sizeof w);
		w.obs = 4; w.find_at = 205;
		vc_init(&v, 0); v.smooth = 0;	/* the old rectangle detour */
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
		vc_init(&v, 0); v.smooth = 0;	/* the old rectangle detour */
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
		vc_init(&v, 0); v.smooth = 0;	/* the old rectangle detour */
		bo = world_step(&w, 1, 100, 90, 0);
		CHECK("S5-H117", v.byp_s_mm == 1000 && v.byp_f_mm == 1000, "shift 100 cm and 90 cm gap: legs capped at 1000 mm");

		/* H118: REVERSE_RETRY: backs away 100 mm straight (-40 mm/s, steer 1), then REVERSE_DONE, then waits */
		memset(&w, 0, sizeof w);
		vc_init(&v, 0); v.smooth = 0;	/* the old rectangle detour */
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
		vc_init(&v, 0); v.smooth = 0;	/* the old rectangle detour */
		bo = world_step(&w, 1, 21, 17, 0);
		{
			VC_IN b = line_in(4, 0);
			b.phase7 = 1; b.obs_state = 4; b.prof_action = 1; b.prof_shift_cm = 21; b.prof_closest_cm = 17;
			b.motion_state = VC_STATE_INHIBITED;
			bo = vc_step(&v, &b);
			CHECK("S5-H119", bo.mode == VC_MODE_STOP && bo.holding && v.byp_step == 1, "stop latch set during the detour: held, detour kept");
		}
	}


	if (getenv("S5_SIM_TRACE") != NULL) {
		SIMRES q = sim_bypass(-1, 60, 100, 0, 20, 0, 0);

		printf("[TRACE] done %d clr %.0f ms %d\n", q.ok, q.clear_mm, q.ms);
		return 0;
	}
	/* H122: the smooth bypass clears boxes of every width, side and distance */
	{
		static const int widths[] = { 60, 100, 150, 200, 250 };
		static const int depths[] = { 100, 200 };
		static const int centres[] = { -20, 0, 20 };
		static const int closest[] = { 12, 15, 17, 20, 25 };
		static const int errs[] = { -8, 0, 8 };
		int side, iw, id, ic, iq, ie, n = 0, n_ok = 0, n_hit = 0, n_lost = 0, n_fail = 0, n_slow = 0, n_off = 0;
		double worst_clear = 1e9, worst_ms = 0, worst_y = 0, worst_head = 0;

		for (side = -1; side <= 1; side += 2) {
			for (iw = 0; iw < 5; iw++) for (id = 0; id < 2; id++) for (ic = 0; ic < 3; ic++)
			for (iq = 0; iq < 5; iq++) for (ie = 0; ie < 3; ie++) {
				SIMRES r = sim_bypass(side, widths[iw], depths[id], centres[ic], closest[iq], errs[ie], 0);

				n++;
				if (r.ok) { n_ok++; }
				if (r.fail_ev) { n_fail++; }
				if (r.hit) { n_hit++; }
				if (r.lost_after) { n_lost++; }
				if (r.ms > 20000) { n_slow++; }
				if (fabs(r.end_y_mm) > 20.0) { n_off++; }
				if (r.clear_mm < worst_clear) { worst_clear = r.clear_mm; }
				if (r.ms > worst_ms) { worst_ms = r.ms; }
				if (fabs(r.end_y_mm) > worst_y) { worst_y = fabs(r.end_y_mm); }
				if (r.max_head_deg > worst_head) { worst_head = r.max_head_deg; }
				if (getenv("S5_SIM_VERBOSE") != NULL && (!r.ok || r.hit || r.clear_mm < 25.0 || fabs(r.end_y_mm) > 20.0 || r.lost_after)) {
					printf("[SIM] side %d W%d dep%d c%d q%d e%d: ok%d fail%d hit%d clear %.0f ms %d y %.0f lost%d head %.0f  back %d lo %d lp %d d %d\n",
					       side, widths[iw], depths[id], centres[ic], closest[iq], errs[ie], r.ok, r.fail_ev, r.hit,
					       r.clear_mm, r.ms, r.end_y_mm, r.lost_after, r.max_head_deg, r.back_mm, r.lo_mm, r.lp_mm, r.d_mm);
				}
			}
		}
		printf("[HOST] INFO H122: %d cases, worst clearance %.0f mm, longest %.1f s, worst lateral error %.0f mm, worst heading at hand-over %.0f deg\n",
		       n, worst_clear, worst_ms / 1000.0, worst_y, worst_head);
		CHECK("S5-H122a", n_ok == n && n_fail == 0, "every case: detour finishes with BYP_DONE (no failure)");
		CHECK("S5-H122b", n_hit == 0 && worst_clear >= 5.0, "no case touches the box; at least 5 mm between the car and the box even when the profile edge lies 10 mm inside the true edge (the worst the fine look can give)");
		CHECK("S5-H122c", n_slow == 0, "every detour is shorter than 20 s (the obstacle task gives up at 25 s)");
		CHECK("S5-H122e", worst_head <= 12.0, "the car faces along the travel (within 12 degrees) when the line controller takes over");
		CHECK("S5-H122d", n_lost == 0 && n_off == 0, "line following takes over and holds the line (no loss, under 20 mm off after 3 s)");
	}

	/* H123: how the curve is chosen from the obstacle */
	{
		int psi_far = 0, back_far = 0, lo_far = 0;
		SIMRES c;
		int psi_near, back_near, lo_near, psi_wide, lo_wide;
		VC_IN b;
		VC_OUT o;
		int k;

		vc_init(&v, 0); v.smooth = 1;
		b = line_in(4, 0); b.phase7 = 1; b.obs_state = 4; b.prof_action = 1; b.prof_shift_cm = 16; b.prof_closest_cm = 30;
		o = vc_step(&v, &b);
		psi_far = v.sw_psi_d10; back_far = v.sw_back_mm; lo_far = v.sw_lo_mm;
		CHECK("S5-H123a", (o.event & VC_EV_BYP_START) && psi_far == 450 && back_far == 0 && o.mode == VC_MODE_DRIVE && !o.holding,
		      "plenty of room: the gentle 45 degree curve, no backing up, starts at once");

		vc_init(&v, 0); v.smooth = 1;
		b = line_in(4, 0); b.phase7 = 1; b.obs_state = 4; b.prof_action = 1; b.prof_shift_cm = 16; b.prof_closest_cm = 17;
		o = vc_step(&v, &b);
		psi_near = v.sw_psi_d10; back_near = v.sw_back_mm; lo_near = v.sw_lo_mm;
		CHECK("S5-H123b", psi_near > psi_far && psi_near <= 600 && lo_near < lo_far + 1 + 0 * back_near,
		      "17 cm from the box: a steeper curve (up to 60 degrees) so the car does not have to back up as far");

		vc_init(&v, 0); v.smooth = 1;
		b = line_in(4, 0); b.phase7 = 1; b.obs_state = 4; b.prof_action = 1; b.prof_shift_cm = 25; b.prof_closest_cm = 30;
		o = vc_step(&v, &b);
		psi_wide = v.sw_psi_d10; lo_wide = v.sw_lo_mm;
		CHECK("S5-H123c", lo_wide > lo_far && psi_wide >= 450 && psi_wide <= 600 && v.sw_d_mm == 250 - VC_SW_TRIM_MM,
		      "wider obstacle (larger shift): longer curve, sideways reach = the shift from the profile less VC_SW_TRIM_MM");

		vc_init(&v, 0); v.smooth = 1;
		b = line_in(4, 0); b.phase7 = 1; b.obs_state = 4; b.prof_action = 1; b.prof_shift_cm = 20; b.prof_closest_cm = 10;
		o = vc_step(&v, &b);
		CHECK("S5-H123d", v.sw_back_mm > 0 && v.sw_back_mm <= VC_SW_BACK_MAX_MM && v.sw_psi_d10 == 600 && o.speed < 0 && o.steer == 1,
		      "box very close: backs up (at most the limit) with the 60 degree curve, steer 1 so S2 does not hold straight");

		/* never a turn on the spot, never steer 0, never more than the limit, in a full run */
		c = sim_bypass(1, 200, 200, 0, 17, 0, 0);
		CHECK("S5-H123e", c.turns == 0 && c.zero_steer == 0 && c.max_steer > 0 && c.max_steer <= VC_SW_STEER_MAX,
		      "no turn on the spot, steer never 0, never over the limit during the detour");
		sim_never_find = 1;
		c = sim_bypass(-1, 150, 200, 0, 20, 0, 0);
		sim_never_find = 0;
		CHECK("S5-H123i", c.fail_ev && !c.ok && c.turns == 0 && !c.hit,
		      "line never found again: the 150 mm look ends in BYP_FAIL (car stops, no turn, no hit)");
		c = sim_bypass(1, 200, 200, 0, 17, 0, 0);
		CHECK("S5-H123f", c.ok && !c.hit && c.clear_mm >= 5.0 && c.ms < 20000,
		      "20 cm box, 17 cm away (the M33 set-up): done, at least 5 mm clear of the box, under 20 s");

		/* pause in the middle of the curve (stop latch), then go on */
		vc_init(&v, 0); v.smooth = 1;
		b = line_in(4, 0); b.phase7 = 1; b.obs_state = 4; b.prof_action = 1; b.prof_shift_cm = 21; b.prof_closest_cm = 30;
		o = vc_step(&v, &b);
		for (k = 0; k < 30; k++) { b.odo_mm = (unsigned)(k * 3); o = vc_step(&v, &b); }
		{
			unsigned kind0 = v.sw_kind;

			b.motion_state = VC_STATE_INHIBITED;
			o = vc_step(&v, &b);
			CHECK("S5-H123g", o.holding && o.mode == VC_MODE_STOP && v.byp_step == 1, "stop latch in the curve: held, the detour is kept");
			b.motion_state = 1;
			b.odo_mm = (unsigned)(30 * 3);
			o = vc_step(&v, &b);
			o = vc_step(&v, &b);
			CHECK("S5-H123h", !o.holding && o.mode == VC_MODE_DRIVE && v.sw_kind == kind0 && o.steer != 0,
			      "released: the same curve goes on from where it was");
		}
	}

	/* H124: the line is seen while the bypass is going on (obstacle near a corner) */
	{
		SIMRES c;
		int side;
		int n_ok = 0, n_early = 0, n_one = 0, n_hit = 0;
		double worst = 1e9, wh = 0;
		static const int corners[] = { 530, 600, 700, 800 };	/* box ends at 430 mm: the corner is past the box, as in M37 */
		int ic;

		sim_obs_lag = 3;			/* the obstacle task needs a few cycles to leave BYPASSING */
		for (side = -1; side <= 1; side += 2) {
			for (ic = 0; ic < 4; ic++) {
				sim_corner_x = corners[ic];
				c = sim_bypass(side, 150, 200, 0, 17, 0, 0);
				if (c.ok) { n_ok++; }
				if (c.early) { n_early++; }
				if (c.starts == 1) { n_one++; }
				if (c.hit) { n_hit++; }
				if (getenv("S5_SIM_VERBOSE") != NULL) { printf("[H124] side %d corner %d ok%d early%d starts%d clear %.0f head %.0f ms %d\n", side, corners[ic], c.ok, c.early, c.starts, c.clear_mm, c.max_head_deg, c.ms); }
				if (c.clear_mm < worst) { worst = c.clear_mm; }
				if (c.max_head_deg > wh) { wh = c.max_head_deg; }
			}
		}
		sim_corner_x = 0;
		sim_obs_lag = 0;
		printf("[HOST] INFO H124: 8 cases, ended early %d times, worst clearance %.0f mm\n", n_early, worst);
		CHECK("S5-H124a", n_ok == 8 && n_early >= 2, "a corner just past the box: the bypass ends the moment the line is seen (both sides); further corners are met at the normal end of the detour");
		CHECK("S5-H124b", n_one == 8, "the bypass is not started a second time while the obstacle task is still in BYPASSING");
		CHECK("S5-H124d", wh <= 12.0, "the car is facing along the travel (within 12 degrees) when it takes the line: it does not rejoin turned back towards the old line");
		CHECK("S5-H124c", n_hit == 0 && worst >= 5.0, "no touch of the box when the line is taken over early (at least 5 mm, worst-case edge)");
	}

	/* H125: just after the line is taken over during a bypass, a lost line is searched for the way the line leads */
	{
		SIMRES r1, r2, r3, r4;

		sim_force_lost = 1;
		sim_corner_x = 350;
		r1 = sim_bypass(1, 150, 200, 0, 17, 0, 0);		/* right bypass, line met on the far side: looks right */
		r2 = sim_bypass(-1, 150, 200, 0, 17, 0, 0);		/* left bypass: looks left */
		sim_corner_x = 0;
		r3 = sim_bypass(1, 150, 200, 0, 17, 0, 0);		/* straight line, met on the way back: looks back, left */
		r4 = sim_bypass(-1, 150, 200, 0, 17, 0, 0);
		sim_force_lost = 0;
		CHECK("S5-H125a", r1.early && r1.post_steer > 0 && r2.early && r2.post_steer < 0,
		      "line met beside the obstacle (corner towards the bypass side), then lost: the car turns towards the bypass side, not back");
		CHECK("S5-H125b", r3.post_steer < 0 && r4.post_steer > 0,
		      "line met on the way back, then lost: the car looks back towards the old line");
	}

	printf("[HOST] vehicle controller SUMMARY %d passed, %d failed\n", pass_n, fail_n);
	return fail_n ? 1 : 0;
}
