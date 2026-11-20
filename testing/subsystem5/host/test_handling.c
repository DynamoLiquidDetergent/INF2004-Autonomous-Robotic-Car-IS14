/* Host tests for handling_logic.c: collision and pick-up (demo rubric item 10). */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../../../app_program/subsystem5/handling_logic.h"

static int pass_n, fail_n;
#define CHECK(id, cond, desc) do { \
	if (cond) { pass_n++; printf("[HOST] %s PASS  %s\n", id, desc); } \
	else { fail_n++; printf("[HOST] %s FAIL  %s (line %d)\n", id, desc, __LINE__); } \
} while (0)

/* the car at rest reads about 0.92 g (S4 handover log), slightly tilted */
#define REST_X	(-32)
#define REST_Y	(-28)
#define REST_Z	916

static unsigned lcg = 12345u;
static int noise(int amp)
{
	lcg = lcg * 1103515245u + 12345u;
	return (int)((lcg >> 16) % (unsigned)(2 * amp + 1)) - amp;
}

typedef struct {
	HL_STATE h;
	uint32_t t;
	unsigned first;		/* first event */
	uint32_t first_t;
	unsigned n_coll, n_pick;
} RUN;

static void run_init(RUN *r)
{
	hl_init(&r->h);
	r->t = 0; r->first = HL_NONE; r->first_t = 0; r->n_coll = 0; r->n_pick = 0;
}

/* n samples (20 ms each) with this acceleration (+ noise) and these line sensors */
static void feed(RUN *r, int n, int ax, int ay, int az, int amp, uint32_t l, uint32_t c, uint32_t g)
{
	int i;

	for (i = 0; i < n; i++) {
		int32_t a[3];
		uint32_t raw[3];
		unsigned ev;

		a[0] = ax + noise(amp); a[1] = ay + noise(amp); a[2] = az + noise(amp);
		raw[0] = l; raw[1] = c; raw[2] = g;
		ev = hl_step(&r->h, a, raw, r->t);
		if (ev != HL_NONE && r->first == HL_NONE) { r->first = ev; r->first_t = r->t; }
		if (ev == HL_COLLISION) r->n_coll++;
		if (ev == HL_PICKED_UP) r->n_pick++;
		r->t += 20;
	}
}

/* line following: left sensor on white, centre on the line edge */
#define ON_LINE	200, 1600, 200

int main(void)
{
	RUN r;
	int k;

	/* a: 60 s of driving with 80 mg of vibration, turns, starts and stops: nothing */
	run_init(&r);
	feed(&r, 100, REST_X, REST_Y, REST_Z, 5, ON_LINE);
	for (k = 0; k < 30; k++) {
		feed(&r, 10, REST_X + 60, REST_Y, REST_Z, 80, ON_LINE);		/* speeding up */
		feed(&r, 60, REST_X, REST_Y + 40, REST_Z, 80, ON_LINE);		/* driving, turning */
		feed(&r, 10, REST_X - 60, REST_Y, REST_Z, 80, ON_LINE);		/* stopping */
	}
	CHECK("S5-H139a", r.first == HL_NONE, "60 s of driving with 80 mg vibration, starts, stops and turns: no event");

	/* b: a junction bar and a barcode: all three sensors dark for up to 0.8 s, car level */
	run_init(&r);
	feed(&r, 100, REST_X, REST_Y, REST_Z, 40, ON_LINE);
	for (k = 0; k < 5; k++) {
		feed(&r, 40, REST_X, REST_Y, REST_Z, 40, 3000, 3000, 2500);	/* 0.8 s on black */
		feed(&r, 50, REST_X, REST_Y, REST_Z, 40, ON_LINE);
	}
	feed(&r, 250, REST_X, REST_Y, REST_Z, 20, 3000, 3000, 2500);		/* standing on a bar for 5 s (a scan) */
	/* rougher floor: 150 mg of vibration over the same bars */
	for (k = 0; k < 5; k++) {
		feed(&r, 20, REST_X, REST_Y, REST_Z, 150, 3000, 3000, 2500);
		feed(&r, 50, REST_X, REST_Y, REST_Z, 150, ON_LINE);
	}
	CHECK("S5-H139b", r.first == HL_NONE, "junction bars and barcodes (all sensors black, car level), standing 5 s on a bar, bars at 150 mg vibration: no event");

	/* c: a hump: the car tilts 15 degrees over 1 s, stays 1 s, comes back; sensors see the floor */
	run_init(&r);
	feed(&r, 100, REST_X, REST_Y, REST_Z, 40, ON_LINE);
	for (k = 0; k <= 50; k++) {
		double th = 15.0 * k / 50.0 * 3.14159265 / 180.0;
		feed(&r, 1, (int)(REST_X + 916 * sin(th)), REST_Y, (int)(916 * cos(th)), 40, ON_LINE);
	}
	feed(&r, 50, (int)(REST_X + 916 * sin(0.2618)), REST_Y, (int)(916 * cos(0.2618)), 40, ON_LINE);
	for (k = 50; k >= 0; k--) {
		double th = -15.0 * k / 50.0 * 3.14159265 / 180.0;
		feed(&r, 1, (int)(REST_X + 916 * sin(th)), REST_Y, (int)(916 * cos(th)), 40, ON_LINE);
	}
	feed(&r, 100, REST_X, REST_Y, REST_Z, 40, ON_LINE);
	CHECK("S5-H139c", r.first == HL_NONE, "a 15 degree hump up and down over 3 s: no event");

	/* d: picked up while driving: 0.25 g up for 0.2 s, held 20 degrees tilted; sensors lose the floor 0.1 s in */
	run_init(&r);
	feed(&r, 100, REST_X, REST_Y, REST_Z, 60, ON_LINE);
	feed(&r, 5, REST_X, REST_Y, REST_Z + 250, 60, ON_LINE);
	feed(&r, 5, REST_X, REST_Y, REST_Z + 250, 60, 3500, 3500, 3000);
	{
		uint32_t lift_t = r.t - 100;
		feed(&r, 100, REST_X + 310, REST_Y, 860, 60, 3500, 3500, 3000);
		CHECK("S5-H139d", r.first == HL_PICKED_UP && r.first_t - lift_t <= 400,
		      "picked up (0.25 g lift, then held tilted, sensors dark): PICKED_UP within 0.4 s");
	}

	/* e: picked up gently and held level: 0.15 g for 0.3 s */
	run_init(&r);
	feed(&r, 100, REST_X, REST_Y, REST_Z, 20, ON_LINE);
	feed(&r, 15, REST_X, REST_Y, REST_Z + 160, 20, 3500, 3500, 3000);
	feed(&r, 50, REST_X, REST_Y, REST_Z, 20, 3500, 3500, 3000);
	CHECK("S5-H139e", r.first == HL_PICKED_UP, "lifted gently (0.16 g) and held level: PICKED_UP");

	/* f: the IMU moved 3 s before the sensors went dark (a knock, then later a junction): not a pick-up */
	run_init(&r);
	feed(&r, 100, REST_X, REST_Y, REST_Z, 20, ON_LINE);
	feed(&r, 2, REST_X, REST_Y, REST_Z + 300, 0, ON_LINE);
	feed(&r, 150, REST_X, REST_Y, REST_Z, 20, ON_LINE);
	feed(&r, 40, REST_X, REST_Y, REST_Z, 20, 3000, 3000, 2500);
	CHECK("S5-H139f", r.first == HL_NONE, "a bump 3 s before a junction bar: no event");

	/* g: hits a box: 0.6 g backwards for one sample */
	run_init(&r);
	feed(&r, 100, REST_X, REST_Y, REST_Z, 60, ON_LINE);
	feed(&r, 20, REST_X, REST_Y, REST_Z, 60, ON_LINE);
	feed(&r, 1, REST_X - 600, REST_Y, REST_Z, 0, ON_LINE);
	feed(&r, 10, REST_X, REST_Y, REST_Z, 60, ON_LINE);
	CHECK("S5-H139g", r.first == HL_COLLISION, "a knock of 0.6 g along the floor for one sample: COLLISION");

	/* h: hit from the side */
	run_init(&r);
	feed(&r, 100, REST_X, REST_Y, REST_Z, 60, ON_LINE);
	feed(&r, 1, REST_X + 200, REST_Y + 450, REST_Z, 0, ON_LINE);
	CHECK("S5-H139h", r.first == HL_COLLISION, "a knock from the side (0.49 g): COLLISION");

	/* i: nothing in the first second (the car is put down on the line) */
	run_init(&r);
	feed(&r, 10, REST_X, REST_Y, REST_Z + 500, 200, 3500, 3500, 3000);
	feed(&r, 40, REST_X, REST_Y, REST_Z, 20, ON_LINE);
	CHECK("S5-H139i", r.first == HL_NONE, "being put down in the first second: no event");

	printf("[HOST] handling SUMMARY %d passed, %d failed\n", pass_n, fail_n);
	return fail_n != 0;
}
