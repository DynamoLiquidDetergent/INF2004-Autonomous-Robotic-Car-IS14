/*
 * Host-side unit tests for app_program/subsystem5/safety_logic.c
 * Build and run on a PC:   sh testing/subsystem5/host/run.sh
 * Output lines: [HOST] S5-H36 PASS ...
 */
#include <stdio.h>
#include "../../../app_program/subsystem5/safety_logic.h"

static int pass_n, fail_n;

#define CHECK(id, cond, desc) do { \
	if (cond) { pass_n++; printf("[HOST] %s PASS  %s\n", id, desc); } \
	else { fail_n++; printf("[HOST] %s FAIL  %s (line %d)\n", id, desc, __LINE__); } \
} while (0)

#define F_MOTION	(1u << 0)
#define F_LINE		(1u << 1)
#define F_OBSTACLE	(1u << 3)

static SM_CFG cfg;
static SM_STATE st;

static void fresh(void)
{
	sm_cfg_default(&cfg);
	sm_init(&st);
}

int main(void)
{
	SM_OUT o;
	unsigned s;
	int ok;

	/* H36: nothing wrong, nothing happens */
	fresh();
	o = sm_step(&st, &cfg, 0, 0, 1000);
	CHECK("S5-H36", !o.set_inhibit && !o.clear_inhibit && o.clear_bits == 0 && !o.reset_obstacle && !st.inhibited,
	      "no fault, obstacle idle: the robot is free to move");

	/* H37: a fault bit sets the latch at once and is cleared (only that bit) */
	fresh();
	o = sm_step(&st, &cfg, F_MOTION, 0, 1000);
	CHECK("S5-H37", o.set_inhibit && o.clear_bits == F_MOTION && st.inhibited,
	      "fault bit: latch set at once, exactly that bit cleared");

	/* H38: held for hold_fault_ms after the fault, then released; no obstacle reset */
	o = sm_step(&st, &cfg, 0, 0, 1000 + 2999);
	CHECK("S5-H38a", !o.clear_inhibit && st.inhibited, "still held 2999 ms after the fault");
	o = sm_step(&st, &cfg, 0, 0, 1000 + 3000);
	CHECK("S5-H38b", o.clear_inhibit && !o.reset_obstacle && !st.inhibited,
	      "released at 3000 ms; a non-obstacle fault does not reset Obstacle Detection");

	/* H39: obstacle fault: released with a RESET for the obstacle task */
	fresh();
	o = sm_step(&st, &cfg, F_OBSTACLE, 7, 5000);
	CHECK("S5-H39a", o.set_inhibit && o.clear_bits == F_OBSTACLE, "obstacle fault: latch set, bit cleared");
	o = sm_step(&st, &cfg, 0, 7, 5000 + 3000);
	CHECK("S5-H39b", o.clear_inhibit && o.reset_obstacle, "released: Obstacle Detection told to re-initialise");

	/* H40: scan states stop the robot, others do not */
	ok = 1;
	for (s = 0; s <= 7; s++) {
		int want = (s >= 1 && s <= 3) ? 1 : 0;
		fresh();
		o = sm_step(&st, &cfg, 0, s, 100);
		if ((o.set_inhibit ? 1 : 0) != want) ok = 0;
	}
	CHECK("S5-H40", ok, "obstacle states 1..3 (scanning, profiling) hold the robot; 0 and 4..7 do not");

	/* H41: scan ends: held 500 ms more, then released; no reset */
	fresh();
	o = sm_step(&st, &cfg, 0, 1, 2000);
	o = sm_step(&st, &cfg, 0, 3, 5000);
	o = sm_step(&st, &cfg, 0, 0, 5100);
	CHECK("S5-H41a", !o.clear_inhibit && st.inhibited, "100 ms after the scan: still held");
	o = sm_step(&st, &cfg, 0, 0, 5100 + 500);
	CHECK("S5-H41b", o.clear_inhibit && !o.reset_obstacle && !st.inhibited, "500 ms after the last scan state: released");

	/* H42: a second fault during the hold extends it */
	fresh();
	o = sm_step(&st, &cfg, F_LINE, 0, 0);
	o = sm_step(&st, &cfg, F_MOTION, 0, 2000);
	o = sm_step(&st, &cfg, 0, 0, 4999);
	CHECK("S5-H42a", !o.clear_inhibit && st.inhibited, "second fault at 2000 ms: not released at 4999 ms");
	o = sm_step(&st, &cfg, 0, 0, 5000);
	CHECK("S5-H42b", o.clear_inhibit, "released 3000 ms after the second fault");

	/* H43: the millisecond counter wraps */
	fresh();
	o = sm_step(&st, &cfg, F_MOTION, 0, 0xFFFFFF00u);
	o = sm_step(&st, &cfg, 0, 0, 0xFFFFFF00u + 2999u);
	CHECK("S5-H43a", !o.clear_inhibit && st.inhibited, "across the wrap: still held at 2999 ms");
	o = sm_step(&st, &cfg, 0, 0, 0xFFFFFF00u + 3000u);
	CHECK("S5-H43b", o.clear_inhibit, "across the wrap: released at 3000 ms");

	/* H44: bits outside the mask are ignored */
	fresh();
	o = sm_step(&st, &cfg, 0xFFFFFFF0u, 0, 100);
	CHECK("S5-H44", !o.set_inhibit && o.clear_bits == 0 && !st.inhibited, "flag bits above bit 3 are not ours: ignored");

	/* H45: a long scan keeps the latch the whole time */
	fresh();
	ok = 1;
	o = sm_step(&st, &cfg, 0, 1, 0);
	for (s = 1; s <= 10; s++) {
		o = sm_step(&st, &cfg, 0, 2, s * 1000u);
		if (o.clear_inhibit || !st.inhibited) ok = 0;
	}
	CHECK("S5-H45", ok, "a 10 s scan never releases the latch");

	/* H46: fault and scan together: the longer hold wins */
	fresh();
	o = sm_step(&st, &cfg, F_MOTION, 1, 0);
	o = sm_step(&st, &cfg, 0, 0, 2999);
	CHECK("S5-H46a", !o.clear_inhibit && st.inhibited, "fault during a scan: held for the fault time");
	o = sm_step(&st, &cfg, 0, 0, 3000);
	CHECK("S5-H46b", o.clear_inhibit, "then released");

	printf("[HOST] safety SUMMARY %d passed, %d failed\n", pass_n, fail_n);
	return fail_n ? 1 : 0;
}
