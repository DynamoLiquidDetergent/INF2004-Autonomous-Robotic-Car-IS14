/*
 *	safety_logic.h
 *	Subsystem 5 -- Safety Monitor decision logic (no hardware, no RTOS).
 *
 *	Pure C so it can be unit-tested on a PC (testing/subsystem5/host/).
 *	safety_monitor.c feeds it the fault flags, the obstacle task state and
 *	the time, and does what it answers.
 *
 *	Rules (IF-11 / IF-12):
 *	  1. Any bit in the fault flags (IF-12) stops the robot at once: the
 *	     inhibit latch (IF-11) is set. It stays set for hold_fault_ms after
 *	     the LAST fault seen.
 *	  2. While Obstacle Detection is scanning (coarse, fine, profiling) the
 *	     robot must stand still: the latch is set. It is released
 *	     hold_scan_ms after the scan state ends. (Guard: it works even if
 *	     the Vehicle Controller forgets to stop the robot.)
 *	  3. A fault bit is cleared as soon as it is seen (only the bits we
 *	     saw). On release, if the Obstacle Detection bit was one of them,
 *	     tell that task to re-initialise.
 *	  4. Only a command with a NEWER epoch moves the robot after a release
 *	     (Motion Control enforces this), so the Vehicle Controller must bump
 *	     the epoch before it drives again.
 */

#ifndef SAFETY_LOGIC_H
#define SAFETY_LOGIC_H

#include <stdint.h>

typedef struct {
	uint32_t hold_fault_ms;		/* keep the latch this long after the last fault */
	uint32_t hold_scan_ms;		/* keep the latch this long after a scan ends */
	uint32_t obstacle_bit;		/* the Obstacle Detection bit in the fault flags */
	uint32_t fault_mask;		/* bits of the fault flags that count */
} SM_CFG;

typedef struct {
	uint8_t		inhibited;	/* the latch is set */
	uint8_t		active_now;	/* a trigger is present right now */
	uint32_t	until_ms;	/* the latch may be released at this time */
	uint32_t	handled;	/* fault bits seen since the latch was set (for the reset) */
} SM_STATE;

typedef struct {
	uint8_t		set_inhibit;	/* set the latch now */
	uint8_t		clear_inhibit;	/* clear the latch now */
	uint32_t	clear_bits;	/* clear exactly these fault bits (the ones seen now) */
	uint8_t		reset_obstacle;	/* send RESET to Obstacle Detection */
} SM_OUT;

void sm_cfg_default(SM_CFG *c);
void sm_init(SM_STATE *s);

/* True when the obstacle task state means the robot must stand still. */
int  sm_scan_state_stops(unsigned obs_state);

/* One decision. faults = current pattern of the fault flags. */
SM_OUT sm_step(SM_STATE *s, const SM_CFG *c, uint32_t faults, unsigned obs_state, uint32_t now_ms);

#endif /* SAFETY_LOGIC_H */
