/*
 *	vc_logic.h
 *	Subsystem 5 -- Vehicle Controller decision logic (no hardware, no RTOS).
 *
 *	Pure C so it can be unit-tested on a PC (testing/subsystem5/host/).
 *	vehicle_controller.c feeds it the line sensor, the Obstacle Detection
 *	state and the Motion Control state every cycle and sends what it answers
 *	to Motion Control (IF-10).
 *
 *	Version 1 (S5_PHASE 6): follow the line; stand still while Obstacle
 *	Detection is not idle or the Safety Monitor's stop latch is set; then
 *	drive on. No bypass yet.
 *
 *	Creep: after the first scan Obstacle Detection may order the car to
 *	drive a short distance straight ahead, or to back away if it stopped too
 *	close (approach_mm, signed), so the second scan is made from about 17 cm.
 *	While that order is open the controller sends one slow DRIVE_DIST command
 *	(negative speed = backwards) instead of following the line; holding for a
 *	scan or for the stop latch still comes first.
 *
 *	Epoch (IF-10): Motion Control drops commands older than the newest it
 *	has seen, and after an inhibit accepts only a NEWER epoch. So the epoch
 *	goes up by one (mod 16) each time the controller starts to hold, and the
 *	same epoch is used for every command until the next hold.
 *
 *	Line following is the controller subsystem 3 used in its drive demo
 *	(same gains), with the steering sign of the car as wired.
 */

#ifndef VC_LOGIC_H
#define VC_LOGIC_H

#include <stdint.h>

/* ---- Motion Control (subsystem2.h), repeated so this file stays pure ---- */
#define VC_MODE_STOP		0u
#define VC_MODE_DRIVE		1u
#define VC_MODE_DRIVE_DIST	3u	/* S2_MODE_DRIVE_DIST: param = distance in mm */
#define VC_STATE_INHIBITED	5u	/* S2_STATE_INHIBITED */

/* ---- Obstacle Detection state that lets the car drive ---- */
#define VC_OBS_IDLE		0u	/* OBS_ST_IDLE_MONITORING */

/* ---- Line following gains (same as subsystem 3's demo, tune on the track) ---- */
#define VC_ZONE_CENTRE		4	/* LINE_ZONE_CENTRE */
#define VC_LF_STEER_SIGN	(-1)	/* -1: GP26 is on the robot's right as it drives */
#define VC_LF_SPEED		100	/* mm/s when centred */
#define VC_LF_MIN_SPEED		40	/* mm/s at the sharpest correction */
#define VC_LF_SLOW_PER_ZONE	20	/* mm/s slower per zone off centre */
#define VC_LF_KP		30	/* mm/s steering per zone off centre */
#define VC_LF_KD		20	/* mm/s steering per zone per cycle of change */
#define VC_LF_STEER_MAX		120	/* mm/s steering limit */
#define VC_LF_SEARCH_SPEED	40	/* mm/s while the line is lost */
#define VC_LF_SEARCH_STEER	100	/* mm/s turn towards where the line was */
#define VC_LF_LOST_STOP_MS	1500	/* lost this long -> stop for good */
#define VC_APPROACH_SPEED	40	/* mm/s while creeping up to the obstacle */

typedef struct {
	int		err_prev;	/* zone error of the previous cycle */
	int		have_prev;	/* err_prev is valid */
	uint32_t	lost_ms;	/* how long the line has been lost */
	int		line_gave_up;	/* lost for VC_LF_LOST_STOP_MS: stopped for good */
	int		holding;	/* standing still for the obstacle / stop latch */
	unsigned	epoch;		/* epoch for every command (0..15) */
} VC_STATE;

typedef struct {
	int		line_fresh;	/* a line message arrived recently (else stop) */
	int		zone;		/* 1..7, 0 = invalid */
	int		lost;		/* line lost flag */
	uint32_t	dt_ms;		/* time since the last call */
	unsigned	obs_state;	/* s5_obs_state */
	unsigned	motion_state;	/* S2 state from s2_get_status() */
	int		approach_mm;	/* s5_approach_mm: 0 = none, > 0 creep forward, < 0 back away (mm) */
} VC_IN;

typedef struct {
	unsigned	mode;		/* VC_MODE_STOP or VC_MODE_DRIVE */
	int		speed;		/* mm/s */
	int		steer;		/* mm/s, + = turn right (IF-10) */
	unsigned	epoch;		/* epoch to put in the command */
	int		holding;	/* 1 = held for an obstacle or the stop latch */
	unsigned	param;		/* DRIVE_DIST: distance in mm (speed carries the direction), else 0 */
} VC_OUT;

void   vc_init(VC_STATE *v, unsigned epoch);
VC_OUT vc_step(VC_STATE *v, const VC_IN *in);

#endif /* VC_LOGIC_H */
