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
 *	move a short way (approach_mm, signed) so the second scan is made from
 *	about 17 cm. Forward it keeps FOLLOWING THE LINE at 40 mm/s; backwards it
 *	reverses straight. Either way the controller measures the distance with
 *	the odometry, stops when it is reached and reports creep_done. Holding for
 *	a scan or for the stop latch still comes first.
 *
 *	Barcode turns (IF-07): the movement command of a barcode is kept until the
 *	next junction. The junction flag of subsystem 3 also rises on ordinary left
 *	corrections (its own note says it needs arming by a barcode), so it is only
 *	looked at while a command is waiting, and has to stay up for a few line
 *	messages. Then: FORWARD drives straight over the junction without steering
 *	by the sensors; LEFT / RIGHT drive on a little (the sensors are ahead of
 *	the wheels), turn 90 degrees on the spot with the Motion Control TURN
 *	command and go back to line following; U-TURN turns 180 degrees on the spot.
 *	The angle turned is also estimated from the wheel speeds, so a turn that
 *	is stopped half-way (pause, obstacle) is finished with the rest only.
 *
 *	Bypass (phase 7, S5_STANDALONE 0): when Obstacle Detection is in BYPASSING the
 *	profile says on which side to pass and how far to move over. The car drives a
 *	rectangle round the obstacle with its own odometry and the wheel-speed turn
 *	estimate: turn 90 degrees away from the line, drive the shift, turn back, drive
 *	past the obstacle (gap + depth + body), turn towards the line, drive until
 *	subsystem 3 sees it again, drive on to put the wheels over it and turn 90
 *	degrees to face along it. Events tell the task when to set BYPASS_DONE (passed),
 *	end the line-lost suppression of subsystem 3 (IF-09) and set LINE_FOUND.
 *	REVERSE_RETRY is a short straight back-away; the task sets REVERSE_DONE.
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
#define VC_MODE_TURN		2u
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
/* Reverse: a steer of 0 would switch on the S2 straight-line hold, which flips its
   correction when reversing and then pushes the wheels APART (seen on the car: L -109,
   R +38 mm/s). A steer of 1 mm/s switches the hold off and is far too small to turn the car.
   The back-away does NOT steer by the line sensors: they are ahead of the wheels, so
   backing up on them is unstable and a simulation (testing/subsystem5/host/reverse_sim.py,
   S5-H80) shows it is no better than reversing straight. The car stays on the line because
   it was already aligned with it when it stopped. */
#define VC_REVERSE_STEER	1

/* ---- Barcode command (IF-07), the numbers subsystem 3 sends (subsystem3.h) ---- */
#define VC_BC_NONE		0u
#define VC_BC_FORWARD		1u
#define VC_BC_UTURN		2u
#define VC_BC_LEFT		3u
#define VC_BC_RIGHT		4u

/* ---- Last action reported to subsystem 1 (S1_MOVE_*) ---- */
#define VC_ACT_FORWARD		0u
#define VC_ACT_UTURN		1u
#define VC_ACT_LEFT		2u
#define VC_ACT_RIGHT		3u

/* ---- Junction manoeuvre (tune on the track, tests S5-M27 to M29) ---- */
/* Subsystem 3 (2026-11-04) raises the junction flag ONCE per crossbar, only after a bar-length
   run of GP26 + centre both black with GP28 dark too, holds it 5 line cycles (100 ms) and then
   keeps 500 ms quiet. So one message is enough; asking for several in a row could miss it when
   this task falls behind and drops older line messages. The flag rises when the sensors have
   just LEFT the crossbar's far edge. Measured by subsystem 3 (s3_drive_demo.c): */
#define VC_SENSOR_AXLE_MM	60	/* line sensors to the wheel axle */
#define VC_BAR_MM		35	/* crossbar / line width */
#define VC_JN_CONFIRM_MSGS	1	/* junction flag in this many line messages in a row */
#define VC_JN_ADVANCE_MM	(VC_SENSOR_AXLE_MM - VC_BAR_MM / 2)	/* after the flag, before a turn: axle over the bar's centre */
#define VC_JN_CROSS_MM		25	/* FORWARD: straight on after the flag, no steering by sensors, until the sensors are back on the line */
#define VC_BC_STEER_MAX		8	/* mm/s steering limit while GP28 reads a barcode and the line is <= 1 zone off */
#define VC_JN_SPEED		60	/* mm/s for those two */
#define VC_JN_REARM_MM		100	/* after a manoeuvre: flag low and this far before the next */
#define VC_BC_EXPIRE_MM		1500	/* a command not used by a junction this far on is dropped */
#define VC_TURN_SPEED		100	/* wheel speed during the spin, mm/s */
#define VC_TURN_DEG		90
#define VC_UTURN_DEG		180
#define VC_TURN_MIN_MS		100	/* ignore the "done" flag this long after sending (it may be old) */
#define VC_TURN_TIMEOUT_MS	4000	/* give up a turn after this long */
#define VC_TURN_LEFTOVER_D10	100	/* less than 10 degrees left after a stop: call it done */
#define VC_TURN_OVERSHOOT_D10	150	/* 15 degrees past the target: stop turning */
#define VC_TRACK_MM		120	/* S2_TRACK_WIDTH_MM */
#define VC_MOTION_IDLE		0u	/* S2_STATE_IDLE */

/* ---- Line search: when the line has been lost for VC_LF_LOST_STOP_MS (after the creeping
   search towards the side it was last seen) the car spins on the spot: VC_SEARCH_DEG towards
   that side, twice as far the other way, and back to where it started. It stops as soon as the
   line has been seen again. Only one sweep per loss; then it gives up and reports a line fault. ---- */
#define VC_SEARCH_DEG		60
#define VC_SEARCH_FOUND_MSGS	2	/* line seen in this many line messages in a row */

/* ---- Events, for the log (VC_OUT.event) ---- */
#define VC_EV_BC_TAKEN		0x01u	/* a barcode command was taken */
#define VC_EV_JN_START		0x02u	/* junction found, manoeuvre started */
#define VC_EV_JN_DONE		0x04u	/* manoeuvre finished */
#define VC_EV_TURN_TIMEOUT	0x08u	/* the turn did not finish in time */
#define VC_EV_BC_EXPIRED	0x10u	/* command dropped: no junction within VC_BC_EXPIRE_MM */
#define VC_EV_SEARCH		0x20u	/* line lost: the spin search starts */
#define VC_EV_FOUND		0x40u	/* line found again by the spin search */
#define VC_EV_GAVE_UP		0x80u	/* search over, line not found: stopped, line fault */
#define VC_EV_BYP_START		0x100u	/* bypass begins: tell subsystem 3 (IF-09) */
#define VC_EV_BYP_PASSED	0x200u	/* obstacle passed: set BYPASS_DONE */
#define VC_EV_BYP_RETURN	0x400u	/* looking for the line: line-lost suppression off */
#define VC_EV_BYP_DONE		0x800u	/* back on the line, facing along it: set LINE_FOUND */
#define VC_EV_BYP_FAIL		0x1000u	/* line not found on the way back: stopped, line fault */

/* ---- Mission state, reported to subsystem 1 in IF-13 (0..14; S1 proposes 1 = READY) ---- */
#define VC_MS_INIT		0u	/* power-up: calibrating, being placed on the line */
#define VC_MS_READY		1u	/* ready; waiting for START (RESUME from the dashboard) */
#define VC_MS_RUNNING		2u	/* following the line */
#define VC_MS_PAUSED		3u	/* PAUSE from the dashboard: standing still */
#define VC_MS_AVOIDING		4u	/* obstacle: scan, creep or back-away in progress */
#define VC_MS_FAULT		5u	/* a fault is being handled (stop latch) */
#define VC_MS_ABORTED		6u	/* ABORT from the dashboard: stopped for good until reset */
#define VC_MS_TURNING		7u	/* barcode command: turning at a junction */

/* ---- Supervisor command (IF-14, bits 1:0) ---- */
#define VC_SUP_NONE		0u
#define VC_SUP_PAUSE		1u
#define VC_SUP_RESUME		2u
#define VC_SUP_ABORT		3u

/* ---- Obstacle Detection states ---- */
#define VC_OBS_BYPASSING	4u
#define VC_OBS_REACQUIRING	5u
#define VC_OBS_REVERSE_RETRY	6u
#define VC_OBS_FAULT		7u

/* ---- Obstacle profile action (OBS_ACT_*) used for the bypass ---- */
#define VC_ACT_OBS_RIGHT	1u
#define VC_ACT_OBS_LEFT		2u

/* ---- Bypass detour (phase 7). Tune on the track, tests S5-M33 to M35 ---- */
#define VC_BYP_SPEED		150	/* mm/s on the two long legs */
#define VC_BYP_TURN_DEG		90
#define VC_BYP_DEPTH_MM		200	/* obstacle depth assumed (spec: obstacles are 10..20 cm) */
#define VC_BYP_CAR_MM		150	/* car body behind the ultrasonic sensor that must also clear it */
#define VC_BYP_MAX_LEG_MM	1000	/* S2 / odometry sanity limit for one leg */
#define VC_BYP_FIND_EXTRA_MM	120	/* look for the line this much farther than the shift */
#define VC_BYP_ADV_MM		(VC_SENSOR_AXLE_MM + VC_BAR_MM / 2)	/* line first seen at its near edge: axle over its centre */
#define VC_RETRY_BACK_MM	100	/* REVERSE_RETRY: back away this far, straight */

typedef struct {
	int		err_prev;	/* zone error of the previous cycle */
	int		have_prev;	/* err_prev is valid */
	uint32_t	lost_ms;	/* how long the line has been lost */
	int		line_gave_up;	/* lost for VC_LF_LOST_STOP_MS: stopped for good */
	int		holding;	/* standing still for the obstacle / stop latch */
	unsigned	epoch;		/* epoch for every command (0..15) */
	int		creep_on;	/* a creep is under way (start recorded) */
	int		creep_done;	/* the ordered distance is reached: stand still */
	unsigned	creep_start;	/* odometry (mm, 16 bit) when the creep began */
	int		creep_have_cmd;	/* creep_speed / creep_steer are valid */
	int		creep_speed;	/* line controller answer, kept between line messages */
	int		creep_steer;
	unsigned	creep_acc;	/* mm of the order already travelled before a pause / hold */
	int		started;	/* START given (RESUME in READY, or auto start) */
	int		paused;		/* PAUSE given, RESUME not yet */
	int		aborted;	/* ABORT given: stays until reset */
	unsigned	bc_pending;	/* barcode command waiting for a junction (VC_BC_*) */
	unsigned	bc_odo;		/* odometry when it arrived */
	unsigned	barcodes;	/* commands taken so far (stops at 255) */
	unsigned	last_action;	/* VC_ACT_* of the last manoeuvre */
	int		jn_armed;	/* a new junction may start a manoeuvre */
	int		jn_conf;	/* line messages in a row with the junction flag */
	unsigned	jn_rearm_odo;	/* odometry at the end of the last manoeuvre */
	unsigned	jn_state;	/* 0 none, 1 driving on, 2 turning */
	unsigned	jn_cmd;		/* the command being carried out */
	int		jn_on;		/* the drive-on has its start recorded */
	unsigned	jn_odo0;
	unsigned	jn_acc;		/* mm of the drive-on done before a stop */
	int		turn_dir;	/* +1 right (clockwise), -1 left */
	int		turn_target_d10;	/* tenths of a degree, positive */
	int		turn_cmd_deg;	/* degrees in the TURN command now sent */
	int		turn_live;	/* the TURN command is being sent */
	unsigned	turn_ms;	/* time since the TURN command was first sent */
	int32_t		turn_arc_um;	/* sum of (left - right) wheel travel while turning, micrometres */
	int		search_leg;	/* 0 = not searching, 1..3 = leg of the spin search */
	int		search_dir;	/* +1 right first, -1 left first */
	int		search_used;	/* the one sweep of this loss is used */
	int		found_cnt;	/* line messages in a row with the line seen, during the search */
	int		byp_step;	/* 0 = no bypass, 1..8 = step of the detour */
	int		byp_dir;	/* +1 round the right, -1 round the left */
	int		byp_s_mm;	/* the shift sideways */
	int		byp_f_mm;	/* the leg past the obstacle */
	int		drv_mm;		/* length of the drive leg now (JN_DRIVE) */
	int		drv_speed;
	int		drv_find;	/* the drive leg ends when the line is seen again */
	int		saw_lost;	/* subsystem 3 has reported the line lost since the leg began */
} VC_STATE;

typedef struct {
	int		line_fresh;	/* a line message arrived recently (else stop) */
	int		zone;		/* 1..7, 0 = invalid */
	int		lost;		/* line lost flag */
	uint32_t	dt_ms;		/* time since the last call */
	unsigned	obs_state;	/* s5_obs_state */
	unsigned	motion_state;	/* S2 state from s2_get_status() */
	int		approach_mm;	/* s5_approach_mm: 0 = none, > 0 creep forward, < 0 back away (mm) */
	unsigned	odo_mm;		/* odometry distance (IF-17), 16 bit, wraps */
	unsigned	sup_cmd;	/* IF-14 command taken this cycle (VC_SUP_*; NONE if no message) */
	int		ready;		/* power-up work (calibration, placing) is finished */
	int		auto_start;	/* 1 = start by itself once ready (bench); 0 = wait for RESUME */
	unsigned	fault_mask;	/* faults the Safety Monitor is handling now (IF-12 bits) */
	int		junction;	/* junction flag of the newest line message */
	int		bc_busy;	/* subsystem 3: GP28 is inside a possible barcode (keep the steering small) */
	int		obst_present;	/* Obstacle Detection has an obstacle (no spinning near it) */
	int		phase7;		/* the Vehicle Controller does the bypass (S5_STANDALONE 0) */
	unsigned	prof_action;	/* newest profile: OBS_ACT_* */
	unsigned	prof_closest_cm;
	unsigned	prof_shift_cm;	/* sideways move to pass on the side chosen by prof_action */
	unsigned	bc_cmd;		/* newest IF-07 command taken this cycle (VC_BC_*, NONE if none) */
	unsigned	bc_n;		/* how many were taken this cycle */
	int		wheel_l;	/* measured wheel speeds (IF-16), mm/s */
	int		wheel_r;
	int		motion_done;	/* IF-16 done flag */
	uint32_t	elapsed_ms;	/* time since the last vc_step call (capped) */
} VC_IN;

typedef struct {
	unsigned	mode;		/* VC_MODE_STOP or VC_MODE_DRIVE */
	int		speed;		/* mm/s */
	int		steer;		/* mm/s, + = turn right (IF-10) */
	unsigned	epoch;		/* epoch to put in the command */
	int		holding;	/* 1 = held for an obstacle or the stop latch */
	int		creep_done;	/* 1 = the ordered creep / back-away distance is reached */
	unsigned	mission_state;	/* VC_MS_*, for IF-13 telemetry */
	int		turn_deg;	/* VC_MODE_TURN: signed degrees, + = clockwise */
	int		busy;		/* a junction manoeuvre is under way: call vc_step every cycle */
	int		reverse_done;	/* REVERSE_RETRY: the back-away is finished (set REVERSE_DONE) */
	unsigned	barcodes;	/* commands taken so far */
	unsigned	last_action;	/* VC_ACT_* */
	unsigned	event;		/* VC_EV_* bits of this call */
} VC_OUT;

/* ---- Status to subsystem 1 (IF-13), every 500 ms ----
   Same fields and "no data" codes as s1_status_t in subsystem1.h; vehicle_controller.c
   copies them over. Everything is capped here the way subsystem 1 expects. */
#define VC_NO_OBST_DIST		511u	/* obst_dist_cm: no obstacle seen */
#define VC_NO_OBST_WIDTH	63u	/* obst_width_cm: unknown */
#define VC_NO_LETTER		31u	/* letter: none stored */
#define VC_NO_HUMP		255u	/* hump_peak_mm: none */
typedef struct {
	unsigned	mission_time_ds;	/* 0.1 s since START, 0 before */
	unsigned	obst_dist_cm;		/* 0..510, 511 none */
	unsigned	mission_state;		/* 0..14 */
	unsigned	fault_mask;		/* IF-12 bits 0..3 */
	unsigned	letter;			/* 0 = A .. 25 = Z, 31 none */
	unsigned	last_action;		/* S1_MOVE_*: 0 forward, 1 U-turn, 2 left, 3 right */
	unsigned	barcodes;		/* barcodes read, stops at 15 */
	unsigned	b_obstacle;		/* 1 = obstacle present */
	unsigned	obst_side;		/* 0 none, 1 left, 2 right */
	unsigned	obst_width_cm;		/* 0..62, 63 unknown */
	unsigned	hump_peak_mm;		/* 0..254, 255 none */
} VC_STATUS;

typedef struct {
	int		obst_present;
	unsigned	obst_dist_cm;
	unsigned	obst_width_cm;
	unsigned	obst_side;
	unsigned	mission_state;
	unsigned	fault_mask;
	uint32_t	run_ms;		/* time since START */
	int		letter;		/* -1 none */
	unsigned	last_action;
	unsigned	barcodes;
	int		hump_mm;	/* -1 none */
} VC_STATUS_IN;

void   vc_status_fill(const VC_STATUS_IN *in, VC_STATUS *out);

void   vc_init(VC_STATE *v, unsigned epoch);
VC_OUT vc_step(VC_STATE *v, const VC_IN *in);

#endif /* VC_LOGIC_H */
