/*
 *	subsystem5.h
 *	Subsystem 5 -- Spatial Optimizer: Obstacle Detection, Vehicle
 *	Controller, Safety Monitor.  Student ID: 2503437.
 *
 *	Public interface of subsystem 5 (what OTHER subsystems may use).
 *	Everything else stays private to the .c files.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "obstacle_profile.h"
#include "s5_phase.h"

/* ---- Task priorities (TEAM.md section 4; PROPOSED, to agree with S2/S3) ---- */
#define S5_PRI_SAFETY		5	/* Safety Monitor */
#define S5_PRI_VEHICLE		7	/* Vehicle Controller */
#define S5_PRI_OBSTACLE		8	/* Obstacle Detection */

/* ---- IF-12 / i14: fault flag group, owned by the Safety Monitor ----
 * One bit per owner. A task sets ONLY its own bit with tk_set_flg (an OR in
 * one kernel call). The Safety Monitor clears only the bits it handled.
 * Wi-Fi loss is NOT a fault and has no bit here. Bit numbers are proposed
 * in TEAM.md and still to be confirmed by the other owners. */
#define FLT_MOTION_BIT		(1u << 0)	/* S2 Motion Control: stall, encoder */
#define FLT_LINE_BIT		(1u << 1)	/* S3 Line Sensor */
#define FLT_IMU_BIT		(1u << 2)	/* S4 IMU / terrain */
#define FLT_OBSTACLE_BIT	(1u << 3)	/* S5 Obstacle Detection */
#define FLT_ALL_MASK		0xFu		/* bits 0..3 */
IMPORT ID	flg_fault_i14;

/* ---- IF-11: latched motor inhibit, owned by the Safety Monitor ----
 * Passed to Motion Control with subsystem2_bind_safety(). While S5_INHIBIT_BIT
 * is set the motors stay off and queued commands are dropped. Only the Safety
 * Monitor sets and clears it. */
#define S5_INHIBIT_BIT		(1u << 0)
IMPORT ID	flg_inhibit;

/* ---- Control flags into Obstacle Detection (flg_s5_ctrl) ----
 * Set by the Vehicle Controller / Safety Monitor; Obstacle Detection clears
 * the bit it consumed. */
#define S5_CTRL_MONITOR_EN	(1u << 0)	/* VC: line following, obstacle watch on (level) */
#define S5_CTRL_BYPASS_DONE	(1u << 1)	/* VC: drove past the obstacle, now search for the line */
#define S5_CTRL_LINE_FOUND	(1u << 2)	/* VC: line reacquired, bypass finished */
#define S5_CTRL_REVERSE_DONE	(1u << 3)	/* VC: backed away, scan again */
#define S5_CTRL_RESET		(1u << 4)	/* Safety Monitor: leave FAULT after re-initialising */
IMPORT ID	flg_s5_ctrl;

/* ---- i8: obstacle profile to the Vehicle Controller ----
 * Message buffer, one message = OBS_MSG_BYTES (two packed 32-bit words,
 * layout in obstacle_profile.h). Send is non-blocking; the Vehicle
 * Controller reads with obs_unpack(). */
IMPORT ID	mbf_obstacle;

/* Current Obstacle Detection state (OBS_STATE, 3 bits). Read only. */
IMPORT volatile UB	s5_obs_state;

/* Creep order from Obstacle Detection to the Vehicle Controller (phase 6+):
   0 = none; > 0 = drive this many mm straight ahead, < 0 = back away this many
   mm (DRIVE_DIST, slowly), then Obstacle Detection scans again from the new
   position. Obstacle Detection sets it and clears it; the Vehicle Controller
   only reads it. While it is not 0 the obstacle state is idle but the car is
   NOT following the line. */
IMPORT volatile H	s5_approach_mm;

/* ---- Build options (override with -D on the make line if needed) ---- */
/* 1 = the Vehicle Controller does not yet drive S5_CTRL_*: obstacle watch is
   on from power-up and a finished scan just returns to monitoring (phases
   5 and 6). Phase 7 (bypass) sets it to 0. */
#ifndef S5_STANDALONE
#define S5_STANDALONE		(S5_PHASE < 7)
#endif
/* 1 = print every reading and scan result on the console (bench bring-up) */
#ifndef S5_VERBOSE
#define S5_VERBOSE		1
#endif

/* Private to subsystem 5 */
IMPORT void	obstacle_detect_task(INT stacd, void *exinf);	/* phases 5-6 */
IMPORT void	s5_bringup_task(INT stacd, void *exinf);		/* phases 1-4 */
IMPORT void	safety_monitor_task(INT stacd, void *exinf);		/* phases 5-7 */
IMPORT void	vehicle_controller_task(INT stacd, void *exinf);	/* phases 6-7 */

#endif /* SUBSYSTEM5_H */
