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
#define S5_PRI_SAFETY		5	/* Safety Monitor (not written yet) */
#define S5_PRI_VEHICLE		7	/* Vehicle Controller (not written yet) */
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
IMPORT ID	flg_fault_i14;

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

/* ---- Build options (override with -D on the make line if needed) ---- */
/* 1 = no Vehicle Controller yet: obstacle watch is on from power-up and a
   finished scan just returns to monitoring. Set 0 once the Vehicle
   Controller drives S5_CTRL_*. */
#ifndef S5_STANDALONE
#define S5_STANDALONE		(S5_PHASE < 6)
#endif
/* 1 = print every reading and scan result on the console (bench bring-up) */
#ifndef S5_VERBOSE
#define S5_VERBOSE		1
#endif

/* Private to subsystem 5 */
IMPORT void	obstacle_detect_task(INT stacd, void *exinf);	/* phases 5-6 */
IMPORT void	s5_bringup_task(INT stacd, void *exinf);		/* phases 1-4 */

#endif /* SUBSYSTEM5_H */
