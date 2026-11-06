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
#define S5_CTRL_APPROACH_DONE	(1u << 5)	/* VC: the creep / back-away ordered in s5_approach_mm is finished */
IMPORT ID	flg_s5_ctrl;

/* ---- i8: obstacle profile to the Vehicle Controller ----
 * Message buffer, one message = OBS_MSG_BYTES (two packed 32-bit words,
 * layout in obstacle_profile.h). Send is non-blocking; the Vehicle
 * Controller reads with obs_unpack(). */
IMPORT ID	mbf_obstacle;

/* ---- IF-14: supervisor commands from subsystem 1, owned by the Vehicle Controller ----
 * Message buffer of 1-byte messages: bits 1:0 command, bit 2 link up, bits 7:3 unused.
 * Subsystem 1 sends with tk_snd_mbf(mbf_sup_cmd, &b, 1, TMO_POL) after it has
 * checked the command (FR7, FR8). The Vehicle Controller reads it every cycle.
 *   PAUSE  stop and stand still (obstacle scans carry on); only after START
 *   RESUME in READY: START the run; after PAUSE: drive on
 *   ABORT  stop for good until the board is reset
 * Link loss is only a status bit, not a fault. */
#define SUP_CMD_NONE		0u
#define SUP_CMD_PAUSE		1u
#define SUP_CMD_RESUME		2u
#define SUP_CMD_ABORT		3u
#define SUP_CMD_MASK		0x3u
#define SUP_LINK_UP		(1u << 2)
IMPORT ID	mbf_sup_cmd;

/* Mission state (VC_MS_* in vc_logic.h, 0..14) for the IF-13 status to subsystem 1.
   Written by the Vehicle Controller only. */
IMPORT volatile UB	s5_mission_state;

/* Obstacle summary for the status to subsystem 1: obs_summary_* in obstacle_profile.h.
   Written by Obstacle Detection in one store, read by the Vehicle Controller. */
IMPORT volatile UW	s5_obs_summary;

/* Faults the Safety Monitor is handling right now (IF-12 bits 0..3). Set when it
   sees them, cleared when it releases the stop latch. Read only elsewhere. */
IMPORT volatile UB	s5_fault_mask;

/* Current Obstacle Detection state (OBS_STATE, 3 bits). Read only. */
IMPORT volatile UB	s5_obs_state;

/* Move order from Obstacle Detection to the Vehicle Controller (phase 6+):
   0 = none; > 0 = creep forward this many mm FOLLOWING THE LINE at 40 mm/s;
   < 0 = back away this many mm straight. The Vehicle Controller measures the
   distance with the odometry (IF-17), stops, and sets S5_CTRL_APPROACH_DONE;
   Obstacle Detection then scans again from the new position. Obstacle
   Detection sets and clears s5_approach_mm; the Vehicle Controller only reads
   it. While it is not 0 the obstacle state is idle. */
IMPORT volatile H	s5_approach_mm;

/* ---- Build options (override with -D on the make line if needed) ---- */
/* 1 = the Vehicle Controller does not yet drive S5_CTRL_*: obstacle watch is
   on from power-up and a finished scan just returns to monitoring (phases
   5 and 6). Phase 7 (bypass) sets it to 0. */
#ifndef S5_STANDALONE
#define S5_STANDALONE		(S5_PHASE < 7)
#endif
/* 1 = the Vehicle Controller starts the run by itself once the power-up work is
   done (bench tests, no dashboard). 0 = it waits in READY for RESUME from
   subsystem 1 (IF-14). Set to 0 for the integrated demo. */
#ifndef VC_AUTO_START
#define VC_AUTO_START		0

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
