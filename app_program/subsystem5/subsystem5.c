/*
 *	subsystem5.c
 *	Subsystem 5 -- Spatial Optimizer: creates its kernel objects and
 *	starts its tasks.
 *
 *	Objects created here (TEAM.md section 5/6):
 *	  flg_fault_i14  event flag group  IF-12  fault bits, one per owner
 *	  flg_inhibit    event flag group  IF-11  latched motor inhibit, to Motion Control
 *	  flg_s5_ctrl    event flag group         control into Obstacle Detection
 *	  mbf_obstacle   message buffer    i8     obstacle profile -> Vehicle Controller
 *	  mbf_sup_cmd    message buffer    IF-14  PAUSE / RESUME / ABORT from subsystem 1
 *
 *	Tasks: s5_bringup_task (phases 1-4) or obstacle_detect_task and
 *	safety_monitor_task (phases 5-6), plus vehicle_controller_task (phase 6).
 */

#include "subsystem5.h"
#include "../subsystem2/subsystem2.h"	/* subsystem2_bind_safety() */
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/s4_integration.h"

#if S3_FAULT_BIT != FLT_LINE_BIT || FLT_IMU_BIT == FLT_LINE_BIT
#error Fault owners must use separate bits in the existing S5 event group
#endif

EXPORT ID		flg_fault_i14;
EXPORT ID		flg_s5_ctrl;
EXPORT ID		mbf_obstacle;
EXPORT ID		mbf_sup_cmd;

EXPORT ER subsystem5_init(void)
{
	T_CFLG	cflg = {0};
	T_CMBF	cmbf = {0};
	ER er;

	/* Fault flags: starts with every bit clear. Several tasks set bits,
	   one task (the Safety Monitor) waits and clears. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_fault_i14 = tk_cre_flg(&cflg);
	if (flg_fault_i14 < E_OK) {
		return flg_fault_i14;
	}
	/* i19/IF-17: bind copied S2 odometry before either core starts S4.
	 * i14/IF-12: S4 sets only the IMU bit of this existing group. */
	er = subsystem4_connect_motion(flg_fault_i14, FLT_IMU_BIT);
	if (er != E_OK) {
		return er;
	}

	/* Control flags into Obstacle Detection. In stand-alone bench mode the
	   obstacle watch starts switched on. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = S5_STANDALONE ? S5_CTRL_MONITOR_EN : 0;
	flg_s5_ctrl = tk_cre_flg(&cflg);
	if (flg_s5_ctrl < E_OK) {
		return flg_s5_ctrl;
	}

	/* IF-11: the motor inhibit latch. Clear at power-up: the robot may move. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_inhibit = tk_cre_flg(&cflg);
	if (flg_inhibit < E_OK) {
		return flg_inhibit;
	}
#if S5_PHASE >= 5
	/* Motion Control now obeys the latch and reports a stall on bit 0 of the
	   fault flags (FLT_MOTION_BIT). Phases 1-4 leave Motion Control free for
	   bench work. */
	subsystem2_bind_safety(flg_inhibit, S5_INHIBIT_BIT, flg_fault_i14, FLT_MOTION_BIT);
#endif

	/* i8: room for 4 profiles (8 bytes + 4 bytes kernel overhead each) */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 4 * (OBS_MSG_BYTES + 4);
	cmbf.maxmsz = OBS_MSG_BYTES;
	mbf_obstacle = tk_cre_mbf(&cmbf);
	if (mbf_obstacle < E_OK) {
		return mbf_obstacle;
	}

	/* IF-14: supervisor commands from subsystem 1, 1 byte each, room for 4.
	   The kernel stores every message as a 4 byte header plus the data rounded up to 4 bytes,
	   so a 1 byte command takes 8 bytes (found by S5-U10 on the board: 4 * (1 + 4) held only 2). */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 4 * (4 + 4);
	cmbf.maxmsz = 1;
	mbf_sup_cmd = tk_cre_mbf(&cmbf);
	return (mbf_sup_cmd < E_OK) ? mbf_sup_cmd : E_OK;
}

EXPORT ER subsystem5_start(void)
{
	ID tid;

	/* Phases 1-4 (s5_phase.h) run the small bring-up task that touches
	   only the pins wired so far; phases 5-6 run the real task.
	   No core affinity: the scheduler may run it on either core. */
#if S5_PHASE <= 4
	tid = app_start_task(s5_bringup_task, S5_PRI_OBSTACLE, 0, "s5_bringup");
#else
	tid = app_start_task(obstacle_detect_task, S5_PRI_OBSTACLE, 0, "obstacle_task");
#endif
	if (tid < E_OK) {
		return tid;
	}
#if S5_PHASE >= 5
	tid = app_start_task(safety_monitor_task, S5_PRI_SAFETY, 0, "safety_monitor");
	if (tid < E_OK) {
		return tid;
	}
#endif
#if S5_PHASE >= 6
	tid = app_start_task(vehicle_controller_task, S5_PRI_VEHICLE, 0, "vehicle_ctrl");
	if (tid < E_OK) {
		return tid;
	}
#endif
	return E_OK;
}
