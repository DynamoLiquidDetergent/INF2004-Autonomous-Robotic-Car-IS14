/*
 *	subsystem5.c
 *	Subsystem 5 -- Spatial Optimizer: creates its kernel objects and
 *	starts its tasks.
 *
 *	Objects created here (TEAM.md section 5/6):
 *	  flg_fault_i14  event flag group  IF-12  fault bits, one per owner
 *	  flg_s5_ctrl    event flag group         control into Obstacle Detection
 *	  mbf_obstacle   message buffer    i8     obstacle profile -> Vehicle Controller
 *
 *	Tasks: s5_bringup_task (phases 1-4) or obstacle_detect_task (phase 5-6). vehicle_controller_task and
 *	safety_monitor_task come next.
 */

#include "subsystem5.h"

EXPORT ID		flg_fault_i14;
EXPORT ID		flg_s5_ctrl;
EXPORT ID		mbf_obstacle;

EXPORT ER subsystem5_init(void)
{
	T_CFLG	cflg = {0};
	T_CMBF	cmbf = {0};

	/* Fault flags: starts with every bit clear. Several tasks set bits,
	   one task (the Safety Monitor) waits and clears. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_fault_i14 = tk_cre_flg(&cflg);
	if (flg_fault_i14 < E_OK) {
		return flg_fault_i14;
	}

	/* Control flags into Obstacle Detection. In stand-alone bench mode the
	   obstacle watch starts switched on. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = S5_STANDALONE ? S5_CTRL_MONITOR_EN : 0;
	flg_s5_ctrl = tk_cre_flg(&cflg);
	if (flg_s5_ctrl < E_OK) {
		return flg_s5_ctrl;
	}

	/* i8: room for 4 profiles (8 bytes + 4 bytes kernel overhead each) */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 4 * (OBS_MSG_BYTES + 4);
	cmbf.maxmsz = OBS_MSG_BYTES;
	mbf_obstacle = tk_cre_mbf(&cmbf);
	return (mbf_obstacle < E_OK) ? mbf_obstacle : E_OK;
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
	return (tid < E_OK) ? tid : E_OK;
}
