/*
 *	motion_control.c
 *	Subsystem 2 -- the Motion Control task (D1) and its cyclic handler.
 *
 *	Every S2_CYCLE_MS (20 ms, 50 Hz) a cyclic handler wakes the task, which:
 *	  1. checks the Safety Monitor's latched inhibit (IF-11)
 *	  2. drains the command buffer q_motion_cmd (IF-10), keeping only the
 *	     newest command that is not stale
 *	  3. stops the robot if no command arrived for S2_CMD_TIMEOUT_MS
 *	  4. reads both encoders (snapshot with interrupts masked)
 *	  5. turns the active command into a speed target per wheel
 *	  6. runs one PI step per wheel and checks for a stall
 *	  7. checks the inhibit AGAIN and only then writes the PWM
 *	  8. publishes the status (IF-16) and odometry (IF-17) words
 *
 *	Agreed at Tribunal 1 (Buddy 2 + Buddy 5):
 *	  - inhibit checked before every PWM write; Safety always wins
 *	  - commands carry an epoch; stale epochs are discarded
 *	  - no command for S2_CMD_TIMEOUT_MS -> the motors stop
 *	  - only the Safety Monitor clears the inhibit, after re-initialising;
 *	    after it clears, only a command with a NEWER epoch moves the robot
 */

#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "motion_math.h"

/* Debug: print one CSV line every N cycles (0 = off).  Columns:
   S2,t_ms,state,sp_l,spd_l,duty_l,sp_r,spd_r,duty_r,dist_mm */
#ifndef S2_LOG_EVERY_CYCLES
#define S2_LOG_EVERY_CYCLES	0
#endif

/* Straight-line hold (DRIVE with steering 0): where each wheel's distance
   comes from.
     1 = integrate the MEASURED SPEED of each wheel (mm/s x ms = um).
         The speed comes from the time between edges, so it stays right
         even when one quadrature channel drops out -- which freezes the
         pulse COUNT (the A edges cancel out) and used to send the hold
         to its +/-S2_HEADING_MAX_MMPS limit within seconds.
     0 = use the encoder pulse count (original behaviour).
   DRIVE_DIST, TURN and the published odometry (IF-17) always use the
   pulse count. */
#ifndef S2_HEADING_FROM_SPEED
#define S2_HEADING_FROM_SPEED	1
#endif

/* Straight-line trim (S2_HEADING_FROM_SPEED only), in permille of the
   right wheel's distance.  Corrects small wheel-size / sensor differences.
     robot veers LEFT  -> make it positive (try +10, then adjust)
     robot veers RIGHT -> make it negative (try -10, then adjust)
   Each 10 = 1 %.  Record the final value in testing/subsystem2/TEST_LOG.md. */
#ifndef S2_HEADING_TRIM_R_PERMILLE
#define S2_HEADING_TRIM_R_PERMILLE	0
#endif

#define CMD_BUF_DEPTH		4		/* messages the buffer holds */
#define CMD_MSG_HEADER		sizeof(INT)	/* kernel's per-message header */
#define CMD_BUF_SIZE		(CMD_BUF_DEPTH * (sizeof(UW) + CMD_MSG_HEADER))
#define CYCLE_US		((UW)S2_CYCLE_MS * 1000U)
#define STALL_CYCLES		(S2_STALL_TIME_MS / S2_CYCLE_MS)
#define UM_PER_MM		1000L
#define HALF_PULSE_UM		(S2_UM_PER_PULSE / 2)	/* stop at the NEAREST pulse */

EXPORT ID	q_motion_cmd;			/* IF-10 */

LOCAL ID	cyc_motion;
LOCAL ID	tid_motion;

/* Published words: each written with one aligned 32-bit store */
LOCAL volatile UW	pub_status = 0U;
LOCAL volatile UW	pub_odometry = 0U;

/* Safety Monitor link (IF-11, IF-12), set by subsystem2_bind_safety() */
LOCAL volatile ID	safety_inhibit_flg = 0;
LOCAL volatile UINT	safety_inhibit_ptn = 0U;
LOCAL volatile ID	safety_fault_flg = 0;
LOCAL volatile UINT	safety_fault_ptn = 0U;

LOCAL S2_TIMING		timing;

/* State of the control loop, used only by motion_task */
typedef struct {
	UINT	mode;			/* S2_MODE_* being executed */
	UINT	state;			/* S2_STATE_* reported */
	UW	active_word;		/* command that started the current move */
	INT	cmd_speed;
	INT	cmd_param;
	BOOL	done;			/* TURN / DRIVE_DIST finished */
	BOOL	have_epoch;		/* at least one command accepted */
	UINT	newest_epoch;
	BOOL	blocked;		/* after inhibit / stall: need a newer epoch */
	UINT	blocked_epoch;
	UW	last_cmd_ms;
	W	start_um[ENC_COUNT];	/* wheel travel when the move started */
	W	ref_um[ENC_COUNT];	/* straight-line reference */
	W	hold_um[ENC_COUNT];	/* speed-integrated travel since the
					   reference (S2_HEADING_FROM_SPEED) */
	INT	hold_sp[ENC_COUNT];	/* setpoints frozen by HOLD */
	INT	sp[ENC_COUNT];		/* ramped setpoints, mm/s */
	PI_STATE pi[ENC_COUNT];
	UW	last_edges[ENC_COUNT];
	UINT	stall_cnt[ENC_COUNT];
} MOTION_CTX;

/* ---------------------------------------------------------------------
 *  Public functions
 * --------------------------------------------------------------------- */
EXPORT UW s2_get_status(void)
{
	return pub_status;
}

EXPORT UW s2_get_odometry(void)
{
	return pub_odometry;
}

EXPORT void s2_get_timing(S2_TIMING *t)
{
	if (t != NULL) {
		*t = timing;		/* diagnostics only: a torn copy is harmless */
	}
}

EXPORT void subsystem2_bind_safety(ID flg_inhibit, UINT inhibit_ptn,
				   ID flg_fault, UINT fault_ptn)
{
	safety_inhibit_ptn = inhibit_ptn;
	safety_fault_ptn   = fault_ptn;
	safety_fault_flg   = flg_fault;
	safety_inhibit_flg = flg_inhibit;	/* last: the task checks this one */
}

EXPORT BOOL s2_safety_is_bound(void)
{
	return ((safety_inhibit_flg > 0) || (safety_fault_flg > 0)) ? TRUE : FALSE;
}

/* ---------------------------------------------------------------------
 *  Helpers
 * --------------------------------------------------------------------- */
LOCAL BOOL inhibit_active(void)
{
	T_RFLG	rflg;
	ID	flg = safety_inhibit_flg;

	if (flg <= 0) {
		return FALSE;
	}
	if (tk_ref_flg(flg, &rflg) != E_OK) {
		return TRUE;			/* cannot read the latch: fail safe */
	}
	return ((rflg.flgptn & safety_inhibit_ptn) != 0U) ? TRUE : FALSE;
}

LOCAL void raise_fault(void)
{
	ID	flg = safety_fault_flg;

	if ((flg > 0) && (safety_fault_ptn != 0U)) {
		(void)tk_set_flg(flg, safety_fault_ptn);	/* one atomic kernel call */
	}
}

LOCAL UW now_ms(void)
{
	SYSTIM	t;

	(void)tk_get_tim(&t);
	return t.lo;
}

LOCAL INT sign_of(INT v)
{
	return (v > 0) ? 1 : ((v < 0) ? -1 : 0);
}

LOCAL W abs_w(W v)
{
	return (v < 0) ? -v : v;
}

LOCAL void reset_loop(MOTION_CTX *c)
{
	UINT	w;

	for (w = 0U; w < ENC_COUNT; w++) {
		c->sp[w] = 0;
		c->pi[w].integ = 0;
		c->stall_cnt[w] = 0U;
	}
}

/* New straight-line reference: both wheels count from here */
LOCAL void reset_heading_ref(MOTION_CTX *c, const W travel_um[])
{
	UINT	w;

	for (w = 0U; w < ENC_COUNT; w++) {
		c->ref_um[w]  = travel_um[w];
		c->hold_um[w] = 0;
	}
}

/* Stop now and require a newer epoch before moving again */
LOCAL void latch_stop(MOTION_CTX *c, UINT state)
{
	motor_stop_all();
	reset_loop(c);
	c->mode = S2_MODE_STOP;
	c->state = state;
	c->active_word = 0U;
	if (c->have_epoch) {
		c->blocked = TRUE;
		c->blocked_epoch = c->newest_epoch;
	}
}

LOCAL BOOL command_acceptable(const MOTION_CTX *c, UINT epoch)
{
	if (!c->have_epoch) {
		return TRUE;
	}
	if (motion_epoch_is_stale(epoch, c->newest_epoch)) {
		return FALSE;
	}
	if (c->blocked && (epoch == c->blocked_epoch)) {
		return FALSE;
	}
	return TRUE;
}

LOCAL void start_move(MOTION_CTX *c, UINT mode, const W travel_um[])
{
	UINT	w;

	for (w = 0U; w < ENC_COUNT; w++) {
		c->start_um[w] = travel_um[w];
		c->hold_sp[w]  = c->sp[w];
	}
	reset_heading_ref(c, travel_um);
	c->done = FALSE;
	c->mode = mode;
}

LOCAL void apply_command(MOTION_CTX *c, UW word, UW t_ms, const W travel_um[])
{
	UINT	mode  = s2_cmd_mode(word);
	UINT	epoch = s2_cmd_epoch(word);
	BOOL	same  = (word == c->active_word) ? TRUE : FALSE;

	c->have_epoch   = TRUE;
	c->newest_epoch = epoch;
	c->blocked      = FALSE;
	c->last_cmd_ms  = t_ms;
	c->cmd_speed    = s2_cmd_speed(word);

	switch (mode) {
	case S2_MODE_DRIVE:
		c->cmd_param = s2_cmd_param_s(word);
		if (c->mode != S2_MODE_DRIVE) {
			start_move(c, S2_MODE_DRIVE, travel_um);
		}
		if (c->cmd_param != 0) {
			reset_heading_ref(c, travel_um);	/* new straight-line reference */
		}
		c->state = S2_STATE_DRIVING;
		break;

	case S2_MODE_TURN:
		c->cmd_param = s2_cmd_param_s(word);
		if (!same) {
			start_move(c, S2_MODE_TURN, travel_um);
		}
		c->state = c->done ? S2_STATE_IDLE : S2_STATE_TURNING;
		break;

	case S2_MODE_DRIVE_DIST:
		c->cmd_param = (INT)s2_cmd_param_u(word);
		if (!same) {
			start_move(c, S2_MODE_DRIVE_DIST, travel_um);
		}
		c->state = c->done ? S2_STATE_IDLE : S2_STATE_DRIVING_DIST;
		break;

	case S2_MODE_HOLD:
		if (c->mode != S2_MODE_HOLD) {
			start_move(c, S2_MODE_HOLD, travel_um);
		}
		c->state = S2_STATE_HOLDING;
		break;

	case S2_MODE_STOP:
	default:				/* 5..7 are not defined: stop */
		c->mode = S2_MODE_STOP;
		c->state = S2_STATE_IDLE;
		reset_loop(c);
		motor_stop_all();
		break;
	}
	c->active_word = word;
}

/* Turn the active command into a target speed for each wheel */
LOCAL void compute_targets(MOTION_CTX *c, const W travel_um[], INT target[])
{
	W	moved_l = travel_um[ENC_LEFT]  - c->start_um[ENC_LEFT];
	W	moved_r = travel_um[ENC_RIGHT] - c->start_um[ENC_RIGHT];
	W	avg_abs = (abs_w(moved_l) + abs_w(moved_r)) / 2;
	INT	v;
	INT	steer;

	target[ENC_LEFT]  = 0;
	target[ENC_RIGHT] = 0;

	switch (c->mode) {
	case S2_MODE_DRIVE:
		steer = c->cmd_param;
		if (steer == 0) {
#if S2_HEADING_FROM_SPEED
			steer = motion_heading_correction(c->hold_um[ENC_LEFT],
							  c->hold_um[ENC_RIGHT]);
#else
			steer = motion_heading_correction(
				travel_um[ENC_LEFT]  - c->ref_um[ENC_LEFT],
				travel_um[ENC_RIGHT] - c->ref_um[ENC_RIGHT]);
#endif
			if (c->cmd_speed < 0) {
				steer = -steer;		/* reversing: correction flips */
			}
		}
		target[ENC_LEFT]  = s2_clamp10(c->cmd_speed + steer);
		target[ENC_RIGHT] = s2_clamp10(c->cmd_speed - steer);
		break;

	case S2_MODE_TURN:
		if (c->done || (avg_abs >= (motion_turn_arc_um(c->cmd_param) - HALF_PULSE_UM))) {
			c->done = TRUE;
			c->state = S2_STATE_IDLE;
			break;
		}
		v = (c->cmd_speed < 0) ? -c->cmd_speed : c->cmd_speed;
		if (v == 0) {
			v = S2_TURN_DEFAULT_MMPS;
		}
		if (c->cmd_param < 0) {
			v = -v;				/* anticlockwise: right wheel forward */
		}
		target[ENC_LEFT]  = v;
		target[ENC_RIGHT] = -v;
		break;

	case S2_MODE_DRIVE_DIST:
		if (c->done || (avg_abs >= (((W)c->cmd_param * UM_PER_MM) - HALF_PULSE_UM))) {
			c->done = TRUE;
			c->state = S2_STATE_IDLE;
			break;
		}
		target[ENC_LEFT]  = c->cmd_speed;
		target[ENC_RIGHT] = c->cmd_speed;
		break;

	case S2_MODE_HOLD:
		target[ENC_LEFT]  = c->hold_sp[ENC_LEFT];
		target[ENC_RIGHT] = c->hold_sp[ENC_RIGHT];
		break;

	case S2_MODE_STOP:
	default:
		break;
	}
}

/* Cyclic handler: interrupt context, only wakes the task */
LOCAL void motion_cyclic_handler(void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(tid_motion);
}

/* ---------------------------------------------------------------------
 *  Creation (called from subsystem2_init / subsystem2_start)
 * --------------------------------------------------------------------- */
EXPORT ER motion_init(void)
{
	T_CMBF	cmbf = {0};
	T_CCYC	ccyc = {0};
	ER	er;

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)CMD_BUF_SIZE;
	cmbf.maxmsz = (INT)sizeof(UW);
	q_motion_cmd = tk_cre_mbf(&cmbf);
	if (q_motion_cmd < E_OK) {
		return q_motion_cmd;
	}

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)motion_cyclic_handler;
	ccyc.cyctim = S2_CYCLE_MS;
	ccyc.cycphs = S2_CYCLE_MS;
	cyc_motion = tk_cre_cyc(&ccyc);
	if (cyc_motion < E_OK) {
		return cyc_motion;
	}

	/* Pins and the interrupt handler; the IRQ itself is enabled by the
	   task, on its own core.  Done here (phase 1) so that other
	   subsystems may register GPIO hooks in their own _init(). */
	er = encoder_init();
	return er;
}

EXPORT ER motion_start(void)
{
	tid_motion = app_start_task(motion_task, S2_PRI_MOTION, S2_MOTION_CORE,
				    "motion_task");
	if (tid_motion < E_OK) {
		return tid_motion;
	}
	return tk_sta_cyc(cyc_motion);
}

/* ---------------------------------------------------------------------
 *  The task
 * --------------------------------------------------------------------- */
EXPORT void motion_task(INT stacd, void *exinf)
{
	MOTION_CTX	c = {0};
	ENC_SNAPSHOT	snap[ENC_COUNT];
	W		travel_um[ENC_COUNT];
	INT		speed[ENC_COUNT];
	INT		target[ENC_COUNT];
	INT		duty[ENC_COUNT];
	UW		t0;
	UW		prev_t0 = 0U;
	UW		t_ms;
	UW		dt;
	UW		late;
	UW		word;
	UW		newest_word = 0U;
	BOOL		have_new;
	BOOL		inhibit;
	INT		n;
	UINT		w;
	W		avg_um;
	W		hold_inc;

	(void)stacd; (void)exinf;

	c.mode  = S2_MODE_STOP;
	c.state = S2_STATE_IDLE;

	CHECK(motor_init(), "motor_init");
	CHECK(encoder_enable_irq(), "encoder_enable_irq");

	while (1) {
		(void)tk_slp_tsk(TMO_FEVR);
		(void)tk_can_wup(TSK_SELF);	/* drop wake-ups from an overrun */

		t0   = encoder_time_us();
		t_ms = now_ms();
		if (timing.cycles != 0U) {
			dt   = t0 - prev_t0;
			late = (dt > CYCLE_US) ? (dt - CYCLE_US) : (CYCLE_US - dt);
			if (late > timing.max_late_us) {
				timing.max_late_us = late;
			}
		}
		prev_t0 = t0;

		/* 4 (first, so moves can start from the current position) */
		for (w = 0U; w < ENC_COUNT; w++) {
			encoder_snapshot(w, &snap[w]);
			travel_um[w] = snap[w].count * S2_UM_PER_PULSE;
			/* read the time AFTER the snapshot: an edge between t0 and
			   the snapshot would otherwise give a negative age, which
			   wraps to a huge UW and reads as speed 0 */
			speed[w] = motion_speed_mmps(snap[w].period_us,
						     encoder_time_us() - snap[w].last_edge_us,
						     snap[w].edges,
						     sign_of(c.sp[w]) != 0 ? sign_of(c.sp[w])
									   : sign_of(motor_get_duty(w)));
			/* speed-integrated travel for the straight-line hold:
			   mm/s x ms = um, right wheel trimmed */
			hold_inc = (W)speed[w] * (W)S2_CYCLE_MS;
			if (w == ENC_RIGHT) {
				hold_inc = (hold_inc * (1000L + S2_HEADING_TRIM_R_PERMILLE)) / 1000L;
			}
			c.hold_um[w] += hold_inc;
		}

		/* 1 + 2: inhibit, then the newest acceptable command */
		inhibit  = inhibit_active();
		have_new = FALSE;
		while ((n = tk_rcv_mbf(q_motion_cmd, &word, TMO_POL)) > 0) {
			timing.cmds_rx++;
			if (inhibit || !command_acceptable(&c, s2_cmd_epoch(word))) {
				timing.cmds_stale++;
				continue;
			}
			newest_word = word;
			have_new = TRUE;
		}

		if (inhibit) {
			if (c.state != S2_STATE_INHIBITED) {
				latch_stop(&c, S2_STATE_INHIBITED);
			}
		} else if (have_new) {
			apply_command(&c, newest_word, t_ms, travel_um);
		} else if ((c.mode != S2_MODE_STOP)
			   && ((t_ms - c.last_cmd_ms) > (UW)S2_CMD_TIMEOUT_MS)) {
			/* 3: command watchdog.  Not latched: any valid command
			   (same epoch is fine) starts again. */
			motor_stop_all();
			reset_loop(&c);
			c.mode = S2_MODE_STOP;
			c.state = S2_STATE_TIMEOUT;
			c.active_word = 0U;
		} else {
			/* nothing new */
		}

		/* 5: targets */
		compute_targets(&c, travel_um, target);

		/* 6: setpoints and PI.  Stopping is immediate (NFR12);
		   speeding up and changing speed is ramped (soft start). */
		for (w = 0U; w < ENC_COUNT; w++) {
			if ((target[w] == 0) || (sign_of(target[w]) == -sign_of(c.sp[w]))) {
				c.sp[w] = (target[w] == 0) ? 0 : motion_ramp(0, target[w], S2_ACCEL_STEP_MMPS);
			} else {
				c.sp[w] = motion_ramp(c.sp[w], target[w], S2_ACCEL_STEP_MMPS);
			}
			duty[w] = motion_pi_step(&c.pi[w], c.sp[w], speed[w]);

			/* stall: driven hard, but the wheel is (almost) not turning.
			   Judged on the measured speed, not on "no new edge": with
			   hundreds of edges per turn, a wheel held by hand still
			   creeps a few edges through the gearbox backlash. */
			if (((duty[w] >= S2_STALL_DUTY_PERMILLE) || (duty[w] <= -S2_STALL_DUTY_PERMILLE))
			    && ((c.sp[w] >= S2_STALL_MIN_MMPS) || (c.sp[w] <= -S2_STALL_MIN_MMPS))
			    && (speed[w] < S2_STALL_SPEED_MMPS) && (speed[w] > -S2_STALL_SPEED_MMPS)) {
				c.stall_cnt[w]++;
			} else if ((c.sp[w] == 0) || (c.stall_cnt[w] == 0U)) {
				c.stall_cnt[w] = 0U;
			} else {
				c.stall_cnt[w]--;	/* one noisy reading only delays it */
			}
			c.last_edges[w] = snap[w].edges;
		}
		if ((c.stall_cnt[ENC_LEFT] >= (UINT)STALL_CYCLES)
		    || (c.stall_cnt[ENC_RIGHT] >= (UINT)STALL_CYCLES)) {
			latch_stop(&c, S2_STATE_STALLED);
			raise_fault();
			duty[ENC_LEFT]  = 0;
			duty[ENC_RIGHT] = 0;
		}

		/* 7: the Safety Monitor may have latched since step 1 */
		if (inhibit_active() || (c.mode == S2_MODE_STOP)) {
			duty[ENC_LEFT]  = 0;
			duty[ENC_RIGHT] = 0;
		}
		for (w = 0U; w < ENC_COUNT; w++) {
			if (duty[w] != 0) {
				encoder_set_direction(w, duty[w]);
			}
			motor_set_duty(w, duty[w]);
		}

		/* 8: publish */
		avg_um = (travel_um[ENC_LEFT] + travel_um[ENC_RIGHT]) / 2;
		pub_odometry = s2_odo_pack((UH)((UW)(avg_um / UM_PER_MM) & S2_MASK_16),
					   (UH)(t_ms & S2_MASK_16));
		pub_status = s2_status_pack(speed[ENC_LEFT], speed[ENC_RIGHT],
					    c.state, c.done, c.newest_epoch);

		dt = encoder_time_us() - t0;
		if (dt > timing.max_exec_us) {
			timing.max_exec_us = dt;
		}
		timing.cycles++;

#if S2_LOG_EVERY_CYCLES > 0
		if ((timing.cycles % S2_LOG_EVERY_CYCLES) == 0U) {
			tm_printf((UB*)"S2,%u,%u,%d,%d,%d,%d,%d,%d,%d\n",
				  t_ms, c.state,
				  c.sp[ENC_LEFT], speed[ENC_LEFT], duty[ENC_LEFT],
				  c.sp[ENC_RIGHT], speed[ENC_RIGHT], duty[ENC_RIGHT],
				  (INT)(avg_um / UM_PER_MM));
		}
#endif
	}
}
