/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion Control (motors, wheel encoders, PID).
 *	Student ID: 2500461 (Aaron Alison Dsilva).
 *
 *	Public interface of subsystem 2.  Other subsystems include it as
 *	"../subsystem2/subsystem2.h".
 *
 *	This header has two parts:
 *	  A. Motion Control: the real subsystem (IF-10, IF-11, IF-12, IF-16,
 *	     IF-17 in TEAM.md section 5).
 *	  B. The template's example producer (IF-01, IF-02).  Kept unchanged
 *	     because subsystems 3 and 4 still use its objects.  Remove it once
 *	     they no longer include sem_slots / mpf_samples / SAMPLE_MSG.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

/* =====================================================================
 *  A.  MOTION CONTROL
 * ===================================================================== */

/* ---------------------------------------------------------------------
 *  Task and timing (TEAM.md section 4; agreed at Tribunal 1:
 *  Safety Monitor 5, Motion Control 6, Vehicle Controller 7).
 * --------------------------------------------------------------------- */
#define S2_PRI_MOTION		6	/* just below the Safety Monitor */
#define S2_MOTION_CORE		TP_PRC2	/* core 1; encoder IRQ runs on the same core */
#define S2_CYCLE_MS		20	/* PID loop period: 50 Hz (NFR9) */
#define S2_CMD_TIMEOUT_MS	100	/* no command for this long -> stop */

/* ---------------------------------------------------------------------
 *  Pins (TEAM.md section 2)
 * --------------------------------------------------------------------- */
#define S2_PIN_MOTOR_L_A	8U	/* M1A, PWM slice 4 A */
#define S2_PIN_MOTOR_L_B	9U	/* M1B, PWM slice 4 B */
#define S2_PIN_MOTOR_R_A	10U	/* M2A, PWM slice 5 A */
#define S2_PIN_MOTOR_R_B	11U	/* M2B, PWM slice 5 B */
#define S2_PIN_ENC_L_A		14U	/* left encoder A  (moved from GP0) */
#define S2_PIN_ENC_L_B		15U	/* left encoder B  (moved from GP1) */
#define S2_PIN_ENC_R_A		4U	/* right encoder A (Grove 3) */
#define S2_PIN_ENC_R_B		5U	/* right encoder B (Grove 3) */

/* ---------------------------------------------------------------------
 *  Hardware settings to CHECK ON THE BENCH (testing/subsystem2, S2-M01..)
 * --------------------------------------------------------------------- */
#define S2_MOTOR_L_INVERT	FALSE	/* TRUE if the left wheel runs backwards */
#define S2_MOTOR_R_INVERT	FALSE
#define S2_ENC_L_INVERT		FALSE	/* only matters with S2_ENC_QUADRATURE */
#define S2_ENC_R_INVERT		FALSE
#define S2_ENC_QUADRATURE	0	/* 1 if channel B is a true quadrature output */
#define S2_ENC_PULL_UP		1	/* pull-ups for open-collector encoder outputs */
#define S2_ENC_MIN_EDGE_US	500U	/* edges closer than this are noise */

/* ---------------------------------------------------------------------
 *  Geometry: MEASURE AND UPDATE (A2).  Distance per pulse is derived.
 * --------------------------------------------------------------------- */
#define S2_WHEEL_DIAMETER_MM	65	/* wheel diameter */
#define S2_ENC_PULSES_PER_REV	20	/* rising edges of A per wheel turn */
#define S2_TRACK_WIDTH_MM	120	/* centre-to-centre distance of the wheels */

/* ---------------------------------------------------------------------
 *  IF-10  Motion command  (Vehicle Controller -> Motion Control)
 *
 *  One 32-bit word (minimum-bit requirement, Tribunal 1 curveball):
 *
 *   bit 31..27  26..23   22..13           12..3          2..0
 *      unused   epoch   param (10 b)    speed (10 b)    mode
 *               (4 b)                   signed mm/s     (3 b)
 *
 *  mode       speed (signed mm/s)        param
 *  ---------  -------------------------  ---------------------------------
 *  STOP       ignored                    ignored
 *  DRIVE      forward speed, -511..511   steering, signed mm/s added to the
 *                                        left wheel and taken from the
 *                                        right one (+ = turn right).
 *                                        0 = hold a straight line (NFR10)
 *  TURN       wheel speed (sign ignored, signed degrees, + = clockwise
 *             0 = default)               (right), -180..180, turn on the spot
 *  DRIVE_DIST speed, sign = direction    distance in mm, 0..1023
 *  HOLD       ignored                    ignored: keep the current speed
 *                                        steady (e.g. over a hump, Buddy 4)
 *
 *  epoch: the Vehicle Controller adds 1 (mod 16) on every pause, fault
 *  and reset.  Motion Control drops commands older than the newest one it
 *  has seen, and after an inhibit or stall accepts only a NEWER epoch.
 *
 *  Send: tk_snd_mbf(q_motion_cmd, &word, sizeof(UW), TMO_POL), at least
 *  every S2_CMD_TIMEOUT_MS while the robot should move.  Sending the same
 *  word again only keeps the robot moving; it does not restart a TURN or
 *  DRIVE_DIST.  Send STOP (or a new epoch) between two identical moves.
 * --------------------------------------------------------------------- */
#define S2_MODE_STOP		0U
#define S2_MODE_DRIVE		1U
#define S2_MODE_TURN		2U
#define S2_MODE_DRIVE_DIST	3U
#define S2_MODE_HOLD		4U

#define S2_CMD_MODE_SHIFT	0U
#define S2_CMD_SPEED_SHIFT	3U
#define S2_CMD_PARAM_SHIFT	13U
#define S2_CMD_EPOCH_SHIFT	23U
#define S2_MASK_3		0x7U
#define S2_MASK_4		0xFU
#define S2_MASK_10		0x3FFU
#define S2_SIGN_10		0x200U
#define S2_SPEED_LIMIT		511	/* largest value a 10-bit signed field holds */
#define S2_EPOCH_MOD		16U

IMPORT ID	q_motion_cmd;		/* IF-10 message buffer, owned by S2 */

/* Sign-extend a 10-bit two's complement field */
Inline INT s2_sext10(UW v)
{
	UW	x = v & S2_MASK_10;
	return (INT)(x ^ S2_SIGN_10) - (INT)S2_SIGN_10;
}

Inline INT s2_clamp10(INT v)
{
	if (v > S2_SPEED_LIMIT) {
		return S2_SPEED_LIMIT;
	}
	if (v < -S2_SPEED_LIMIT) {
		return -S2_SPEED_LIMIT;
	}
	return v;
}

Inline UW s2_cmd_pack(UINT mode, INT speed_mmps, INT param, UINT epoch)
{
	return  (((UW)mode & S2_MASK_3) << S2_CMD_MODE_SHIFT)
	      | (((UW)s2_clamp10(speed_mmps) & S2_MASK_10) << S2_CMD_SPEED_SHIFT)
	      | (((UW)param & S2_MASK_10) << S2_CMD_PARAM_SHIFT)
	      | (((UW)epoch & S2_MASK_4) << S2_CMD_EPOCH_SHIFT);
}

Inline UINT s2_cmd_mode(UW w)   { return (UINT)((w >> S2_CMD_MODE_SHIFT) & S2_MASK_3); }
Inline INT  s2_cmd_speed(UW w)  { return s2_sext10(w >> S2_CMD_SPEED_SHIFT); }
Inline INT  s2_cmd_param_s(UW w){ return s2_sext10(w >> S2_CMD_PARAM_SHIFT); }
Inline UINT s2_cmd_param_u(UW w){ return (UINT)((w >> S2_CMD_PARAM_SHIFT) & S2_MASK_10); }
Inline UINT s2_cmd_epoch(UW w)  { return (UINT)((w >> S2_CMD_EPOCH_SHIFT) & S2_MASK_4); }

/* ---------------------------------------------------------------------
 *  IF-16  Motion status  (Motion Control -> Vehicle Controller, telemetry)
 *
 *   bit 31..28  27..24   23     22..20    19..10          9..0
 *      unused   epoch   done   state    speed R (10 b)   speed L (10 b)
 *                                       signed mm/s      signed mm/s
 *
 *  Read with s2_get_status().  The word is written in one store, so it is
 *  always consistent.  "done" = the last TURN / DRIVE_DIST has finished.
 *  "epoch" = newest command epoch accepted.
 * --------------------------------------------------------------------- */
#define S2_STATE_IDLE		0U
#define S2_STATE_DRIVING	1U
#define S2_STATE_TURNING	2U
#define S2_STATE_DRIVING_DIST	3U
#define S2_STATE_HOLDING	4U
#define S2_STATE_INHIBITED	5U	/* Safety Monitor stop latch is set */
#define S2_STATE_STALLED	6U	/* stall fault: wheels driven but not turning */
#define S2_STATE_TIMEOUT	7U	/* stopped: no command for S2_CMD_TIMEOUT_MS */

#define S2_ST_SPDL_SHIFT	0U
#define S2_ST_SPDR_SHIFT	10U
#define S2_ST_STATE_SHIFT	20U
#define S2_ST_DONE_SHIFT	23U
#define S2_ST_EPOCH_SHIFT	24U

Inline UW s2_status_pack(INT spd_l, INT spd_r, UINT state, BOOL done, UINT epoch)
{
	return  (((UW)s2_clamp10(spd_l) & S2_MASK_10) << S2_ST_SPDL_SHIFT)
	      | (((UW)s2_clamp10(spd_r) & S2_MASK_10) << S2_ST_SPDR_SHIFT)
	      | (((UW)state & S2_MASK_3) << S2_ST_STATE_SHIFT)
	      | (((UW)(done ? 1U : 0U)) << S2_ST_DONE_SHIFT)
	      | (((UW)epoch & S2_MASK_4) << S2_ST_EPOCH_SHIFT);
}

Inline INT  s2_status_speed_l(UW w) { return s2_sext10(w >> S2_ST_SPDL_SHIFT); }
Inline INT  s2_status_speed_r(UW w) { return s2_sext10(w >> S2_ST_SPDR_SHIFT); }
Inline UINT s2_status_state(UW w)   { return (UINT)((w >> S2_ST_STATE_SHIFT) & S2_MASK_3); }
Inline BOOL s2_status_done(UW w)    { return (((w >> S2_ST_DONE_SHIFT) & 1U) != 0U) ? TRUE : FALSE; }
Inline UINT s2_status_epoch(UW w)   { return (UINT)((w >> S2_ST_EPOCH_SHIFT) & S2_MASK_4); }

IMPORT UW	s2_get_status(void);

/* ---------------------------------------------------------------------
 *  IF-17  Odometry  (Motion Control -> IMU/Terrain via the VC, or direct)
 *
 *   bit 31..16                       15..0
 *      distance, mm (uint16, wraps)  timestamp, ms (uint16, wraps)
 *
 *  Distance = average forward travel of both wheels since boot.  Both
 *  fields wrap; always use differences: (UH)(d_now - d_before).  The
 *  timestamp is the low 16 bits of tk_get_tim() ms, the same clock every
 *  task can read, so the IMU task can line its samples up with it.
 *  Updated every cycle (20 ms), written in one store.
 * --------------------------------------------------------------------- */
#define S2_ODO_DIST_SHIFT	16U
#define S2_MASK_16		0xFFFFU

Inline UW s2_odo_pack(UH dist_mm, UH time_ms)
{
	return ((UW)dist_mm << S2_ODO_DIST_SHIFT) | (UW)time_ms;
}
Inline UH s2_odo_dist_mm(UW w) { return (UH)((w >> S2_ODO_DIST_SHIFT) & S2_MASK_16); }
Inline UH s2_odo_time_ms(UW w) { return (UH)(w & S2_MASK_16); }

IMPORT UW	s2_get_odometry(void);

/* ---------------------------------------------------------------------
 *  IF-11 / IF-12  Safety Monitor link (objects owned by subsystem 5)
 *
 *  Subsystem 5 calls this ONCE in subsystem5_init(), after creating its
 *  event flags:
 *    flg_inhibit, inhibit_ptn : latched motor inhibit (IF-11).  While any
 *                               bit of inhibit_ptn is set, Motion Control
 *                               keeps PWM at 0 and discards commands.
 *    flg_fault,   fault_ptn   : fault group (IF-12).  Motion Control sets
 *                               fault_ptn (bit 0 agreed) on a motor stall.
 *  Pass 0 as an ID to leave that link unused.  Until it is called, Motion
 *  Control runs without a Safety Monitor (bench testing).
 * --------------------------------------------------------------------- */
IMPORT void	subsystem2_bind_safety(ID flg_inhibit, UINT inhibit_ptn,
				       ID flg_fault, UINT fault_ptn);

/* ---------------------------------------------------------------------
 *  Shared GPIO interrupt (IO_IRQ_BANK0).  Subsystem 2 owns the handler.
 *  Another subsystem can receive edges on its own pin (e.g. subsystem 5,
 *  HC-SR04 echo on GP17) by registering a hook, ideally in its _init():
 *      s2_gpio_hook_register(17U, S2_GPIO_EDGE_RISE | S2_GPIO_EDGE_FALL,
 *                            echo_hook);
 *  The hook runs in INTERRUPT context on the Motion Control core: keep it
 *  to a few instructions (store the timestamp, set an event flag).
 *    gpio   : pin number
 *    events : S2_GPIO_EDGE_* bits that fired
 *    t_us   : 1 MHz timestamp of the interrupt (wraps every ~71 min)
 * --------------------------------------------------------------------- */
#define S2_GPIO_EDGE_FALL	0x4U
#define S2_GPIO_EDGE_RISE	0x8U

typedef void (*S2_GPIO_HOOK)(UINT gpio, UINT events, UW t_us);

IMPORT ER	s2_gpio_hook_register(UINT gpio, UINT events, S2_GPIO_HOOK cb);

/* ---------------------------------------------------------------------
 *  Timing evidence (NFR1, NFR9, Week 13): measured by the task itself.
 * --------------------------------------------------------------------- */
typedef struct {
	UW	cycles;		/* control cycles run */
	UW	max_exec_us;	/* longest cycle (WCET observed) */
	UW	max_late_us;	/* worst release jitter: period error, +/- */
	UW	cmds_rx;	/* commands received */
	UW	cmds_stale;	/* commands dropped as stale / blocked epoch */
} S2_TIMING;

IMPORT void	s2_get_timing(S2_TIMING *t);

/* Private to subsystem 2 */
IMPORT BOOL	s2_safety_is_bound(void);
IMPORT ER	motion_init(void);
IMPORT ER	motion_start(void);
IMPORT void	motion_task(INT stacd, void *exinf);

/* =====================================================================
 *  B.  TEMPLATE EXAMPLE PRODUCER (unchanged; still used by S3 and S4)
 * ===================================================================== */

#define S2_KEEP_EXAMPLE		1	/* 0 = do not start producer_task */

#define S2_PRI_PRODUCER		14	/* priority band 14-15 (TEAM.md) */
#define S2_PERIOD_MS		300	/* one sample every 300 ms */

/* ---------------------------------------------------------------------
 *  IF-01 data format: one sample, sent to subsystem 3's mailbox.
 *  A mailbox message MUST start with a T_MSG header: the kernel links
 *  messages together through it, so the data itself is never copied.
 * --------------------------------------------------------------------- */
typedef struct {
	T_MSG	hdr;		/* used by the kernel, do not touch */
	UW	seq;		/* sample sequence number */
	UW	time_ms;	/* system time when the sample was taken */
	INT	value;		/* the "measurement", 0..100 */
	INT	prc;		/* processor (core) that produced it: 1 or 2 */
} SAMPLE_MSG;

#define S2_POOL_COUNT	4	/* at most 4 samples in flight */

/* IF-02: the receiver of a sample gives its memory block back to
   mpf_samples (tk_rel_mpf) and then a slot back to sem_slots
   (tk_sig_sem, count 1).  Nothing else may use these two objects. */
IMPORT ID	sem_slots;
IMPORT ID	mpf_samples;

/* Private to subsystem 2 */
IMPORT void	producer_task(INT stacd, void *exinf);
IMPORT INT	s2_make_value(UW seq);

#endif /* SUBSYSTEM2_H */
