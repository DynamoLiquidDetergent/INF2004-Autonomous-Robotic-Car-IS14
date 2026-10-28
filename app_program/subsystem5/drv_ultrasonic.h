/*
 *	drv_ultrasonic.h
 *	Subsystem 5 driver: HC-SR04 ranging (GP16 TRIG, GP17 ECHO) and the
 *	SG90 scan servo (GP12).  Called ONLY from the Obstacle Detection task,
 *	which is the single owner of these pins (TEAM.md section 2/3).
 *
 *	How the echo is timed (no interrupt, no busy-wait, no shared GPIO IRQ):
 *	GP17 is PWM slice 0, channel B.  The slice is put in "gated by B" mode
 *	with a 1 MHz clock, so its counter counts microseconds ONLY while the
 *	echo pin is high.  After the echo window the task reads the counter.
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include "../app.h"

#define S5_PIN_SERVO		12	/* Robo Pico servo port 1, PWM slice 6 A */
#define S5_PIN_TRIG		16	/* Grove 4 */
#define S5_PIN_ECHO		17	/* Grove 4, PWM slice 0 B. 5 V module: DIVIDE THE ECHO to 3.3 V */

/* 1 if the servo turns the wrong way round for the mounting, so that
   index 0 (0 deg) is the robot's RIGHT */
#ifndef S5_SERVO_REVERSED
#define S5_SERVO_REVERSED	0
#endif

#define US_PING_GAP_MS		65	/* HC-SR04: more than 60 ms between pings */
#define US_ECHO_WAIT_MS		45	/* covers the ~38 ms "nothing there" pulse */

typedef enum {
	US_OK		= 0,	/* echo pulse measured, distance valid */
	US_OUT_OF_RANGE	= 1,	/* pulse longer than 400 cm: nothing in range (normal) */
	US_NO_ECHO	= 2	/* no pulse at all: sensor missing or dead (maybe a fault) */
} US_STATUS;

typedef struct {
	UB	status;		/* US_STATUS */
	UW	echo_us;	/* measured pulse width, microseconds */
	UH	dist_cm;	/* valid when status == US_OK */
} US_RESULT;

/* Which parts of the hardware the driver may touch. A part that is not
   switched on is NEVER driven: its pins stay at their power-up state
   (input, no function). Bring the board up one part at a time. */
#define DRV_PART_SERVO	(1u << 0)	/* GP12 PWM -> servo */
#define DRV_PART_TRIG	(1u << 1)	/* GP16 output -> HC-SR04 TRIG */
#define DRV_PART_ECHO	(1u << 2)	/* GP17 input  <- HC-SR04 ECHO (through the divider!) */
#define DRV_PART_ALL	(DRV_PART_SERVO | DRV_PART_TRIG | DRV_PART_ECHO)

/* Release the PWM block from reset and set up the requested parts.
   Safe to call more than once (parts accumulate). Returns E_OK. */
IMPORT ER	drv_s5_init(UW parts);

/* Fire ONE trigger pulse and nothing else (no echo timing). For phase 3. */
IMPORT void	drv_us_trigger_only(void);

/* Move the servo to 0..180 degrees (does not wait for it to arrive) */
IMPORT void	drv_servo_set_deg(UINT deg);

/* Trigger one measurement and return. Task context only: it sleeps
   (tk_dly_tsk) about 65 ms, so the whole call takes 65-75 ms. */
IMPORT US_RESULT	drv_us_ping(void);

/* Raw readings for tests */
IMPORT UW	drv_pwm_reset_done(void);		/* non-zero once PWM is out of reset */
IMPORT UW	drv_servo_pulse_us(void);		/* current servo pulse width, us */

#endif /* DRV_ULTRASONIC_H */
