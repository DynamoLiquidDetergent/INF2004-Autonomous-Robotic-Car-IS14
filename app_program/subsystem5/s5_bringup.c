/*
 *	s5_bringup.c
 *	Subsystem 5 -- step-by-step hardware bring-up (phases 1 to 4,
 *	see s5_phase.h).  Each phase touches only the pins listed there, so
 *	nothing is driven until you have wired and checked it.
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"

#if (S5_PHASE >= 1) && (S5_PHASE <= 4)

/* Servo sweep limits for phase 2, in degrees. Start narrow so the servo
   cannot hit its mechanical stops; widen after you have watched it. */
#define S5_SWEEP_MIN		60
#define S5_SWEEP_MAX		120
#define S5_SWEEP_STEP		5
#define S5_SWEEP_STEP_MS	100

EXPORT void s5_bringup_task(INT stacd, void *exinf)
{
	UINT	deg;
	UW	n = 0;
	US_RESULT r;
	UW	min_us = 0xFFFFFFFFu, max_us = 0;

	(void)deg; (void)n; (void)r; (void)min_us; (void)max_us;	/* each phase uses a few */
	tm_printf((UB*)"\ns5 bring-up PHASE %d\n", S5_PHASE);

#if S5_PHASE == 1
	drv_s5_init(0);			/* releases PWM reset only; no pin touched */
	for (;;) {
		tm_printf((UB*)"s5 phase 1: alive %d s, PWM out of reset = %d\n",
			  (INT)n++, (INT)(drv_pwm_reset_done() != 0));
		tk_dly_tsk(1000);
	}

#elif S5_PHASE == 2
	drv_s5_init(DRV_PART_SERVO);
	drv_servo_set_deg(90);
	tm_printf((UB*)"s5 phase 2: servo at 90 deg (pulse %d us). Fit the horn now. Sweep starts in 5 s.\n",
		  (INT)drv_servo_pulse_us());
	tk_dly_tsk(5000);
	for (;;) {
		for (deg = S5_SWEEP_MIN; deg <= S5_SWEEP_MAX; deg += S5_SWEEP_STEP) {
			drv_servo_set_deg(deg);
			tm_printf((UB*)"s5 phase 2: %3d deg  pulse %d us\n", (INT)deg, (INT)drv_servo_pulse_us());
			tk_dly_tsk(S5_SWEEP_STEP_MS);
		}
		for (deg = S5_SWEEP_MAX; deg >= S5_SWEEP_MIN; deg -= S5_SWEEP_STEP) {
			drv_servo_set_deg(deg);
			tm_printf((UB*)"s5 phase 2: %3d deg  pulse %d us\n", (INT)deg, (INT)drv_servo_pulse_us());
			tk_dly_tsk(S5_SWEEP_STEP_MS);
		}
	}

#elif S5_PHASE == 3
	drv_s5_init(DRV_PART_SERVO | DRV_PART_TRIG);
	drv_servo_set_deg(90);
	tm_printf((UB*)"s5 phase 3: TRIG pulse on GP%d twice a second. ECHO must stay UNCONNECTED to the Pico.\n",
		  S5_PIN_TRIG);
	for (;;) {
		drv_us_trigger_only();
		tm_printf((UB*)"s5 phase 3: trigger %d\n", (INT)n++);
		tk_dly_tsk(500);
	}

#else /* S5_PHASE == 4 */
	drv_s5_init(DRV_PART_ALL);
	drv_servo_set_deg(90);
	tm_printf((UB*)"s5 phase 4: raw echo readings. Expect about 58 us per cm.\n");
	for (;;) {
		r = drv_us_ping();
		if (r.status == US_OK) {
			if (r.echo_us < min_us) min_us = r.echo_us;
			if (r.echo_us > max_us) max_us = r.echo_us;
		}
		tm_printf((UB*)"s5 phase 4: #%d echo %5d us  status %d  = %d cm   (min %d, max %d us)\n",
			  (INT)n++, (INT)r.echo_us, (INT)r.status, (INT)r.dist_cm,
			  (INT)((min_us == 0xFFFFFFFFu) ? 0 : min_us), (INT)max_us);
	}
#endif
}

#endif /* phases 1-4 */
