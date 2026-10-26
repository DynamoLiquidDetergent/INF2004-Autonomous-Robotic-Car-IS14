/*
 *	drv_motor.c
 *	Subsystem 2 -- DC motor driver using the RP2040 PWM block.
 *
 *	Uses the BSP helpers in <bsp/libbsp.h> (pwm_set_pin, pwm_set_wrap,
 *	pwm_set_cc, pwm_set_enabled).  Called only from the Motion Control
 *	task, so no locking is needed: pwm_set_cc() does a read-modify-write
 *	of a register shared by channels A and B of the same slice, and both
 *	channels of slices 4 and 5 belong to this driver alone.
 */

#include "drv_motor.h"
#include "subsystem2.h"
#include <tk/syslib.h>		/* in_w/out_w, DI/EI, EnableInt */

/* ---------------------------------------------------------------------
 *  PWM timing.  clk_sys = 125 MHz, slice divider left at its reset value
 *  of 1.0, so the counter runs at 125 MHz.  TOP = 6249 gives
 *  125 MHz / 6250 = 20 kHz: above hearing range and well within what the
 *  Robo Pico motor driver accepts.
 * --------------------------------------------------------------------- */
#define MOTOR_PWM_TOP		6249U
#define MOTOR_PWM_STEPS		(MOTOR_PWM_TOP + 1U)

/* RP2040 RESETS block, atomic clear alias (write 1 to take out of reset) */
#define MOTOR_RESETS_CLR	(RESETS_BASE + 0x3000U)

typedef struct {
	UINT	pin_a;
	UINT	pin_b;
	BOOL	invert;
} MOTOR_PINS;

LOCAL const MOTOR_PINS motor_pins[MOTOR_COUNT] = {
	{ S2_PIN_MOTOR_L_A, S2_PIN_MOTOR_L_B, S2_MOTOR_L_INVERT },
	{ S2_PIN_MOTOR_R_A, S2_PIN_MOTOR_R_B, S2_MOTOR_R_INVERT },
};

LOCAL INT motor_duty[MOTOR_COUNT] = { 0, 0 };

/*
 * The kernel only releases the PWM block from reset when the physical
 * timer is enabled (config.h USE_PTMR), which it is not.  Release it here.
 * Only the "clear" alias is written, so if another subsystem (e.g. the
 * servo in subsystem 5) has already released it, nothing is reset.
 */
LOCAL void motor_release_pwm_reset(void)
{
	if ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U) {
		out_w(MOTOR_RESETS_CLR, RESETS_RESET_PWM);
		while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U) {
			/* wait for the block to come out of reset */
		}
	}
}

LOCAL UW motor_permille_to_cc(INT permille)
{
	if (permille <= 0) {
		return 0U;
	}
	if (permille >= MOTOR_DUTY_MAX) {
		return MOTOR_PWM_STEPS;		/* CC > TOP: output stays high */
	}
	return ((UW)permille * MOTOR_PWM_STEPS) / (UW)MOTOR_DUTY_MAX;
}

EXPORT ER motor_init(void)
{
	UINT	m;
	ER	er = E_OK;

	motor_release_pwm_reset();

	for (m = 0U; m < MOTOR_COUNT; m++) {
		/* Outputs low before the pins are switched to PWM */
		(void)pwm_set_cc(motor_pins[m].pin_a, 0U);
		(void)pwm_set_cc(motor_pins[m].pin_b, 0U);
		(void)pwm_set_wrap(motor_pins[m].pin_a, MOTOR_PWM_TOP);

		er = pwm_set_pin(motor_pins[m].pin_a);
		if (er == E_OK) {
			er = pwm_set_pin(motor_pins[m].pin_b);
		}
		if (er == E_OK) {
			er = pwm_set_enabled(motor_pins[m].pin_a, TRUE);
		}
		if (er != E_OK) {
			return er;
		}
		motor_duty[m] = 0;
	}
	return E_OK;
}

EXPORT void motor_set_duty(UINT motor, INT duty_permille)
{
	INT	d = duty_permille;
	UW	cc;

	if (motor >= MOTOR_COUNT) {
		return;
	}
	if (d > MOTOR_DUTY_MAX) {
		d = MOTOR_DUTY_MAX;
	} else if (d < -MOTOR_DUTY_MAX) {
		d = -MOTOR_DUTY_MAX;
	}
	motor_duty[motor] = d;

	if (motor_pins[motor].invert) {
		d = -d;
	}

	if (d >= 0) {
		cc = motor_permille_to_cc(d);
		(void)pwm_set_cc(motor_pins[motor].pin_b, 0U);
		(void)pwm_set_cc(motor_pins[motor].pin_a, cc);
	} else {
		cc = motor_permille_to_cc(-d);
		(void)pwm_set_cc(motor_pins[motor].pin_a, 0U);
		(void)pwm_set_cc(motor_pins[motor].pin_b, cc);
	}
}

EXPORT void motor_stop_all(void)
{
	UINT	m;

	for (m = 0U; m < MOTOR_COUNT; m++) {
		motor_set_duty(m, 0);
	}
}

EXPORT INT motor_get_duty(UINT motor)
{
	return (motor < MOTOR_COUNT) ? motor_duty[motor] : 0;
}
