/*
 *	drv_ultrasonic.c
 *	Subsystem 5 driver: HC-SR04 echo timing and SG90 scan servo.
 *	See drv_ultrasonic.h.  Written against the RP2040 datasheet and the
 *	register names in include/sys/sysdepend/cpu/rp2040/sysdef.h.
 *
 *	NOT YET TESTED ON HARDWARE.  Bring-up order (testing/subsystem5):
 *	servo moves -> raw echo counter changes with distance -> ruler test.
 */

#include "drv_ultrasonic.h"
#include "obstacle_profile.h"
#include <sys/machine.h>

#define SLICE_OF(pin)		(((pin) >> 1) & 7)
#define SLICE_BASE(n)		(PWM_BASE + 0x14 * (n))
#define SERVO_SLICE		SLICE_OF(S5_PIN_SERVO)		/* 6 */
#define ECHO_SLICE		SLICE_OF(S5_PIN_ECHO)		/* 0 */

/* PWM clock is clk_sys = 125 MHz (cpu_clock.c). Divide by 125 -> 1 MHz. */
#define PWM_DIV_1MHZ		(125u << 4)	/* INT in bits 11:4, FRAC in 3:0 */
#define SERVO_TOP		19999u		/* 20 ms period -> 50 Hz */
#define PWM_CSR_DIVMODE_LEVEL	(1u << 4)	/* count only while B pin is high */
#define PWM_CSR_EN		(1u << 0)

#define TRIG_LOOPS		400u		/* ~16-40 us at 125 MHz: needs >= 10 us */
#define ECHO_MAX_US		24000u		/* 400 cm is 23200 us; anything longer = out of range */

LOCAL volatile UW	s_servo_pulse_us;
LOCAL UW		s_last_trigger_ms;
LOCAL BOOL		s_have_trigger;
LOCAL UW		s_parts;		/* parts initialised so far */

LOCAL UW now_ms(void)
{
	SYSTIM	t;

	tk_get_otm(&t);
	return t.lo;
}

EXPORT UW drv_pwm_reset_done(void)
{
	return in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM;
}

EXPORT UW drv_servo_pulse_us(void)
{
	return s_servo_pulse_us;
}

EXPORT ER drv_s5_init(UW parts)
{
	UW	guard;

	/* The kernel leaves the PWM block in reset (USE_PTMR = 0 in
	   config.h, so hw_setting.c skips it). Release it here. Releasing it
	   twice (Motion Control does too) is harmless. This touches no pin. */
	clr_w(RESETS_RESET, RESETS_RESET_PWM);
	for (guard = 0; guard < 100000u; guard++) {
		if (drv_pwm_reset_done()) {
			break;
		}
	}
	if (!drv_pwm_reset_done()) {
		return E_SYS;
	}

	/* TRIG: plain output, low */
	if ((parts & DRV_PART_TRIG) && !(s_parts & DRV_PART_TRIG)) {
		gpio_set_pin(S5_PIN_TRIG, GPIO_MODE_OUT);
		gpio_set_val(S5_PIN_TRIG, 0);
		s_parts |= DRV_PART_TRIG;
	}

	/* ECHO: PWM slice 0 channel B as an input gate.  Pull-down so an
	   unplugged sensor reads low (= no echo), not noise. */
	if ((parts & DRV_PART_ECHO) && !(s_parts & DRV_PART_ECHO)) {
		pwm_set_pin(S5_PIN_ECHO);
		set_w(GPIO(S5_PIN_ECHO), GPIO_PDE);
		out_w(SLICE_BASE(ECHO_SLICE) + PWM_CHx_CSR, 0);
		out_w(SLICE_BASE(ECHO_SLICE) + PWM_CHx_DIV, PWM_DIV_1MHZ);
		out_w(SLICE_BASE(ECHO_SLICE) + PWM_CHx_TOP, 0xFFFFu);
		out_w(SLICE_BASE(ECHO_SLICE) + PWM_CHx_CTR, 0);
		s_parts |= DRV_PART_ECHO;
	}

	/* Servo: 50 Hz, pulse width sets the angle. Start at 90 degrees. */
	if ((parts & DRV_PART_SERVO) && !(s_parts & DRV_PART_SERVO)) {
		pwm_set_pin(S5_PIN_SERVO);
		out_w(SLICE_BASE(SERVO_SLICE) + PWM_CHx_CSR, 0);
		out_w(SLICE_BASE(SERVO_SLICE) + PWM_CHx_DIV, PWM_DIV_1MHZ);
		pwm_set_wrap(S5_PIN_SERVO, SERVO_TOP);
		s_servo_pulse_us = obs_angle_to_pulse_us(90);
		pwm_set_cc(S5_PIN_SERVO, s_servo_pulse_us);
		pwm_set_enabled(S5_PIN_SERVO, TRUE);
		s_parts |= DRV_PART_SERVO;
	}
	return E_OK;
}

LOCAL void trigger_pulse(void)
{
	volatile UW	i;

	gpio_set_val(S5_PIN_TRIG, 1);
	for (i = 0; i < TRIG_LOOPS; i++) {
		/* short busy-wait: a few microseconds, far shorter than one
		   kernel tick or a context switch; pre-emption only makes the
		   pulse longer, which the sensor accepts */
	}
	gpio_set_val(S5_PIN_TRIG, 0);
}

EXPORT void drv_us_trigger_only(void)
{
	if (s_parts & DRV_PART_TRIG) {
		trigger_pulse();
	}
}

EXPORT void drv_servo_set_deg(UINT deg)
{
	if (!(s_parts & DRV_PART_SERVO)) {
		return;			/* servo not switched on: do not touch it */
	}
	if (deg > 180u) {
		deg = 180u;
	}
#if S5_SERVO_REVERSED
	deg = 180u - deg;
#endif
	s_servo_pulse_us = obs_angle_to_pulse_us(deg);
	pwm_set_cc(S5_PIN_SERVO, s_servo_pulse_us);
}

EXPORT US_RESULT drv_us_ping(void)
{
	US_RESULT	r;
	UW		elapsed, us;
	UW		csr_run = PWM_CSR_DIVMODE_LEVEL | PWM_CSR_EN;

	r.status = US_NO_ECHO;
	r.echo_us = 0;
	r.dist_cm = 0;

	if ((s_parts & (DRV_PART_TRIG | DRV_PART_ECHO)) != (DRV_PART_TRIG | DRV_PART_ECHO)) {
		return r;		/* sensor not switched on: no pin is touched */
	}

	/* HC-SR04 needs more than 60 ms between measurements */
	if (s_have_trigger) {
		elapsed = now_ms() - s_last_trigger_ms;
		if (elapsed < US_PING_GAP_MS) {
			tk_dly_tsk((RELTIM)(US_PING_GAP_MS - elapsed));
		}
	}

	/* Arm the echo counter at 0, then fire the trigger pulse */
	out_w(SLICE_BASE(ECHO_SLICE) + PWM_CHx_CSR, 0);
	out_w(SLICE_BASE(ECHO_SLICE) + PWM_CHx_CTR, 0);
	out_w(SLICE_BASE(ECHO_SLICE) + PWM_CHx_CSR, csr_run);

	s_last_trigger_ms = now_ms();
	s_have_trigger = TRUE;
	trigger_pulse();

	/* The echo pulse ends within ~38 ms even with nothing in front */
	tk_dly_tsk(US_ECHO_WAIT_MS);

	out_w(SLICE_BASE(ECHO_SLICE) + PWM_CHx_CSR, 0);
	us = in_w(SLICE_BASE(ECHO_SLICE) + PWM_CHx_CTR) & 0xFFFFu;
	r.echo_us = us;

	if (us == 0) {
		r.status = US_NO_ECHO;
	} else if (us > ECHO_MAX_US) {
		r.status = US_OUT_OF_RANGE;
	} else {
		r.dist_cm = obs_echo_us_to_cm(us);
		r.status = (r.dist_cm > 400) ? US_OUT_OF_RANGE : US_OK;
	}
	return r;
}
