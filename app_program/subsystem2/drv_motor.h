/*
 *	drv_motor.h
 *	Subsystem 2 -- DC motor driver (Robo Pico motor outputs).
 *
 *	Private to subsystem 2: only the Motion Control task calls these
 *	functions, because it is the only task allowed to write motor PWM
 *	(TEAM.md section 3, interface rule "only Motion Control writes PWM").
 *
 *	Pins (TEAM.md section 2):
 *	  left  motor  M1A = GP8,  M1B = GP9   (PWM slice 4, channels A/B)
 *	  right motor  M2A = GP10, M2B = GP11  (PWM slice 5, channels A/B)
 *
 *	Direction is set by which pin of the pair carries the PWM:
 *	  forward  A = PWM, B = 0
 *	  reverse  A = 0,   B = PWM
 *	  coast    A = 0,   B = 0
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "../app.h"

#define MOTOR_LEFT		0U
#define MOTOR_RIGHT		1U
#define MOTOR_COUNT		2U

/* Duty is given in permille: -1000 (full reverse) .. +1000 (full forward) */
#define MOTOR_DUTY_MAX		1000

IMPORT ER	motor_init(void);
IMPORT void	motor_set_duty(UINT motor, INT duty_permille);
IMPORT void	motor_stop_all(void);
IMPORT INT	motor_get_duty(UINT motor);

#endif /* DRV_MOTOR_H */
