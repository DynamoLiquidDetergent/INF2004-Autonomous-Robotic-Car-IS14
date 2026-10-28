/*
 *	s5_phase.h
 *	Subsystem 5 bring-up phase.  CHANGE THIS ONE NUMBER, rebuild, flash.
 *
 *	Go up one phase at a time. Do not move on until the phase's check in
 *	testing/subsystem5/BRINGUP.md passes and you have logged it.
 *
 *	  1  Nothing wired. Only prints. Releases the PWM block from reset.
 *	     Pins touched: NONE.
 *	  2  Servo only (GP12). Centre, then a small slow sweep.
 *	     Pins touched: GP12.
 *	  3  Servo + HC-SR04 TRIG (GP16). Fires a trigger pulse twice a second.
 *	     ECHO is NOT connected and GP17 is not touched.
 *	     Pins touched: GP12, GP16.
 *	  4  Servo + TRIG + ECHO (GP17, through the divider). Prints the raw
 *	     echo time and distance. No scanning.
 *	     Pins touched: GP12, GP16, GP17.
 *	  5  Full Obstacle Detection scan on its own (no Vehicle Controller).
 *	  6  Obstacle Detection driven by the Vehicle Controller (later).
 */

#ifndef S5_PHASE_H
#define S5_PHASE_H

#ifndef S5_PHASE
#define S5_PHASE	1
#endif

#endif /* S5_PHASE_H */
