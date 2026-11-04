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
 *	  6  Vehicle Controller v1: follows the line, stands still while Obstacle
 *	     Detection scans or the Safety Monitor's latch is set, then drives on.
 *	     No bypass. Needs S3_BC_DRIVE_DEMO 0 (s3_private.h): only one task may
 *	     send motion commands.
 *	     After the first scan (obstacle within 50 cm) the car moves to a gap of
 *	     about 17 cm (creeps forward above 20 cm, backs away below 15 cm) and
 *	     scans again from there.
 *	  7  Vehicle Controller does the bypass (later).
 */

#ifndef S5_PHASE_H
#define S5_PHASE_H

#ifndef S5_PHASE
#define S5_PHASE	2
#endif

#endif /* S5_PHASE_H */
