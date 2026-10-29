/*
 *	s3_private.h
 *	Functions shared ONLY between subsystem 3's own files.
 *	Other subsystems include subsystem3.h, never this file.
 */

#ifndef S3_PRIVATE_H
#define S3_PRIVATE_H

#include "subsystem3.h"

/* ---------------------------------------------------------------------
 *  DEBUG SWITCH for the Commit 4 Part B video.
 *  1 = in a NORMAL build (make -j8, not TEST=1) the robot drives forward
 *      over the barcode by itself, stops after a decode, then drives back
 *      over it in reverse.  Also turns on the A,B,C,D test mapping and the
 *      "BARCODE: ch=.. cmd=.." prints in barcode_decode.c.
 *  MUST be 0 when committing.
 * --------------------------------------------------------------------- */
#define S3_BC_DRIVE_DEMO    1 //was 0

/* ---- line_sensor.c ---- */
IMPORT ER   line_sensor_init(void);     /* phase 1: create objects      */
IMPORT ER   line_sensor_start(void);    /* phase 2: start the task      */

/* Classify one GP28 reading as dark (bar / line) or light, with
   hysteresis, using the calibrated GP28 range and sensor polarity. */
IMPORT bool line_sensor_gp28_dark(uint16_t adc, bool was_dark);

/* Stop / restart sending GP28 samples to q_gp28_samples.
   Used by the tests to inject a hand-made barcode. */
IMPORT void line_sensor_pause_gp28(bool pause);

/* Count of GP28 samples successfully put into q_gp28_samples (INT-02). */
IMPORT UW   line_sensor_gp28_sent(void);

/* An ADC value that is certainly dark / certainly light (for tests). */
IMPORT uint16_t line_sensor_gp28_test_level(bool dark);

/* ---- barcode_decode.c ---- */
IMPORT ER   barcode_decode_start(void); /* phase 2: start the task      */

/* true if the mapping is four distinct letters A-Z (IF char map) */
IMPORT bool s3_char_map_valid(const CHAR_MAP_MSG *m);

/* Number of valid letters decoded since boot */
IMPORT UW   barcode_decode_count(void);

/* ---- s3_drive_demo.c (debug only, see S3_BC_DRIVE_DEMO) ---- */
IMPORT ER   s3_drive_demo_start(void);  /* phase 2; does nothing when off */

#endif /* S3_PRIVATE_H */
