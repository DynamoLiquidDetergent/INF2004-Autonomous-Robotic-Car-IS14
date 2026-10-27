/*
 *	subsystem3.h
 *	Subsystem 3 -- IR perception (line tracking and barcode decoding) Student ID: 2500172.
 *
 *	Public interface of subsystem 3.  Other subsystems include it as
 *	"../subsystem3/subsystem3.h".
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

/* -- QUEUE IDs -- */

/* Line position data sent to the Vehicle Controller every 20 ms */
IMPORT ID	q_line_data;

/* Raw barcode sensor readings (one every 2 ms) sent to the Barcode Decode Task */
IMPORT ID	q_gp28_samples;

/* Decoded letter (A-Z) sent to the Vehicle Controller after a successful barcode read */
IMPORT ID	q_decoded_char;

/* Movement command sent to the Vehicle Controller after a barcode is decoded and mapped */
IMPORT ID	q_barcode_cmd;

/* Character-to-movement mapping received from the WiFi/MQTT subsystem */
IMPORT ID	mbx_char_map;

/* Safety monitor event-flag group (owned by Buddy 5). Subsystem 3 sets this flag when the line is lost; the safety monitor reads and clears it */
IMPORT ID	flg_fault_i14;
#define S3_FAULT_BIT		(1u << 2)	/* Buddy 3's bit in the flag group */

/* Pack line position into one byte: zone in bits 7-5, junction in bit 4, and line-lost in bit 3 */
typedef uint8_t LINE_DATA_MSG;

#define LINE_ZONE_SHIFT		5u			                /* Number of bits to shift the zone left */
#define LINE_ZONE_MASK		(0x07u << LINE_ZONE_SHIFT)	/* Mask to isolate zone bits */
#define LINE_JUNCTION_BIT	(1u << 4)		            /* Bit 4 indicates a junction */
#define LINE_LOST_BIT		(1u << 3)		            /* Bit 3 indicates line lost */
#define LINE_ZONE_CENTRE	4u			                /* Zone value for "centred" */

/* Pack a zone number (0-7), junction flag, and line-lost flag into one byte */
static inline LINE_DATA_MSG line_data_pack(uint8_t zone, bool jn, bool lost)
{
	return (LINE_DATA_MSG)(
		((zone & 0x07u) << LINE_ZONE_SHIFT) | /* Put zone in bits 7-5 */
		(jn ? LINE_JUNCTION_BIT : 0u) |
		(lost ? LINE_LOST_BIT : 0u));
}

/* Get the zone number (0-7) from a packed line message */
static inline uint8_t line_data_zone(LINE_DATA_MSG m)
{
	return (uint8_t)((m & LINE_ZONE_MASK) >> LINE_ZONE_SHIFT);
}

/* Get the junction flag from a packed line message */
static inline bool line_data_junction(LINE_DATA_MSG m)
{
	return (m & LINE_JUNCTION_BIT) != 0u;
}

/* Get the line-lost flag from a packed line message */
static inline bool line_data_lost(LINE_DATA_MSG m)
{
	return (m & LINE_LOST_BIT) != 0u;
}

/* One GP28 barcode sensor reading packed into 16 bits: upper 12 bits are the ADC value (0-4095), lower 4 bits the rolling slot (0-15). */
typedef uint16_t GP28_SAMPLE_MSG;

#define GP28_ADC_SHIFT		4u			                /* ADC value lives in the upper 12 bits */
#define GP28_ADC_MASK		(0x0FFFu << GP28_ADC_SHIFT)	/* Mask to isolate ADC bits */
#define GP28_SLOT_MASK		0x000Fu			            /* Mask to isolate the lower 4 bits */

/* Pack a 12-bit ADC reading and 4-bit slot number into one 16-bit value */
static inline GP28_SAMPLE_MSG gp28_pack(uint16_t adc12, uint8_t slot4)
{
	return (GP28_SAMPLE_MSG)(
		((adc12 & 0x0FFFu) << GP28_ADC_SHIFT) | /* Put ADC value in bits 15-4 */
		(slot4 & 0x0Fu));
}

/* Get the ADC value (0-4095) from a packed sample */
static inline uint16_t gp28_adc(GP28_SAMPLE_MSG m)
{
	return (uint16_t)((m & GP28_ADC_MASK) >> GP28_ADC_SHIFT);
}

/* Get the rolling slot number (0-15) from a packed sample */
static inline uint8_t gp28_slot(GP28_SAMPLE_MSG m)
{
	return (uint8_t)(m & GP28_SLOT_MASK);
}

/* Movement commands sent after decoding a barcode and looking up its letter
 * in the character-to-movement map.
 */
typedef uint8_t BARCODE_CMD;

#define BARCODE_CMD_NONE	0u	/* No valid barcode read, or no map yet */
#define BARCODE_CMD_FORWARD	1u	/* Go straight through the junction */
#define BARCODE_CMD_UTURN	2u	/* Turn 180 degrees and go back */
#define BARCODE_CMD_TURN_LEFT	3u	/* Turn left at the junction */
#define BARCODE_CMD_TURN_RIGHT	4u	/* Turn right at the junction */

/* Four-letter mapping received from WiFi/MQTT: each letter identifies a movement command */
typedef struct {
	uint8_t ch_forward;	/* Letter for go forward */
	uint8_t ch_uturn;	/* Letter for U-turn */
	uint8_t ch_left;	/* Letter for turn left */
	uint8_t ch_right;	/* Letter for turn right */
} CHAR_MAP_MSG;

/* Calibration and barcode-detection parameters */
#define S3_CALIB_TIMEOUT_MS	5000u	    /* Calibration timeout in milliseconds */
#define S3_CALIB_SAMPLES	64u	        /* Number of samples collected for calibration */
#define BC_MIN_NARROW_SAMPLES	4u	    /* Minimum consecutive samples for a narrow element */
#define BC_WIDE_RATIO_X10	15u	        /* Minimum wide/narrow ratio, multiplied by 10 */
#define BC_MAX_WINDOW_SAMPLES	800u	/* Maximum samples in the barcode detection window */

/* Initialize subsystem 3 */
IMPORT void subsystem3_init(void);

/* Calibrate the GP28 IR sensors. Returns true on success, false on timeout */
IMPORT bool subsystem3_run_calibration(void);

/* Enable or disable subsystem 3 bypass mode */
IMPORT void subsystem3_set_bypass(bool active);

#endif
