/*
 *	subsystem3.h
 *	Subsystem 3 -- IR Perception: Line Tracking, Junction Detection, Barcode Decode. Student ID: 2500172
 *
 *	Public interface of subsystem 3.  Other subsystems include it as
 *	"../subsystem3/subsystem3.h".
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include <stdint.h>
#include <stdbool.h>

// task priorities for the two S3 tasks (line_sensor_task, barcode_decode_task)
#define S3_LINE_TASK_PRI    16   // line_sensor_task  – high priority, 20 ms / 2 ms cycles
#define S3_BC_TASK_PRI      17   // barcode_decode_task – event-driven on i16 queue


// IF-05: line position data sent to the Vehicle Controller every 20 ms
IMPORT ID q_line_data;

// IF-06: raw barcode sensor readings (one every 2 ms) going to the Barcode Decode Task (internal to S3)
IMPORT ID q_gp28_samples;

// IF-07: the decoded letter (A–Z) sent to the Vehicle Controller after a successful barcode read
IMPORT ID q_decoded_char;

// IF-07: the MOVEMENT command (turn left, turn right) sent to the Vehicle Controller after a barcode is decoded and looked up in the map
IMPORT ID q_barcode_cmd;

// IF (MQTT): character-to-movement mapping arriving from WiFi/MQTT. Buddy 1 writes into this queue; Buddy 3 reads from it
IMPORT ID mbx_char_map;

// IF-08: Safety Monitor event-flag group (owned by Buddy 5). Buddy 3 sets S3_FAULT_BIT on debounced LINE_LOST outside bypass; Safety Monitor reads and clears it — S3 never clears this flag
IMPORT ID flg_fault_i14;

// Bit inside flg_fault_i14 that belongs to S3. (1u << 2) = bit 2; confirm with S5 owner
#define S3_FAULT_BIT   (1u << 2)

// line_data_pack() packs the zone (0–7), junction flag and line-lost flag into one byte. The following constants define the bit layout of that byte.
typedef uint8_t LINE_DATA_MSG;

#define LINE_ZONE_SHIFT    5u                          // zone occupies bits 7–5
#define LINE_ZONE_MASK     (0x07u << LINE_ZONE_SHIFT)  // mask to isolate zone
#define LINE_JUNCTION_BIT  (1u << 4)                   // bit 4 = junction flag
#define LINE_LOST_BIT      (1u << 3)                   // bit 3 = line-lost flag
#define LINE_ZONE_CENTRE   4u                          // zone value = "centred"

// Pack zone (0–7), junction flag and line-lost flag into one byte
static inline LINE_DATA_MSG line_data_pack(uint8_t zone, bool jn, bool lost)
{
    return (LINE_DATA_MSG)(
        ((zone & 0x07u) << LINE_ZONE_SHIFT) |   // zone in bits 7–5
        (jn   ? LINE_JUNCTION_BIT : 0u)     |   // bit 4 if junction
        (lost ? LINE_LOST_BIT     : 0u)         // bit 3 if lost
    );
}

// Getters – unpack individual fields from a packed LINE_DATA_MSG byte
static inline uint8_t line_data_zone    (LINE_DATA_MSG m) { return (m & LINE_ZONE_MASK) >> LINE_ZONE_SHIFT; }
static inline bool    line_data_junction(LINE_DATA_MSG m) { return (m & LINE_JUNCTION_BIT) != 0u; }
static inline bool    line_data_lost    (LINE_DATA_MSG m) { return (m & LINE_LOST_BIT)     != 0u; }

// gp28_pack() packs a 12-bit ADC reading and a 4-bit slot counter into one 16-bit value. The following constants define the bit layout of that value.
typedef uint16_t GP28_SAMPLE_MSG;

#define GP28_ADC_SHIFT  4u                           // ADC value lives in bits 15–4
#define GP28_ADC_MASK   (0x0FFFu << GP28_ADC_SHIFT)  // mask to isolate those 12 bits
#define GP28_SLOT_MASK  0x000Fu                      // mask to isolate bits 3–0

// Pack a 12-bit ADC reading and a 4-bit slot counter into one 16-bit value
static inline GP28_SAMPLE_MSG gp28_pack(uint16_t adc12, uint8_t slot4)
{
    return (GP28_SAMPLE_MSG)(
        ((adc12 & 0x0FFFu) << GP28_ADC_SHIFT) |   /* ADC in bits 15–4 */
        (slot4  & 0x0Fu)                           /* slot in bits 3–0 */
    );
}

// Getters – unpack each field from a packed GP28_SAMPLE_MSG value
static inline uint16_t gp28_adc (GP28_SAMPLE_MSG m) { return (m & GP28_ADC_MASK) >> GP28_ADC_SHIFT; }
static inline uint8_t  gp28_slot(GP28_SAMPLE_MSG m) { return (uint8_t)(m & GP28_SLOT_MASK); }

// barcode_cmd_pack() packs a movement command (forward, left, right, U-turn) into one byte. The following constants define the bit layout of that byte.
typedef uint8_t BARCODE_CMD;
#define BARCODE_CMD_NONE        0u   // no valid barcode read (or no map yet)
#define BARCODE_CMD_FORWARD     1u   // go straight through the junction
#define BARCODE_CMD_UTURN       2u   // turn 180 degrees and go back
#define BARCODE_CMD_TURN_LEFT   3u   /* turn left at the junction              */
#define BARCODE_CMD_TURN_RIGHT  4u   /* turn right at the junction             */

// character mapping msg
typedef struct {
    uint8_t ch_forward;   /* which letter means go forward  */
    uint8_t ch_uturn;     /* which letter means U-turn      */
    uint8_t ch_left;      /* which letter means turn left   */
    uint8_t ch_right;     /* which letter means turn right  */
} CHAR_MAP_MSG;


// Calibration: 5 s timeout, 64 samples collected
#define S3_CALIB_TIMEOUT_MS    5000u
#define S3_CALIB_SAMPLES       64u

/* Barcode decode tuning */
#define BC_MIN_NARROW_SAMPLES  4u    // minimum consecutive samples for a narrow element
#define BC_WIDE_RATIO_X10      15u   // wide/narrow ratio threshold × 10
#define BC_MAX_WINDOW_SAMPLES  800u  // abandon a read if more samples arrive before 29 elems


// Called by usermain() phase 1: create all kernel objects
IMPORT ER subsystem3_init(void);

// Called by usermain() phase 2: start all tasks
IMPORT ER subsystem3_start(void);

// Run the GP28 calibration routine.
// Returns true on success, false if the 5 s timeout expired.
IMPORT bool subsystem3_run_calibration(void);

// Set/clear the bypass flag. While active, LINE_LOST is suppressed (robot intentionally off-line).
IMPORT void subsystem3_set_bypass(bool active);


/* =====================================================================
 *  LEGACY TEMPLATE INTERFACE (example "Consumer")
 *  Subsystem 2 (task_producer.c) and subsystem 4 (task_monitor.c) still
 *  use these.  Kept only so the whole project builds.  Remove this block
 *  and task_consumer.c once S2/S4 drop their example code.
 * ===================================================================== */
#define S3_PRI_CONSUMER   18    /* legacy consumer task priority          */
#define S3_BATCH_SIZE     5     /* one summary line to S4 per 5 samples    */

typedef struct {
    INT  min;           /* smallest sample value seen   */
    INT  max;           /* largest sample value seen    */
    W    sum;           /* sum of all values            */
    UW   received;      /* number of samples received   */
} APP_STATS;

IMPORT ID        mbx_samples;   /* IF-01: S2 -> S3 sample mailbox      */
IMPORT ID        mtx_stats;     /* IF-04: guards app_stats             */
IMPORT APP_STATS app_stats;     /* IF-04: read by S4 under mtx_stats   */

IMPORT void consumer_task(INT stacd, void *exinf);   /* task_consumer.c */

#endif // SUBSYSTEM3_H
