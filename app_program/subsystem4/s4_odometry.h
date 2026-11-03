#ifndef S4_ODOMETRY_H
#define S4_ODOMETRY_H
#include "s4_processing.h"

enum { S4_ODOM_US32=0, S4_ODOM_MS16=1 };
typedef struct
{
    uint16_t distance_mm;
    uint32_t timestamp;
    uint8_t clock_format;
    bool valid;
    bool forward;
} s4_odometry_t;

/* Local callback ABI, not a packed wire format. Bind before any tasks start.
 * Provider must return a bounded, coherent snapshot and validate its freshness
 * and forward direction. A missing/stale/reversing source must not be valid. */
typedef bool (*s4_read_odometry_fn)(s4_odometry_t *out);

enum
{
    S4_HEIGHT_OK=0, S4_HEIGHT_NO_ODOMETRY, S4_HEIGHT_TIMEBASE,
    S4_HEIGHT_IMU, S4_HEIGHT_BASELINE, S4_HEIGHT_STALE,
    S4_HEIGHT_GAP, S4_HEIGHT_REVERSE, S4_HEIGHT_JUMP,
    S4_HEIGHT_FORMAT, S4_HEIGHT_ABORTED, S4_HEIGHT_NO_MOVEMENT
};
typedef struct
{
    /* Development guards; confirm against S2 resolution/rate/speed and S5. */
    uint32_t max_age_us, max_gap_us, max_candidate_ms;
    uint16_t max_step_mm;
    s4_odometry_t previous;
    uint32_t previous_receipt_ms, start_ms;
    float previous_sine, current_mm, peak_mm, last_mm, highest_mm;
    uint8_t reason;
    bool have_previous, baseline_ready, active, chain_valid;
    bool saw_climbing, moved, odometry_valid, last_valid, highest_valid;
} s4_height_t;

void s4_height_init(s4_height_t *height);
/* Proposed IF-17: high 16 bits = cumulative mm; low 16 = kernel ms.
 * The caller supplies confirmed forward direction separately. */
void s4_odometry_decode_word(uint32_t word,bool forward,s4_odometry_t *out);
void s4_height_step(s4_height_t *height,const s4_odometry_t *odometry,
                    const s4_orientation_t *orientation,bool imu_fresh,
                    bool timebase_consistent,uint32_t now_us,uint32_t now_ms,
                    uint8_t terrain_state,float level_exit_deg,
                    bool completed,bool aborted);
const char *s4_height_reason_name(uint8_t reason);
#endif
