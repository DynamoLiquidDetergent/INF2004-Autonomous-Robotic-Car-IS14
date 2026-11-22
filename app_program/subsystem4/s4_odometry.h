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
/* Wheelbase-aware height track: one entry per mm of travel. The track must
 * hold at least one wheelbase of history, so wheelbase_mm < this length. */
#define S4_HEIGHT_TRACK_LEN 192U
typedef struct
{
    /* Development guards; confirm against S2 resolution/rate/speed and S5. */
    uint32_t max_age_us, max_gap_us, max_candidate_ms;
    uint16_t max_step_mm;
    /* Qualified IMU endpoint policy; separate from encoder continuity. */
    uint32_t max_imu_gap_us,quality_receipt_ms;
    uint16_t max_imu_step_mm;
    s4_odometry_t quality_previous;
    bool quality_have_previous;
    s4_odometry_t previous;
    uint32_t previous_receipt_ms, start_ms;
    float previous_sine, current_mm, peak_mm, last_mm, highest_mm;
    uint32_t interpolated_samples, max_interpolated_us, pending_samples;
    uint8_t reason;
    /* Published reason saved when a candidate starts; restored if that
     * candidate turns out to be a false start (never reached CLIMBING). */
    uint8_t idle_reason;
    /* 0 = legacy sum of ds*sin(pitch). Otherwise the drive-axle-to-caster
     * distance: front height = rear height + L*sin(pitch), and the rear wheel
     * is where the front contact was one wheelbase of travel earlier. */
    uint16_t wheelbase_mm;
    uint32_t track_mm;          /* travel since the candidate started */
    float front_mm;             /* front contact height at track_mm */
    float track[S4_HEIGHT_TRACK_LEN];
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
/* Quality is explicit: I/O errors, saturation and impacts must never be bridged.
 * WAIT/DYNAMIC may defer an interval, using only qualified endpoint angles,
 * within a bounded IMU endpoint window and continuous fresh odometry. Counts are local
 * diagnostic metadata, not additions to the agreed packed wire interface. */
enum { S4_IMU_GOOD=0, S4_IMU_WAIT, S4_IMU_DYNAMIC, S4_IMU_FAULT };
void s4_height_step_quality(s4_height_t *height,const s4_odometry_t *odometry,
                    const s4_orientation_t *orientation,uint8_t quality,
                    bool timebase_consistent,uint32_t now_us,uint32_t now_ms,
                    uint8_t terrain_state,float level_exit_deg,
                    bool completed,bool aborted);
const char *s4_height_reason_name(uint8_t reason);
#endif
