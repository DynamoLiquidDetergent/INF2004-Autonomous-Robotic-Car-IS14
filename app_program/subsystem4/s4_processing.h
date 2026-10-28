#ifndef S4_PROCESSING_H
#define S4_PROCESSING_H
#include "drv_motion_sensor.h"
typedef struct
{
    unsigned int calibration_samples;
    float calibration_spread_g;
    float gravity_tolerance_g;
    float filter_alpha;
    float tilt_warning_deg;
    float magnetic_min_gauss, magnetic_max_gauss;
    float magnetic_bias[3], magnetic_scale[3];
    bool magnetic_calibrated;
    float max_turn_rate_deg_s;
} s4_processing_config_t;
typedef struct
{
    s4_processing_config_t config;
    unsigned int count;
    float sum[3], minimum[3], maximum[3], filtered[3];
    float pitch_zero, roll_zero;
    float previous_heading;
    uint32_t previous_heading_us;
    bool calibrated, filter_ready, have_heading;
} s4_processor_t;
typedef struct
{
    float pitch_deg, roll_deg, heading_deg, turn_rate_deg_s;
    float dynamic_g;
    bool orientation_valid, heading_valid, turn_rate_valid, tilt_warning;
} s4_orientation_t;
s4_processing_config_t s4_processing_defaults(void);
void s4_processor_init(s4_processor_t *p, const s4_processing_config_t *config);
bool s4_process(s4_processor_t *p, const s4_sample_t *sample,
                 uint32_t time_us, s4_orientation_t *out);
#endif
