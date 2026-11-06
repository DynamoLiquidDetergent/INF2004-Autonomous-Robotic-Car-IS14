#ifndef S4_TERRAIN_H
#define S4_TERRAIN_H
#include "s4_processing.h"

typedef enum
{
    S4_INITIALISING=0, S4_CALIBRATING=1, S4_LEVEL=2,
    S4_CLIMBING=3, S4_DESCENDING=4, S4_FAULT=5
} s4_terrain_state_t;
typedef enum
{
    S4_STATIONARY=0, S4_ACCELERATING=1, S4_TURNING=2,
    S4_MOTION_CLIMBING=3, S4_MOTION_DESCENDING=4,
    S4_UNCLASSIFIED=7
} s4_motion_class_t;
typedef struct
{
    float enter_deg, exit_deg, impact_g, acceleration_g, turning_deg_s;
    unsigned int debounce_samples;
    uint32_t calibration_timeout_us, traversal_timeout_us;
} s4_terrain_config_t;
typedef struct
{
    s4_terrain_config_t config;
    uint8_t state_motion; /* terrain bits 0..2; exclusive motion bits 3..5 */
    unsigned int transition_count;
    uint32_t calibration_start_us, traversal_start_us;
    bool impact_armed, saw_descent;
    bool impact_event, hump_event, traversal_aborted; /* one call only */
} s4_terrain_t;
s4_terrain_config_t s4_terrain_defaults(void);
void s4_terrain_init(s4_terrain_t *t,uint32_t now);
/* sensor_ok includes the caller's I/O and missing-sample timeout checks.
 * sample_fresh prevents reusing old angles or rearming impacts on a data gap. */
void s4_terrain_step(s4_terrain_t *t,const s4_orientation_t *o,
                     bool calibrated,bool sensor_ok,bool sample_fresh,uint32_t now);
uint8_t s4_terrain_state(const s4_terrain_t *t);
uint8_t s4_motion_class(const s4_terrain_t *t);
void s4_set_motion(s4_terrain_t *t,uint8_t motion);
const char *s4_terrain_state_name(uint8_t state);
const char *s4_motion_class_name(uint8_t motion);
#endif
