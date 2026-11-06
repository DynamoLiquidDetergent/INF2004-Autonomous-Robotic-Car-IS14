#ifndef DRV_MOTION_SENSOR_H
#define DRV_MOTION_SENSOR_H
#include <stdbool.h>
#include "s4_types.h"
#include <stdint.h>
typedef int (*s4_transfer_fn)(void *context, uint8_t address, uint8_t reg,
                              uint8_t *bytes, s4_size_t count, bool read);
typedef struct { s4_transfer_fn transfer; void *context; } s4_bus_t;
typedef struct
{
    float accel_g[3];
    float magnetic_gauss[3];
    bool accel_fresh;
    bool magnetic_fresh;
    bool saturated;
} s4_sample_t;
int s4_sensor_init(const s4_bus_t *bus);
int s4_sensor_read(const s4_bus_t *bus, s4_sample_t *sample);
int s4_board_bus_init(s4_bus_t *bus);
uint32_t s4_board_time_us(void);
#endif
