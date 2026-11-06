#ifndef S4_INTEGRATION_H
#define S4_INTEGRATION_H
#include "subsystem4.h"

/* S5 init binds S2 odometry and its existing IMU fault bit, before tasks start.
 * No queue/event group is created here. Safety Monitor owns fault handling. */
IMPORT ER subsystem4_connect_motion(ID fault_group,UINT imu_fault_bit);
IMPORT ER s4_integration_publish(const s4_height_t *height,BOOL failed,UW completed_humps);
/* One aligned 32-bit snapshot: -1 unavailable, otherwise rounded highest mm.
 * S5 passes it through its existing telemetry clamp (0..254; 255 unavailable).
 * This getter is a local API, not a new message/wire representation. */
IMPORT INT subsystem4_highest_hump_mm(void);
IMPORT UW subsystem4_completed_humps(void);
#endif
