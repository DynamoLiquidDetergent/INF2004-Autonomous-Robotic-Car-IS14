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
/* TRUE while the right wheel is on a hump (from a 3 mm rise until the height is
 * done or dropped). The height is only kept if both wheels drive forward the whole
 * time (a reversing wheel = NOT_FORWARD), so S5 keeps its steering gentle then. */
IMPORT BOOL subsystem4_hump_active(void);
/* Demo rubric "stop when lifted / overturned": subsystem 5's handling_logic runs in the
 * IMU task on every sample. Returns the kind of the last event (HL_PICKED_UP or
 * HL_OVERTURNED, 0 none yet) and the number of events so far in *count. */
IMPORT UINT subsystem4_handling(UW *count);
IMPORT void s4_handling_publish(UINT kind);
#endif
