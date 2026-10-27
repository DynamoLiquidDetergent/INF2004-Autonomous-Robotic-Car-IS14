#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H
#include "../app.h"

#include "s4_config.h"

/* Transitional ABI: S3 main still includes these names. Remove only when
 * its owner has migrated the example consumer, not before. */
#define S4_SUMMARY_MAX 48
#define FLG_BATCH     0x0002U
#define FLG_ALARM     0x0001U
IMPORT ID mbf_summary;
IMPORT ID flg_events;
IMPORT UW s4_lines_received;
IMPORT ID tid_imu_terrain;
IMPORT void imu_terrain_task(INT stacd, void *exinf);
IMPORT void s4_drain_example(void);

#endif
