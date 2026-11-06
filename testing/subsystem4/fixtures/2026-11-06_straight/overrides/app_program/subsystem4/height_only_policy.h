#ifndef S4_HEIGHT_ONLY_POLICY_H
#define S4_HEIGHT_ONLY_POLICY_H
#include <stdint.h>
#define H4_HEALTHY 1U
#define H4_LEVEL 2U
#define H4_FAULT 4U
enum { H4_WAIT,H4_RUN,H4_STOP };
enum { H4_NONE,H4_HUMP,H4_LIMIT,H4_TIMEOUT,H4_FAULT_STOP,H4_INPUT,H4_NOTREADY };
typedef struct { uint32_t boot_ms,start_ms,ready_ms,start_humps;uint16_t start_mm;
    uint8_t phase,reason,ready; } h4_policy_t;
void h4_init(h4_policy_t *p,uint32_t ms);
void h4_step(h4_policy_t *p,uint32_t ms,uint32_t flags,uint16_t mm,uint32_t humps);
const char *h4_reason(uint8_t reason);
uint32_t h4_snapshot(uint32_t ms,uint32_t flags);
uint32_t h4_live_flags(uint32_t snapshot,uint32_t ms,int running);
uint32_t h4_inputs(int calibrated,int clock_ok,int sensor_alive,int odometry_ok,int level_ready,int fault);
#endif
