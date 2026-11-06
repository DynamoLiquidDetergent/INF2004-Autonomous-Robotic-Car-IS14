#ifndef S4_BENCH_STOP_H
#define S4_BENCH_STOP_H
#include <stdint.h>
enum { S4_BENCH_CONTINUE,S4_BENCH_DISTANCE,S4_BENCH_HUMP,
       S4_BENCH_LIMIT,S4_BENCH_TIMEOUT };
typedef struct {
    uint32_t start_ms,start_humps;
    uint16_t start_mm;
    uint8_t kind,reason;
} s4_bench_stop_t;
void s4_bench_stop_init(s4_bench_stop_t *s,uint8_t kind,uint32_t ms,
                        uint16_t mm,uint32_t humps);
uint8_t s4_bench_stop_step(s4_bench_stop_t *s,uint32_t ms,uint16_t mm,uint32_t humps);
const char *s4_bench_stop_reason(uint8_t reason);
#endif
