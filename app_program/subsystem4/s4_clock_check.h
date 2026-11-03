#ifndef S4_CLOCK_CHECK_H
#define S4_CLOCK_CHECK_H
#include <stdint.h>

enum { S4_CLOCK_PENDING=0, S4_CLOCK_CONSISTENT=1, S4_CLOCK_BAD=2 };
typedef struct
{
    uint32_t reference_us, reference_ms;
    uint8_t status;
} s4_clock_check_t;

void s4_clock_check_init(s4_clock_check_t *check,uint32_t us,uint32_t ms);
void s4_clock_check_step(s4_clock_check_t *check,uint32_t us,uint32_t ms);
const char *s4_clock_status_name(uint8_t status);
#endif
