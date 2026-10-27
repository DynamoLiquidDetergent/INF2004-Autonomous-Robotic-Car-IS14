#ifndef S4_TIME_H
#define S4_TIME_H
#include <stdint.h>
/* All comparisons assume intervals shorter than half the counter range. */
uint32_t s4_delay_to_release(uint32_t now, uint32_t *release,
                             uint32_t period, uint32_t *missed);
#endif
