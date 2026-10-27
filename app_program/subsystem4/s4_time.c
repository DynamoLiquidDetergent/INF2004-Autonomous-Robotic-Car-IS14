#include "s4_time.h"

uint32_t s4_delay_to_release(uint32_t now, uint32_t *release,
                             uint32_t period, uint32_t *missed)
{
    int32_t remaining = (int32_t)(*release - now);
    if ((remaining < 0) && (period != 0U))
    {
        uint32_t skipped = (now - *release) / period + 1U;
        *release += skipped * period;
        *missed += skipped;
        remaining = (int32_t)(*release - now);
    }
    return (remaining > 0) ? (uint32_t)remaining : 0U;
}
