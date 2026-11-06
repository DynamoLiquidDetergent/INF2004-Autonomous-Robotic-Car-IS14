#include "s4_clock_check.h"

void s4_clock_check_init(s4_clock_check_t *c,uint32_t us,uint32_t ms)
{
    c->reference_us=us;c->reference_ms=ms;
    c->startup_us=us;c->startup_ms=ms;
    c->status=S4_CLOCK_PENDING;c->good_windows=0U;c->startup_rejections=0U;
}
void s4_clock_check_step(s4_clock_check_t *c,uint32_t us,uint32_t ms)
{
    uint32_t elapsed_ms=ms-c->reference_ms;
    uint32_t elapsed_us=us-c->reference_us;
    uint32_t expected,tolerance,error;
    unsigned int good=0U;
    if(c->status==S4_CLOCK_BAD){return;}
    /* Boot qualification, not a wider operating tolerance. A measured first
     * window contained 332809 us for 204 kernel ms, while later clocks agreed.
     * Height stays unavailable until THREE consecutive valid windows. Either
     * clock reaching this 2 s development budget without qualification faults.
     * Never adjust the shared timer, tick, clock offset or sampling period. */
    if(c->status==S4_CLOCK_PENDING&&
       (((uint32_t)(ms-c->startup_ms)>=2000U)||
        ((uint32_t)(us-c->startup_us)>=2000000U)))
    {c->status=S4_CLOCK_BAD;return;}
    if(elapsed_ms<200U)
    {
        /* After qualification, a stalled kernel clock must not leave a
         * previously CONSISTENT status valid indefinitely. */
        if(c->status==S4_CLOCK_CONSISTENT&&elapsed_us>1000000U)
        {c->status=S4_CLOCK_BAD;}
        return;
    }
    /* Original operating bounds: 200..1000 ms windows; 10% plus 10 ms.
     * This is a coarse rate check, not calibrated time or sample alignment. */
    if(elapsed_ms<=1000U)
    {
        expected=elapsed_ms*1000U;tolerance=expected/10U+10000U;
        error=elapsed_us>expected?elapsed_us-expected:expected-elapsed_us;
        good=(error<=tolerance);
    }
    if(c->status==S4_CLOCK_CONSISTENT)
    {
        if(!good){c->status=S4_CLOCK_BAD;}
    }
    else if(good)
    {
        ++c->good_windows;
        if(c->good_windows>=3U){c->status=S4_CLOCK_CONSISTENT;}
    }
    else
    {
        c->good_windows=0U;
        if(c->startup_rejections<255U){++c->startup_rejections;}
    }
    c->reference_us=us;c->reference_ms=ms;
}
const char *s4_clock_status_name(uint8_t status)
{
    if(status==S4_CLOCK_CONSISTENT){return "CONSISTENT";}
    return status==S4_CLOCK_BAD?"BAD":"PENDING";
}
