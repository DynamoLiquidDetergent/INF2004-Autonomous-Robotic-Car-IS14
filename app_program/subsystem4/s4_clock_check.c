#include "s4_clock_check.h"

void s4_clock_check_init(s4_clock_check_t *c,uint32_t us,uint32_t ms)
{
    c->reference_us=us;c->reference_ms=ms;c->status=S4_CLOCK_PENDING;
}
void s4_clock_check_step(s4_clock_check_t *c,uint32_t us,uint32_t ms)
{
    uint32_t elapsed_ms=ms-c->reference_ms;
    uint32_t elapsed_us,expected,tolerance,error;
    if((c->status==S4_CLOCK_BAD)||(elapsed_ms<200U)){return;}
    /* A coarse cross-check, not calibrated timing or proof of clock alignment.
     * The generous tolerance allows the 10 ms kernel tick and read skew.
     * Never change the global timer/watchdog from this subsystem. */
    if(elapsed_ms>1000U){c->status=S4_CLOCK_BAD;return;}
    elapsed_us=us-c->reference_us;expected=elapsed_ms*1000U;
    tolerance=expected/10U+10000U;
    error=(elapsed_us>expected)?elapsed_us-expected:expected-elapsed_us;
    c->status=(error<=tolerance)?S4_CLOCK_CONSISTENT:S4_CLOCK_BAD;
    c->reference_us=us;c->reference_ms=ms;
}
const char *s4_clock_status_name(uint8_t status)
{
    if(status==S4_CLOCK_CONSISTENT){return "CONSISTENT";}
    return (status==S4_CLOCK_BAD)?"BAD":"PENDING";
}
