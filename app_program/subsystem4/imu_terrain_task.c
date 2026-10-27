#include "subsystem4.h"
#include "s4_time.h"

EXPORT void imu_terrain_task(INT stacd, void *exinf)
{
    SYSTIM clock;
    uint32_t release, missed = 0U, cycles = 0U;
    (void)stacd;
    (void)exinf;
    (void)tk_get_tim(&clock);
    release = clock.lo + S4_PERIOD_MS;
    for (;;)
    {
        uint32_t delay;
        s4_drain_example();
        ++cycles;
#if S4_LOG_EVERY_CYCLES > 0
        if ((cycles % S4_LOG_EVERY_CYCLES) == 0U)
        {
            tm_printf((UB *)"S4 phase=%u cycles=%u missed=%u\n",
                      S4_PHASE, cycles, missed);
        }
#endif
        (void)tk_get_tim(&clock);
        delay = s4_delay_to_release(clock.lo, &release, S4_PERIOD_MS, &missed);
        if (delay != 0U) { (void)tk_dly_tsk(delay); }
        release += S4_PERIOD_MS;
    }
}
