#include "subsystem4.h"
#include "s4_time.h"
#include "drv_motion_sensor.h"

EXPORT void imu_terrain_task(INT stacd, void *exinf)
{
    SYSTIM clock;
    uint32_t release, missed = 0U, cycles = 0U;
    s4_bus_t bus;
    s4_sample_t sample = {0};
    int sensor_ok;
    (void)stacd; (void)exinf;
    sensor_ok = (s4_board_bus_init(&bus) == 0) && (s4_sensor_init(&bus) == 0);
    (void)tk_get_tim(&clock);
    release = clock.lo + S4_PERIOD_MS;
    for (;;)
    {
        uint32_t delay;
        int result = sensor_ok ? s4_sensor_read(&bus, &sample) : -1;
        s4_drain_example();
        ++cycles;
#if S4_LOG_EVERY_CYCLES > 0
        if ((cycles % S4_LOG_EVERY_CYCLES) == 0U)
        {
            tm_printf((UB *)"S4 raw mg=%d,%d,%d fresh=%d rc=%d missed=%u\n",
                      (int)(sample.accel_g[0]*1000.0F),
                      (int)(sample.accel_g[1]*1000.0F),
                      (int)(sample.accel_g[2]*1000.0F),
                      (int)sample.accel_fresh, result, missed);
        }
#endif
        (void)tk_get_tim(&clock);
        delay = s4_delay_to_release(clock.lo, &release, S4_PERIOD_MS, &missed);
        if (delay != 0U) { (void)tk_dly_tsk(delay); }
        release += S4_PERIOD_MS;
    }
}
