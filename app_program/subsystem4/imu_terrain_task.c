#include "subsystem4.h"
#include "s4_time.h"
#include "drv_motion_sensor.h"
#include "s4_processing.h"

static void map_sample(s4_sample_t *s)
{
    const unsigned int index[3]={S4_AXIS_X,S4_AXIS_Y,S4_AXIS_Z};
    const int sign[3]={S4_SIGN_X,S4_SIGN_Y,S4_SIGN_Z};
    s4_sample_t raw=*s;
    unsigned int i;
    for(i=0U;i<3U;++i)
    {
        s->accel_g[i]=raw.accel_g[index[i]]*(float)sign[i];
        s->magnetic_gauss[i]=raw.magnetic_gauss[index[i]]*(float)sign[i];
    }
}
EXPORT void imu_terrain_task(INT stacd, void *exinf)
{
    SYSTIM clock;
    uint32_t release, missed = 0U, cycles = 0U;
    s4_bus_t bus;
    s4_sample_t sample = {0};
    int sensor_ok;
    s4_processing_config_t config = s4_processing_defaults();
    s4_processor_t processor;
    s4_orientation_t orientation={0};
    config.magnetic_calibrated=(S4_MAG_CALIBRATED!=0);
    config.magnetic_bias[0]=S4_MAG_BIAS_X;config.magnetic_bias[1]=S4_MAG_BIAS_Y;
    config.magnetic_bias[2]=S4_MAG_BIAS_Z;
    config.magnetic_scale[0]=S4_MAG_SCALE_X;config.magnetic_scale[1]=S4_MAG_SCALE_Y;
    config.magnetic_scale[2]=S4_MAG_SCALE_Z;
    s4_processor_init(&processor, &config);
    (void)stacd; (void)exinf;
    sensor_ok = (s4_board_bus_init(&bus) == 0) && (s4_sensor_init(&bus) == 0);
    (void)tk_get_tim(&clock);
    release = clock.lo + S4_PERIOD_MS;
    for (;;)
    {
        uint32_t delay;
        int result = sensor_ok ? s4_sensor_read(&bus, &sample) : -1;
        if ((result == 0) && sample.accel_fresh)
        { map_sample(&sample); (void)s4_process(&processor, &sample, s4_board_time_us(), &orientation); }
        s4_drain_example();
        ++cycles;
#if S4_LOG_EVERY_CYCLES > 0
        if ((cycles % S4_LOG_EVERY_CYCLES) == 0U)
        {
            tm_printf((UB *)"S4 mg=%d,%d,%d fresh=%d rc=%d missed=%u\n",
                      (int)(sample.accel_g[0]*1000.0F),
                      (int)(sample.accel_g[1]*1000.0F),
                      (int)(sample.accel_g[2]*1000.0F),
                      (int)sample.accel_fresh, result, missed);
        }
#endif
#if S4_LOG_EVERY_CYCLES > 0
        if ((cycles % S4_LOG_EVERY_CYCLES) == 0U)
        { tm_printf((UB *)"S4 calibrated=%d pitch_md=%d roll_md=%d heading_ok=%d rate_ok=%d\n", (int)processor.calibrated, (int)(orientation.pitch_deg*1000.0F), (int)(orientation.roll_deg*1000.0F), (int)orientation.heading_valid, (int)orientation.turn_rate_valid); }
#endif
        (void)tk_get_tim(&clock);
        delay = s4_delay_to_release(clock.lo, &release, S4_PERIOD_MS, &missed);
        if (delay != 0U) { (void)tk_dly_tsk(delay); }
        release += S4_PERIOD_MS;
    }
}
