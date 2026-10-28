#include "subsystem4.h"
#include "s4_time.h"
#include "drv_motion_sensor.h"
#include "s4_processing.h"
#include "s4_terrain.h"

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
    uint32_t release,missed=0U,cycles=0U,startup,last_accel;
    uint32_t max_work_us=0U,humps=0U,impacts=0U,aborts=0U;
    uint8_t previous_state=S4_INITIALISING;
    s4_bus_t bus;
    s4_sample_t sample={0};
    s4_processing_config_t config=s4_processing_defaults();
    s4_processor_t processor;
    s4_orientation_t orientation={0};
    s4_terrain_t terrain;
    int sensor_ok;
    (void)stacd;(void)exinf;
    config.magnetic_calibrated=(S4_MAG_CALIBRATED!=0);
    config.magnetic_bias[0]=S4_MAG_BIAS_X;config.magnetic_bias[1]=S4_MAG_BIAS_Y;
    config.magnetic_bias[2]=S4_MAG_BIAS_Z;
    config.magnetic_scale[0]=S4_MAG_SCALE_X;config.magnetic_scale[1]=S4_MAG_SCALE_Y;
    config.magnetic_scale[2]=S4_MAG_SCALE_Z;
    s4_processor_init(&processor,&config);
    sensor_ok=(s4_board_bus_init(&bus)==0)&&(s4_sensor_init(&bus)==0);
    startup=s4_board_time_us();last_accel=startup;s4_terrain_init(&terrain,startup);
    (void)tk_get_tim(&clock);release=clock.lo+S4_PERIOD_MS;
    for(;;)
    {
        uint32_t delay,work_start=s4_board_time_us(),work;
        bool fresh;
        uint8_t state;
        int result=sensor_ok?s4_sensor_read(&bus,&sample):-1;
        fresh=(result==0)&&sample.accel_fresh;
        if(fresh)
        {
            last_accel=work_start;map_sample(&sample);
            (void)s4_process(&processor,&sample,work_start,&orientation);
        }
        else
        {
            orientation.orientation_valid=false;orientation.heading_valid=false;
            orientation.turn_rate_valid=false;
        }
        s4_terrain_step(&terrain,&orientation,processor.calibrated,
                       (result==0)&&((uint32_t)(work_start-last_accel)<=100000U),
                       fresh,work_start);
        if(terrain.hump_event){++humps;}
        if(terrain.impact_event){++impacts;}
        if(terrain.traversal_aborted){++aborts;}
        state=s4_terrain_state(&terrain);
        s4_drain_example();++cycles;
#if S4_LOG_EVERY_CYCLES > 0
        /* Changes/events are logged immediately, not only at the heartbeat. */
        if((state!=previous_state)||terrain.hump_event||terrain.impact_event||
           terrain.traversal_aborted||((cycles%S4_LOG_EVERY_CYCLES)==0U))
        {
            tm_printf((UB *)"S4 t_us=%u state=%s motion=%s calibrated=%d valid=%d fresh=%d rc=%d pitch_md=%d roll_md=%d impact=%d hump=%d aborted=%d humps=%u impacts=%u aborts=%u max_us=%u missed=%u\n",
                work_start,s4_terrain_state_name(state),
                s4_motion_class_name(s4_motion_class(&terrain)),
                (int)processor.calibrated,(int)(fresh&&orientation.orientation_valid),
                (int)fresh,result,(int)(orientation.pitch_deg*1000.0F),
                (int)(orientation.roll_deg*1000.0F),(int)terrain.impact_event,
                (int)terrain.hump_event,(int)terrain.traversal_aborted,
                humps,impacts,aborts,max_work_us,missed);
        }
#endif
        previous_state=state;
        /* Elapsed work-section high-water mark, including console calls and
         * preemption; shown on the next log. Not a measured complete WCET. */
        work=s4_board_time_us()-work_start;if(work>max_work_us){max_work_us=work;}
        (void)tk_get_tim(&clock);
        delay=s4_delay_to_release(clock.lo,&release,S4_PERIOD_MS,&missed);
        if(delay!=0U){(void)tk_dly_tsk(delay);}
        release+=S4_PERIOD_MS;
    }
}
