#include "subsystem4.h"
#include "s4_time.h"
#include "drv_motion_sensor.h"
#include "s4_processing.h"
#include "s4_terrain.h"
#include "s4_odometry.h"
#include "s4_clock_check.h"
#include "s4_integration.h"
#if S4_HEIGHT_WHEELBASE_MM >= S4_HEIGHT_TRACK_LEN
#error S4_HEIGHT_WHEELBASE_MM must be below S4_HEIGHT_TRACK_LEN
#endif

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
    bool previous_chain=true,previous_active=false;
    uint32_t previous_interpolated=0U;
    s4_bus_t bus;
    s4_sample_t sample={0};
    s4_processing_config_t config=s4_processing_defaults();
    s4_processor_t processor;
    s4_orientation_t orientation={0};
    s4_terrain_t terrain;
    static s4_height_t height; /* holds the height track: keep off the 2 KB stack */
    s4_clock_check_t timebase;
    s4_odometry_t odometry={0};
    ER fault_rc=E_OK;
    int bus_rc,init_rc,sensor_ok;
    (void)stacd;(void)exinf;
    config.magnetic_calibrated=(S4_MAG_CALIBRATED!=0);
    config.magnetic_bias[0]=S4_MAG_BIAS_X;config.magnetic_bias[1]=S4_MAG_BIAS_Y;
    config.magnetic_bias[2]=S4_MAG_BIAS_Z;
    config.magnetic_scale[0]=S4_MAG_SCALE_X;config.magnetic_scale[1]=S4_MAG_SCALE_Y;
    config.magnetic_scale[2]=S4_MAG_SCALE_Z;
    s4_processor_init(&processor,&config);
    bus_rc=s4_board_bus_init(&bus);
    init_rc=(bus_rc==0)?s4_sensor_init(&bus):-100; /* -100: not attempted. */
    sensor_ok=(bus_rc==0)&&(init_rc==0);
    startup=s4_board_time_us();last_accel=startup;s4_terrain_init(&terrain,startup);
    s4_height_init(&height);
    height.wheelbase_mm=S4_HEIGHT_WHEELBASE_MM;
    (void)tk_get_tim(&clock);
    s4_clock_check_init(&timebase,startup,clock.lo);
    release=clock.lo+S4_PERIOD_MS;
    for(;;)
    {
        uint32_t delay,work_start=s4_board_time_us(),work;
        bool fresh;
        uint8_t state,imu_quality;
        const char *imu_status;
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
        if(result!=0){imu_quality=S4_IMU_FAULT;imu_status="IO_ERROR";}
        else if(!fresh){imu_quality=S4_IMU_WAIT;imu_status="NO_NEW_SAMPLE";}
        else if(sample.saturated){imu_quality=S4_IMU_FAULT;imu_status="SATURATED";}
        else if(!processor.calibrated){imu_quality=S4_IMU_FAULT;imu_status="CALIBRATING";}
        else if(orientation.dynamic_g>=terrain.config.impact_g)
        {imu_quality=S4_IMU_FAULT;imu_status="IMPACT";}
        else if(!orientation.orientation_valid)
        {imu_quality=S4_IMU_DYNAMIC;imu_status="NOT_GRAVITY_DOMINANT";}
        else {imu_quality=S4_IMU_GOOD;imu_status="OK";}
        s4_terrain_step(&terrain,&orientation,processor.calibrated,
                       (result==0)&&((uint32_t)(work_start-last_accel)<=100000U),
                       fresh,work_start);
        if(terrain.hump_event){++humps;}
        if(terrain.impact_event){++impacts;}
        if(terrain.traversal_aborted){++aborts;}
        state=s4_terrain_state(&terrain);
        {
            uint32_t input_time;
            (void)s4_read_odometry(&odometry);
            (void)tk_get_tim(&clock);input_time=s4_board_time_us();
            s4_clock_check_step(&timebase,input_time,clock.lo);
            s4_height_step_quality(&height,&odometry,&orientation,imu_quality,
                timebase.status==S4_CLOCK_CONSISTENT,input_time,clock.lo,
                state,terrain.config.exit_deg,terrain.hump_event,
                terrain.traversal_aborted);
        }
        fault_rc=s4_integration_publish(&height,
            (state==S4_FAULT)||(timebase.status==S4_CLOCK_BAD),humps);
        s4_drain_example();++cycles;
#if S4_LOG_EVERY_CYCLES > 0
        /* Changes/events are logged immediately, not only at the heartbeat. */
        if((state!=previous_state)||terrain.hump_event||terrain.impact_event||
           terrain.traversal_aborted||((previous_chain||!previous_active)&&!height.chain_valid&&height.active)||
           (height.interpolated_samples!=0U&&previous_interpolated==0U)||
           ((cycles%S4_LOG_EVERY_CYCLES)==0U))
        {
            tm_printf((UB *)"S4 t_us=%u state=%s motion=%s calibrated=%d valid=%d fresh=%d rc=%d pitch_md=%d roll_md=%d impact=%d hump=%d aborted=%d humps=%u impacts=%u aborts=%u max_us=%u missed=%u bus_rc=%d init_rc=%d timebase=%s odom_valid=%d height_valid=%d height_mm=%d highest_valid=%d highest_mm=%d height_reason=%s odo_mm=%u odo_ms=%u odo_forward=%d fault_rc=%d build_id=%s profile=%s encoder_valid=%d imu_status=%s dynamic_mg=%d interpolated=%u max_interp_us=%u pending_imu=%u chain_valid=%d\n",
                work_start,s4_terrain_state_name(state),
                s4_motion_class_name(s4_motion_class(&terrain)),
                (int)processor.calibrated,(int)(fresh&&orientation.orientation_valid),
                (int)fresh,result,(int)(orientation.pitch_deg*1000.0F),
                (int)(orientation.roll_deg*1000.0F),(int)terrain.impact_event,
                (int)terrain.hump_event,(int)terrain.traversal_aborted,
                humps,impacts,aborts,max_work_us,missed,bus_rc,init_rc,
                s4_clock_status_name(timebase.status),(int)height.odometry_valid,
                (int)height.last_valid,
                height.last_valid?(int)(height.last_mm+0.5F):-1,
                (int)height.highest_valid,
                height.highest_valid?(int)(height.highest_mm+0.5F):-1,
                s4_height_reason_name(height.reason),
                (unsigned)odometry.distance_mm,(unsigned)(uint16_t)odometry.timestamp,
                (int)odometry.forward,(int)fault_rc,S4_BUILD_TAG,S4_BUILD_PROFILE,
                (int)odometry.valid,imu_status,(int)(orientation.dynamic_g*1000.0F),
                height.interpolated_samples,height.max_interpolated_us,
                height.pending_samples,(int)height.chain_valid);
        }
#endif
        previous_state=state;previous_chain=height.chain_valid;previous_active=height.active;
        previous_interpolated=height.interpolated_samples;
        /* Elapsed work-section high-water mark, including console calls and
         * preemption; shown on the next log. Not a measured complete WCET. */
        work=s4_board_time_us()-work_start;if(work>max_work_us){max_work_us=work;}
        (void)tk_get_tim(&clock);
        delay=s4_delay_to_release(clock.lo,&release,S4_PERIOD_MS,&missed);
        if(delay!=0U){(void)tk_dly_tsk(delay);}
        release+=S4_PERIOD_MS;
    }
}
