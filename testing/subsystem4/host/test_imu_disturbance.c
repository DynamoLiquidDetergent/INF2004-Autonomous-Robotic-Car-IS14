/* Synthetic regression checks only: no physical height/accuracy claim. */
#include <stdio.h>
#include "test_math.h"
#include "s4_math.h"
#include "s4_terrain.h"
#include "s4_motion_input.h"
static unsigned checks,failures;
#define CHECK(x) do{++checks;if(!(x)){++failures;printf("FAIL %d: %s\n",__LINE__,#x);}}while(0)
typedef struct {s4_height_t h;s4_odometry_t o;s4_orientation_t a;uint32_t ms;} run_t;
static void tick(run_t *r,uint8_t quality,bool done)
{
    r->ms+=20U;r->o.timestamp=r->o.clock_format==S4_ODOM_MS16?(uint16_t)r->ms:r->ms*1000U;
    ++r->o.distance_mm;r->a.orientation_valid=quality==S4_IMU_GOOD;
    r->a.pitch_deg=quality==S4_IMU_GOOD?15.0F:89.0F;
    s4_height_step_quality(&r->h,&r->o,&r->a,quality,true,r->ms*1000U,r->ms,
        done?S4_LEVEL:S4_CLIMBING,2.0F,done,false);
}
static void init(run_t *r,uint8_t clock)
{
    *r=(run_t){0};s4_height_init(&r->h);
    r->h.max_imu_gap_us=100000U;r->h.max_imu_step_mm=5U;
    r->o.valid=true;r->o.forward=true;r->o.clock_format=clock;
    r->ms=65500U;r->o.distance_mm=65530U;
    r->o.timestamp=clock==S4_ODOM_MS16?(uint16_t)r->ms:r->ms*1000U;
    r->a.orientation_valid=true;
    s4_height_step_quality(&r->h,&r->o,&r->a,S4_IMU_GOOD,true,r->ms*1000U,r->ms,S4_LEVEL,2,false,false);
    tick(r,S4_IMU_GOOD,false);
}
static void finish(run_t *r)
{tick(r,S4_IMU_GOOD,false);tick(r,S4_IMU_GOOD,true);}
static void filter_test(void)
{
    s4_processing_config_t c=s4_processing_defaults();
    s4_processor_t p,control;s4_orientation_t a,b;
    s4_sample_t flat={{0,0,1},{0,0,0},true,false,false},s=flat;
    unsigned i;c.calibration_samples=4U;
    s4_processor_init(&p,&c);s4_processor_init(&control,&c);
    for(i=0;i<4U;++i)
    {(void)s4_process(&p,&flat,i*20000U,&a);(void)s4_process(&control,&flat,i*20000U,&b);}
    /* Similar magnitude to the latest physical rejection, not a replay of
     * unobserved 50-Hz data. Only this one raw vector was in the capture. */
    s.accel_g[0]=-0.188F;s.accel_g[1]=-0.078F;s.accel_g[2]=0.570F;
    for(i=0;i<4U;++i)
    {
        CHECK(!s4_process(&p,&s,(i+4U)*20000U,&a));
        CHECK(!a.orientation_valid&&!a.heading_valid&&a.dynamic_g>0.39F);
        CHECK(p.filtered[0]==control.filtered[0]&&p.filtered[1]==control.filtered[1]&&p.filtered[2]==control.filtered[2]);
    }
    s.accel_g[0]=0.258819F;s.accel_g[1]=0;s.accel_g[2]=0.965926F;
    CHECK(s4_process(&p,&s,160000U,&a));CHECK(s4_process(&control,&s,160000U,&b));
    CHECK(a.orientation_valid&&test_abs(a.pitch_deg-b.pitch_deg)<0.001F);
    CHECK(test_abs(a.roll_deg-b.roll_deg)<0.001F);
    s=flat;s.saturated=true;
    CHECK(!s4_process(&p,&s,180000U,&a)&&a.dynamic_g==4.0F&&!a.orientation_valid);
    s=flat;s.accel_fresh=false;
    CHECK(!s4_process(&p,&s,200000U,&a)&&!a.orientation_valid);
    /* Unstable startup must still fail calibration. */
    s4_processor_init(&p,&c);
    for(i=0;i<100U;++i)
    {s=flat;s.accel_g[0]=(i&1U)?0.3F:0;(void)s4_process(&p,&s,i*20000U,&a);}
    CHECK(!p.calibrated);
    /* Both real functions see exactly the same sequence. The old observer
     * still rejects the jolt as orientation, but feeds it into its filter. */
    s4_processor_init(&p,&c);s4_processor_init(&control,&c);
    for(i=0;i<4U;++i)
    {(void)s4_process(&p,&flat,i*20000U,&a);(void)s4_process_reference(&control,&flat,i*20000U,&b);}
    CHECK(a.orientation_valid&&b.orientation_valid&&test_abs(a.pitch_deg-b.pitch_deg)<0.001F);
    s=flat;s.accel_g[0]=-0.188F;s.accel_g[1]=-0.078F;s.accel_g[2]=0.570F;
    for(i=0;i<4U;++i)
    {(void)s4_process(&p,&s,(i+4U)*20000U,&a);(void)s4_process_reference(&control,&s,(i+4U)*20000U,&b);}
    CHECK(!a.orientation_valid&&!b.orientation_valid);
    CHECK(p.filtered[0]==0.0F&&control.filtered[0]<-0.1F);
    (void)s4_process(&p,&flat,160000U,&a);(void)s4_process_reference(&control,&flat,160000U,&b);
    CHECK(a.orientation_valid&&b.orientation_valid&&test_abs(a.pitch_deg-b.pitch_deg)>1.0F);
}
static void endpoints(void)
{
    run_t r;unsigned n,i,clock;
    for(clock=0;clock<2;++clock)
    {
        for(n=1;n<=4;++n)
        {
            init(&r,(uint8_t)clock);
            for(i=0;i<n;++i){tick(&r,i&1U?S4_IMU_WAIT:S4_IMU_DYNAMIC,false);}
            CHECK(r.h.chain_valid&&r.h.pending_samples==n&&!r.h.odometry_valid);
            finish(&r);
            CHECK(r.h.last_valid&&r.h.highest_valid&&r.h.interpolated_samples==n);
            CHECK(r.h.max_interpolated_us==(n+1U)*20000U);
            CHECK(r.h.max_gap_us==60000U&&r.h.max_step_mm==50U);
        }
        init(&r,(uint8_t)clock);
        for(i=0;i<5U;++i){tick(&r,S4_IMU_DYNAMIC,false);}
        finish(&r);CHECK(!r.h.last_valid&&!r.h.highest_valid);
        /* Too much real travel, even within the time budget. */
        init(&r,(uint8_t)clock);tick(&r,S4_IMU_DYNAMIC,false);
        r.o.distance_mm+=4U;tick(&r,S4_IMU_GOOD,false);
        finish(&r);CHECK(!r.h.last_valid);
        /* Missing encoder updates cannot borrow the IMU's 100-ms budget. */
        init(&r,(uint8_t)clock);tick(&r,S4_IMU_DYNAMIC,false);
        r.ms+=60U;tick(&r,S4_IMU_GOOD,false);finish(&r);
        CHECK(!r.h.last_valid&&r.h.reason==S4_HEIGHT_GAP);
        /* Intermediate reverse, loss, I/O error or impact remains fatal. */
        init(&r,(uint8_t)clock);r.o.forward=false;tick(&r,S4_IMU_DYNAMIC,false);
        r.o.forward=true;finish(&r);CHECK(!r.h.last_valid&&r.h.reason==S4_HEIGHT_REVERSE);
        init(&r,(uint8_t)clock);r.o.valid=false;tick(&r,S4_IMU_DYNAMIC,false);
        r.o.valid=true;finish(&r);CHECK(!r.h.last_valid);
        init(&r,(uint8_t)clock);tick(&r,S4_IMU_FAULT,false);finish(&r);CHECK(!r.h.last_valid);
        /* Changed distance with an identical timestamp is a torn sample. */
        init(&r,(uint8_t)clock);tick(&r,S4_IMU_DYNAMIC,false);++r.o.distance_mm;
        s4_height_step_quality(&r.h,&r.o,&r.a,S4_IMU_DYNAMIC,true,r.ms*1000U,r.ms,S4_CLIMBING,2,false,false);
        finish(&r);CHECK(!r.h.last_valid&&r.h.reason==S4_HEIGHT_JUMP);
        /* Duplicate timestamps never become a qualified new endpoint. */
        init(&r,(uint8_t)clock);r.ms+=20U;r.a.orientation_valid=false;
        s4_height_step_quality(&r.h,&r.o,&r.a,S4_IMU_DYNAMIC,true,r.ms*1000U,r.ms,S4_CLIMBING,2,false,false);
        r.a.orientation_valid=true;
        s4_height_step_quality(&r.h,&r.o,&r.a,S4_IMU_GOOD,true,r.ms*1000U,r.ms,S4_CLIMBING,2,false,false);
        CHECK(r.h.pending_samples==1U&&r.h.interpolated_samples==0U);
        tick(&r,S4_IMU_GOOD,false);CHECK(r.h.pending_samples==0U&&r.h.interpolated_samples==1U);
        /* No trusted baseline, no inferred starting height. */
        init(&r,(uint8_t)clock);r.h.active=false;r.h.have_previous=false;r.h.baseline_ready=false;
        tick(&r,S4_IMU_DYNAMIC,false);tick(&r,S4_IMU_GOOD,false);finish(&r);CHECK(!r.h.last_valid);
        init(&r,(uint8_t)clock);tick(&r,S4_IMU_DYNAMIC,false);
        s4_height_step_quality(&r.h,&r.o,&r.a,S4_IMU_DYNAMIC,false,r.ms*1000U,r.ms,S4_CLIMBING,2,false,false);
        finish(&r);CHECK(!r.h.last_valid&&r.h.reason==S4_HEIGHT_TIMEBASE);
    }
}
/* Real processor + terrain + packed S2 reader + height wrapper together. */
static void pipeline(unsigned bad_count,bool hard,bool wheelbase,bool *valid,unsigned *bridged)
{
    s4_processing_config_t c=s4_processing_defaults();s4_processor_t p;
    s4_sample_t s={{0,0,1},{0,0,0},true,false,false};s4_orientation_t a;
    s4_height_t h;s4_motion_reader_t reader;s4_terrain_t t;s4_odometry_t o;
    unsigned i;uint16_t mm=0;c.calibration_samples=4U;
    s4_processor_init(&p,&c);s4_height_init(&h);s4_motion_reader_init(&reader);s4_terrain_init(&t,0);
    h.max_imu_gap_us=100000U;h.max_imu_step_mm=5U;h.wheelbase_mm=wheelbase?110U:0U;
    for(i=0;i<100U;++i)
    {
        uint32_t ms=(i+1U)*20U;uint8_t q=S4_IMU_GOOD;
        float pitch=i<12U?0:i<48U?12.0F:i<80U?-12.0F:0;
        s.accel_g[0]=s4_sin_deg(pitch);s.accel_g[1]=0;s.accel_g[2]=s4_cos_deg(pitch);
        if(i>=30U&&i<30U+bad_count)
        {s.accel_g[0]=-0.188F;s.accel_g[1]=-0.078F;s.accel_g[2]=0.570F;}
        (void)s4_process(&p,&s,ms*1000U,&a);
        if(!p.calibrated){q=S4_IMU_FAULT;}
        else if(!a.orientation_valid){q=hard?S4_IMU_FAULT:S4_IMU_DYNAMIC;}
        ++mm;(void)s4_motion_snapshot(&reader,((uint32_t)mm<<16U)|(uint16_t)ms,i>12U?1U:0U,false,ms,&o);
        s4_terrain_step(&t,&a,p.calibrated,true,true,ms*1000U);
        s4_height_step_quality(&h,&o,&a,q,true,ms*1000U,ms,s4_terrain_state(&t),2.0F,t.hump_event,t.traversal_aborted);
    }
    *valid=h.last_valid;*bridged=h.interpolated_samples;
}
int main(void)
{
    unsigned i,n,b;bool valid;
    filter_test();endpoints();
    for(b=0;b<2U;++b)
    {
        for(i=1;i<=4U;++i)
        {pipeline(i,false,b!=0,&valid,&n);CHECK(valid&&n==i);}
        pipeline(5,false,b!=0,&valid,&n);CHECK(!valid);
        pipeline(1,true,b!=0,&valid,&n);CHECK(!valid);
    }
    printf("IMU disturbance: %s (%u checks, %u failures); synthetic filter/endpoint/reader/FSM guards only.\n",failures?"FAIL":"PASS",checks,failures);
    return failures?1:0;
}
