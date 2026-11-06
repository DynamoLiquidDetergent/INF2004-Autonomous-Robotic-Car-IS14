/* Synthetic endpoint/gap policy checks; no physical accuracy claim. */
#include <stdio.h>
#include "test_math.h"
#include "s4_terrain.h"
#include "s4_motion_input.h"
static unsigned checks,failures;
#define CHECK(x) do{++checks;if(!(x)){++failures;printf("FAIL %d: %s\n",__LINE__,#x);}}while(0)
typedef struct {s4_height_t h;s4_odometry_t o;s4_orientation_t a;uint32_t ms;} run_t;
static void step(run_t *r,float pitch,uint8_t quality,uint8_t state,bool done)
{
    r->ms+=20U;r->o.timestamp=(uint16_t)r->ms;r->o.distance_mm+=2U;
    r->a.pitch_deg=pitch;r->a.orientation_valid=(quality==S4_IMU_GOOD);
    s4_height_step_quality(&r->h,&r->o,&r->a,quality,true,r->ms*1000U,r->ms,state,2.0F,done,false);
}
static void init(run_t *r)
{
    *r=(run_t){0};s4_height_init(&r->h);r->o.valid=true;r->o.forward=true;r->o.clock_format=S4_ODOM_MS16;
    step(r,0.0F,S4_IMU_GOOD,S4_LEVEL,false);
    step(r,30.0F,S4_IMU_GOOD,S4_CLIMBING,false);
}
static void finish(run_t *r)
{
    step(r,30.0F,S4_IMU_GOOD,S4_CLIMBING,false);
    step(r,-30.0F,S4_IMU_GOOD,S4_DESCENDING,false);
    step(r,0.0F,S4_IMU_GOOD,S4_LEVEL,true);
}
static void pipeline(unsigned bad_count,uint8_t quality,bool *valid,unsigned *bridged)
{
    s4_height_t h;s4_motion_reader_t reader;s4_terrain_t t;
    s4_odometry_t o={0};s4_orientation_t a={0};unsigned i;uint16_t mm=0;
    s4_height_init(&h);s4_motion_reader_init(&reader);s4_terrain_init(&t,0);
    for(i=0;i<35U;++i)
    {
        uint32_t ms=(i+1U)*20U;uint8_t q=S4_IMU_GOOD;
        float p=i<6U?0.0F:i<18U?15.0F:i<27U?-15.0F:0.0F;
        if(i>=12U&&i<12U+bad_count){q=quality;}
        a.pitch_deg=(q==S4_IMU_GOOD)?p:89.0F; /* rejected angle must not be integrated */
        a.orientation_valid=(q==S4_IMU_GOOD);mm+=2U;
        (void)s4_motion_snapshot(&reader,((uint32_t)mm<<16U)|(uint16_t)ms,(i>6U)?1U:0U,false,ms,&o);
        s4_terrain_step(&t,&a,true,true,q!=S4_IMU_WAIT,ms*1000U);
        s4_height_step_quality(&h,&o,&a,q,true,ms*1000U,ms,s4_terrain_state(&t),2.0F,t.hump_event,t.traversal_aborted);
    }
    *valid=h.last_valid;*bridged=h.interpolated_samples;
}
int main(void)
{
    run_t good,one,two,bad;unsigned i,n;bool valid;
    init(&good);step(&good,30,S4_IMU_GOOD,S4_CLIMBING,false);finish(&good);
    init(&one);step(&one,89,S4_IMU_WAIT,S4_CLIMBING,false);
    CHECK(one.h.active&&one.h.chain_valid&&one.h.pending_samples==1U&&!one.h.odometry_valid);
    finish(&one);CHECK(one.h.last_valid&&one.h.highest_valid);
    CHECK(test_abs(one.h.last_mm-good.h.last_mm)<0.001F);
    CHECK(one.h.interpolated_samples==1U&&one.h.max_interpolated_us==40000U);
    init(&two);step(&two,89,S4_IMU_DYNAMIC,S4_CLIMBING,false);step(&two,89,S4_IMU_WAIT,S4_CLIMBING,false);
    finish(&two);CHECK(two.h.last_valid&&two.h.interpolated_samples==2U&&two.h.max_interpolated_us==60000U);
    init(&bad);for(i=0;i<3U;++i){step(&bad,89,S4_IMU_WAIT,S4_CLIMBING,false);}
    CHECK(!bad.h.chain_valid&&bad.h.reason==S4_HEIGHT_IMU);finish(&bad);CHECK(!bad.h.last_valid&&!bad.h.highest_valid);
    init(&bad);step(&bad,89,S4_IMU_FAULT,S4_CLIMBING,false);finish(&bad);CHECK(!bad.h.last_valid);
    init(&bad);bad.o.forward=false;step(&bad,89,S4_IMU_WAIT,S4_CLIMBING,false);CHECK(!bad.h.chain_valid&&bad.h.reason==S4_HEIGHT_REVERSE);
    init(&bad);bad.o.valid=false;step(&bad,89,S4_IMU_WAIT,S4_CLIMBING,false);CHECK(!bad.h.chain_valid);
    init(&bad);bad.o.distance_mm+=51U;step(&bad,89,S4_IMU_WAIT,S4_CLIMBING,false);CHECK(!bad.h.chain_valid);
    init(&bad);bad.ms+=80U;step(&bad,89,S4_IMU_WAIT,S4_CLIMBING,false);CHECK(!bad.h.chain_valid);
    init(&bad);step(&bad,89,S4_IMU_WAIT,S4_FAULT,false);CHECK(!bad.h.active&&!bad.h.last_valid);
    init(&bad);step(&bad,89,S4_IMU_WAIT,S4_CLIMBING,false);bad.o.timestamp=bad.ms;
    s4_height_step_quality(&bad.h,&bad.o,&bad.a,S4_IMU_WAIT,false,bad.ms*1000U,bad.ms,S4_CLIMBING,2,false,false);
    CHECK(!bad.h.chain_valid&&bad.h.reason==S4_HEIGHT_TIMEBASE);
    /* Deferred baseline remains real; no baseline is invented at startup. */
    init(&bad);bad.h.active=false;bad.h.have_previous=false;bad.h.baseline_ready=false;
    step(&bad,89,S4_IMU_WAIT,S4_LEVEL,false);step(&bad,15,S4_IMU_GOOD,S4_CLIMBING,false);
    CHECK(!bad.h.chain_valid&&bad.h.reason==S4_HEIGHT_BASELINE);
    for(i=0;i<2U;++i)
    {
        pipeline(1U,i?S4_IMU_DYNAMIC:S4_IMU_WAIT,&valid,&n);CHECK(valid&&n==1U);
        pipeline(2U,i?S4_IMU_DYNAMIC:S4_IMU_WAIT,&valid,&n);CHECK(valid&&n==2U);
        pipeline(3U,i?S4_IMU_DYNAMIC:S4_IMU_WAIT,&valid,&n);CHECK(!valid);
    }
    pipeline(1U,S4_IMU_FAULT,&valid,&n);CHECK(!valid);
    printf("S4-U14: %s (%u checks, %u failures); bounded quality gaps, real reader/FSM pipeline.\n",failures?"FAIL":"PASS",checks,failures);
    return failures?1:0;
}
