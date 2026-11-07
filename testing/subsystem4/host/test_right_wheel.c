/* Synthetic axle projection/contact sequence. Not physical accuracy evidence. */
#include <assert.h>
#include <stdio.h>
#include "s4_right_wheel.h"
#include "s4_math.h"
#include "test_math.h"
typedef struct {s4_right_wheel_t r;s4_height_t h;s4_terrain_t t;s4_odometry_t o;unsigned ms,events;} run_t;
static void tick(run_t *v,float height,uint8_t q,bool moving)
{
    s4_orientation_t a={0};float sy=-height/v->r.track_mm,sx=-0.08F;
    if(height==0){sx=0;}
    a.pitch_deg=s4_atan2_deg(sx,s4_sqrt(1-sx*sx));
    a.roll_deg=s4_atan2_deg(sy,s4_sqrt(1-sx*sx-sy*sy));a.orientation_valid=q==S4_IMU_GOOD;
    v->ms+=20;v->o.timestamp=(uint16_t)v->ms;if(moving){++v->o.distance_mm;}
    s4_right_step(&v->r,&v->h,&v->t,&v->o,&a,q,true,true,true,v->ms*1000,v->ms);
    if(v->t.hump_event){++v->events;}
}
static void init(run_t *v,float track)
{
    unsigned i;*v=(run_t){0};v->r.track_mm=track;s4_height_init(&v->h);s4_terrain_init(&v->t,0);
    v->o.valid=true;v->o.forward=true;v->o.clock_format=S4_ODOM_MS16;
    for(i=0;i<8;++i){tick(v,0,S4_IMU_GOOD,false);}assert(v->h.baseline_ready);
}
static void cross(run_t *v,float peak)
{
    unsigned i;for(i=1;i<=12;++i){tick(v,peak*i/12,S4_IMU_GOOD,true);}
    for(i=0;i<5;++i){tick(v,peak,S4_IMU_GOOD,true);}
    for(i=12;i>0;--i){tick(v,peak*(i-1)/12,S4_IMU_GOOD,true);}
    for(i=0;i<9;++i){tick(v,0,S4_IMU_GOOD,true);}
}
int main(void)
{
    run_t v;unsigned i,j;float tracks[3]={90,120,180};float peaks[3]={15,23,35};
    for(i=0;i<3;++i){for(j=0;j<3;++j){init(&v,tracks[i]);cross(&v,peaks[j]);
        assert(v.events==1&&v.h.last_valid&&v.h.highest_valid);
        /* Existing atan2 approximation is bounded in test_processing; allow
         * its sub-mm projection error at the largest synthetic spacing. */
        assert(test_abs(v.h.last_mm-peaks[j])<0.5F);assert(s4_terrain_state(&v.t)==S4_LEVEL);}}
    init(&v,120);for(i=0;i<200;++i){tick(&v,i&1?1.0F:-1.0F,S4_IMU_GOOD,true);}
    assert(!v.events&&!v.h.highest_valid);
    init(&v,120);for(i=0;i<20;++i){tick(&v,-23,S4_IMU_GOOD,true);}assert(!v.events&&!v.h.active);
    init(&v,120);for(i=0;i<8;++i){tick(&v,15,S4_IMU_GOOD,true);}v.o.forward=false;
    tick(&v,20,S4_IMU_GOOD,true);v.o.forward=true;cross(&v,23);assert(!v.h.last_valid&&!v.h.highest_valid);
    init(&v,120);for(i=0;i<8;++i){tick(&v,15,S4_IMU_GOOD,true);}
    tick(&v,15,S4_IMU_FAULT,true);cross(&v,23);assert(!v.h.last_valid);
    init(&v,120);for(i=0;i<8;++i){tick(&v,15,S4_IMU_GOOD,true);}
    for(i=0;i<5;++i){tick(&v,15,S4_IMU_WAIT,true);}cross(&v,23);assert(!v.h.last_valid);
    init(&v,120);cross(&v,23);cross(&v,15);assert(v.events==2&&v.h.highest_mm>22.8F&&v.h.last_mm<15.2F);
    init(&v,120);for(i=0;i<8;++i){tick(&v,15,S4_IMU_GOOD,true);}
    tick(&v,15,S4_IMU_WAIT,true);cross(&v,23);assert(v.h.last_valid&&v.h.interpolated_samples==1);
    init(&v,120);for(i=0;i<8;++i){tick(&v,15,S4_IMU_GOOD,true);}
    {s4_orientation_t a={0};a.orientation_valid=true;
     s4_right_step(&v.r,&v.h,&v.t,&v.o,&a,S4_IMU_GOOD,true,true,false,v.ms*1000,v.ms);}
    cross(&v,23);assert(!v.h.last_valid&&v.h.reason==S4_HEIGHT_TIMEBASE);
    init(&v,120);for(i=0;i<20;++i){tick(&v,15,S4_IMU_GOOD,false);}
    for(i=0;i<20;++i){tick(&v,0,S4_IMU_GOOD,false);}
    assert(v.events==1&&!v.h.last_valid&&v.h.reason==S4_HEIGHT_NO_MOVEMENT);
    init(&v,120);for(i=0;i<8;++i){tick(&v,15,S4_IMU_GOOD,true);}v.ms+=20000;
    tick(&v,15,S4_IMU_GOOD,true);assert(v.t.traversal_aborted&&!v.h.last_valid);
    puts("Right-wheel model: PASS (3 spacings x 3 heights, flat/wrong side, reverse, fault, long gap, highest). Synthetic only.");return 0;
}
