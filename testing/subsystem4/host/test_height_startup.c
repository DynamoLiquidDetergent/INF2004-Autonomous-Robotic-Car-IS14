/* S4-U13: synthetic real-reader -> terrain FSM -> height integration checks.
 * Command epochs change while cumulative S2 odometry remains continuous.
 * No physical hump measurement is claimed. */
#include <stdio.h>
#include "test_math.h"
#include "s4_motion_input.h"
#include "s4_terrain.h"
static unsigned checks,failures;
#define CHECK(x) do{++checks;if(!(x)){++failures;printf("FAIL %d: %s\n",__LINE__,#x);}}while(0)
typedef struct {s4_motion_reader_t reader;s4_height_t height;s4_terrain_t terrain;
    s4_odometry_t odo;s4_orientation_t angle;uint32_t ms;uint16_t mm;unsigned humps;} run_t;
static void init(run_t *r)
{
    *r=(run_t){0};s4_motion_reader_init(&r->reader);s4_height_init(&r->height);
    s4_terrain_init(&r->terrain,0U);r->angle.orientation_valid=true;
}
static void sample(run_t *r,float pitch,int moved,uint8_t epoch,bool blocked,unsigned delay)
{
    uint32_t word;r->ms+=delay;r->mm=(uint16_t)(r->mm+moved);
    word=((uint32_t)r->mm<<16U)|(uint16_t)r->ms;
    (void)s4_motion_snapshot(&r->reader,word,epoch,blocked,r->ms,&r->odo);
    r->angle.pitch_deg=pitch;
    s4_terrain_step(&r->terrain,&r->angle,true,true,true,r->ms*1000U);
    if(r->terrain.hump_event){++r->humps;}
    s4_height_step(&r->height,&r->odo,&r->angle,true,true,r->ms*1000U,r->ms,
        s4_terrain_state(&r->terrain),2.0F,r->terrain.hump_event,r->terrain.traversal_aborted);
}
static void traverse(run_t *r,unsigned epoch_at,unsigned failure)
{
    unsigned i;init(r);
    for(i=0;i<25U;++i)
    {
        float pitch=i<5U?0.0F:i==5U?1.0F:i==6U?3.0F:i<10U?10.0F:i<15U?20.0F:i<19U?-10.0F:0.0F;
        int mm=i<5U?0:2;unsigned delay=20U;bool blocked=false;
        if(i==12U){if(failure==1U){blocked=true;}if(failure==2U){mm=-1;}
                   if(failure==3U){delay=80U;}if(failure==4U){mm=51;}}
        sample(r,pitch,mm,(uint8_t)(i>=epoch_at?1U:0U),blocked,delay);
    }
}
int main(void)
{
    run_t normal,start,mid,bad;unsigned i;
    traverse(&normal,99U,0U);CHECK(normal.humps==1U&&normal.height.last_valid);
    traverse(&start,5U,0U);
    CHECK(start.humps==1U&&start.height.last_valid); /* old reader loses the level baseline here */
    CHECK(start.height.reason==S4_HEIGHT_OK&&start.height.highest_valid);
    CHECK(test_abs(start.height.last_mm-normal.height.last_mm)<0.001F);
    traverse(&mid,12U,0U);CHECK(mid.humps==1U&&mid.height.last_valid);
    CHECK(test_abs(mid.height.last_mm-normal.height.last_mm)<0.001F);
    for(i=1U;i<=4U;++i)
    {traverse(&bad,5U,i);CHECK(!bad.height.last_valid&&!bad.height.highest_valid);}
    /* No actual flat baseline still means no height, even with good new data. */
    init(&bad);
    (void)s4_motion_snapshot(&bad.reader,0U,1U,false,0U,&bad.odo); /* reader primed, no flat height sample */
    for(i=0U;i<5U;++i){sample(&bad,10.0F,2,1U,false,20U);}
    for(i=0U;i<4U;++i){sample(&bad,-10.0F,2,1U,false,20U);}
    for(i=0U;i<4U;++i){sample(&bad,0.0F,2,1U,false,20U);}
    CHECK(bad.humps==1U&&!bad.height.last_valid&&bad.height.reason==S4_HEIGHT_BASELINE);
    printf("S4-U13: %s (%u checks, %u failures); synthetic command-start/height regression only.\n",failures?"FAIL":"PASS",checks,failures);
    return failures?1:0;
}
