#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "s4_terrain.h"

static void tick(s4_terrain_t *t,s4_orientation_t *o,uint32_t *now,bool fresh)
{*now+=20000U;s4_terrain_step(t,o,true,true,fresh,*now);}
static void pose(s4_terrain_t *t,s4_orientation_t *o,uint32_t *now,float pitch,unsigned n)
{unsigned i;o->pitch_deg=pitch;for(i=0U;i<n;++i){tick(t,o,now,true);}}
static void ready(s4_terrain_t *t,s4_orientation_t *o,uint32_t *now)
{
    s4_terrain_init(t,*now);*o=(s4_orientation_t){0};o->orientation_valid=true;
    pose(t,o,now,0.0F,1U);assert(s4_terrain_state(t)==S4_LEVEL);
    assert(s4_motion_class(t)==S4_UNCLASSIFIED);
}
static void ordered_hump(void)
{
    s4_terrain_t t;s4_orientation_t o;uint32_t now=0U;unsigned i;
    ready(&t,&o,&now);
    /* Brief vibration cannot start climbing. */
    pose(&t,&o,&now,8.0F,2U);assert(s4_terrain_state(&t)==S4_LEVEL);
    pose(&t,&o,&now,0.0F,1U);pose(&t,&o,&now,8.0F,3U);
    assert(s4_terrain_state(&t)==S4_CLIMBING);
    /* A long flat crest still has not proved a downhill segment. */
    for(i=0U;i<20U;++i)
    {pose(&t,&o,&now,0.0F,1U);assert(s4_terrain_state(&t)==S4_CLIMBING);assert(!t.hump_event);}
    pose(&t,&o,&now,-8.0F,2U);assert(s4_terrain_state(&t)==S4_CLIMBING);
    o.orientation_valid=false;tick(&t,&o,&now,true);o.orientation_valid=true;
    pose(&t,&o,&now,-8.0F,2U);assert(s4_terrain_state(&t)==S4_CLIMBING);
    pose(&t,&o,&now,-8.0F,1U);assert(s4_terrain_state(&t)==S4_DESCENDING);
    pose(&t,&o,&now,0.0F,2U);assert(!t.hump_event);
    pose(&t,&o,&now,0.0F,1U);assert(t.hump_event&&s4_terrain_state(&t)==S4_LEVEL);
    for(i=0U;i<10U;++i){tick(&t,&o,&now,true);assert(!t.hump_event);}
    /* A downhill-only profile cannot be a complete hump. */
    pose(&t,&o,&now,-8.0F,10U);pose(&t,&o,&now,0.0F,5U);
    assert(s4_terrain_state(&t)==S4_LEVEL&&!t.hump_event);
}
static void freshness_and_impact(void)
{
    s4_terrain_t t;s4_orientation_t o;uint32_t now=0U;unsigned i;
    ready(&t,&o,&now);pose(&t,&o,&now,8.0F,2U);
    tick(&t,&o,&now,false);pose(&t,&o,&now,8.0F,1U);
    assert(s4_terrain_state(&t)==S4_LEVEL); /* stale value broke the debounce */
    ready(&t,&o,&now);
    o.dynamic_g=0.8F;tick(&t,&o,&now,true);assert(t.impact_event);
    for(i=0U;i<10U;++i){tick(&t,&o,&now,true);assert(!t.impact_event);}
    /* A stale low reading must not rearm a sustained impact. */
    o.dynamic_g=0.0F;tick(&t,&o,&now,false);assert(!t.impact_armed);
    o.dynamic_g=0.8F;tick(&t,&o,&now,true);assert(!t.impact_event);
    o.dynamic_g=0.4F;tick(&t,&o,&now,true);assert(!t.impact_armed);
    o.dynamic_g=0.39F;tick(&t,&o,&now,true);assert(t.impact_armed);
    o.dynamic_g=1.0F;o.orientation_valid=false;tick(&t,&o,&now,true);
    assert(t.impact_event); /* raw impact survives a rejected tilt estimate */
    tick(&t,&o,&now,true);assert(!t.impact_event);
}
static void timeouts_and_faults(void)
{
    s4_terrain_t t;s4_orientation_t o={0};uint32_t now=0U,start;
    s4_terrain_init(&t,now);
    s4_terrain_step(&t,&o,false,true,false,29999999U);
    assert(s4_terrain_state(&t)==S4_CALIBRATING);
    s4_terrain_step(&t,&o,false,true,false,30000000U);
    assert(s4_terrain_state(&t)==S4_FAULT);
    o.orientation_valid=true;pose(&t,&o,&now,0.0F,5U);
    assert(s4_terrain_state(&t)==S4_FAULT); /* good samples cannot clear fault */
    /* Calibration deadline across the 32-bit microsecond wrap. */
    start=UINT32_MAX-1000000U;s4_terrain_init(&t,start);
    now=start+t.config.calibration_timeout_us-1U;
    s4_terrain_step(&t,&o,false,true,true,now);
    assert(s4_terrain_state(&t)==S4_CALIBRATING);
    s4_terrain_step(&t,&o,false,true,true,now+1U);
    assert(s4_terrain_state(&t)==S4_FAULT);
    /* Incomplete traversals abort exactly at the deadline, without a hump
     * or a same-call restart even when the current pitch is still uphill. */
    now=UINT32_MAX-100000U;ready(&t,&o,&now);pose(&t,&o,&now,8.0F,3U);
    assert(s4_terrain_state(&t)==S4_CLIMBING);
    now=t.traversal_start_us+t.config.traversal_timeout_us-1U;
    s4_terrain_step(&t,&o,true,true,true,now);assert(!t.traversal_aborted);
    s4_terrain_step(&t,&o,true,true,true,now+1U);
    assert(t.traversal_aborted&&!t.hump_event&&s4_terrain_state(&t)==S4_LEVEL);
    pose(&t,&o,&now,0.0F,1U);assert(!t.traversal_aborted&&!t.hump_event);
    ready(&t,&o,&now);pose(&t,&o,&now,8.0F,3U);
    s4_terrain_step(&t,&o,true,false,true,now+20000U);
    assert(s4_terrain_state(&t)==S4_FAULT&&t.traversal_aborted);
    pose(&t,&o,&now,0.0F,1U);assert(s4_terrain_state(&t)==S4_FAULT&&!t.traversal_aborted);
    ready(&t,&o,&now);s4_terrain_step(&t,&o,false,true,true,now+20000U);
    assert(s4_terrain_state(&t)==S4_FAULT); /* unexpected loss of calibration */
}
static void classification(void)
{
    s4_terrain_t t;s4_orientation_t o;uint32_t now=0U;
    ready(&t,&o,&now);tick(&t,&o,&now,true);
    assert(s4_motion_class(&t)==S4_UNCLASSIFIED); /* no odometry yet */
    o.dynamic_g=0.2F;tick(&t,&o,&now,true);
    assert(s4_motion_class(&t)==S4_ACCELERATING);
    o.turn_rate_valid=true;o.turn_rate_deg_s=-20.0F;tick(&t,&o,&now,true);
    assert(s4_motion_class(&t)==S4_TURNING);
    o.turn_rate_valid=false;o.dynamic_g=0.0F;tick(&t,&o,&now,true);
    assert(s4_motion_class(&t)==S4_UNCLASSIFIED);
    o.turn_rate_valid=true;o.dynamic_g=0.2F;pose(&t,&o,&now,8.0F,3U);
    assert(s4_motion_class(&t)==S4_MOTION_CLIMBING);
    pose(&t,&o,&now,-8.0F,3U);assert(s4_motion_class(&t)==S4_MOTION_DESCENDING);
    tick(&t,&o,&now,false);assert(s4_motion_class(&t)==S4_UNCLASSIFIED);
    assert(sizeof(t.state_motion)==1U&&(t.state_motion&0xc0U)==0U);
    assert(S4_FAULT<8&&S4_UNCLASSIFIED<8);
}
int main(void)
{
    ordered_hump();freshness_and_impact();timeouts_and_faults();classification();
    puts("terrain: PASS (ordered/debounced hump, freshness, impact rearm, timeout/wrap, fault latch, classification)");
    return 0;
}
