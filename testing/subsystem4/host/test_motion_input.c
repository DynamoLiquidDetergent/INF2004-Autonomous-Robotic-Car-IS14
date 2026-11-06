/* S4-U10: real IF-17 decoder/guard, with synthetic S2 words only. */
#include <stdio.h>
#include "s4_motion_input.h"
static unsigned checks,failures;
#define CHECK(x) do { ++checks; if(!(x)){++failures;printf("FAIL %d: %s\n",__LINE__,#x);} } while(0)
static uint32_t word(unsigned distance,unsigned ms)
{return ((uint32_t)(uint16_t)distance<<16U)|(uint16_t)ms;}
int main(void)
{
    s4_motion_reader_t r;s4_odometry_t o;
    s4_motion_reader_init(&r);
    CHECK(!s4_motion_snapshot(&r,word(0,0),0,false,0,&o)&&!o.valid);
    CHECK(s4_motion_snapshot(&r,word(0,20),0,false,20,&o)&&o.forward);
    CHECK(s4_motion_snapshot(&r,word(10,40),0,false,40,&o)&&o.distance_mm==10U);
    CHECK(s4_motion_snapshot(&r,word(10,40),0,false,60,&o)&&o.forward);
    CHECK(!s4_motion_snapshot(&r,word(10,40),0,false,81,&o));
    CHECK(!s4_motion_snapshot(&r,word(10,40),0,false,40U+65536U,&o));
    s4_motion_reader_init(&r);
    (void)s4_motion_snapshot(&r,word(10,40),0,false,40,&o);
    CHECK(!s4_motion_snapshot(&r,word(20,110),0,false,110,&o)); /* 70 ms gap */
    CHECK(s4_motion_snapshot(&r,word(30,120),0,false,120,&o));
    CHECK(s4_motion_snapshot(&r,word(25,140),0,false,140,&o)&&!o.forward);
    CHECK(s4_motion_snapshot(&r,word(25,140),0,false,150,&o)&&!o.forward);
    CHECK(s4_motion_snapshot(&r,word(35,160),0,true,160,&o)&&!o.forward);
    CHECK(!s4_motion_snapshot(&r,word(45,180),1,false,180,&o)); /* new command epoch */
    CHECK(s4_motion_snapshot(&r,word(55,200),1,false,200,&o)&&o.forward);
    CHECK(!s4_motion_snapshot(&r,word(56,200),1,false,200,&o)); /* changed distance, same time */
    CHECK(!s4_motion_snapshot(&r,word(107,220),1,false,220,&o)); /* excessive jump */
    CHECK(s4_motion_snapshot(&r,word(117,240),1,false,240,&o));
    CHECK(!s4_motion_snapshot(&r,word(127,320),1,false,320,&o)); /* source gap */
    CHECK(s4_motion_snapshot(&r,word(137,340),1,false,340,&o));
    CHECK(!s4_motion_snapshot(&r,word(147,360),1,false,401,&o)); /* stale new word */
    s4_motion_reader_init(&r);
    CHECK(!s4_motion_snapshot(&r,word(65530,65530),0,false,65530,&o));
    CHECK(s4_motion_snapshot(&r,word(4,14),0,false,65550,&o)&&o.forward);
    CHECK(o.clock_format==S4_ODOM_MS16&&o.timestamp==14U&&o.distance_mm==4U);
    s4_motion_reader_init(&r);
    CHECK(!s4_motion_snapshot(&r,word(0,65530),0,false,0xfffffffaU,&o));
    CHECK(s4_motion_snapshot(&r,word(5,14),0,false,14U,&o)); /* full local-ms wrap */
    printf("S4-U10: %s (%u checks, %u failures); synthetic IF-17 words.\n",failures?"FAIL":"PASS",checks,failures);
    return failures?1:0;
}
