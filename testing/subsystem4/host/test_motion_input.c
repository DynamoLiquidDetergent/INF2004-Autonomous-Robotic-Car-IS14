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
    CHECK(s4_motion_snapshot(&r,word(45,180),1,false,180,&o)&&o.forward); /* fresh cumulative data across new command */
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
    s4_motion_reader_init(&r);
    (void)s4_motion_snapshot(&r,word(100,100),0,false,100,&o);
    CHECK(s4_motion_snapshot(&r,word(102,120),0,false,120,&o)&&o.forward);
    CHECK(s4_motion_snapshot(&r,word(102,120),1,false,125,&o)&&o.forward); /* same live word, new command */
    CHECK(s4_motion_snapshot(&r,word(102,120),1,true,130,&o)&&!o.forward);
    CHECK(s4_motion_snapshot(&r,word(102,120),2,false,135,&o)&&!o.forward); /* cannot unblock old data */
    CHECK(s4_motion_snapshot(&r,word(104,140),2,false,140,&o)&&o.forward);
    CHECK(s4_motion_snapshot(&r,word(103,160),3,false,160,&o)&&!o.forward); /* reverse still rejected */
    CHECK(!s4_motion_snapshot(&r,word(154,180),4,false,180,&o)); /* jump still rejected */
    CHECK(!s4_motion_snapshot(&r,word(155,200),5,false,241,&o)); /* stale despite new command */
    s4_motion_reader_init(&r);
    (void)s4_motion_snapshot(&r,word(65530,65530),15,false,65530,&o);
    CHECK(s4_motion_snapshot(&r,word(4,14),0,false,65550,&o)&&o.forward); /* data and epoch wrap */
    printf("S4-U10: %s (%u checks, %u failures); synthetic IF-17 words.\n",failures?"FAIL":"PASS",checks,failures);
    return failures?1:0;
}
