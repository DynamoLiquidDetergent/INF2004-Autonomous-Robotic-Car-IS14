/* S4-U12: synthetic powered-test stop policy; not stopping-distance proof. */
#include <stdio.h>
#include "s4_bench_stop.h"
#include "../../../app_program/subsystem5/vc_logic.c"
static unsigned checks,failures;
#define CHECK(x) do {++checks;if(!(x)){++failures;printf("FAIL %d: %s\n",__LINE__,#x);}}while(0)
int main(void)
{
    s4_bench_stop_t s;
    VC_STATE vehicle;VC_IN input={0};VC_OUT output;
    s4_bench_stop_init(&s,0,0,0,0);CHECK(!s4_bench_stop_step(&s,40000,4000,1));
    s4_bench_stop_init(&s,1,0,0,0);CHECK(!s4_bench_stop_step(&s,1000,999,0));
    CHECK(s4_bench_stop_step(&s,1020,1000,0)==S4_BENCH_DISTANCE);
    CHECK(s4_bench_stop_step(&s,1040,0,0)==S4_BENCH_DISTANCE); /* latched */
    s4_bench_stop_init(&s,2,0,100,5);CHECK(!s4_bench_stop_step(&s,1000,500,5));
    CHECK(s4_bench_stop_step(&s,1020,600,6)==S4_BENCH_HUMP);
    s4_bench_stop_init(&s,2,0,0,0);CHECK(!s4_bench_stop_step(&s,1000,2999,0));
    CHECK(s4_bench_stop_step(&s,1020,3000,0)==S4_BENCH_LIMIT);
    s4_bench_stop_init(&s,2,0,100,0);CHECK(!s4_bench_stop_step(&s,1000,90,0)); /* reverse isn't 65 m forward */
    CHECK(s4_bench_stop_step(&s,30000,90,0)==S4_BENCH_TIMEOUT);
    s4_bench_stop_init(&s,1,0xfffffff0U,65000,0);
    CHECK(s4_bench_stop_step(&s,20,464,0)==S4_BENCH_DISTANCE); /* distance/clock wraps */
    s4_bench_stop_init(&s,2,0,0,0xffffffffU);
    CHECK(s4_bench_stop_step(&s,20,1,0)==S4_BENCH_HUMP); /* count wrap */
    s4_bench_stop_init(&s,2,0,0,5);
    CHECK(!s4_bench_stop_step(&s,20,1,0)); /* count restart isn't a completion */
    CHECK(s4_bench_stop_reason(S4_BENCH_HUMP)[0]=='H');
    vc_init(&vehicle,1U);input.ready=1;input.auto_start=1;input.line_fresh=1;
    input.zone=VC_ZONE_CENTRE;input.dt_ms=20U;input.elapsed_ms=20U;
    output=vc_step(&vehicle,&input);
    CHECK(output.mode==VC_MODE_DRIVE&&output.speed>0);
    s4_bench_stop_init(&s,2,0,0,0);
    if(s4_bench_stop_step(&s,1000,500,1)){input.sup_cmd=VC_SUP_ABORT;}
    output=vc_step(&vehicle,&input);
    CHECK(output.mode==VC_MODE_STOP&&vehicle.aborted&&output.mission_state==VC_MS_ABORTED);
    input.sup_cmd=VC_SUP_RESUME;output=vc_step(&vehicle,&input);
    CHECK(output.mode==VC_MODE_STOP&&vehicle.aborted); /* reset, not RESUME, is required */
    vc_init(&vehicle,1U);input.sup_cmd=VC_SUP_NONE;input.motion_state=VC_STATE_INHIBITED;
    output=vc_step(&vehicle,&input);CHECK(output.mode==VC_MODE_STOP); /* normal safety path */
    printf("S4-U12: %s (%u checks, %u failures); synthetic stop policy only.\n",failures?"FAIL":"PASS",checks,failures);
    return failures?1:0;
}
