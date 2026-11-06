"""Explicit scratch-image overrides; the committed integration is not edited."""
from pathlib import Path
import hashlib

POLICY_H=r'''#ifndef S4_HEIGHT_ONLY_POLICY_H
#define S4_HEIGHT_ONLY_POLICY_H
#include <stdint.h>
#define H4_HEALTHY 1U
#define H4_LEVEL 2U
#define H4_FAULT 4U
enum { H4_WAIT,H4_RUN,H4_STOP };
enum { H4_NONE,H4_HUMP,H4_LIMIT,H4_TIMEOUT,H4_FAULT_STOP,H4_INPUT,H4_NOTREADY };
typedef struct { uint32_t boot_ms,start_ms,ready_ms,start_humps;uint16_t start_mm;
    uint8_t phase,reason,ready; } h4_policy_t;
void h4_init(h4_policy_t *p,uint32_t ms);
void h4_step(h4_policy_t *p,uint32_t ms,uint32_t flags,uint16_t mm,uint32_t humps);
const char *h4_reason(uint8_t reason);
uint32_t h4_snapshot(uint32_t ms,uint32_t flags);
uint32_t h4_live_flags(uint32_t snapshot,uint32_t ms,int running);
#endif
'''
POLICY_C=r'''#include "height_only_policy.h"
uint32_t h4_snapshot(uint32_t ms,uint32_t flags)
{return ((ms&0x1fffffffU)<<3U)|(flags&7U);}
uint32_t h4_live_flags(uint32_t snapshot,uint32_t ms,int running)
{
    uint32_t age=((ms&0x1fffffffU)-(snapshot>>3U))&0x1fffffffU;
    return age<=100U?(snapshot&7U):(running?H4_FAULT:0U);
}
void h4_init(h4_policy_t *p,uint32_t ms)
{p->boot_ms=ms;p->start_ms=0;p->ready_ms=0;p->start_humps=0;p->start_mm=0;
 p->phase=H4_WAIT;p->reason=H4_NONE;p->ready=0;}
void h4_step(h4_policy_t *p,uint32_t ms,uint32_t flags,uint16_t mm,uint32_t humps)
{
    uint16_t moved=(uint16_t)(mm-p->start_mm);
    uint32_t added=humps-p->start_humps;
    if(p->phase==H4_STOP){return;}
    if(flags&H4_FAULT){p->reason=H4_FAULT_STOP;}
    else if(p->phase==H4_WAIT)
    {
        if((flags&(H4_HEALTHY|H4_LEVEL))==(H4_HEALTHY|H4_LEVEL))
        {if(!p->ready){p->ready_ms=ms;p->ready=1;}}
        else {p->ready=0;}
        if((uint32_t)(ms-p->boot_ms)>=30000U){p->reason=H4_NOTREADY;}
        else if(p->ready&&(uint32_t)(ms-p->ready_ms)>=500U&&
                (uint32_t)(ms-p->boot_ms)>=8000U)
        {p->phase=H4_RUN;p->start_ms=ms;p->start_mm=mm;p->start_humps=humps;}
    }
    else if(!(flags&H4_HEALTHY)){p->reason=H4_INPUT;}
    else if(added>0U&&added<0x80000000U){p->reason=H4_HUMP;}
    else if(moved<0x8000U&&moved>=3000U){p->reason=H4_LIMIT;}
    else if((uint32_t)(ms-p->start_ms)>=30000U){p->reason=H4_TIMEOUT;}
    if(p->reason){p->phase=H4_STOP;}
}
const char *h4_reason(uint8_t reason)
{static const char *const text[]={"NONE","HUMP","LIMIT","TIMEOUT","FAULT","INPUT","NOTREADY"};
 return reason<=H4_NOTREADY?text[reason]:"UNKNOWN";}
'''
DRIVER_C=r'''/* S4 height-only fixture: sole motion sender, no mission controller.
 * One task at existing controller priority 7; no added flag or message buffer.
 * 100 mm/s is a preliminary bench setting, not a measured speed guarantee. */
#include "../subsystem5/subsystem5.h"
#include "../subsystem2/subsystem2.h"
#include "s4_integration.h"
#include "height_only_policy.h"
volatile UW s4_height_only_flags;
static UW h4_now(void){SYSTIM t;(void)tk_get_tim(&t);return t.lo;}
EXPORT void s4_height_only_drive_task(INT stacd,void *exinf)
{
    h4_policy_t p;UINT epoch;UW word;UB previous=H4_WAIT;ER er;
    (void)stacd;(void)exinf;h4_init(&p,h4_now());
    epoch=(s2_status_epoch(s2_get_status())+1U)&S2_MASK_4;
    tm_printf((UB*)"S4 HEIGHT ONLY: flat/still startup; straight drive, no line or obstacle task.\n");
    for(;;)
    {
        UW flags=h4_live_flags(s4_height_only_flags,h4_now(),p.phase==H4_RUN);
        UW status=s2_get_status();
        if(s2_status_state(status)==S2_STATE_INHIBITED||
           s2_status_state(status)==S2_STATE_STALLED){flags|=H4_FAULT;}
        h4_step(&p,h4_now(),flags,(uint16_t)s2_odo_dist_mm(s2_get_odometry()),
                subsystem4_completed_humps());
        if(previous!=p.phase&&p.phase==H4_RUN)
            tm_printf((UB*)"S4_HEIGHT_ONLY START kind=2 odo_mm=%u humps=%u\n",
                (UINT)p.start_mm,(UINT)p.start_humps);
        word=s2_cmd_pack(p.phase==H4_RUN?S2_MODE_DRIVE:S2_MODE_STOP,
                         p.phase==H4_RUN?100:0,0,epoch);
        er=tk_snd_mbf(q_motion_cmd,&word,sizeof(word),TMO_POL);
        if(er!=E_OK){tm_printf((UB*)"S4_HEIGHT_ONLY COMMAND_ERROR rc=%d\n",er);
            for(;;){(void)tk_dly_tsk(100);} /* Never return/restart this task. */}
        if(previous!=p.phase&&p.phase==H4_STOP)
            tm_printf((UB*)"S4_HEIGHT_ONLY STOP kind=2 reason=%s odo_mm=%u humps=%u\n",
                h4_reason(p.reason),(UINT)s2_odo_dist_mm(s2_get_odometry()),
                (UINT)subsystem4_completed_humps());
        previous=p.phase;(void)tk_dly_tsk(50);
    }
}
'''
START_BODY=r'''EXPORT ER subsystem5_start(void)
{
    ID tid=app_start_task(safety_monitor_task,S5_PRI_SAFETY,0,"safety_monitor");
    if(tid<E_OK){return tid;}
    tid=app_start_task(s4_height_only_drive_task,S5_PRI_VEHICLE,0,"height_test");
    return tid<E_OK?tid:E_OK;
}
'''
MAIN_BODY=r'''EXPORT INT usermain(void)
{
    ER result;
    /* Create every object before starting tasks on either core. */
#define H4_CHECK(call) do {result=(call);if(result!=E_OK){ \
 tm_printf((UB*)"S4_HEIGHT_ONLY INIT_ERROR rc=%d\n",result);goto failed;}}while(0)
    H4_CHECK(subsystem2_init());
    H4_CHECK(subsystem4_init());
    H4_CHECK(subsystem5_init());
    H4_CHECK(subsystem5_start());
    H4_CHECK(subsystem2_start());
    H4_CHECK(subsystem4_start());
failed:
    (void)tk_slp_tsk(TMO_FEVR);return 0;
#undef H4_CHECK
}
'''

def apply(source,metadata):
    changes={}
    def write(rel,text):
        path=source/rel
        before=path.read_bytes() if path.exists() else None
        path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text,encoding='utf-8',newline='\n')
        changes[rel]={'before_sha256':hashlib.sha256(before).hexdigest() if before else None,
                      'effective_sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    def replace(rel,old,new):
        text=(source/rel).read_text()
        if text.count(old)!=1:raise RuntimeError('Unexpected test-image source: '+rel)
        write(rel,text.replace(old,new))
    root='app_program/subsystem4/'
    write(root+'height_only_policy.h',POLICY_H)
    write(root+'height_only_policy.c',POLICY_C)
    write(root+'height_only_drive.c',DRIVER_C)
    path=source/'app_program/app_main.c';text=path.read_text();token='EXPORT INT usermain(void)'
    if text.count(token)!=1:raise RuntimeError('Unexpected app_main entry')
    write('app_program/app_main.c',text[:text.index(token)]+MAIN_BODY)
    path=source/'app_program/subsystem5/subsystem5.c';text=path.read_text();token='EXPORT ER subsystem5_start(void)'
    if text.count(token)!=1:raise RuntimeError('Unexpected S5 start entry')
    text=text[:text.index(token)]+START_BODY
    write('app_program/subsystem5/subsystem5.c',text.replace('EXPORT ER subsystem5_start(void)',
          'IMPORT void s4_height_only_drive_task(INT,void *);\nEXPORT ER subsystem5_start(void)',1))
    # S1 and S3 are neither initialised nor started; no line/ADC task can steer.
    # Legacy bridge is unused in this isolated image; keep committed code intact.
    replace(root+'s4_config.h','#define S4_KEEP_EXAMPLE_BRIDGE    1','#define S4_KEEP_EXAMPLE_BRIDGE    0')
    # Disable any legacy sample producer in the test image, independently of default.
    import re
    path=source/'app_program/subsystem2/subsystem2.h';text=path.read_text()
    text,count=re.subn(r'(?m)^(#define\s+S2_KEEP_EXAMPLE\s+)\d+',lambda m:m[1]+'0',text)
    if count!=1:raise RuntimeError('Unexpected S2 legacy setting')
    write('app_program/subsystem2/subsystem2.h',text)
    replace(root+'imu_terrain_task.c','#include "s4_integration.h"',
            '#include "s4_integration.h"\n#include "height_only_policy.h"\nextern volatile UW s4_height_only_flags;')
    replace(root+'imu_terrain_task.c','        s4_drain_example();++cycles;',
       '''        /* Test readiness only; estimator, axis map and live odometry unchanged. */
        s4_height_only_flags=h4_snapshot(clock.lo,((processor.calibrated&&fresh&&orientation.orientation_valid&&
            timebase.status==S4_CLOCK_CONSISTENT&&height.odometry_valid&&odometry.forward)?H4_HEALTHY:0U)|
            (state==S4_LEVEL?H4_LEVEL:0U)|
            ((state==S4_FAULT||timebase.status==S4_CLOCK_BAD||fault_rc!=E_OK)?H4_FAULT:0U));
        s4_drain_example();++cycles;''')
    # Merge if a generic tag/profile override already touched one of these paths.
    for rel,value in changes.items():
        if rel in metadata['image_overrides']:value['before_sha256']=metadata['image_overrides'][rel]['before_sha256']
        metadata['image_overrides'][rel]=value
    metadata['test_fixture']={'speed_command_mm_s':100,'period_ms':50,
       'startup_min_ms':8000,'ready_hold_ms':500,'startup_limit_ms':30000,
       'run_limit_ms':30000,'distance_limit_mm':3000,
       'active_tasks':['S2 Motion Control','S4 IMU/Terrain','S5 Safety Monitor','temporary straight-drive fixture'],
       'inactive_tasks':['S1 networking','S3 line/barcode','S5 obstacle detection','S5 mission controller'],
       'scope':'Physical S4 hump-height experiment; no line following or independent odometry certification.'}
