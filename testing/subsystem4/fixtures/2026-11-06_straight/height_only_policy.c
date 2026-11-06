#include "height_only_policy.h"
uint32_t h4_inputs(int calibrated,int clock_ok,int sensor_alive,int odometry_ok,int level_ready,int fault)
{
    return ((calibrated&&clock_ok&&sensor_alive&&odometry_ok)?H4_HEALTHY:0U)|
           (level_ready?H4_LEVEL:0U)|(fault?H4_FAULT:0U);
}
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
