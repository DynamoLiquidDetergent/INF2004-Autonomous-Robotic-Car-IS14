#include "s4_bench_stop.h"
void s4_bench_stop_init(s4_bench_stop_t *s,uint8_t kind,uint32_t ms,
                        uint16_t mm,uint32_t humps)
{
    s->kind=kind;s->start_ms=ms;s->start_mm=mm;s->start_humps=humps;
    s->reason=S4_BENCH_CONTINUE;
}
uint8_t s4_bench_stop_step(s4_bench_stop_t *s,uint32_t ms,uint16_t mm,uint32_t humps)
{
    uint16_t travelled=(uint16_t)(mm-s->start_mm);
    uint32_t added=humps-s->start_humps;
    if(s->reason||!s->kind){return s->reason;}
    if((s->kind==2U)&&(added>0U)&&(added<0x80000000U)){s->reason=S4_BENCH_HUMP;}
    else if((travelled<0x8000U)&&(travelled>=((s->kind==1U)?1000U:3000U)))
    {s->reason=(s->kind==1U)?S4_BENCH_DISTANCE:S4_BENCH_LIMIT;}
    else if((uint32_t)(ms-s->start_ms)>=30000U){s->reason=S4_BENCH_TIMEOUT;}
    return s->reason;
}
const char *s4_bench_stop_reason(uint8_t reason)
{
    static const char *const text[]={"CONTINUE","DISTANCE","HUMP","LIMIT","TIMEOUT"};
    return reason<=S4_BENCH_TIMEOUT?text[reason]:"UNKNOWN";
}
