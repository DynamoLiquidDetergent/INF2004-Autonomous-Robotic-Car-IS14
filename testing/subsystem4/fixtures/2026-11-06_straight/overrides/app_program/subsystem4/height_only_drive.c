/* S4 height-only fixture: sole motion sender, no mission controller.
 * One task at existing controller priority 7; no added flag or message buffer.
 * Speed is set by the local build profile, not a measured speed guarantee. */
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
                         p.phase==H4_RUN?50:0,0,epoch);
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
