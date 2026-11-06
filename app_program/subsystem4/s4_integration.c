#include "s4_integration.h"
#include "s4_motion_input.h"
#include "../subsystem2/subsystem2.h"

static s4_motion_reader_t reader;
static ID fault_group_id;
static UINT fault_pattern;
static volatile INT highest_hump_mm=-1;
static volatile UW completed_hump_count;

static bool read_motion(s4_odometry_t *out)
{
    UW before=s2_get_odometry();
    UW status=s2_get_status();
    UW after=s2_get_odometry();
    SYSTIM clock;
    UINT state=s2_status_state(status);
    bool blocked=(state==S2_STATE_TURNING)||(state==S2_STATE_INHIBITED)||
                 (state==S2_STATE_STALLED)||(s2_status_speed_l(status)<0)||
                 (s2_status_speed_r(status)<0);
    /* A bounded attempt: skip a changing pair rather than spin/retry across
     * cores. S2 publishes the two words separately, not as one transaction.
     * Direction/distance guards remain conservative; exact alignment is a
     * physical integration check, not guaranteed by this read pattern. */
    if(before!=after){return false;}
    (void)tk_get_tim(&clock); /* Same kernel clock as actual S2 now_ms(). */
    return s4_motion_snapshot(&reader,(uint32_t)after,
                              (uint8_t)s2_status_epoch(status),blocked,
                              (uint32_t)clock.lo,out);
}
EXPORT ER subsystem4_connect_motion(ID group,UINT bit)
{
    ER result;
    if((group<=0)||(bit==0U)||((bit&(bit-1U))!=0U)){return E_PAR;}
    result=subsystem4_bind_odometry(read_motion);
    if(result!=E_OK){return result;}
    s4_motion_reader_init(&reader);
    fault_group_id=group;fault_pattern=bit;highest_hump_mm=-1;completed_hump_count=0U;
    return E_OK;
}
EXPORT ER s4_integration_publish(const s4_height_t *height,BOOL failed,UW completed_humps)
{
    /* Only the IMU task writes this aligned scalar. Keep the last valid
     * historical maximum even if a subsequent traversal is invalid. */
    highest_hump_mm=height->highest_valid?(INT)(height->highest_mm+0.5F):-1;
    completed_hump_count=completed_humps;
    if(failed)
    {
        if(fault_group_id<=0){return E_NOEXS;}
        /* Persistent local FAULT/BAD timing reasserts the assigned bit.
         * S5 handles/clears it and retains its stop latch while faults recur.
         * Recovery for the latched S4 fault currently requires a restart. */
        return tk_set_flg(fault_group_id,fault_pattern);
    }
    return E_OK;
}
EXPORT INT subsystem4_highest_hump_mm(void){return highest_hump_mm;}
EXPORT UW subsystem4_completed_humps(void){return completed_hump_count;}
