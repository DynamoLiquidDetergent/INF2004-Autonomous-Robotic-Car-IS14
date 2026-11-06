#include "subsystem4.h"

EXPORT ID mbf_summary = 0;
EXPORT ID flg_events = 0;
EXPORT UW s4_lines_received = 0U;
EXPORT ID tid_imu_terrain = 0;

EXPORT ER subsystem4_init(void)
{
#if S4_KEEP_EXAMPLE_BRIDGE
    T_CFLG flags = {0};
    T_CMBF buffer = {0};
    flags.flgatr = TA_TFIFO | TA_WMUL;
    flg_events = tk_cre_flg(&flags);
    if (flg_events < E_OK) { return flg_events; }
    buffer.mbfatr = TA_TFIFO;
    buffer.bufsz = 4 * S4_SUMMARY_MAX;
    buffer.maxmsz = S4_SUMMARY_MAX;
    mbf_summary = tk_cre_mbf(&buffer);
    if (mbf_summary < E_OK)
    {
        (void)tk_del_flg(flg_events);
        return mbf_summary;
    }
#endif
    return E_OK;
}

EXPORT ER subsystem4_start(void)
{
    /* S4 init guard: app_main logs init failures but still calls _start. */
#if S4_KEEP_EXAMPLE_BRIDGE
    if((mbf_summary<=0)||(flg_events<=0)){return E_NOEXS;}
#endif

    tid_imu_terrain = app_start_task((FP)imu_terrain_task, S4_PRIORITY,
                                    0U, "imu_terrain_task");
    return (tid_imu_terrain < E_OK) ? tid_imu_terrain : E_OK;
}
