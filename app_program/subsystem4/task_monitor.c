/* Compatibility drain only. No LED control, stats access or alarm task. */
#include "subsystem4.h"

EXPORT void s4_drain_example(void)
{
#if S4_KEEP_EXAMPLE_BRIDGE
    UB text[S4_SUMMARY_MAX];
    unsigned int count;
    for (count = 0U; count < 4U; ++count)
    {
        if (tk_rcv_mbf(mbf_summary, text, TMO_POL) <= 0) { break; }
        ++s4_lines_received;
    }
    /* The old flag has no safety meaning. Only one consumer uses it. */
    (void)tk_clr_flg(flg_events, ~FLG_BATCH);
#endif
}
