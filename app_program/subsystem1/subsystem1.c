/*
 * subsystem1.c
 * Subsystem 1 (Network Gatekeeper): creates objects, starts tasks.
 */

#include "subsystem1.h"
#include "telemetry_task.h"
#include "cmd_parse.h"

EXPORT ID g_s1_tid_telem = 0;

/* IF-18 map word.  volatile: S5's task may read it from the other core. */
static volatile UW s_nav_map = 0U;

/* The bit layouts in subsystem1.h and cmd_parse.h must agree */
typedef char s1_check_map_valid[(S1_NAV_MAP_VALID == CMD_MAP_VALID) ? 1 : -1];
typedef char s1_check_map_moves[((S1_MOVE_FORWARD == CMD_MOVE_FORWARD) &&
                                 (S1_MOVE_UTURN == CMD_MOVE_UTURN) &&
                                 (S1_MOVE_LEFT == CMD_MOVE_LEFT) &&
                                 (S1_MOVE_RIGHT == CMD_MOVE_RIGHT)) ? 1 : -1];

#if S1_IF05_COMPAT
EXPORT ID tid_led = 0;

/* Stub for S4's IF-05: sleeps forever, safe to suspend/resume. DELETE when S4 removes IF-05 code */
static void if05_stub_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}
#endif

/* Create kernel objects.  Message buffers are added in step 2. */
EXPORT ER subsystem1_init(void)
{
    return E_OK;
}

/* Create and start subsystem 1's tasks. */
EXPORT ER subsystem1_start(void)
{
    ER er = E_OK;

    g_s1_tid_telem = app_start_task(telemetry_task, S1_PRI_TELEM, 0,
                                    "telemetry_task");
    if (g_s1_tid_telem < E_OK)
    {
        er = g_s1_tid_telem;
    }

#if S1_IF05_COMPAT
    if (er == E_OK)
    {
        tid_led = app_start_task(if05_stub_task, S1_PRI_LED, 0,
                                 "if05_stub");
        if (tid_led < E_OK)
        {
            er = tid_led;
        }
    }
#endif

    return er;
}

/* IF-18: read the map word (one 32-bit load). */
EXPORT UW s1_get_nav_map(void)
{
    return s_nav_map;
}

/* IF-18: movement for a letter, from one snapshot of the map word. */
EXPORT BOOL s1_nav_lookup(UB letter, UB *p_move)
{
    uint8_t move = 0U;
    BOOL    b_found = FALSE;

    if ((p_move != NULL) && cmd_map_lookup(s_nav_map, letter, &move))
    {
        *p_move = move;
        b_found = TRUE;
    }
    return b_found;
}

/* IF-18: store an accepted map (one 32-bit store). */
EXPORT void s1_set_nav_map(UW map_word)
{
    s_nav_map = map_word;
}