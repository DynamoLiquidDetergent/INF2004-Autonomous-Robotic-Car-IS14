/*
 * subsystem1.h
 * Subsystem 1 (Network Gatekeeper): public interface.
 * Owner: 2502578 (DynamoLiquidDetergent)
 *
 * Only what other subsystems may use goes here.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

#define S1_PRI_TELEM        (22)    /* TEAM.md sec 4: lowest app task */

IMPORT ID g_s1_tid_telem;           /* telemetry task ID (for tests) */

/*
 * IF-18: letter-to-movement map (S1 writes, S5 reads).
 * Word: bits 0-4 FORWARD, 5-9 U-TURN, 10-14 LEFT, 15-19 RIGHT (letter
 * codes, 0 = A .. 25 = Z), bit 20 = valid.  0 until a valid map arrives,
 * so S5 can refuse to start until S1_NAV_MAP_VALID is set.
 * Written in one 32-bit store, so a reader never sees half a map.
 */
#define S1_NAV_MAP_VALID    (0x00100000UL)
#define S1_MOVE_FORWARD     (0U)
#define S1_MOVE_UTURN       (1U)
#define S1_MOVE_LEFT        (2U)
#define S1_MOVE_RIGHT       (3U)

IMPORT UW   s1_get_nav_map(void);
/* Movement for a letter (0 = A .. 25 = Z).  FALSE if no valid map or the
   letter is not in it. */
IMPORT BOOL s1_nav_lookup(UB letter, UB *p_move);
/* S1 only: store an accepted map word (from cmd_parse_map). */
IMPORT void s1_set_nav_map(UW map_word);

/*
 * TEMPORARY: IF-05 compatibility for S4's example monitor, which
 * suspends/resumes tid_led.  tid_led is a stub task that only sleeps.
 * Set to 0 and DELETE once S4 removes IF-05 code.
 */
#define S1_IF05_COMPAT      (1)
#if S1_IF05_COMPAT
#define S1_PRI_LED          (S1_PRI_TELEM)
IMPORT ID tid_led;
#endif

#endif /* SUBSYSTEM1_H */