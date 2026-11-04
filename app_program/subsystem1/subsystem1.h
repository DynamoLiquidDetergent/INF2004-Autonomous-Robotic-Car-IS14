/*
 * subsystem1.h
 * Subsystem 1 (Network Gatekeeper): public interface.
 *
 * Only what other subsystems may use goes here.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include <stdint.h>
#include "../app.h"

#define S1_PRI_TELEM        (22)    /* TEAM.md sec 4: lowest app task */

IMPORT ID g_s1_tid_telem;           /* telemetry task ID (for tests) */

/*
 * IF-13: S5 (Vehicle Controller) -> S1, every 500 ms.
 * S5 fills an s1_status_t and calls s1_status_send(): it is packed into
 * two UW (61 bits used, Change Requirement 1) and sent into
 * mbf_telem_status with TMO_POL.  S1 keeps only the newest.
 * PROPOSED: fields and the READY value are still to be confirmed with S5.
 */
#define S1_MSTATE_READY     (1U)    /* PROPOSED: S5's READY state value */
#define S1_MSTATE_NONE      (15U)   /* not known yet                    */

/* Status as S5 fills it in (not sent as is: see s1_status_msg_t) */
typedef struct
{
    uint16_t mission_time_ds;   /* 0.1 s since START                    */
    uint16_t obst_dist_cm;      /* 0..510, 511 = no obstacle seen       */
    uint8_t  mission_state;     /* 0..14; S1_MSTATE_READY = READY; 15 none */
    uint8_t  fault_mask;        /* IF-12 bits: 0 S2, 1 S3, 2 S4, 3 S5   */
    uint8_t  letter;            /* stored letter 0 = A .. 25 = Z, 31 none */
    uint8_t  last_action;       /* S1_MOVE_xxx                          */
    uint8_t  barcodes;          /* barcodes read so far, stops at 15    */
    uint8_t  b_obstacle;        /* 1 = obstacle present                 */
    uint8_t  obst_side;         /* 0 none, 1 left, 2 right              */
    uint8_t  obst_width_cm;     /* 0..62, 63 = unknown                  */
    uint8_t  hump_peak_mm;      /* highest hump so far 0..254, 255 none.
                                   Sender caps real heights at 254: a
                                   uint8_t 255 would read as "none" */
} s1_status_t;

/*
 * The IF-13 message itself: 8 bytes.
 *   w0 bits  0-15 mission_time_ds   w1 bits  0-8  obst_dist_cm
 *      bits 16-19 mission_state        bits  9-10 obst_side
 *      bits 20-23 fault_mask           bits 11-16 obst_width_cm
 *      bits 24-28 letter               bits 17-20 barcodes
 *      bits 29-30 last_action          bits 21-28 hump_peak_mm
 *      bit  31    b_obstacle              bits 29-31 spare, always 0
 * Too-large values are capped before packing, as telemetry does (U13):
 *   - fields with a "no data" value (obstacle distance and width, hump)
 *     keep that value, and anything else is capped at the largest REAL
 *     value (distance 600 -> 510, never 511 "none");
 *   - codes with no larger real value (mission state, letter) and
 *     counters are capped at the field maximum (letter 40 -> 31 none,
 *     barcodes 20 -> 15).
 */
typedef struct
{
    UW w0;
    UW w1;
} s1_status_msg_t;

/* Pack / unpack one IF-13 message.  NULL pointers are ignored. */
IMPORT void s1_status_pack(const s1_status_t *p_st, s1_status_msg_t *p_msg);
IMPORT void s1_status_unpack(const s1_status_msg_t *p_msg, s1_status_t *p_st);
/* S5: pack and send one status (TMO_POL).  E_OK, or the kernel error
   (E_TMOUT if S1 has not taken the last two yet). */
IMPORT ER   s1_status_send(const s1_status_t *p_st);

IMPORT ID mbf_telem_status;         /* IF-13 (owned by S1) */

/*
 * IF-15: S1 <-> mqtt_utk.c.  Message format and topic IDs: s1_topics.h.
 */
IMPORT ID mbf_mqtt_tx;              /* S1 -> broker (owned by S1) */
IMPORT ID mbf_mqtt_rx;              /* broker -> S1 (owned by S1) */

/*
 * IF-18: letter-to-movement map (S1 writes, S5 reads).
 * Word: bits 0-4 FORWARD, 5-9 U-TURN, 10-14 LEFT, 15-19 RIGHT (letter
 * codes, 0 = A .. 25 = Z), bit 20 = valid.  0 until a valid map arrives,
 * so S5 can refuse to start until S1_NAV_MAP_VALID is set.
 * Written in one 32-bit store, so a reader never sees half a map.
 * S5 should read it once at START and use that copy for the whole run.
 */
#define S1_NAV_MAP_VALID    (0x00100000UL)
#define S1_NAV_CODE_BITS    (5U)            /* bits per letter code    */
#define S1_NAV_CODE_MASK    (0x1FUL)        /* one letter code         */
/* Shift of a movement's letter code in the word (move = S1_MOVE_xxx) */
#define S1_NAV_SHIFT(move)  ((UW)(move) * S1_NAV_CODE_BITS)
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
 * TEMPORARY: stand-in for S5 until its real IF-13 sender exists.
 * Normal builds only (test builds send IF-13 from the tests).
 * Set to 0 and DELETE once S5 sends IF-13.
 */
#define S1_FAKE_S5          (1)

/*
 * Debug: in builds WITHOUT MQTT=1, a task takes each message from
 * mbf_mqtt_tx and prints what would be published.  With MQTT=1,
 * mqtt_utk.c publishes them instead and this task is not built.
 * Test builds drain the buffer themselves.  Set to 0 to drop it.
 */
#define S1_TX_SINK          (1)

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