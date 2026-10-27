/*
 * telem_layout.h
 * Bit layout of every frame subsystem 1 publishes (Change Requirement 1).
 *
 * THIS IS THE ONLY PLACE S1 FIELD WIDTHS ARE DEFINED.  The packer and the
 * tests use these constants, so changing a width here is enough.
 * Bump TELEM_VERSION whenever any layout changes.
 *
 * Widths marked PROVISIONAL are working assumptions until other buddies confirm them.
 *
 * Every frame: 1 header byte, then the body packed LSB-first from bit 0
 * of byte 1, fields in the order listed.
 * "NONE" = all-ones value meaning "no data yet" (team convention).
 */

#ifndef TELEM_LAYOUT_H
#define TELEM_LAYOUT_H

/* ---- Header byte ---------------------------------------------------- */
#define TELEM_VERSION            (1U)
#define TELEM_HDR_VER_MASK       (0x07U)   /* bits 0-2 */
#define TELEM_HDR_TYPE_SHIFT     (3U)      /* bits 3-5 */
#define TELEM_HDR_TYPE_MASK      (0x07U)
#define TELEM_HDR_REPLAYED       (0x40U)   /* bit 6    */
#define TELEM_HDR_LEN            (1U)      /* bytes    */

#define TELEM_TYPE_TELEM         (0U)
#define TELEM_TYPE_HB            (1U)
#define TELEM_TYPE_EVT           (2U)

/* ---- Telemetry body: 106 bits -> 14 bytes --------------------------- */
#define TELEM_W_TIME             (16U)  /* 0.1 s units        PROVISIONAL */
#define TELEM_W_MSTATE           (4U)   /* mission state, IF-13        */
#define TELEM_W_FAULT            (4U)   /* fault mask, IF-12           */
#define TELEM_W_SPEED            (10U)  /* signed mm/s, IF-16 (x2)     */
#define TELEM_W_MOTION           (3U)   /* motion state, IF-16         */
#define TELEM_W_DIST             (16U)  /* mm since mission start      */
#define TELEM_W_ZONE             (3U)   /* line zone, IF-05  PROVISIONAL */
#define TELEM_W_FLAG             (1U)   /* any 0/1 field               */
#define TELEM_W_LETTER           (5U)   /* 0..25 = A..Z                */
#define TELEM_W_ACTION           (2U)   /* F, U, L, R                  */
#define TELEM_W_BARCODES         (4U)   /* count          PROVISIONAL  */
#define TELEM_W_HUMP             (8U)   /* mm             PROVISIONAL  */
#define TELEM_W_OBST_DIST        (9U)   /* cm             PROVISIONAL  */
#define TELEM_W_OBST_SIDE        (2U)   /* 0 none, 1 L, 2 R            */
#define TELEM_W_OBST_WIDTH       (6U)   /* cm             PROVISIONAL  */

#define TELEM_BODY_BITS          (106U)
#define TELEM_BODY_LEN           (14U)  /* bytes, 6 bits spare */
#define TELEM_FRAME_LEN          (TELEM_HDR_LEN + TELEM_BODY_LEN)  /* 15 */

/* Largest real value, and the "no data" value, per field */
#define TELEM_MSTATE_MAX         (14U)
#define TELEM_MSTATE_NONE        (15U)
#define TELEM_FAULT_MAX          (15U)
#define TELEM_SPEED_MAX          (511)
#define TELEM_SPEED_MIN          (-511)
#define TELEM_SPEED_NONE         (-512)
#define TELEM_MOTION_MAX         (7U)
#define TELEM_ZONE_MAX           (6U)
#define TELEM_ZONE_NONE          (7U)
#define TELEM_LETTER_MAX         (25U)  /* 'Z' */
#define TELEM_LETTER_NONE        (31U)
#define TELEM_ACTION_MAX         (3U)
#define TELEM_BARCODES_MAX       (15U)  /* stops at 15 */
#define TELEM_HUMP_MAX           (254U)
#define TELEM_HUMP_NONE          (255U)
#define TELEM_OBST_DIST_MAX      (510U)
#define TELEM_OBST_DIST_NONE     (511U)
#define TELEM_OBST_SIDE_MAX      (3U)
#define TELEM_OBST_WIDTH_MAX     (62U)
#define TELEM_OBST_WIDTH_NONE    (63U)

/* ---- Heartbeat body: 24 bits -> 3 bytes ----------------------------- */
#define TELEM_W_HB_SEQ           (8U)
#define TELEM_W_HB_OUTAGE        (5U)   /* frames in outage buffer, 0..20 */
#define TELEM_W_HB_REJECTED      (4U)   /* stops at 15 */

#define TELEM_HB_BODY_BITS       (24U)
#define TELEM_HB_BODY_LEN        (3U)
#define TELEM_HB_FRAME_LEN       (TELEM_HDR_LEN + TELEM_HB_BODY_LEN)  /* 4 */

#define TELEM_HB_OUTAGE_MAX      (31U)
#define TELEM_HB_REJECTED_MAX    (15U)

/* ---- Event body: 3-bit type + 13-bit data -> 2 bytes ----------------- */
#define TELEM_W_EVT_TYPE         (3U)
#define TELEM_W_EVT_DATA         (13U)
#define TELEM_EVT_DATA_MAX       (0x1FFFU)

#define TELEM_EVT_BODY_LEN       (2U)
#define TELEM_EVT_FRAME_LEN      (TELEM_HDR_LEN + TELEM_EVT_BODY_LEN)  /* 3 */

#define TELEM_EVT_BARCODE        (0U)   /* letter 5, reversed 1        */
#define TELEM_EVT_JUNCTION       (1U)   /* letter 5, action 2          */
#define TELEM_EVT_MAP_ACK        (2U)   /* accepted 1, reason 2        */
#define TELEM_EVT_CMD_ACK        (3U)   /* command 2, accepted 1       */
#define TELEM_EVT_HUMP           (4U)   /* height mm 8                 */
#define TELEM_EVT_OBSTACLE       (5U)   /* width cm 6, side 2          */
#define TELEM_EVT_STATE_CHANGE   (6U)   /* new state 4, old state 4    */
#define TELEM_EVT_FAULT          (7U)   /* fault mask 4                */
#define TELEM_EVT_TYPE_MAX       (7U)

/* Largest frame of any type: the outage buffer must hold this */
#define TELEM_FRAME_MAX_LEN      (TELEM_FRAME_LEN)

#endif /* TELEM_LAYOUT_H */