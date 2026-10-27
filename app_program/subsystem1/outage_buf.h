/*
 * outage_buf.h
 * Outage buffer for subsystem 1.
 *
 * While the link is down, telemetry frames are kept here and replayed
 * when it comes back (IF-15: keep 20 messages, drop the oldest).
 *
 * Notes: This is pure C (no kernel calls, no malloc).  Each buffer is a caller-owned
 * outage_buf_t, so the tests use their OWN & never touch the one the
 * telemetry task uses.
 *
 * Not thread-safe: only one task may use a given buffer.  In S1,
 * telemetry_task is the only task that owns the outage buffer.
 */

#ifndef OUTAGE_BUF_H
#define OUTAGE_BUF_H

#include <stdbool.h>
#include <stdint.h>

#define OUTAGE_BUF_DEPTH      (20U)   /* frames kept (IF-15)            */
#define OUTAGE_BUF_FRAME_MAX  (15U)   /* bytes; = largest S1 frame
                                         (TELEM_FRAME_MAX_LEN, checked
                                         at build time in telem_pack.c) */

typedef struct
{
    uint8_t len;
    uint8_t data[OUTAGE_BUF_FRAME_MAX];
} outage_frame_t;

typedef struct
{
    outage_frame_t frames[OUTAGE_BUF_DEPTH];
    uint8_t        head;      /* index of the oldest frame           */
    uint8_t        count;     /* frames stored, 0..OUTAGE_BUF_DEPTH  */
    uint16_t       dropped;   /* oldest frames overwritten (saturates) */
} outage_buf_t;

void     outage_buf_init(outage_buf_t *p_buf);
bool     outage_buf_push(outage_buf_t *p_buf,
                         const uint8_t *p_frame, uint8_t len);
bool     outage_buf_pop(outage_buf_t *p_buf,
                        uint8_t *p_out, uint8_t out_size, uint8_t *p_len);
uint8_t  outage_buf_count(const outage_buf_t *p_buf);
uint16_t outage_buf_dropped(const outage_buf_t *p_buf);

#endif /* OUTAGE_BUF_H */