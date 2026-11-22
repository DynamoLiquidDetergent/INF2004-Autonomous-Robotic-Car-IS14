/*
 * outage_buf.c
 * Fixed-size ring of telemetry frames.  When full, a push overwrites the
 * oldest frame and counts it as dropped (IF-15: 20 frames, drop-oldest).
 */

#include <stddef.h>
#include <string.h>
#include "outage_buf.h"

#define OUTAGE_BUF_DROPPED_MAX  (0xFFFFU)

static uint8_t outage_buf_next(uint8_t index);

/*
 * Empty buffer & clear drop count.
 */
void outage_buf_init(outage_buf_t *p_buf)
{
    if (p_buf != NULL)
    {
        p_buf->head    = 0U;
        p_buf->count   = 0U;
        p_buf->dropped = 0U;
    }
}

/*
 * Copy one frame in.  If buffer is full, oldest frame is
 * overwritten & counted as dropped.
 * Returns false (stores nothing) for a NULL pointer, an empty frame
 * or a frame longer than OUTAGE_BUF_FRAME_MAX.
 */
bool outage_buf_push(outage_buf_t *p_buf,
                     const uint8_t *p_frame, uint8_t len)
{
    uint8_t tail;

    if ((p_buf == NULL) || (p_frame == NULL) ||
        (len == 0U) || (len > OUTAGE_BUF_FRAME_MAX))
    {
        return false;
    }

    if (p_buf->count == OUTAGE_BUF_DEPTH)
    {
        /* Full: forget the oldest frame to make room */
        p_buf->head = outage_buf_next(p_buf->head);
        p_buf->count--;

        if (p_buf->dropped < OUTAGE_BUF_DROPPED_MAX)
        {
            p_buf->dropped++;
        }
    }

    tail = (uint8_t)((p_buf->head + p_buf->count) % OUTAGE_BUF_DEPTH);
    (void)memcpy(p_buf->frames[tail].data, p_frame, len);
    p_buf->frames[tail].len = len;
    p_buf->count++;

    return true;
}

/*
 * Take the oldest frame out, copying it to p_out & its length to p_len.
 * Returns false if buffer is empty, a pointer is NULL, or out_size is
 * too small.  When out_size is too small, frame stays in buffer.
 */
bool outage_buf_pop(outage_buf_t *p_buf,
                    uint8_t *p_out, uint8_t out_size, uint8_t *p_len)
{
    const outage_frame_t *p_frame;

    if ((p_buf == NULL) || (p_out == NULL) || (p_len == NULL) ||
        (p_buf->count == 0U))
    {
        return false;
    }

    p_frame = &p_buf->frames[p_buf->head];

    if (p_frame->len > out_size)
    {
        return false;
    }

    (void)memcpy(p_out, p_frame->data, p_frame->len);
    *p_len = p_frame->len;

    p_buf->head = outage_buf_next(p_buf->head);
    p_buf->count--;

    return true;
}

/*
 * Number of frames waiting to be replayed.
 */
uint8_t outage_buf_count(const outage_buf_t *p_buf)
{
    return (p_buf == NULL) ? 0U : p_buf->count;
}

/*
 * Number of frames lost because the buffer was full (saturates at 65535).
 */
uint16_t outage_buf_dropped(const outage_buf_t *p_buf)
{
    return (p_buf == NULL) ? 0U : p_buf->dropped;
}

/*
 * Ring index after 'index', wrapping at OUTAGE_BUF_DEPTH.
 */
static uint8_t outage_buf_next(uint8_t index)
{
    return (uint8_t)((index + 1U) % OUTAGE_BUF_DEPTH);
}