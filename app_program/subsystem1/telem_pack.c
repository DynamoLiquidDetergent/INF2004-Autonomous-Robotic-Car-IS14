/*
 * telem_pack.c
 * Bit packing for subsystem 1's outgoing frames.  See telem_pack.h for
 * the rules and telem_layout.h for the layout.
 */

#include <stddef.h>
#include "telem_pack.h"
#include "outage_buf.h"

/* ---- Compile-time checks: fail the build if the layout is wrong ----- */

/* The telemetry field widths must add up to TELEM_BODY_BITS */
typedef char telem_check_body_bits[
    ((TELEM_W_TIME + TELEM_W_MSTATE + TELEM_W_FAULT +
      (2U * TELEM_W_SPEED) + TELEM_W_MOTION + TELEM_W_DIST +
      TELEM_W_ZONE + (2U * TELEM_W_FLAG) + TELEM_W_LETTER +
      TELEM_W_ACTION + TELEM_W_FLAG + TELEM_W_BARCODES + TELEM_W_HUMP +
      TELEM_W_FLAG + TELEM_W_OBST_DIST + TELEM_W_OBST_SIDE +
      TELEM_W_OBST_WIDTH) == TELEM_BODY_BITS) ? 1 : -1];

/* Each body must fit in its byte count */
typedef char telem_check_body_len[
    (TELEM_BODY_BITS <= (TELEM_BODY_LEN * 8U)) ? 1 : -1];
typedef char telem_check_hb_len[
    ((TELEM_W_HB_SEQ + TELEM_W_MSTATE + TELEM_W_HB_OUTAGE +
      (3U * TELEM_W_FLAG) + TELEM_W_HB_REJECTED) == TELEM_HB_BODY_BITS) &&
    (TELEM_HB_BODY_BITS <= (TELEM_HB_BODY_LEN * 8U)) ? 1 : -1];

/* The outage buffer must hold the largest frame */
typedef char telem_check_outage_fits[
    (TELEM_FRAME_MAX_LEN <= OUTAGE_BUF_FRAME_MAX) ? 1 : -1];

/* ---- Bit cursor ----------------------------------------------------- */

#define TELEM_SPEED_SIGN_BIT  (1UL << (TELEM_W_SPEED - 1U))   /* 0x200 */
#define TELEM_SPEED_SPAN      (1L << TELEM_W_SPEED)           /* 1024  */

typedef struct
{
    uint8_t  *p_buf;     /* writer: body to fill (zeroed first) */
    const uint8_t *p_rd; /* reader: body to read                */
    uint16_t pos;        /* next bit, 0 = bit 0 of byte 0       */
} telem_bits_t;

static void     telem_put(telem_bits_t *p_bits, uint32_t value, uint8_t width);
static uint32_t telem_get(telem_bits_t *p_bits, uint8_t width);
static uint32_t telem_fit(uint32_t value, uint32_t max, uint32_t none);
static uint32_t telem_speed_encode(int16_t speed);
static int16_t  telem_speed_decode(uint32_t raw);
static void     telem_zero(uint8_t *p_buf, uint8_t len);
static uint8_t  telem_header(uint8_t type);
static bool     telem_header_ok(const uint8_t *p_in, uint8_t len,
                                uint8_t want_len, uint8_t want_type);

/* ---- Packing -------------------------------------------------------- */

/*
 * Pack a telemetry frame (TELEM_FRAME_LEN bytes).
 */
uint8_t telem_pack_status(const telem_status_t *p_in,
                          uint8_t *p_out, uint8_t out_size)
{
    telem_bits_t bits;
    uint32_t     letter;

    if ((p_in == NULL) || (p_out == NULL) || (out_size < TELEM_FRAME_LEN))
    {
        return 0U;
    }

    telem_zero(p_out, TELEM_FRAME_LEN);
    p_out[0] = telem_header(TELEM_TYPE_TELEM);

    bits.p_buf = &p_out[TELEM_HDR_LEN];
    bits.p_rd  = NULL;
    bits.pos   = 0U;

    /* A letter past 'Z' is not a letter: send NONE, never clamp to 'Z' */
    letter = p_in->letter;
    if ((letter > TELEM_LETTER_MAX) && (letter != TELEM_LETTER_NONE))
    {
        letter = TELEM_LETTER_NONE;
    }

    telem_put(&bits, p_in->time_ds, TELEM_W_TIME);
    telem_put(&bits, telem_fit(p_in->mission_state, TELEM_MSTATE_MAX,
                               TELEM_MSTATE_NONE), TELEM_W_MSTATE);
    telem_put(&bits, telem_fit(p_in->fault_mask, TELEM_FAULT_MAX,
                               TELEM_FAULT_MAX), TELEM_W_FAULT);
    telem_put(&bits, telem_speed_encode(p_in->speed_left), TELEM_W_SPEED);
    telem_put(&bits, telem_speed_encode(p_in->speed_right), TELEM_W_SPEED);
    telem_put(&bits, telem_fit(p_in->motion_state, TELEM_MOTION_MAX,
                               TELEM_MOTION_MAX), TELEM_W_MOTION);
    telem_put(&bits, p_in->dist_mm, TELEM_W_DIST);
    telem_put(&bits, telem_fit(p_in->line_zone, TELEM_ZONE_MAX,
                               TELEM_ZONE_NONE), TELEM_W_ZONE);
    telem_put(&bits, p_in->b_junction ? 1U : 0U, TELEM_W_FLAG);
    telem_put(&bits, p_in->b_line_lost ? 1U : 0U, TELEM_W_FLAG);
    telem_put(&bits, letter, TELEM_W_LETTER);
    telem_put(&bits, telem_fit(p_in->last_action, TELEM_ACTION_MAX,
                               TELEM_ACTION_MAX), TELEM_W_ACTION);
    telem_put(&bits, p_in->b_map_ok ? 1U : 0U, TELEM_W_FLAG);
    telem_put(&bits, telem_fit(p_in->barcodes, TELEM_BARCODES_MAX,
                               TELEM_BARCODES_MAX), TELEM_W_BARCODES);
    telem_put(&bits, telem_fit(p_in->hump_mm, TELEM_HUMP_MAX,
                               TELEM_HUMP_NONE), TELEM_W_HUMP);
    telem_put(&bits, p_in->b_obstacle ? 1U : 0U, TELEM_W_FLAG);
    telem_put(&bits, telem_fit(p_in->obst_dist_cm, TELEM_OBST_DIST_MAX,
                               TELEM_OBST_DIST_NONE), TELEM_W_OBST_DIST);
    telem_put(&bits, telem_fit(p_in->obst_side, TELEM_OBST_SIDE_MAX,
                               TELEM_OBST_SIDE_MAX), TELEM_W_OBST_SIDE);
    telem_put(&bits, telem_fit(p_in->obst_width_cm, TELEM_OBST_WIDTH_MAX,
                               TELEM_OBST_WIDTH_NONE), TELEM_W_OBST_WIDTH);

    return TELEM_FRAME_LEN;
}

/*
 * Pack a heartbeat frame (TELEM_HB_FRAME_LEN bytes).
 */
uint8_t telem_pack_hb(const telem_hb_t *p_in,
                      uint8_t *p_out, uint8_t out_size)
{
    telem_bits_t bits;

    if ((p_in == NULL) || (p_out == NULL) || (out_size < TELEM_HB_FRAME_LEN))
    {
        return 0U;
    }

    telem_zero(p_out, TELEM_HB_FRAME_LEN);
    p_out[0] = telem_header(TELEM_TYPE_HB);

    bits.p_buf = &p_out[TELEM_HDR_LEN];
    bits.p_rd  = NULL;
    bits.pos   = 0U;

    telem_put(&bits, p_in->seq, TELEM_W_HB_SEQ);
    telem_put(&bits, telem_fit(p_in->mission_state, TELEM_MSTATE_MAX,
                               TELEM_MSTATE_NONE), TELEM_W_MSTATE);
    telem_put(&bits, telem_fit(p_in->outage_count, TELEM_HB_OUTAGE_MAX,
                               TELEM_HB_OUTAGE_MAX), TELEM_W_HB_OUTAGE);
    telem_put(&bits, p_in->b_wifi_up ? 1U : 0U, TELEM_W_FLAG);
    telem_put(&bits, p_in->b_mqtt_up ? 1U : 0U, TELEM_W_FLAG);
    telem_put(&bits, p_in->b_map_ok ? 1U : 0U, TELEM_W_FLAG);
    telem_put(&bits, telem_fit(p_in->rejected, TELEM_HB_REJECTED_MAX,
                               TELEM_HB_REJECTED_MAX), TELEM_W_HB_REJECTED);

    return TELEM_HB_FRAME_LEN;
}

/*
 * Pack an event frame (TELEM_EVT_FRAME_LEN bytes).  The caller builds
 * 'data' from the event's fields; see telem_layout.h for each layout.
 */
uint8_t telem_pack_event(uint8_t type, uint16_t data,
                         uint8_t *p_out, uint8_t out_size)
{
    telem_bits_t bits;

    if ((p_out == NULL) || (out_size < TELEM_EVT_FRAME_LEN) ||
        (type > TELEM_EVT_TYPE_MAX) || (data > TELEM_EVT_DATA_MAX))
    {
        return 0U;
    }

    telem_zero(p_out, TELEM_EVT_FRAME_LEN);
    p_out[0] = telem_header(TELEM_TYPE_EVT);

    bits.p_buf = &p_out[TELEM_HDR_LEN];
    bits.p_rd  = NULL;
    bits.pos   = 0U;

    telem_put(&bits, type, TELEM_W_EVT_TYPE);
    telem_put(&bits, data, TELEM_W_EVT_DATA);

    return TELEM_EVT_FRAME_LEN;
}

/* ---- Unpacking (tests and PC-decoder cross-check) ------------------- */

/*
 * Unpack a telemetry frame.  False if the length, type or version is wrong.
 */
bool telem_unpack_status(const uint8_t *p_in, uint8_t len,
                         telem_status_t *p_out)
{
    telem_bits_t bits;

    if ((p_out == NULL) ||
        !telem_header_ok(p_in, len, TELEM_FRAME_LEN, TELEM_TYPE_TELEM))
    {
        return false;
    }

    bits.p_buf = NULL;
    bits.p_rd  = &p_in[TELEM_HDR_LEN];
    bits.pos   = 0U;

    p_out->time_ds       = (uint16_t)telem_get(&bits, TELEM_W_TIME);
    p_out->mission_state = (uint8_t)telem_get(&bits, TELEM_W_MSTATE);
    p_out->fault_mask    = (uint8_t)telem_get(&bits, TELEM_W_FAULT);
    p_out->speed_left    = telem_speed_decode(telem_get(&bits, TELEM_W_SPEED));
    p_out->speed_right   = telem_speed_decode(telem_get(&bits, TELEM_W_SPEED));
    p_out->motion_state  = (uint8_t)telem_get(&bits, TELEM_W_MOTION);
    p_out->dist_mm       = (uint16_t)telem_get(&bits, TELEM_W_DIST);
    p_out->line_zone     = (uint8_t)telem_get(&bits, TELEM_W_ZONE);
    p_out->b_junction    = (telem_get(&bits, TELEM_W_FLAG) != 0U);
    p_out->b_line_lost   = (telem_get(&bits, TELEM_W_FLAG) != 0U);
    p_out->letter        = (uint8_t)telem_get(&bits, TELEM_W_LETTER);
    p_out->last_action   = (uint8_t)telem_get(&bits, TELEM_W_ACTION);
    p_out->b_map_ok      = (telem_get(&bits, TELEM_W_FLAG) != 0U);
    p_out->barcodes      = (uint8_t)telem_get(&bits, TELEM_W_BARCODES);
    p_out->hump_mm       = (uint8_t)telem_get(&bits, TELEM_W_HUMP);
    p_out->b_obstacle    = (telem_get(&bits, TELEM_W_FLAG) != 0U);
    p_out->obst_dist_cm  = (uint16_t)telem_get(&bits, TELEM_W_OBST_DIST);
    p_out->obst_side     = (uint8_t)telem_get(&bits, TELEM_W_OBST_SIDE);
    p_out->obst_width_cm = (uint8_t)telem_get(&bits, TELEM_W_OBST_WIDTH);

    return true;
}

/*
 * Unpack a heartbeat frame.  False if the length, type or version is wrong.
 */
bool telem_unpack_hb(const uint8_t *p_in, uint8_t len, telem_hb_t *p_out)
{
    telem_bits_t bits;

    if ((p_out == NULL) ||
        !telem_header_ok(p_in, len, TELEM_HB_FRAME_LEN, TELEM_TYPE_HB))
    {
        return false;
    }

    bits.p_buf = NULL;
    bits.p_rd  = &p_in[TELEM_HDR_LEN];
    bits.pos   = 0U;

    p_out->seq           = (uint8_t)telem_get(&bits, TELEM_W_HB_SEQ);
    p_out->mission_state = (uint8_t)telem_get(&bits, TELEM_W_MSTATE);
    p_out->outage_count  = (uint8_t)telem_get(&bits, TELEM_W_HB_OUTAGE);
    p_out->b_wifi_up     = (telem_get(&bits, TELEM_W_FLAG) != 0U);
    p_out->b_mqtt_up     = (telem_get(&bits, TELEM_W_FLAG) != 0U);
    p_out->b_map_ok      = (telem_get(&bits, TELEM_W_FLAG) != 0U);
    p_out->rejected      = (uint8_t)telem_get(&bits, TELEM_W_HB_REJECTED);

    return true;
}

/*
 * Unpack an event frame.  False if the length, type or version is wrong.
 */
bool telem_unpack_event(const uint8_t *p_in, uint8_t len,
                        uint8_t *p_type, uint16_t *p_data)
{
    telem_bits_t bits;

    if ((p_type == NULL) || (p_data == NULL) ||
        !telem_header_ok(p_in, len, TELEM_EVT_FRAME_LEN, TELEM_TYPE_EVT))
    {
        return false;
    }

    bits.p_buf = NULL;
    bits.p_rd  = &p_in[TELEM_HDR_LEN];
    bits.pos   = 0U;

    *p_type = (uint8_t)telem_get(&bits, TELEM_W_EVT_TYPE);
    *p_data = (uint16_t)telem_get(&bits, TELEM_W_EVT_DATA);

    return true;
}

/* ---- Header helpers ------------------------------------------------- */

/*
 * Set the "replayed" bit on a frame taken from the outage buffer.
 */
void telem_mark_replayed(uint8_t *p_frame)
{
    if (p_frame != NULL)
    {
        p_frame[0] = (uint8_t)(p_frame[0] | TELEM_HDR_REPLAYED);
    }
}

uint8_t telem_frame_type(const uint8_t *p_frame)
{
    return (p_frame == NULL) ? 0xFFU :
           (uint8_t)((p_frame[0] >> TELEM_HDR_TYPE_SHIFT) & TELEM_HDR_TYPE_MASK);
}

uint8_t telem_frame_version(const uint8_t *p_frame)
{
    return (p_frame == NULL) ? 0U :
           (uint8_t)(p_frame[0] & TELEM_HDR_VER_MASK);
}

bool telem_is_replayed(const uint8_t *p_frame)
{
    return (p_frame != NULL) && ((p_frame[0] & TELEM_HDR_REPLAYED) != 0U);
}

/* ---- Private helpers ------------------------------------------------ */

/*
 * Write the low 'width' bits of value, LSB first.  The body must already
 * be zeroed.
 */
static void telem_put(telem_bits_t *p_bits, uint32_t value, uint8_t width)
{
    uint8_t i;

    for (i = 0U; i < width; i++)
    {
        if (((value >> i) & 1UL) != 0UL)
        {
            p_bits->p_buf[p_bits->pos >> 3] =
                (uint8_t)(p_bits->p_buf[p_bits->pos >> 3] |
                          (1U << (p_bits->pos & 7U)));
        }
        p_bits->pos++;
    }
}

/*
 * Read 'width' bits, LSB first.
 */
static uint32_t telem_get(telem_bits_t *p_bits, uint8_t width)
{
    uint32_t value = 0UL;
    uint8_t  i;

    for (i = 0U; i < width; i++)
    {
        if (((p_bits->p_rd[p_bits->pos >> 3] >> (p_bits->pos & 7U)) & 1U) != 0U)
        {
            value |= (1UL << i);
        }
        p_bits->pos++;
    }
    return value;
}

/*
 * Keep 'none' as it is; clamp anything above 'max' down to 'max'.
 * For a field with no "no data" value, pass none = max.
 */
static uint32_t telem_fit(uint32_t value, uint32_t max, uint32_t none)
{
    if (value == none)
    {
        return value;
    }
    return (value > max) ? max : value;
}

/*
 * Signed speed -> 10-bit two's complement.  NONE (-512) passes through;
 * everything else is clamped to -511..+511.
 */
static uint32_t telem_speed_encode(int16_t speed)
{
    int32_t s = speed;

    if (s != TELEM_SPEED_NONE)
    {
        if (s > TELEM_SPEED_MAX)
        {
            s = TELEM_SPEED_MAX;
        }
        else if (s < TELEM_SPEED_MIN)
        {
            s = TELEM_SPEED_MIN;
        }
        else
        {
            /* in range */
        }
    }

    if (s < 0)
    {
        s += TELEM_SPEED_SPAN;
    }
    return (uint32_t)s;
}

/*
 * 10-bit two's complement -> signed speed.
 */
static int16_t telem_speed_decode(uint32_t raw)
{
    int32_t s = (int32_t)raw;

    if ((raw & TELEM_SPEED_SIGN_BIT) != 0UL)
    {
        s -= TELEM_SPEED_SPAN;
    }
    return (int16_t)s;
}

static void telem_zero(uint8_t *p_buf, uint8_t len)
{
    uint8_t i;

    for (i = 0U; i < len; i++)
    {
        p_buf[i] = 0U;
    }
}

static uint8_t telem_header(uint8_t type)
{
    return (uint8_t)((TELEM_VERSION & TELEM_HDR_VER_MASK) |
                     ((type & TELEM_HDR_TYPE_MASK) << TELEM_HDR_TYPE_SHIFT));
}

/*
 * True if p_in is want_len bytes and its header has our version and
 * want_type (the replayed bit is ignored).
 */
static bool telem_header_ok(const uint8_t *p_in, uint8_t len,
                            uint8_t want_len, uint8_t want_type)
{
    return (p_in != NULL) && (len == want_len) &&
           (telem_frame_version(p_in) == TELEM_VERSION) &&
           (telem_frame_type(p_in) == want_type);
}