/*
 * telem_pack.h
 * Packs subsystem 1's outgoing frames into compact binary
 * (Change Requirement 1), and unpacks them again for tests and for
 * checking the PC decoder.  Layout: telem_layout.h.
 *
 * Masks and shifts only, no bit-fields (the compiler decides bit-field
 * layout).  Pure C: no kernel calls.
 *
 * Out-of-range values are clamped to the field's largest real value, so
 * they never turn into the "no data" value by accident.  A field passed
 * its NONE value keeps it.
 *
 * Every pack function returns the frame length in bytes, or 0 (nothing
 * written) for a NULL pointer, a buffer that is too small, or an
 * invalid event type / data.
 */

#ifndef TELEM_PACK_H
#define TELEM_PACK_H

#include <stdbool.h>
#include <stdint.h>
#include "telem_layout.h"

typedef struct
{
    uint16_t time_ds;        /* mission time, 0.1 s units           */
    uint8_t  mission_state;
    uint8_t  fault_mask;
    int16_t  speed_left;     /* mm/s                                */
    int16_t  speed_right;    /* mm/s                                */
    uint8_t  motion_state;
    uint16_t dist_mm;        /* since mission start                 */
    uint8_t  line_zone;
    bool     b_junction;
    bool     b_line_lost;
    uint8_t  letter;         /* 0..25 = A..Z, TELEM_LETTER_NONE     */
    uint8_t  last_action;
    bool     b_map_ok;
    uint8_t  barcodes;
    uint8_t  hump_mm;
    bool     b_obstacle;
    uint16_t obst_dist_cm;
    uint8_t  obst_side;
    uint8_t  obst_width_cm;
} telem_status_t;

typedef struct
{
    uint8_t seq;
    uint8_t mission_state;
    uint8_t outage_count;
    bool    b_wifi_up;
    bool    b_mqtt_up;
    bool    b_map_ok;
    uint8_t rejected;
} telem_hb_t;

uint8_t telem_pack_status(const telem_status_t *p_in,
                          uint8_t *p_out, uint8_t out_size);
uint8_t telem_pack_hb(const telem_hb_t *p_in,
                      uint8_t *p_out, uint8_t out_size);
uint8_t telem_pack_event(uint8_t type, uint16_t data,
                         uint8_t *p_out, uint8_t out_size);

bool    telem_unpack_status(const uint8_t *p_in, uint8_t len,
                            telem_status_t *p_out);
bool    telem_unpack_hb(const uint8_t *p_in, uint8_t len,
                        telem_hb_t *p_out);
bool    telem_unpack_event(const uint8_t *p_in, uint8_t len,
                           uint8_t *p_type, uint16_t *p_data);

/* Header helpers */
void    telem_mark_replayed(uint8_t *p_frame);
uint8_t telem_frame_type(const uint8_t *p_frame);
uint8_t telem_frame_version(const uint8_t *p_frame);
bool    telem_is_replayed(const uint8_t *p_frame);

#endif /* TELEM_PACK_H */