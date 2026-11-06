#ifndef S4_MOTION_INPUT_H
#define S4_MOTION_INPUT_H
#include "s4_odometry.h"

typedef struct
{
    uint32_t previous_word, changed_ms;
    uint8_t previous_epoch;
    bool have_previous, forward;
} s4_motion_reader_t;

/* Decode the existing S2 IF-17 word. Direction is checked using signed
 * modular distance differences, not an assumed command speed. blocked is
 * supplied from S2 state/speed (turn, inhibit, stall or reverse command). */
void s4_motion_reader_init(s4_motion_reader_t *reader);
bool s4_motion_snapshot(s4_motion_reader_t *reader,uint32_t word,uint8_t epoch,
                        bool blocked,uint32_t now_ms,s4_odometry_t *out);
#endif
