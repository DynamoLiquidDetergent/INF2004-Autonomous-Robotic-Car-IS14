#include "s4_motion_input.h"
#include "s4_memory.h"

void s4_motion_reader_init(s4_motion_reader_t *r){s4_zero(r,sizeof(*r));}

bool s4_motion_snapshot(s4_motion_reader_t *r,uint32_t word,uint8_t epoch,
                        bool blocked,uint32_t now_ms,s4_odometry_t *out)
{
    uint16_t age=(uint16_t)((uint16_t)now_ms-(uint16_t)word);
    bool valid=false;
    s4_odometry_decode_word(word,false,out);
    out->valid=false;
    if(!r->have_previous)
    {
        r->previous_word=word;r->changed_ms=now_ms;r->previous_epoch=epoch;
        r->have_previous=true;return false; /* Establish an actual baseline. */
    }
    if(word==r->previous_word)
    {
        valid=(age<=40U)&&((uint32_t)(now_ms-r->changed_ms)<=40U)&&
              (epoch==r->previous_epoch);
        out->forward=r->forward&&!blocked;
        /* If a manoeuvre starts between odometry updates, keep it blocked
         * until a new snapshot; an unchanged word cannot prove recovery. */
        if(blocked||epoch!=r->previous_epoch){r->forward=false;}
    }
    else
    {
        uint16_t elapsed=(uint16_t)((uint16_t)word-(uint16_t)r->previous_word);
        uint16_t moved=(uint16_t)((uint16_t)(word>>16U)-
                                  (uint16_t)(r->previous_word>>16U));
        valid=(age<=40U)&&(elapsed>0U)&&(elapsed<=60U)&&
              ((uint32_t)(now_ms-r->changed_ms)<=60U)&&
              (epoch==r->previous_epoch);
        /* Same 50 mm per-update guard as the Phase 5 height module. These
         * development bounds still require physical distance validation. */
        if(moved<0x8000U&&moved>50U){valid=false;}
        r->forward=!blocked&&(moved<0x8000U);
        out->forward=r->forward;
        r->previous_word=word;r->changed_ms=now_ms;r->previous_epoch=epoch;
    }
    out->valid=valid;
    return valid;
}
