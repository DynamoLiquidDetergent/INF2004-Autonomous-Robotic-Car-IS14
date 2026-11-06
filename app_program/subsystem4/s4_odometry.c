#include "s4_odometry.h"
#include "s4_terrain.h"
#include "s4_math.h"
#include "s4_memory.h"

void s4_height_init(s4_height_t *h)
{
    s4_zero(h,sizeof(*h));
    h->max_gap_us=60000U;h->max_age_us=40000U;
    h->max_step_mm=50U;h->max_candidate_ms=10000U;
    h->reason=S4_HEIGHT_NO_ODOMETRY;
}
void s4_odometry_decode_word(uint32_t word,bool forward,s4_odometry_t *o)
{
    o->distance_mm=(uint16_t)(word>>16U);o->timestamp=(uint16_t)word;
    o->clock_format=S4_ODOM_MS16;o->valid=true;o->forward=forward;
}
static uint32_t interval(const s4_odometry_t *a,const s4_odometry_t *b)
{
    if(a->clock_format==S4_ODOM_MS16)
    {return (uint32_t)(uint16_t)(a->timestamp-b->timestamp)*1000U;}
    return a->timestamp-b->timestamp;
}
static void reject(s4_height_t *h,uint8_t reason)
{
    /* Once a traversal is incomplete, a later good sample cannot repair it. */
    if(h->active)
    {
        if(h->chain_valid){h->reason=reason;}
        h->chain_valid=false;
    }
    else if(!h->last_valid){h->reason=reason;}
}
static bool usable(const s4_odometry_t *o,const s4_orientation_t *a,
                   bool fresh,bool clock_ok,uint32_t us,uint32_t ms,
                   s4_height_t *h)
{
    uint32_t age;
    uint8_t reason=S4_HEIGHT_OK;
    if(!clock_ok){reason=S4_HEIGHT_TIMEBASE;}
    else if(!o->valid){reason=S4_HEIGHT_NO_ODOMETRY;}
    else if(!o->forward){reason=S4_HEIGHT_REVERSE;}
    else if(!fresh||!a->orientation_valid||
            !(a->pitch_deg>=-90.0F&&a->pitch_deg<=90.0F))
    {reason=S4_HEIGHT_IMU;}
    else if(o->clock_format!=S4_ODOM_MS16&&o->clock_format!=S4_ODOM_US32)
    {reason=S4_HEIGHT_FORMAT;}
    else
    {
        age=(o->clock_format==S4_ODOM_MS16)?
            (uint32_t)(uint16_t)((uint16_t)ms-(uint16_t)o->timestamp)*1000U:
            us-o->timestamp;
        if(age>h->max_age_us){reason=S4_HEIGHT_STALE;}
    }
    if(reason!=S4_HEIGHT_OK){reject(h,reason);return false;}
    return true;
}
void s4_height_step(s4_height_t *h,const s4_odometry_t *o,
                    const s4_orientation_t *a,bool fresh,bool clock_ok,
                    uint32_t us,uint32_t ms,uint8_t state,float exit_deg,
                    bool completed,bool aborted)
{
    bool good,new_sample=true,starting;
    uint32_t dt=0U;
    uint16_t ds=0U;
    float sine=0.0F;
    /* Start provisionally near the departure from level, before the FSM's
     * higher 6-degree/debounced climbing threshold. Include the preceding
     * level sample in the first interval. A positive tilt alone is not a hump. */
    starting=!h->active&&fresh&&a->orientation_valid&&
        ((state==S4_CLIMBING)||((state==S4_LEVEL)&&(a->pitch_deg>exit_deg)));
    if(starting)
    {
        h->active=true;h->chain_valid=true;h->moved=false;
        h->saw_climbing=false;h->start_ms=ms;
        h->current_mm=0.0F;h->peak_mm=0.0F;
        h->last_valid=false;h->last_mm=0.0F;h->reason=S4_HEIGHT_OK;
    }
    if(h->active&&(state==S4_CLIMBING)){h->saw_climbing=true;}
    good=usable(o,a,fresh,clock_ok,us,ms,h);
    if(starting&&good&&(!h->have_previous||!h->baseline_ready))
    {reject(h,S4_HEIGHT_BASELINE);}
    if(good&&h->have_previous)
    {
        if(o->clock_format!=h->previous.clock_format)
        {reject(h,S4_HEIGHT_FORMAT);good=false;}
        else
        {
            dt=interval(o,&h->previous);
            ds=(uint16_t)(o->distance_mm-h->previous.distance_mm);
            if(dt==0U)
            {
                new_sample=false;
                if(ds!=0U){reject(h,S4_HEIGHT_JUMP);good=false;}
                else if((uint32_t)(ms-h->previous_receipt_ms)>h->max_gap_us/1000U)
                {reject(h,S4_HEIGHT_STALE);good=false;}
            }
            else if((dt>h->max_gap_us)||
                    ((uint32_t)(ms-h->previous_receipt_ms)>h->max_gap_us/1000U))
            {reject(h,S4_HEIGHT_GAP);good=false;}
            else if(ds>h->max_step_mm)
            {reject(h,S4_HEIGHT_JUMP);good=false;}
        }
    }
    h->odometry_valid=good;
    if(good&&new_sample)
    {
        sine=s4_sin_deg(a->pitch_deg);
        if(h->have_previous&&h->active&&h->chain_valid)
        {
            /* ds is distance ALONG the travelled path, not horizontal range.
             * Trapezoidal sin(pitch) integration approximates vertical rise. */
            h->current_mm+=(float)ds*0.5F*(h->previous_sine+sine);
            if(h->current_mm>h->peak_mm){h->peak_mm=h->current_mm;}
            if(ds!=0U){h->moved=true;}
        }
        h->previous=*o;h->previous_sine=sine;h->previous_receipt_ms=ms;
        h->have_previous=true;
        h->baseline_ready=(state==S4_LEVEL)&&(s4_abs(a->pitch_deg)<=exit_deg);
    }
    else if(!good)
    {h->have_previous=false;h->baseline_ready=false;}
    /* Duplicate odometry never replaces the pitch attached to that sample. */
    if(aborted||(state==S4_FAULT)||
       (h->active&&((uint32_t)(ms-h->start_ms)>h->max_candidate_ms))||
       (h->active&&!completed&&(state==S4_LEVEL)&&
        (h->saw_climbing||(fresh&&a->orientation_valid&&a->pitch_deg<=exit_deg))))
    {
        if(h->active){reject(h,S4_HEIGHT_ABORTED);h->last_valid=false;}
        h->active=false;
    }
    if(completed)
    {
        h->last_valid=h->active&&h->chain_valid&&h->saw_climbing&&
                      h->moved&&(h->peak_mm>0.0F);
        if(h->last_valid)
        {
            h->last_mm=h->peak_mm;h->reason=S4_HEIGHT_OK;
            if(!h->highest_valid||(h->last_mm>h->highest_mm))
            {h->highest_mm=h->last_mm;h->highest_valid=true;}
        }
        else if(h->reason==S4_HEIGHT_OK){h->reason=S4_HEIGHT_NO_MOVEMENT;}
        h->active=false;
    }
}
const char *s4_height_reason_name(uint8_t reason)
{
    static const char *const names[]={"OK","NO_ODOMETRY","TIMEBASE","IMU",
        "NO_LEVEL_BASELINE","STALE","GAP","NOT_FORWARD","DISTANCE_JUMP",
        "CLOCK_FORMAT","INCOMPLETE","NO_MOVEMENT"};
    return reason<=S4_HEIGHT_NO_MOVEMENT?names[reason]:"UNKNOWN";
}
