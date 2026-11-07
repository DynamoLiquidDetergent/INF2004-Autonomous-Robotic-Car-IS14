#include "s4_odometry.h"
#include "s4_terrain.h"
#include "s4_math.h"
#include "s4_memory.h"

void s4_height_init(s4_height_t *h)
{
    s4_zero(h,sizeof(*h));
    h->max_gap_us=60000U;h->max_age_us=40000U;
    h->max_step_mm=50U;h->max_candidate_ms=10000U;
    h->max_imu_gap_us=h->max_gap_us;h->max_imu_step_mm=h->max_step_mm;
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
static uint32_t track_wheelbase(const s4_height_t *h)
{
    return (h->wheelbase_mm<S4_HEIGHT_TRACK_LEN)?
           h->wheelbase_mm:(S4_HEIGHT_TRACK_LEN-1U);
}
static void track_start(s4_height_t *h)
{
    /* The baseline sample was level, so the rear wheel is at height 0 and
     * the front contact is L*sin(baseline pitch) above it. */
    h->track_mm=0U;
    h->front_mm=h->have_previous?
        (float)track_wheelbase(h)*h->previous_sine:0.0F;
    h->track[0]=h->front_mm;
}
static float track_step(s4_height_t *h,uint16_t ds,float sine)
{
    uint32_t l=track_wheelbase(h), d0=h->track_mm, d1=d0+ds, back, i;
    float rear=0.0F, front;
    if(d1>=l)
    {
        back=d1-l;
        if(back>d0){back=d0;}  /* ds > wheelbase: use the newest entry */
        rear=h->track[back%S4_HEIGHT_TRACK_LEN];
    }
    front=rear+(float)l*sine;
    if(ds==0U){h->track[d0%S4_HEIGHT_TRACK_LEN]=front;}
    for(i=1U;i<=ds;++i)
    {
        h->track[(d0+i)%S4_HEIGHT_TRACK_LEN]=
            h->front_mm+(front-h->front_mm)*(float)i/(float)ds;
    }
    h->track_mm=d1;h->front_mm=front;
    return front;
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
        h->interpolated_samples=0U;h->max_interpolated_us=0U;
        /* The published last/highest result is NOT cleared here: a pitch
         * blip on flat ground must not erase the previous hump's height. */
        h->idle_reason=h->reason;h->reason=S4_HEIGHT_OK;
        track_start(h);
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
            if(h->wheelbase_mm!=0U)
            {
                /* Chassis pitch is the slope between the wheels, not under
                 * one wheel, so summing ds*sin(pitch) gives the height
                 * averaged over the wheelbase and under-reads short humps. */
                h->current_mm=track_step(h,ds,sine);
            }
            else
            {
                /* Legacy: ds is distance ALONG the travelled path.
                 * Trapezoidal sin(pitch) integration approximates rise. */
                h->current_mm+=(float)ds*0.5F*(h->previous_sine+sine);
            }
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
        if(h->active&&h->saw_climbing)
        {reject(h,S4_HEIGHT_ABORTED);h->last_valid=false;} /* a real hump failed */
        else if(h->active)
        {h->reason=h->idle_reason;} /* false start: nothing to publish */
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
/* Odometry continuity is checked on EVERY call, independently of the
 * longer IMU endpoint interval. Fresh encoder updates cannot be replaced by
 * one stale cumulative sample. Never use a rejected acceleration as tilt. */
static uint8_t quality_odometry(s4_height_t *h,const s4_odometry_t *o,
                               bool clock_ok,uint32_t us,uint32_t ms)
{
    uint32_t age,dt;
    uint16_t ds;
    uint8_t reason=S4_HEIGHT_OK;
    if(!clock_ok){reason=S4_HEIGHT_TIMEBASE;}
    else if(!o->valid){reason=S4_HEIGHT_NO_ODOMETRY;}
    else if(!o->forward){reason=S4_HEIGHT_REVERSE;}
    else if(o->clock_format!=S4_ODOM_MS16&&o->clock_format!=S4_ODOM_US32)
    {reason=S4_HEIGHT_FORMAT;}
    else
    {
        age=o->clock_format==S4_ODOM_MS16?
            (uint32_t)(uint16_t)((uint16_t)ms-(uint16_t)o->timestamp)*1000U:
            us-o->timestamp;
        if(age>h->max_age_us){reason=S4_HEIGHT_STALE;}
        else if(h->quality_have_previous)
        {
            if(o->clock_format!=h->quality_previous.clock_format)
            {reason=S4_HEIGHT_FORMAT;}
            else
            {
                dt=interval(o,&h->quality_previous);
                ds=(uint16_t)(o->distance_mm-h->quality_previous.distance_mm);
                if((dt==0U&&ds!=0U)||ds>h->max_step_mm)
                {reason=S4_HEIGHT_JUMP;}
                else if(dt>h->max_gap_us||
                    (uint32_t)(ms-h->quality_receipt_ms)>h->max_gap_us/1000U)
                {reason=S4_HEIGHT_GAP;}
            }
        }
    }
    if(reason!=S4_HEIGHT_OK){h->quality_have_previous=false;return reason;}
    if(!h->quality_have_previous||interval(o,&h->quality_previous)!=0U)
    {h->quality_previous=*o;h->quality_receipt_ms=ms;}
    h->quality_have_previous=true;
    return reason;
}
void s4_height_step_quality(s4_height_t *h,const s4_odometry_t *o,
                    const s4_orientation_t *a,uint8_t quality,bool clock_ok,
                    uint32_t us,uint32_t ms,uint8_t state,float exit_deg,
                    bool completed,bool aborted)
{
    uint32_t skipped=h->pending_samples,gap=0U;
    uint32_t strict_gap=h->max_gap_us;
    uint16_t strict_step=h->max_step_mm;
    uint16_t endpoint_step=h->max_imu_step_mm<strict_step?
                           h->max_imu_step_mm:strict_step;
    bool deferrable=quality==S4_IMU_WAIT||quality==S4_IMU_DYNAMIC;
    bool endpoint=quality==S4_IMU_GOOD&&a->orientation_valid;
    bool was_valid=h->have_previous&&(!h->active||h->chain_valid);
    uint8_t odometry_reason=quality_odometry(h,o,clock_ok,us,ms);
    bool bounded=false;
    if(was_valid&&odometry_reason==S4_HEIGHT_OK&&
       o->clock_format==h->previous.clock_format)
    {
        gap=interval(o,&h->previous);
        bounded=gap<=h->max_imu_gap_us&&
            (uint32_t)(ms-h->previous_receipt_ms)<=h->max_imu_gap_us/1000U&&
            (uint16_t)(o->distance_mm-h->previous.distance_mm)<=endpoint_step&&
            (gap!=0U||o->distance_mm==h->previous.distance_mm);
    }
    if(deferrable&&bounded&&gap<h->max_imu_gap_us&&
       (uint32_t)(ms-h->previous_receipt_ms)<h->max_imu_gap_us/1000U&&
       state!=S4_FAULT&&!aborted&&!completed&&
       (!h->active||(uint32_t)(ms-h->start_ms)<=h->max_candidate_ms))
    {
        ++h->pending_samples;h->odometry_valid=false;return;
    }
    h->pending_samples=0U;
    /* A broken encoder stream cannot be healed by a later IMU endpoint. */
    if(odometry_reason!=S4_HEIGHT_OK&&was_valid)
    {
        reject(h,odometry_reason);endpoint=false;
    }
    if(skipped!=0U&&endpoint)
    {
        if(bounded)
        {
            /* Only this verified interval uses the IMU budget. The strict
             * odometry stream above retains its original 60 ms guard. */
            h->max_gap_us=h->max_imu_gap_us;h->max_step_mm=endpoint_step;
        }
        else {reject(h,S4_HEIGHT_IMU);endpoint=false;}
    }
    s4_height_step(h,o,a,endpoint,clock_ok,us,ms,state,exit_deg,completed,aborted);
    h->max_gap_us=strict_gap;h->max_step_mm=strict_step;
    if(endpoint&&skipped!=0U&&was_valid&&h->chain_valid&&h->odometry_valid)
    {
        if(gap!=0U&&h->previous_receipt_ms==ms)
        {
            h->interpolated_samples+=skipped;
            if(gap>h->max_interpolated_us){h->max_interpolated_us=gap;}
        }
        else {h->pending_samples=skipped;}
    }
}
const char *s4_height_reason_name(uint8_t reason)
{
    static const char *const names[]={"OK","NO_ODOMETRY","TIMEBASE","IMU",
        "NO_LEVEL_BASELINE","STALE","GAP","NOT_FORWARD","DISTANCE_JUMP",
        "CLOCK_FORMAT","INCOMPLETE","NO_MOVEMENT"};
    return reason<=S4_HEIGHT_NO_MOVEMENT?names[reason]:"UNKNOWN";
}

#define S4_RIGHT_IMPLEMENTATION
#include "s4_right_wheel.h"
