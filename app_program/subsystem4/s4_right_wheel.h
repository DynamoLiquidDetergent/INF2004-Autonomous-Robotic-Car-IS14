#ifndef S4_RIGHT_WHEEL_H
#define S4_RIGHT_WHEEL_H
#include "s4_odometry.h"
#include "s4_terrain.h"
typedef struct {
    float track_mm,baseline_sum,baseline;
    unsigned baseline_count,rise_count,fall_count,flat_count;
    uint16_t start_distance;
    uint32_t last_imu_ms;
    bool baseline_valid;
} s4_right_wheel_t;
void s4_right_step(s4_right_wheel_t *,s4_height_t *,s4_terrain_t *,
    const s4_odometry_t *,const s4_orientation_t *,uint8_t,bool,bool,bool,uint32_t,uint32_t);
#ifdef S4_RIGHT_IMPLEMENTATION
/* Requires the strict odometry helpers in s4_odometry.c. The left tyre must
 * remain on the same flat floor. This estimates RIGHT-minus-LEFT contact
 * height, not arbitrary terrain height when both wheels rise.
 * Gravity projection along the axle is cos(pitch)*sin(roll); caster motion
 * does not enter the difference between equal-radius wheel centres.
 * Tyre flex/slip and dynamic acceleration still require physical validation. */
static void rw_state(s4_terrain_t *t,unsigned state)
{
    unsigned motion=state==S4_CLIMBING?S4_MOTION_CLIMBING:
                    state==S4_DESCENDING?S4_MOTION_DESCENDING:S4_UNCLASSIFIED;
    t->state_motion=(uint8_t)(state|(motion<<3U));
}
static void rw_abort(s4_right_wheel_t *r,s4_height_t *h,s4_terrain_t *t)
{
    if(h->active&&h->saw_climbing){t->traversal_aborted=true;h->last_valid=false;}
    h->active=false;r->rise_count=r->fall_count=r->flat_count=0U;
    rw_state(t,S4_LEVEL);
}
void s4_right_step(s4_right_wheel_t *r,s4_height_t *h,s4_terrain_t *t,
    const s4_odometry_t *o,const s4_orientation_t *a,uint8_t q,bool calibrated,
    bool sensor_ok,bool clock_ok,uint32_t us,uint32_t ms)
{
    unsigned state=s4_terrain_state(t);uint8_t reason;
    bool endpoint=q==S4_IMU_GOOD&&a->orientation_valid;
    uint32_t pending=h->pending_samples;
    float projection,lift;
    t->hump_event=t->impact_event=t->traversal_aborted=false;
    if(state==S4_FAULT||!sensor_ok)
    {reject(h,S4_HEIGHT_IMU);rw_abort(r,h,t);rw_state(t,S4_FAULT);return;}
    if(!calibrated)
    {
        rw_state(t,S4_CALIBRATING);h->baseline_ready=false;
        if((uint32_t)(us-t->calibration_start_us)>=t->config.calibration_timeout_us){rw_state(t,S4_FAULT);}
        return;
    }
    if(state<=S4_CALIBRATING){rw_state(t,S4_LEVEL);state=S4_LEVEL;}
    if(t->impact_armed&&a->dynamic_g>=t->config.impact_g)
    {t->impact_event=true;t->impact_armed=false;}
    if(a->dynamic_g<t->config.impact_g*0.5F){t->impact_armed=true;}
    reason=quality_odometry(h,o,clock_ok,us,ms);
    if(h->active&&(uint32_t)(ms-h->start_ms)>20000U)
    {reject(h,S4_HEIGHT_ABORTED);rw_abort(r,h,t);return;}
    if(reason==S4_HEIGHT_OK&&!endpoint&&(q==S4_IMU_WAIT||q==S4_IMU_DYNAMIC)&&
       h->have_previous&&(uint32_t)(ms-r->last_imu_ms)<100U&&
       interval(o,&h->previous)<100000U&&
       (uint16_t)(o->distance_mm-h->previous.distance_mm)<=5U)
    {++h->pending_samples;h->odometry_valid=false;return;}
    if(reason!=S4_HEIGHT_OK||!endpoint||
       !(a->pitch_deg>=-90&&a->pitch_deg<=90&&a->roll_deg>=-90&&a->roll_deg<=90))
    {
        reject(h,reason!=S4_HEIGHT_OK?reason:S4_HEIGHT_IMU);
        h->have_previous=false;h->pending_samples=0;h->odometry_valid=false;
        if(!r->baseline_valid){r->baseline_count=0;r->baseline_sum=0;}
        r->rise_count=r->fall_count=r->flat_count=0;
        return;
    }
    if(h->have_previous&&pending)
    {
        uint32_t gap=interval(o,&h->previous);
        if(gap>100000U||(uint32_t)(ms-r->last_imu_ms)>100U||
            (uint16_t)(o->distance_mm-h->previous.distance_mm)>5U)
        {reject(h,S4_HEIGHT_GAP);}
        else {h->interpolated_samples+=pending;if(gap>h->max_interpolated_us){h->max_interpolated_us=gap;}}
    }
    h->pending_samples=0;h->odometry_valid=true;h->previous=*o;
    h->have_previous=true;r->last_imu_ms=ms;
    projection=-r->track_mm*s4_cos_deg(a->pitch_deg)*s4_sin_deg(a->roll_deg);
    if(!r->baseline_valid)
    {
        if(s4_abs(a->pitch_deg)>3.0F||s4_abs(a->roll_deg)>3.0F)
        {r->baseline_count=0;r->baseline_sum=0;return;}
        r->baseline_sum+=projection;
        if(++r->baseline_count<5U){return;}
        r->baseline=r->baseline_sum/(float)r->baseline_count;
        r->baseline_valid=true;h->baseline_ready=true;h->chain_valid=true;h->reason=S4_HEIGHT_OK;
    }
    lift=projection-r->baseline;h->current_mm=lift;
    /* Provisional 3/6/2 mm and 3/5-sample guards: validation targets, not
     * sensor specifications. No reference hump height enters this estimator. */
    if(!h->active&&lift>=3.0F)
    {
        h->active=true;h->chain_valid=true;h->saw_climbing=false;
        h->start_ms=ms;r->start_distance=o->distance_mm;h->peak_mm=lift;
        h->reason=S4_HEIGHT_OK;h->interpolated_samples=0;h->max_interpolated_us=0;
        r->rise_count=r->fall_count=r->flat_count=0;
    }
    if(!h->active){return;}
    if(lift>h->peak_mm){h->peak_mm=lift;}
    if(state==S4_LEVEL)
    {
        r->rise_count=lift>=6.0F?r->rise_count+1U:0U;
        if(r->rise_count>=3U){rw_state(t,S4_CLIMBING);h->saw_climbing=true;}
        else if(s4_abs(lift)<=2.0F){h->active=false;}
    }
    else if(state==S4_CLIMBING)
    {
        r->fall_count=h->peak_mm-lift>=2.0F?r->fall_count+1U:0U;
        if(r->fall_count>=3U){rw_state(t,S4_DESCENDING);}
    }
    else if(state==S4_DESCENDING)
    {
        r->flat_count=s4_abs(lift)<=2.0F?r->flat_count+1U:0U;
        if(r->flat_count>=5U)
        {
            h->moved=(uint16_t)(o->distance_mm-r->start_distance)>=5U;
            h->last_valid=h->chain_valid&&h->saw_climbing&&h->moved;
            if(h->last_valid)
            {
                h->last_mm=h->peak_mm;h->reason=S4_HEIGHT_OK;
                if(!h->highest_valid||h->last_mm>h->highest_mm)
                {h->highest_mm=h->last_mm;h->highest_valid=true;}
            }
            else if(h->chain_valid){h->reason=S4_HEIGHT_NO_MOVEMENT;}
            t->hump_event=true;h->active=false;rw_state(t,S4_LEVEL);
        }
    }
}
#endif
#endif
