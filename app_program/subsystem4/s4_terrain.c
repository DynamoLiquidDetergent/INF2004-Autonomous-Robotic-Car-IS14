#include "s4_terrain.h"
#include "s4_math.h"
#include "s4_memory.h"

uint8_t s4_terrain_state(const s4_terrain_t *t){return t->state_motion&7U;}
uint8_t s4_motion_class(const s4_terrain_t *t){return (t->state_motion>>3U)&7U;}
void s4_set_motion(s4_terrain_t *t,uint8_t m)
{t->state_motion=(uint8_t)((t->state_motion&7U)|((m&7U)<<3U));}
static void set_state(s4_terrain_t *t,uint8_t state)
{t->state_motion=(uint8_t)((t->state_motion&0x38U)|(state&7U));t->transition_count=0U;}
const char *s4_terrain_state_name(uint8_t state)
{
    static const char *const names[]={"INITIALISING","CALIBRATING","LEVEL",
                                     "CLIMBING","DESCENDING","FAULT"};
    return state<=S4_FAULT?names[state]:"UNKNOWN";
}
const char *s4_motion_class_name(uint8_t motion)
{
    static const char *const names[]={"STATIONARY","ACCELERATING","TURNING",
        "CLIMBING","DESCENDING","UNCLASSIFIED","UNCLASSIFIED","UNCLASSIFIED"};
    return motion<8U?names[motion]:"UNCLASSIFIED";
}
s4_terrain_config_t s4_terrain_defaults(void)
{
    /* Development settings, to validate against real flat/hump/vibration runs.
     * A 30 s startup timeout allows repositioning; it is not a claimed boot time. */
    s4_terrain_config_t c={6.0F,2.0F,0.8F,0.15F,10.0F,3U,30000000U,10000000U};
    return c;
}
void s4_terrain_init(s4_terrain_t *t,uint32_t now)
{
    s4_zero(t,sizeof(*t));t->config=s4_terrain_defaults();
    t->calibration_start_us=now;t->impact_armed=true;
    set_state(t,S4_INITIALISING);s4_set_motion(t,S4_UNCLASSIFIED);
}
static bool consecutive(s4_terrain_t *t,bool condition)
{
    unsigned int required=t->config.debounce_samples;
    if(required==0U){required=1U;}
    if(!condition){t->transition_count=0U;return false;}
    if(t->transition_count<required){++t->transition_count;}
    return t->transition_count>=required;
}
static void classify(s4_terrain_t *t,const s4_orientation_t *o)
{
    uint8_t state=s4_terrain_state(t);
    if(state==S4_CLIMBING){s4_set_motion(t,S4_MOTION_CLIMBING);}
    else if(state==S4_DESCENDING){s4_set_motion(t,S4_MOTION_DESCENDING);}
    else if(o->orientation_valid&&o->turn_rate_valid&&
            (s4_abs(o->turn_rate_deg_s)>=t->config.turning_deg_s))
    {s4_set_motion(t,S4_TURNING);}
    else if(o->dynamic_g>=t->config.acceleration_g)
    {s4_set_motion(t,S4_ACCELERATING);}
    else {s4_set_motion(t,S4_UNCLASSIFIED);}
    /* Near-zero acceleration alone cannot prove the robot is stationary.
     * Turning needs a valid rate input; no unvalidated heading is substituted. */
}
void s4_terrain_step(s4_terrain_t *t,const s4_orientation_t *o,
                     bool calibrated,bool sensor_ok,bool sample_fresh,uint32_t now)
{
    uint8_t state=s4_terrain_state(t);
    bool traversing=(state==S4_CLIMBING)||(state==S4_DESCENDING);
    t->impact_event=false;t->hump_event=false;t->traversal_aborted=false;
    if(state==S4_FAULT||!sensor_ok)
    {
        t->traversal_aborted=traversing;
        set_state(t,S4_FAULT);s4_set_motion(t,S4_UNCLASSIFIED);return;
    }
    if(state==S4_INITIALISING){set_state(t,S4_CALIBRATING);state=S4_CALIBRATING;}
    if(state==S4_CALIBRATING)
    {
        if(calibrated&&sample_fresh){set_state(t,S4_LEVEL);}
        else if((uint32_t)(now-t->calibration_start_us)>=t->config.calibration_timeout_us)
        {set_state(t,S4_FAULT);}
        s4_set_motion(t,S4_UNCLASSIFIED);return;
    }
    if(!calibrated)
    {
        t->traversal_aborted=traversing;
        set_state(t,S4_FAULT);s4_set_motion(t,S4_UNCLASSIFIED);return;
    }
    if(traversing&&((uint32_t)(now-t->traversal_start_us)>=t->config.traversal_timeout_us))
    {
        set_state(t,S4_LEVEL);t->saw_descent=false;t->traversal_aborted=true;
        s4_set_motion(t,S4_UNCLASSIFIED);return; /* no same-call restart */
    }
    if(!sample_fresh)
    {t->transition_count=0U;s4_set_motion(t,S4_UNCLASSIFIED);return;}
    if(t->impact_armed&&(o->dynamic_g>=t->config.impact_g))
    {t->impact_event=true;t->impact_armed=false;}
    if(o->dynamic_g<t->config.impact_g*0.5F){t->impact_armed=true;}
    if(o->orientation_valid)
    {
        switch(state)
        {
            case S4_LEVEL:
                if(consecutive(t,o->pitch_deg>=t->config.enter_deg))
                {
                    set_state(t,S4_CLIMBING);t->traversal_start_us=now;
                    t->saw_descent=false;
                }
                break;
            case S4_CLIMBING:
                /* A flat crest is not evidence of a descent. */
                if(consecutive(t,o->pitch_deg<=-t->config.enter_deg))
                {set_state(t,S4_DESCENDING);t->saw_descent=true;}
                break;
            case S4_DESCENDING:
                if(consecutive(t,t->saw_descent&&
                                 (s4_abs(o->pitch_deg)<=t->config.exit_deg)))
                {set_state(t,S4_LEVEL);t->hump_event=true;t->saw_descent=false;}
                break;
            default: break;
        }
    }
    else {t->transition_count=0U;}
    classify(t,o);
}
