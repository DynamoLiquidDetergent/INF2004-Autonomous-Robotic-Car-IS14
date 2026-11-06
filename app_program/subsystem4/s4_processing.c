#include "s4_processing.h"
#include "s4_math.h"
#include "s4_memory.h"
s4_processing_config_t s4_processing_defaults(void)
{
    /* Bench starting values, not measured acceptance limits. */
    s4_processing_config_t c = {0};
    unsigned int i;
    c.calibration_samples=100U; c.calibration_spread_g=0.05F;
    c.gravity_tolerance_g=0.15F; c.filter_alpha=0.25F;
    c.tilt_warning_deg=25.0F;
    c.magnetic_min_gauss=0.15F; c.magnetic_max_gauss=0.80F;
    c.max_turn_rate_deg_s=400.0F;
    c.magnetic_calibrated=false;
    for(i=0U;i<3U;++i) { c.magnetic_scale[i]=1.0F; }
    return c;
}
void s4_processor_init(s4_processor_t *p,const s4_processing_config_t *c)
{
    s4_zero(p,sizeof(*p));
    p->config=*c;
}
static float magnitude(const float a[3])
{
    return s4_sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
}
bool s4_process(s4_processor_t *p,const s4_sample_t *s,
                 uint32_t time_us,s4_orientation_t *o)
{
    unsigned int i;
    float gravity, raw_pitch, raw_roll, pitch, roll;
    s4_zero(o,sizeof(*o));
    if (!s->accel_fresh || s->saturated) { p->have_heading=false; if(s->saturated){o->dynamic_g=4.0F;} return false; }
    gravity=magnitude(s->accel_g);
    o->dynamic_g=s4_abs(gravity-1.0F); /* Also catches impacts before filtering. */
    if (!p->calibrated)
    {
        bool stable=s4_abs(gravity-1.0F)<=p->config.gravity_tolerance_g;
        if(p->count==0U)
        {
            for(i=0U;i<3U;++i)
            { p->minimum[i]=s->accel_g[i]; p->maximum[i]=s->accel_g[i]; }
        }
        for(i=0U;i<3U;++i)
        {
            if(s->accel_g[i]<p->minimum[i]) {p->minimum[i]=s->accel_g[i];}
            if(s->accel_g[i]>p->maximum[i]) {p->maximum[i]=s->accel_g[i];}
            if((p->maximum[i]-p->minimum[i])>p->config.calibration_spread_g)
            {stable=false;}
        }
        if(!stable)
        {
            p->count=0U;
            for(i=0U;i<3U;++i){p->sum[i]=0.0F;}
            return false;
        }
        for(i=0U;i<3U;++i){p->sum[i]+=s->accel_g[i];}
        ++p->count;
        if(p->count<p->config.calibration_samples){return false;}
        for(i=0U;i<3U;++i){p->filtered[i]=p->sum[i]/(float)p->count;}
        p->pitch_zero=s4_atan2_deg(p->filtered[0],
                       s4_sqrt(p->filtered[1]*p->filtered[1]+p->filtered[2]*p->filtered[2]));
        p->roll_zero=s4_atan2_deg(p->filtered[1],p->filtered[2]);
        /* Boot calibration must happen on flat ground, Z up. */
        if((s4_abs(p->pitch_zero)>10.0F)||(s4_abs(p->roll_zero)>10.0F))
        {p->count=0U;for(i=0U;i<3U;++i){p->sum[i]=0.0F;}return false;}
        p->filter_ready=true;
        p->calibrated=true;
    }
    for(i=0U;i<3U;++i)
    {p->filtered[i]+=p->config.filter_alpha*(s->accel_g[i]-p->filtered[i]);}
    raw_pitch=s4_atan2_deg(p->filtered[0],
                s4_sqrt(p->filtered[1]*p->filtered[1]+p->filtered[2]*p->filtered[2]));
    raw_roll=s4_atan2_deg(p->filtered[1],p->filtered[2]);
    o->pitch_deg=raw_pitch-p->pitch_zero;
    o->roll_deg=raw_roll-p->roll_zero;
    o->orientation_valid=s4_abs(gravity-1.0F)<=p->config.gravity_tolerance_g;
    o->tilt_warning=o->orientation_valid&&((s4_abs(o->pitch_deg)>=p->config.tilt_warning_deg)||
                    (s4_abs(o->roll_deg)>=p->config.tilt_warning_deg));
    pitch=raw_pitch;roll=raw_roll;
    if(p->config.magnetic_calibrated && s->magnetic_fresh && o->orientation_valid)
    {
        float m[3],norm,xh,yh,heading;
        for(i=0U;i<3U;++i)
        {m[i]=(s->magnetic_gauss[i]-p->config.magnetic_bias[i])*p->config.magnetic_scale[i];}
        norm=magnitude(m);
        xh=m[0]*s4_cos_deg(pitch)-s4_sin_deg(pitch)*
              (m[1]*s4_sin_deg(roll)+m[2]*s4_cos_deg(roll));
        yh=m[1]*s4_cos_deg(roll)-m[2]*s4_sin_deg(roll);
        if((norm>=p->config.magnetic_min_gauss)&&(norm<=p->config.magnetic_max_gauss)&&
           ((xh*xh+yh*yh)>0.000001F))
        {
            uint32_t dt=time_us-p->previous_heading_us;
            heading=s4_atan2_deg(yh,xh);
            if(heading<0.0F){heading+=360.0F;}
            o->heading_deg=heading;o->heading_valid=true;
            if(p->have_heading&&(dt>=1000U)&&(dt<=100000U))
            {
                o->turn_rate_deg_s=s4_angle_delta(heading-p->previous_heading)*
                                      (1000000.0F/(float)dt);
                o->turn_rate_valid=s4_abs(o->turn_rate_deg_s)<=p->config.max_turn_rate_deg_s;
                if(!o->turn_rate_valid){o->heading_valid=false;}
            }
            p->previous_heading=heading;p->previous_heading_us=time_us;
            p->have_heading=o->heading_valid;
        }
        else {p->have_heading=false;}
    }
    else {p->have_heading=false;}
    return true;
}
