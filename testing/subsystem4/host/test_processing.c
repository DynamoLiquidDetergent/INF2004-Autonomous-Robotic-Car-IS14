#include <assert.h>
#include "test_math.h"
#include <stdio.h>
#include "s4_processing.h"
#include "s4_math.h"
int main(void)
{
    s4_processing_config_t c=s4_processing_defaults();
    s4_processor_t p; s4_orientation_t o;
    s4_sample_t s={{0,0,1},{0.3F,0,0},true,true,false};
    int i; float maxe=0;
    for(i=-1800;i<=1800;++i)
    {
        float d=(float)i/10.0F,r=d*0.01745329252F;
        float error=test_abs(s4_angle_delta(s4_atan2_deg(test_sin(r),test_cos(r))-d));
        if(error>maxe){maxe=error;}
        assert(test_abs(s4_sin_deg(d)-test_sin(r))<0.00001F);
    }
    assert(maxe<0.12F);
    c.calibration_samples=4;c.filter_alpha=1.0F;
    s4_processor_init(&p,&c);
    for(i=0;i<3;++i){assert(!s4_process(&p,&s,(uint32_t)i*20000U,&o));}
    assert(s4_process(&p,&s,60000U,&o));
    assert(o.orientation_valid&&!o.heading_valid);
    s.accel_g[0]=0.5F;s.accel_g[2]=0.8660254F;
    assert(s4_process(&p,&s,80000U,&o));assert(test_abs(o.pitch_deg-30.0F)<0.12F);
    c.magnetic_calibrated=true;s4_processor_init(&p,&c);
    s.accel_g[0]=0;s.accel_g[2]=1;
    for(i=0;i<4;++i){(void)s4_process(&p,&s,(uint32_t)i*20000U,&o);}
    s.magnetic_gauss[1]=-0.005236F;
    (void)s4_process(&p,&s,80000U,&o);
    s.magnetic_gauss[1]=0.005236F;
    (void)s4_process(&p,&s,100000U,&o);
    assert(o.turn_rate_valid&&o.turn_rate_deg_s>90.0F&&o.turn_rate_deg_s<110.0F);
    s.magnetic_gauss[0]=4.0F;
    (void)s4_process(&p,&s,120000U,&o);assert(!o.heading_valid);
    s4_processor_init(&p,&c);
    for(i=0;i<20;++i)
    {s.accel_g[0]=(i%2)?0.2F:0.0F;(void)s4_process(&p,&s,(uint32_t)i*20000U,&o);}
    assert(!p.calibrated);
    puts("processing: PASS (math bounds, flat calibration, tilt, heading wrap, interference)");
    return 0;
}
