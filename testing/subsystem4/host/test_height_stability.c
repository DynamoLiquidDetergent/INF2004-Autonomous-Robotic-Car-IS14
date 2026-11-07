/* Synthetic spike rejection, sustained tilt and raw impact regression. */
#include <assert.h>
#include <stdio.h>
#include "s4_processing.h"
#include "s4_math.h"
#include "test_math.h"
int main(void)
{
    s4_processing_config_t c=s4_processing_defaults();s4_processor_t p;
    s4_orientation_t o;s4_sample_t s={{0,0,1},{0,0,0},true,false,false};
    unsigned i;uint32_t us=0;c.calibration_samples=4;
    s4_processor_init(&p,&c);
    for(i=0;i<10U;++i){us+=20000U;(void)s4_process(&p,&s,us,&o);}
    assert(p.calibrated&&o.orientation_valid);
    /* Unit-norm tilt spike passes the magnitude gate; median must reject it. */
    s.accel_g[0]=0.5F;s.accel_g[2]=0.8660254F;
    for(i=0;i<2U;++i){us+=20000U;(void)s4_process(&p,&s,us,&o);assert(test_abs(o.pitch_deg)<0.1F);}
    s.accel_g[0]=0;s.accel_g[2]=1;
    for(i=0;i<10U;++i){us+=20000U;(void)s4_process(&p,&s,us,&o);assert(test_abs(o.pitch_deg)<0.1F);}
    s.accel_g[0]=s4_sin_deg(12);s.accel_g[2]=s4_cos_deg(12);
    for(i=0;i<25U;++i){us+=20000U;(void)s4_process(&p,&s,us,&o);}
    assert(o.orientation_valid&&test_abs(o.pitch_deg-12)<0.2F);
    s.accel_g[0]=-s4_sin_deg(12);
    for(i=0;i<25U;++i){us+=20000U;(void)s4_process(&p,&s,us,&o);}
    assert(o.orientation_valid&&test_abs(o.pitch_deg+12)<0.2F);
    s.accel_g[0]=0;s.accel_g[2]=2;
    us+=20000U;(void)s4_process(&p,&s,us,&o);
    assert(!o.orientation_valid&&o.dynamic_g>0.9F);
    s.accel_g[0]=s4_sin_deg(10);s.accel_g[2]=s4_cos_deg(10);us+=200000U;
    (void)s4_process(&p,&s,us,&o);
    assert(o.orientation_valid&&p.median_history[0][0]>0.17F);
    puts("Height stability: PASS (short spikes, sustained rise/descent, raw impact, history expiry); synthetic only.");
    return 0;
}
