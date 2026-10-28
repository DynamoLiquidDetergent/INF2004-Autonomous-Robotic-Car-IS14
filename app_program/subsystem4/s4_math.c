/* Small local approximations: no new -lm dependency in the shared makefile.
 * Host tests bound atan error <0.12 degrees and sine error <0.00001.
 * These bounds describe arithmetic, not sensor or hump-height accuracy. */
#include "s4_math.h"
float s4_abs(float x) { return (x < 0.0F) ? -x : x; }
float s4_sqrt(float x)
{
    float a = (x > 1.0F) ? x : 1.0F;
    unsigned int i;
    if (x <= 0.0F) { return 0.0F; }
    for (i=0U; i<12U; ++i) { a = 0.5F * (a + x/a); }
    return a;
}
float s4_atan2_deg(float y, float x)
{
    float a, z;
    if ((x == 0.0F) && (y == 0.0F)) { return 0.0F; }
    if (s4_abs(x) >= s4_abs(y))
    {
        z = s4_abs(y/x);
        a = 45.0F*z - z*(z-1.0F)*(14.02001F+3.7852F*z);
    }
    else
    {
        z = s4_abs(x/y);
        a = 90.0F-(45.0F*z-z*(z-1.0F)*(14.02001F+3.7852F*z));
    }
    if (x < 0.0F) { a = 180.0F-a; }
    return (y < 0.0F) ? -a : a;
}
float s4_angle_delta(float d)
{
    /* Callers use bounded headings, so at most one correction is needed. */
    if (d > 180.0F) { d -= 360.0F; }
    if (d < -180.0F) { d += 360.0F; }
    return d;
}
float s4_sin_deg(float d)
{
    float x, x2;
    while (d > 180.0F) { d -= 360.0F; }
    while (d < -180.0F) { d += 360.0F; }
    if (d > 90.0F) { d = 180.0F-d; }
    if (d < -90.0F) { d = -180.0F-d; }
    x = d * 0.01745329252F;
    x2 = x*x;
    return x*(1.0F+x2*(-1.0F/6.0F+x2*(1.0F/120.0F+
           x2*(-1.0F/5040.0F+x2/362880.0F))));
}
float s4_cos_deg(float d) { return s4_sin_deg(d+90.0F); }
