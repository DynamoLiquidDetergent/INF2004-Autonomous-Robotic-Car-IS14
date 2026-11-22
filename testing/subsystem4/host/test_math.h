#ifndef S4_TEST_MATH_H
#define S4_TEST_MATH_H
double sin(double x);
double cos(double x);
static inline float test_sin(float x){return (float)sin((double)x);}
static inline float test_cos(float x){return (float)cos((double)x);}
static inline float test_abs(float x){return (x<0.0F)?-x:x;}
#endif
