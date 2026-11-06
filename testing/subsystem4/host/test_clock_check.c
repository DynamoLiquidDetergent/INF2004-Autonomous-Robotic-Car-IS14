/* S4-U08: coarse clock comparison logic, not actual oscillator calibration. */
#include <stdio.h>
#include "s4_clock_check.h"
static unsigned checks,failures;
#define CHECK(x) do { ++checks; if(!(x)) { ++failures; \
    printf("FAIL line %d: %s\n",__LINE__,#x); } } while(0)
int main(void)
{
    s4_clock_check_t c;
    s4_clock_check_init(&c,0U,0U);CHECK(c.status==S4_CLOCK_PENDING);
    s4_clock_check_step(&c,199000U,199U);CHECK(c.status==S4_CLOCK_PENDING);
    s4_clock_check_step(&c,200000U,200U);CHECK(c.status==S4_CLOCK_CONSISTENT);
    s4_clock_check_step(&c,400000U,400U);CHECK(c.status==S4_CLOCK_CONSISTENT);
    s4_clock_check_init(&c,0U,0U);s4_clock_check_step(&c,4688U,200U);
    CHECK(c.status==S4_CLOCK_BAD);
    s4_clock_check_step(&c,404688U,600U);CHECK(c.status==S4_CLOCK_BAD);
    s4_clock_check_init(&c,0U,0U);s4_clock_check_step(&c,230000U,200U);
    CHECK(c.status==S4_CLOCK_CONSISTENT);
    s4_clock_check_init(&c,0U,0U);s4_clock_check_step(&c,230001U,200U);
    CHECK(c.status==S4_CLOCK_BAD);
    s4_clock_check_init(&c,0U,0U);s4_clock_check_step(&c,170000U,200U);
    CHECK(c.status==S4_CLOCK_CONSISTENT);
    s4_clock_check_init(&c,0U,0U);s4_clock_check_step(&c,169999U,200U);
    CHECK(c.status==S4_CLOCK_BAD);
    s4_clock_check_init(&c,0U,0U);s4_clock_check_step(&c,0U,200U);
    CHECK(c.status==S4_CLOCK_BAD);
    s4_clock_check_init(&c,0U,0U);s4_clock_check_step(&c,1001000U,1001U);
    CHECK(c.status==S4_CLOCK_BAD);
    s4_clock_check_init(&c,0xffffff00U,0xffffff80U);
    s4_clock_check_step(&c,(uint32_t)(0xffffff00U+200000U),(uint32_t)(0xffffff80U+200U));
    CHECK(c.status==S4_CLOCK_CONSISTENT);
    s4_clock_check_init(&c,100U,10U);CHECK(c.status==S4_CLOCK_PENDING);
    printf("S4-U08: %s (%u checks, %u failures). Synthetic clocks; not measured timing.\n",
           failures?"FAIL":"PASS",checks,failures);
    return failures?1:0;
}
