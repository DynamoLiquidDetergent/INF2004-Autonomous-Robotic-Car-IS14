/* S4-U08: clock qualification and latched runtime guards; synthetic inputs. */
#include <stdio.h>
#include "s4_clock_check.h"
static unsigned checks,failures;
#define CHECK(x) do {++checks;if(!(x)){++failures;printf("FAIL %d: %s\n",__LINE__,#x);}}while(0)
static void qualify(s4_clock_check_t *c)
{
    s4_clock_check_init(c,0U,0U);
    s4_clock_check_step(c,200000U,200U);
    s4_clock_check_step(c,400000U,400U);
    s4_clock_check_step(c,600000U,600U);
}
int main(void)
{
    s4_clock_check_t c;unsigned i;
    s4_clock_check_init(&c,0,0);CHECK(c.status==S4_CLOCK_PENDING&&c.good_windows==0U);
    s4_clock_check_step(&c,199000,199);CHECK(c.status==S4_CLOCK_PENDING&&c.good_windows==0U);
    s4_clock_check_step(&c,200000,200);CHECK(c.status==S4_CLOCK_PENDING&&c.good_windows==1U);
    s4_clock_check_step(&c,400000,400);CHECK(c.status==S4_CLOCK_PENDING&&c.good_windows==2U);
    s4_clock_check_step(&c,600000,600);CHECK(c.status==S4_CLOCK_CONSISTENT&&c.good_windows==3U);
    /* Exact first rejected hardware observation, then three ideal later windows.
     * Later observations showed 14000085 us for 14000 ms; not board evidence here. */
    s4_clock_check_init(&c,9187U,3355769856U);
    s4_clock_check_step(&c,341996U,3355770060U);
    CHECK(c.status==S4_CLOCK_PENDING&&c.startup_rejections==1U&&c.good_windows==0U);
    s4_clock_check_step(&c,541996U,3355770260U);CHECK(c.good_windows==1U&&c.status==S4_CLOCK_PENDING);
    s4_clock_check_step(&c,741996U,3355770460U);CHECK(c.good_windows==2U&&c.status==S4_CLOCK_PENDING);
    s4_clock_check_step(&c,941996U,3355770660U);CHECK(c.status==S4_CLOCK_CONSISTENT);
    /* Persistent wrong rate cannot qualify and faults within the startup budget. */
    s4_clock_check_init(&c,0,0);
    for(i=1U;i<10U;++i){s4_clock_check_step(&c,i*4688U,i*200U);CHECK(c.status==S4_CLOCK_PENDING);}
    s4_clock_check_step(&c,46880,2000);CHECK(c.status==S4_CLOCK_BAD);
    s4_clock_check_step(&c,646880,2600);CHECK(c.status==S4_CLOCK_BAD); /* no recovery */
    s4_clock_check_init(&c,0,0);s4_clock_check_step(&c,2000000U,0);CHECK(c.status==S4_CLOCK_BAD);
    s4_clock_check_init(&c,0,0);s4_clock_check_step(&c,0,2000);CHECK(c.status==S4_CLOCK_BAD);
    s4_clock_check_init(&c,0,0);s4_clock_check_step(&c,200000,200);
    s4_clock_check_step(&c,630001,400);CHECK(c.good_windows==0U&&c.startup_rejections==1U);
    s4_clock_check_step(&c,830001,600);s4_clock_check_step(&c,1030001,800);
    CHECK(c.status==S4_CLOCK_PENDING&&c.good_windows==2U);
    s4_clock_check_step(&c,1230001,1000);CHECK(c.status==S4_CLOCK_CONSISTENT);
    /* Original +/-30 ms inclusive tolerance around a 200 ms runtime window. */
    qualify(&c);s4_clock_check_step(&c,830000,800);CHECK(c.status==S4_CLOCK_CONSISTENT);
    qualify(&c);s4_clock_check_step(&c,830001,800);CHECK(c.status==S4_CLOCK_BAD);
    qualify(&c);s4_clock_check_step(&c,770000,800);CHECK(c.status==S4_CLOCK_CONSISTENT);
    qualify(&c);s4_clock_check_step(&c,769999,800);CHECK(c.status==S4_CLOCK_BAD);
    qualify(&c);s4_clock_check_step(&c,1600000,1600);CHECK(c.status==S4_CLOCK_CONSISTENT);
    qualify(&c);s4_clock_check_step(&c,1601000,1601);CHECK(c.status==S4_CLOCK_BAD);
    qualify(&c);s4_clock_check_step(&c,1600001,600);CHECK(c.status==S4_CLOCK_BAD); /* kernel stopped */
    qualify(&c);s4_clock_check_step(&c,600000,800);CHECK(c.status==S4_CLOCK_BAD); /* hardware stopped */
    qualify(&c);s4_clock_check_step(&c,610000,590);CHECK(c.status==S4_CLOCK_BAD); /* clock moved backwards */
    s4_clock_check_init(&c,0xffffff00U,0xffffff80U);
    for(i=1U;i<=3U;++i){s4_clock_check_step(&c,(uint32_t)(0xffffff00U+i*200000U),(uint32_t)(0xffffff80U+i*200U));}
    CHECK(c.status==S4_CLOCK_CONSISTENT); /* ordinary wraps */
    s4_clock_check_init(&c,100,10);CHECK(c.status==S4_CLOCK_PENDING&&c.good_windows==0U&&c.startup_rejections==0U);
    printf("S4-U08: %s (%u synthetic checks, %u failures); not measured timing.\n",failures?"FAIL":"PASS",checks,failures);
    return failures?1:0;
}
