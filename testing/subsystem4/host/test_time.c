#include <assert.h>
#include <stdio.h>
#include "s4_time.h"
int main(void)
{
    uint32_t r = 20U, m = 0U;
    assert(s4_delay_to_release(3U, &r, 20U, &m) == 17U);
    assert(s4_delay_to_release(20U, &r, 20U, &m) == 0U);
    assert(m == 0U);
    assert(s4_delay_to_release(65U, &r, 20U, &m) == 15U);
    assert((r == 80U) && (m == 3U));
    r = 5U; m = 0U;
    assert(s4_delay_to_release(UINT32_MAX-4U, &r, 20U, &m) == 10U);
    puts("time: PASS (deadline, overrun, wrap)");
    return 0;
}
