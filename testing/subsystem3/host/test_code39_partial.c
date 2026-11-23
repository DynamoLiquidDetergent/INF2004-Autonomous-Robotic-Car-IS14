/* PC tests for code39_decode_partial() (S3-H07): a Code 39 symbol whose stop
   marker faded out before all 29 elements were read.
   Build and run: sh testing/subsystem3/host/run.sh */
#include <stdio.h>
#include "../../../app_program/subsystem3/code39.h"

static int s_pass = 0, s_fail = 0;

static void check(const char *id, int ok, const char *what)
{
    printf("[HOST] %s %s  %s\n", id, ok ? "PASS" : "FAIL", what);
    if (ok) s_pass++; else s_fail++;
}

static unsigned load(BcElem *e, const unsigned *w, unsigned n)
{
    unsigned i;
    for (i = 0; i < n; i++) e[i].w = (uint16_t)w[i];
    return n;
}

int main(void)
{
    /* lab run, 2026-11-23 (S5-M42 run 17): *Q* driven over right to left,
       the narrow decoder gave up after 25 of 29 elements */
    static const unsigned lab_q[25] = { 17, 21, 74, 20, 80, 28, 15, 95, 20, 25,
                                        86, 80, 71, 23, 17, 31, 10, 34, 18, 32,
                                        12, 34, 76, 34, 73 };
    BcElem e[29];
    unsigned n, i;
    unsigned w[29];

    code39_init();

    n = load(e, lab_q, 25);
    check("S3-H07a", code39_decode_partial(e, n) == 'Q', "lab trace, 25 of 29 elements: Q");

    n = load(e, lab_q, 24);
    check("S3-H07b", code39_decode_partial(e, n) == 'Q', "same trace cut to 24 elements: Q");

    n = load(e, lab_q, 23);
    check("S3-H07c", code39_decode_partial(e, n) == CODE39_INVALID, "23 elements: too few, nothing");

    /* stop element 22 should be wide (76): make it narrow -> stop marker does not match */
    n = load(e, lab_q, 25);
    e[22].w = 30;
    check("S3-H07d", code39_decode_partial(e, n) == CODE39_INVALID, "stop marker element wrong: nothing");

    /* start marker broken (element 2 narrow) */
    n = load(e, lab_q, 25);
    e[2].w = 22;
    check("S3-H07e", code39_decode_partial(e, n) == CODE39_INVALID, "start marker wrong: nothing");

    /* narrow and wide too close (all wides shrunk to 40, widest narrow 34) */
    n = load(e, lab_q, 25);
    for (i = 0; i < n; i++) if (e[i].w > 60) e[i].w = 40;
    check("S3-H07f", code39_decode_partial(e, n) == CODE39_INVALID, "wide only 1.2x the widest narrow: nothing");

    /* forward read of *A*, clean widths, last 4 elements missing */
    {
        /* * = n W n n W n W n n, A = W n n n n W n n W */
        static const unsigned star[9] = { 20, 60, 20, 20, 60, 20, 60, 20, 20 };
        static const unsigned a[9]    = { 60, 20, 20, 20, 20, 60, 20, 20, 60 };
        for (i = 0; i < 9; i++) { w[i] = star[i]; w[10 + i] = a[i]; w[20 + i] = star[i]; }
        w[9] = 20; w[19] = 20;
        n = load(e, w, 25);
        check("S3-H07g", code39_decode_partial(e, n) == 'A', "forward *A*, 25 of 29: A");
        n = load(e, w, 29);
        check("S3-H07h", code39_decode_partial(e, n) == CODE39_INVALID, "all 29 elements: left to the full decoder");
    }

    printf("[HOST] code39_partial SUMMARY %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
