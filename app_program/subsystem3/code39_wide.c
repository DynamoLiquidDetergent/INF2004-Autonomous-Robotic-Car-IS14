/*
 *	code39_wide.c
 *	Subsystem 3 -- Code 39 decoding from the wide bars only.
 *	See code39_wide.h for the method.
 */

#include "code39_wide.h"
#include <string.h>

/* Code 39 patterns, bit 8 = first element (a bar), 1 = wide.
   Same values as k_code39[] in code39.c (kept separate so this file
   builds on its own for the PC tests). */
#define PAT_STAR  0x094u
static const uint16_t k_pat[26] = {
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C, 0x01C, /* A-J */
    0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106, 0x046, 0x016, /* K-T */
    0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0                              /* U-Z */
};

#define NELEM      29u

/* Wide/narrow ratio search range (the track print measures ~3.7-4.2:
   it was cut off at 3.4 before).  The ratio is measured ONCE per symbol
   from the 4 wide bars of the start and stop '*' (identical for every
   letter), so a wrong letter cannot "fit" by choosing its own ratio. */
#define RATIO_MIN   1.8f
#define RATIO_MAX   4.4f
#define RATIO_STEP  0.02f

/* Wide flags of "*X*" (29 elements), optionally reversed */
static void build_flags(uint16_t pat, int rev, uint8_t fl[NELEM])
{
    uint8_t tmp[NELEM];
    unsigned i;

    for (i = 0; i < 9u; i++) {
        tmp[i]       = (uint8_t)((PAT_STAR >> (8u - i)) & 1u);
        tmp[10u + i] = (uint8_t)((pat      >> (8u - i)) & 1u);
        tmp[20u + i] = (uint8_t)((PAT_STAR >> (8u - i)) & 1u);
    }
    tmp[9]  = 0u;
    tmp[19] = 0u;
    for (i = 0; i < NELEM; i++) {
        fl[i] = rev ? tmp[NELEM - 1u - i] : tmp[i];
    }
}

/* Start of each element, in narrow units, and the wide-bar centres */
static unsigned layout(const uint8_t fl[NELEM], float r, float edge[NELEM + 1u],
                       float wc[C39W_NWIDE])
{
    float    pos = 0.0f, w;
    unsigned i, n = 0;

    for (i = 0; i < NELEM; i++) {
        edge[i] = pos;
        w = fl[i] ? r : 1.0f;
        if (((i & 1u) == 0u) && fl[i] && n < C39W_NWIDE) {
            wc[n++] = pos + w * 0.5f;
        }
        pos += w;
    }
    edge[NELEM] = pos;
    return n;
}

/* Least squares x = a + s*t over n points.  Returns the RMS error in
   units of t (1e9 if the fit is degenerate). */
static float fit(const float *x, const float *t, unsigned n, float *a, float *s)
{
    float mx = 0, mt = 0, vt = 0, ct = 0, e = 0, d;
    unsigned i;

    *a = 0.0f;
    *s = 0.0f;
    for (i = 0; i < n; i++) { mx += x[i]; mt += t[i]; }
    mx /= (float)n; mt /= (float)n;
    for (i = 0; i < n; i++) {
        vt += (t[i] - mt) * (t[i] - mt);
        ct += (t[i] - mt) * (x[i] - mx);
    }
    if (vt <= 0.0f || ct <= 0.0f) return 1e9f;
    *s = ct / vt;
    *a = mx - *s * mt;
    for (i = 0; i < n; i++) {
        d = x[i] - (*a + *s * t[i]);
        e += d * d;
    }
    e /= (float)n;
    /* sqrt without libm: a few Newton steps are plenty here */
    {
        float g = (e > 1.0f) ? e : 1.0f;
        int k;
        if (e <= 0.0f) return 0.0f;
        for (k = 0; k < 20; k++) g = 0.5f * (g + e / g);
        return g / *s;
    }
}

/* Square root without libm (a few Newton steps) */
static float isqrt_f(float e)
{
    float g = (e > 1.0f) ? e : 1.0f;
    int   k;
    if (e <= 0.0f) return 0.0f;
    for (k = 0; k < 20; k++) g = 0.5f * (g + e / g);
    return g;
}

typedef struct {
    float err, a, s;
    bool  ok;
} FIT;

/* One candidate (a letter in one scan direction), laid out for the
   tie-break.  Bars are in sample positions x16 relative to 'base'. */
#define NBARS  15u                              /* bars in "*X*"           */
typedef struct {
    int     L, rev;
    float   err;
    uint8_t fl[NELEM];
    float   edge[NELEM + 1u];
    float   a, s;
    int32_t blo[NBARS], bhi[NBARS];             /* bar edges, samples x16  */
    uint8_t bwide[NBARS];
    int32_t hn, hw;                             /* template half-windows   */
    /* least-squares sums over the samples used */
    int64_t s1, s2, s11, s22, s12, sy1, sy2;
    float   rss, amp;
    bool    solved;
} CAND;

/* Fits of every letter in both directions, and the tied candidates (best
   fits first).  Static: together ~3 KB, too big for the 2 KB task stack.
   Only the barcode decode task calls c39w_decode, so this is safe. */
static FIT  s_fits[26][2];
static CAND s_cand[C39W_MAX_TIED];

/* Length of the overlap of [lo,hi) and [a,b) */
static int32_t overlap(int32_t lo, int32_t hi, int32_t a, int32_t b)
{
    int32_t l = (lo > a) ? lo : a;
    int32_t h = (hi < b) ? hi : b;
    return (h > l) ? (h - l) : 0;
}

/* Least squares  y = k0 + k1*f1 + k2*f2  from the sums, in double (few
   operations, once per candidate).  Sets c->rss (sum of squared errors)
   and c->amp (k1 x full window = ADC counts a narrow bar adds when it
   fills the template window). */
static void solve(CAND *c, double n, double sy, double syy)
{
    double m00 = n,              m01 = (double)c->s1,  m02 = (double)c->s2;
    double m11 = (double)c->s11, m12 = (double)c->s12, m22 = (double)c->s22;
    double b0 = sy, b1 = (double)c->sy1, b2 = (double)c->sy2;
    double c00 = m11 * m22 - m12 * m12;
    double c01 = m02 * m12 - m01 * m22;
    double c02 = m01 * m12 - m02 * m11;
    double det = m00 * c00 + m01 * c01 + m02 * c02;
    double k0, k1, k2;

    c->solved = false;
    if (det <= 1e-9 * (m00 * m11 * m22 + 1.0)) return;    /* degenerate */
    k0 = (c00 * b0 + c01 * b1 + c02 * b2) / det;
    k1 = (c01 * b0 + (m00 * m22 - m02 * m02) * b1 + (m02 * m01 - m00 * m12) * b2) / det;
    k2 = (c02 * b0 + (m01 * m02 - m00 * m12) * b1 + (m00 * m11 - m01 * m01) * b2) / det;
    c->rss    = (float)(syy - (k0 * b0 + k1 * b1 + k2 * b2));
    c->amp    = (float)(k1 * 2.0 * (double)c->hn);
    c->solved = true;
}

void c39w_decode(const C39W_STATE *st, C39W_RESULT *res)
{
    FIT    (*fits)[2] = s_fits;
    float    x[C39W_NWIDE], t[C39W_NWIDE], edge[NELEM + 1u], a = 0, s = 0, e;
    float    gx[4], gt[4], r, ratio = 2.5f, gerr = 1e9f;
    uint8_t  fl[NELEM];
    uint32_t base = st->cx2[0] / 2u;
    float    best_err = 1e9f;
    int      L, rev, k, c;
    int      ncand = 0, nletters = 0;
    CAND    *cand = s_cand;

    memset(res, 0, sizeof(*res));
    res->ch    = C39W_NONE;
    res->ch2   = C39W_NONE;

    /* centres in samples relative to 'base' (keeps floats small and exact) */
    for (k = 0; k < (int)C39W_NWIDE; k++) {
        x[k] = 0.5f * (float)(st->cx2[k] - 2u * base);
    }

    /* 1. ratio from the guards: wide bars 0,1 (one '*') and 4,5 (the
          other '*').  Their pattern is the same for every letter and both
          scan directions, so any letter's layout gives the template. */
    gx[0] = x[0]; gx[1] = x[1]; gx[2] = x[4]; gx[3] = x[5];
    build_flags(k_pat[0], 0, fl);
    for (r = RATIO_MIN; r <= RATIO_MAX + 0.001f; r += RATIO_STEP) {
        (void)layout(fl, r, edge, t);
        gt[0] = t[0]; gt[1] = t[1]; gt[2] = t[4]; gt[3] = t[5];
        e = fit(gx, gt, 4u, &a, &s);
        if (s >= 2.0f && s <= 100.0f && e < gerr) {
            gerr = e;
            ratio = r;
        }
    }
    if (gerr * 100.0f > (float)C39W_MAX_ERR_X100) {
        res->err_x100 = (gerr < 99.0f) ? (uint16_t)(gerr * 100.0f) : 9999u;
        return;                                   /* guards are not '*' */
    }

    /* 2. with that ratio, fit all 6 wide bars for every letter in BOTH
          directions.  Both are kept: a letter scanned backwards can have
          exactly the wide bars of another letter scanned forwards
          (N backwards = D forwards), so the direction must not be decided
          from the wide bars alone. */
    for (L = 0; L < 26; L++) {
        for (rev = 0; rev < 2; rev++) {
            FIT *f = &fits[L][rev];
            f->ok  = false;
            if (st->allow != 0u && (st->allow & (1ul << L)) == 0u) {
                continue;                         /* not on this track */
            }
            build_flags(k_pat[L], rev, fl);
            if (layout(fl, ratio, edge, t) != C39W_NWIDE) continue;
            f->err = fit(x, t, C39W_NWIDE, &f->a, &f->s);
            if (f->s < 2.0f || f->s > 100.0f) continue;  /* implausible scale */
            f->ok = true;
            if (f->err < best_err) best_err = f->err;
        }
    }
    if (best_err * 100.0f > (float)C39W_MAX_ERR_X100) {
        res->err_x100 = (best_err < 99.0f) ? (uint16_t)(best_err * 100.0f) : 9999u;
        return;                                   /* nothing fits */
    }

    /* 3. tie group: every letter+direction that fits (almost) as well as
          the best one, best fits first, at most C39W_MAX_TIED */
    for (L = 0; L < 26; L++) {
        for (rev = 0; rev < 2; rev++) {
            const FIT *f = &fits[L][rev];
            int        pos;
            if (!f->ok) continue;
            if ((f->err - best_err) * 100.0f > (float)C39W_TIE_ERR_X100) continue;
            if (f->err * 100.0f > (float)C39W_MAX_ERR_X100) continue;
            pos = ncand;                          /* insert sorted by err */
            while (pos > 0 && cand[pos - 1].err > f->err) pos--;
            if (pos >= (int)C39W_MAX_TIED) continue;
            for (k = (ncand < (int)C39W_MAX_TIED ? ncand : (int)C39W_MAX_TIED - 1); k > pos; k--) {
                cand[k] = cand[k - 1];
            }
            if (ncand < (int)C39W_MAX_TIED) ncand++;
            cand[pos].L     = L;
            cand[pos].rev   = rev;
            cand[pos].err   = f->err;
            cand[pos].a     = f->a;
            cand[pos].s     = f->s;
            build_flags(k_pat[L], rev, cand[pos].fl);
            (void)layout(cand[pos].fl, ratio, cand[pos].edge, t);
        }
    }
    for (c = 0; c < ncand; c++) {                 /* distinct letters */
        for (k = 0; k < c && cand[k].L != cand[c].L; k++) { }
        if (k == c) nletters++;
    }

    /* 4. look-alikes: compare the BLURRED shape.  GP28's spot is wider
          than a narrow bar, so the narrow bars never show as separate
          bumps -- they only lift the reading a little where they are
          crowded together.  For each candidate, predict that lift: at
          every sample, how much of its narrow bars falls inside a window
          C39W_TPL_W_X10 / 10 narrow units wide (and, as a second term, how
          much of its wide bars falls in a window twice that, for their
          blurred edges).  Fit  reading = k0 + k1*narrow + k2*wide  by least
          squares over the samples away from the wide bars.  The letter
          whose prediction leaves the smallest error wins. */
    {
        int32_t  j, j0 = 0x7FFFFFFF, j1 = -0x7FFFFFFF;
        int      w1 = 0, w2 = -1;
        int64_t  n = 0, sy = 0, syy = 0;
        unsigned i;

        res->ch       = (uint8_t)('A' + cand[0].L);
        res->ntied    = (uint8_t)nletters;

        if (nletters > 1) {
            for (c = 0; c < ncand; c++) {
                CAND *q = &cand[c];
                float hw = (float)C39W_TPL_W_X10 / 20.0f;  /* half window, units */
                int   nb = 0;
                for (i = 0; i < NELEM; i += 2u) {
                    q->blo[nb]   = (int32_t)(16.0f * (q->a + q->s * q->edge[i]));
                    q->bhi[nb]   = (int32_t)(16.0f * (q->a + q->s * q->edge[i + 1u]));
                    q->bwide[nb] = q->fl[i];
                    nb++;
                }
                q->hn = (int32_t)(16.0f * q->s * hw);
                q->hw = 2 * q->hn;
                q->s1 = q->s2 = q->s11 = q->s22 = q->s12 = q->sy1 = q->sy2 = 0;
                {
                    int32_t lo = (int32_t)(q->a + q->s * (q->edge[0] - (float)C39W_TPL_MARGIN));
                    int32_t hi = (int32_t)(q->a + q->s * (q->edge[NELEM] + (float)C39W_TPL_MARGIN)) + 1;
                    if (lo < j0) j0 = lo;
                    if (hi > j1) j1 = hi;
                }
            }

            for (j = j0; j <= j1; j++) {
                uint32_t abs_i = base + (uint32_t)j;
                int32_t  x16 = 16 * j, y;
                bool     use = true;

                if ((st->idx - abs_i) == 0u || (st->idx - abs_i) > C39W_BUF_LEN) {
                    continue;                   /* not (or no longer) stored */
                }
                /* skip samples near ANY candidate's wide bar: the same set
                   for every candidate, so their errors compare fairly */
                for (c = 0; c < ncand && use; c++) {
                    int32_t ex = (int32_t)(1.6f * cand[c].s * (float)C39W_TPL_EXCL_X10);
                    for (i = 0; i < NBARS; i++) {
                        if (cand[c].bwide[i] &&
                            x16 > cand[c].blo[i] - ex && x16 < cand[c].bhi[i] + ex) {
                            use = false;
                            break;
                        }
                    }
                }
                if (!use) continue;

                y = (int32_t)st->buf[abs_i & (C39W_BUF_LEN - 1u)];
                n++;
                sy  += y;
                syy += (int64_t)y * y;
                for (c = 0; c < ncand; c++) {
                    CAND   *q = &cand[c];
                    int32_t f1 = 0, f2 = 0;
                    for (i = 0; i < NBARS; i++) {
                        if (q->bwide[i]) f2 += overlap(q->blo[i], q->bhi[i], x16 - q->hw, x16 + q->hw);
                        else             f1 += overlap(q->blo[i], q->bhi[i], x16 - q->hn, x16 + q->hn);
                    }
                    q->s1  += f1;
                    q->s2  += f2;
                    q->s11 += (int64_t)f1 * f1;
                    q->s22 += (int64_t)f2 * f2;
                    q->s12 += (int64_t)f1 * f2;
                    q->sy1 += (int64_t)f1 * y;
                    q->sy2 += (int64_t)f2 * y;
                }
            }

            for (c = 0; c < ncand; c++) {
                solve(&cand[c], (double)n, (double)sy, (double)syy);
            }
            w1 = -1;
            for (c = 0; c < ncand; c++) {
                if (cand[c].solved && (w1 < 0 || cand[c].rss < cand[w1].rss)) w1 = c;
            }
            if (w1 < 0) {                           /* nothing could be fitted */
                w1 = 0;
                res->uncertain = true;
            }
            for (c = 0; c < ncand; c++) {
                if (cand[c].solved && cand[c].L != cand[w1].L &&
                    (w2 < 0 || cand[c].rss < cand[w2].rss)) w2 = c;
            }
        }

        res->ch         = (uint8_t)('A' + cand[w1].L);
        res->reversed   = (uint8_t)cand[w1].rev;
        res->err_x100   = (uint16_t)(cand[w1].err * 100.0f);
        res->ratio_x100 = (uint16_t)(ratio * 100.0f);
        /* a ratio stuck at the end of the search range means the guard
           bars were measured badly: do not trust anything that follows */
        if (ratio < RATIO_MIN + 0.1f || ratio > RATIO_MAX - 0.1f) {
            res->uncertain = true;
        }
        res->unit_x10   = (uint16_t)(cand[w1].s * 10.0f);
        if (nletters > 1 && cand[w1].solved && n > 0) {
            float rms1 = isqrt_f(cand[w1].rss / (float)n);
            res->rms_x10   = (uint16_t)(rms1 * 10.0f);
            res->narrow    = (int16_t)cand[w1].amp;
            res->nsamp     = (uint16_t)n;
            res->uncertain = res->uncertain ||
                ((float)n < (float)C39W_TPL_MIN_UNITS * cand[w1].s) ||
                (cand[w1].amp < (float)C39W_MIN_NARROW);
            if (w2 >= 0) {
                float rms2 = isqrt_f(cand[w2].rss / (float)n);
                res->ch2       = (uint8_t)('A' + cand[w2].L);
                res->rms2_x10  = (uint16_t)(rms2 * 10.0f);
                /* evidence: the runner-up's extra error, measured in units
                   of the winner's noise level (= n * (rss2 - rss1) / rss1).
                   Needs to be large: a wrong letter only leaves a little
                   more error than the right one when GP28 is blurred. */
                {
                    float ev = (cand[w1].rss > 0.0f)
                             ? (float)n * (cand[w2].rss - cand[w1].rss) / cand[w1].rss
                             : 65535.0f;
                    res->evidence  = (ev > 65535.0f) ? 65535u : (uint16_t)ev;
                    res->uncertain = res->uncertain ||
                                     (ev < (float)C39W_MIN_EVIDENCE);
                }
            }
        } else if (nletters > 1) {
            res->uncertain = true;
        }
    }
}

void c39w_set_letters(C39W_STATE *st, uint32_t allow)
{
    st->allow = allow & 0x03FFFFFFul;
}

void c39w_reset(C39W_STATE *st)
{
    st->dark = false;
    st->n    = 0;
}

bool c39w_feed(C39W_STATE *st, uint16_t adc, bool dark, C39W_RESULT *res)
{
    uint32_t cur = st->idx;
    uint32_t w;
    unsigned k;

    st->buf[cur & (C39W_BUF_LEN - 1u)] = adc;
    st->idx++;

    /* Two wide bars with a narrow space between them: the blur fills the
       space, so the reading may only dip to ~650-760 and never get below
       the caller's "light" level.  So a pulse also ends when the reading
       falls below C39W_SPLIT_PCT % of its own peak, and the next one only
       starts once the reading has risen C39W_SPLIT_RISE above the dip. */
    if (st->dark) {
        if (adc > st->peak) st->peak = adc;
        if ((uint32_t)adc * 100u < (uint32_t)st->peak * C39W_SPLIT_PCT) {
            dark = false;                        /* dip between two bars */
        }
    } else {
        if (adc < st->valley) st->valley = adc;
        if ((uint32_t)adc < (uint32_t)st->valley + C39W_SPLIT_RISE) {
            dark = false;                        /* not risen enough yet */
        }
    }

    if (dark && !st->dark) {                     /* pulse starts */
        st->dark  = true;
        st->start = cur;
        st->peak  = adc;
        return false;
    }
    if (!dark && !st->dark) {
        return false;
    }
    if (dark) {
        return false;                            /* pulse continues */
    }
    st->valley = adc;                            /* pulse ends: track the dip */

    /* pulse ends at 'cur' (first light sample) */
    st->dark = false;
    w = cur - st->start;
    if (w < C39W_MIN_PULSE) {
        return false;                            /* speck */
    }
    if (w > C39W_MAX_PULSE) {
        st->n = 0;                               /* floor / line, not a bar */
        return false;
    }
    if (st->n > 0u && (st->start - st->last_end) > C39W_MAX_GAP) {
        st->n = 0;                               /* too far apart: new symbol */
    }
    st->cx2[st->n] = st->start + cur;
    st->pw[st->n]  = (uint16_t)w;
    st->n++;
    st->last_end   = cur;

    if (st->n < C39W_NWIDE) {
        return false;
    }

    /* all six must be wide bars of similar width */
    {
        uint16_t wmin = 0xFFFFu, wmax = 0u;
        for (k = 0; k < C39W_NWIDE; k++) {
            if (st->pw[k] < wmin) wmin = st->pw[k];
            if (st->pw[k] > wmax) wmax = st->pw[k];
        }
        if ((uint32_t)wmin * C39W_WIDTH_SPREAD < wmax) {
            for (k = 1; k < C39W_NWIDE; k++) {   /* not a wide-only symbol */
                st->cx2[k - 1u] = st->cx2[k];
                st->pw[k - 1u]  = st->pw[k];
            }
            st->n = C39W_NWIDE - 1u;
            return false;
        }
    }

    /* the whole symbol must still be in the buffer */
    if ((cur - st->cx2[0] / 2u) > (C39W_BUF_LEN - 64u)) {
        memset(res, 0, sizeof(*res));
        res->ch = C39W_NONE;
        res->ch2 = C39W_NONE;
        res->err_x100 = 9999u;
    } else {
        c39w_decode(st, res);
    }

    if (res->ch != C39W_NONE) {
        st->n = 0;                               /* done with this symbol */
    } else {
        for (k = 1; k < C39W_NWIDE; k++) {       /* slide by one bar */
            st->cx2[k - 1u] = st->cx2[k];
            st->pw[k - 1u]  = st->pw[k];
        }
        st->n = C39W_NWIDE - 1u;
    }
    return true;
}
