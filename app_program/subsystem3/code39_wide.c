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
#define DATA_FIRST 10u      /* elements 10..18 = the data character */
#define DATA_LAST  18u

/* Wide/narrow ratio search range.  The ratio is measured ONCE per symbol
   from the 4 wide bars of the start and stop '*' (identical for every
   letter), so a wrong letter cannot "fit" by choosing its own ratio. */
#define RATIO_MIN   1.8f
#define RATIO_MAX   3.4f
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

/* Mean raw value over the middle half of [u0,u1) (narrow units).
   base = absolute sample index that x = 0 refers to.  Returns false if
   the samples are no longer in the ring buffer. */
static bool mean_mid(const C39W_STATE *st, uint32_t base, float a, float s,
                     float u0, float u1, float *mean)
{
    float    m = 0.5f * (u0 + u1), h = 0.25f * (u1 - u0);
    int32_t  i0 = (int32_t)(a + s * (m - h));
    int32_t  i1 = (int32_t)(a + s * (m + h));
    uint32_t sum = 0, cnt = 0, abs_i;
    int32_t  i;

    if (i1 < i0) i1 = i0;
    for (i = i0; i <= i1; i++) {
        abs_i = base + (uint32_t)i;
        if ((st->idx - abs_i) == 0u || (st->idx - abs_i) > C39W_BUF_LEN) {
            return false;               /* future, or already overwritten */
        }
        sum += st->buf[abs_i & (C39W_BUF_LEN - 1u)];
        cnt++;
    }
    *mean = (float)sum / (float)cnt;
    return true;
}

typedef struct {
    float err, a, s, r;
    int   rev;
    bool  ok;
} FIT;

void c39w_decode(const C39W_STATE *st, C39W_RESULT *res)
{
    FIT      best[26];
    float    x[C39W_NWIDE], t[C39W_NWIDE], edge[NELEM + 1u], a = 0, s = 0, e;
    float    gx[4], gt[4], r, ratio = 2.5f, gerr = 1e9f;
    uint8_t  fl[NELEM];
    uint32_t base = st->cx2[0] / 2u;
    float    best_err = 1e9f;
    int      L, rev, k;
    float    c1 = -1e9f, c2 = -1e9f;
    int      w1 = -1, w2 = -1, ntied = 0;

    memset(res, 0, sizeof(*res));
    res->ch  = C39W_NONE;
    res->ch2 = C39W_NONE;

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

    /* 2. with that ratio, fit all 6 wide bars for every letter and
          both directions */
    for (L = 0; L < 26; L++) {
        best[L].ok = false;
        best[L].err = 1e9f;
        for (rev = 0; rev < 2; rev++) {
            build_flags(k_pat[L], rev, fl);
            if (layout(fl, ratio, edge, t) != C39W_NWIDE) continue;
            e = fit(x, t, C39W_NWIDE, &a, &s);
            if (s < 2.0f || s > 100.0f) continue;  /* implausible scale */
            if (e < best[L].err) {
                best[L].err = e; best[L].a = a; best[L].s = s;
                best[L].r = ratio; best[L].rev = rev; best[L].ok = true;
            }
        }
        if (best[L].ok && best[L].err < best_err) best_err = best[L].err;
    }
    if (best_err * 100.0f > (float)C39W_MAX_ERR_X100) {
        res->err_x100 = (best_err < 99.0f) ? (uint16_t)(best_err * 100.0f) : 9999u;
        return;                                   /* nothing fits */
    }

    /* 3. tie group: letters that fit (almost) as well as the best one.
          Score each by narrow-bar contrast in the data character. */
    for (L = 0; L < 26; L++) {
        float sb = 0, ss = 0, m = 0, sc;
        int   nb = 0;
        bool  ok = true;
        unsigned i;

        if (!best[L].ok) continue;
        if ((best[L].err - best_err) * 100.0f > (float)C39W_TIE_ERR_X100) continue;
        if (best[L].err * 100.0f > (float)C39W_MAX_ERR_X100) continue;
        ntied++;

        /* Each narrow bar of the data character against the white right
           next to it (the spaces either side), so slow drift of the white
           level cancels out. */
        build_flags(k_pat[L], best[L].rev, fl);
        (void)layout(fl, best[L].r, edge, t);
        for (i = DATA_FIRST; i <= DATA_LAST && ok; i += 2u) {
            float ml = 0, mr = 0, ref = 0;
            int   nref = 0;
            if (fl[i]) continue;                  /* narrow bars only */
            ok = mean_mid(st, base, best[L].a, best[L].s, edge[i], edge[i + 1u], &m);
            /* reference white: a neighbouring narrow space that does not
               touch a wide bar (a wide bar's blurred edge spills into it) */
            if (ok && !fl[i - 2u]) {
                ok = mean_mid(st, base, best[L].a, best[L].s, edge[i - 1u], edge[i], &ml);
                ref += ml; nref++;
            }
            if (ok && !fl[i + 2u]) {
                ok = mean_mid(st, base, best[L].a, best[L].s, edge[i + 1u], edge[i + 2u], &mr);
                ref += mr; nref++;
            }
            if (!ok || nref == 0) continue;
            sb += m;
            ss += ref / (float)nref;
            nb++;
        }
        sc = (ok && nb) ? (sb - ss) / (float)nb : -1e9f;

        if (sc > c1)      { c2 = c1; w2 = w1; c1 = sc; w1 = L; }
        else if (sc > c2) { c2 = sc; w2 = L; }
    }
    if (w1 < 0 || c1 <= -1e8f) return;

    res->ch         = (uint8_t)('A' + w1);
    res->reversed   = (uint8_t)best[w1].rev;
    res->err_x100   = (uint16_t)(best[w1].err * 100.0f);
    res->ratio_x100 = (uint16_t)(best[w1].r * 100.0f);
    res->unit_x10   = (uint16_t)(best[w1].s * 10.0f);
    res->ntied      = (uint8_t)ntied;
    res->contrast   = (int16_t)c1;
    if (w2 >= 0 && c2 > -1e8f) {
        res->ch2       = (uint8_t)('A' + w2);
        res->contrast2 = (int16_t)c2;
        res->uncertain = (c1 - c2) < (float)C39W_MIN_MARGIN;
    }
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

    if (dark && !st->dark) {                     /* pulse starts */
        st->dark  = true;
        st->start = cur;
        return false;
    }
    if (dark || !st->dark) {
        return false;
    }

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
