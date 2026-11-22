/*
 *	test_code39_wide.c -- PC tests for subsystem 3's wide-bar Code 39 decoder
 *	(app_program/subsystem3/code39_wide.c), fed sample by sample exactly as
 *	barcode_decode.c does: raw ADC plus the dark/light decision with the
 *	same hysteresis (dark at 840, light again below 610).
 *
 *	S3-H01  (b: any letter allowed: unsure is fine, never a wrong letter)
 *	        real capture: *A* on the printed test sheet, driven over at about
 *	        118 mm/s (S5-M42 run 4, 2026-11-22) -> A
 *	(S3-H02..H04: a 4-letter map per case, as on the car: the letter and 3 others)
 *	S3-H02  synthetic, sharp spot and faint narrow bars (like the track print
 *	        the decoder was tuned on), both directions, 50 / 100 / 150 mm/s,
 *	        with noise: a: all 26 letters, never a wrong letter (look-alikes in
 *	        the same map may be sent as unsure); b: the map C D A B, all read
 *	S3-H03  synthetic, blurred spot (as measured in S3-H01: an 8 mm edge) and
 *	        solid narrow bars (the printed test sheet): same cases
 *	S3-H04  the 4 mapped letters only (C D A B), as on the car, blurred spot
 *	S3-H06  wide bars of uneven darkness (each bar 0..x lighter): a: sharp, x = 45 %;
 *	        b: today's blur, faint narrow bars, x = 20 %; c: today's blur, solid
 *	        narrow bars, x = 20 %: every letter read; d: blur and x = 45 %: never wrong
 *	S3-H05  no false letter: white floor, a junction crossbar, the black line
 *
 *	Build and run (WSL):  sh testing/subsystem3/host/run.sh
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../../../app_program/subsystem3/code39_wide.h"

#define DARK_ON   840u
#define DARK_OFF  610u
#define WHITE     230.0
#define BLACK     2560.0

static int n_pass, n_fail;

static void check(int ok, const char *id, const char *what)
{
	printf("[HOST] %s %s  %s\n", id, ok ? "PASS" : "FAIL", what);
	if (ok) n_pass++; else n_fail++;
}

/* Feed a trace; return the first letter decoded (not uncertain), or 0. */
static int decode_trace(const unsigned short *v, int n, unsigned allow, int *n_tries, int *bad)
{
	static C39W_STATE st;
	C39W_RESULT r;
	int dark = 0, i, got = 0;

	memset(&st, 0, sizeof(st));
	c39w_reset(&st);
	c39w_set_letters(&st, allow);
	*n_tries = 0;
	*bad = 0;
	for (i = 0; i < n; i++) {
		dark = dark ? (v[i] >= DARK_OFF) : (v[i] >= DARK_ON);
		if (c39w_feed(&st, v[i], dark != 0, &r)) {
			(*n_tries)++;
			if (getenv("C39_DEBUG")) printf("        try @%d: ch=%c unc=%d err=%u ratio=%u unit=%u ntied=%d ch2=%c ev=%u narrow=%d\n",
				i, r.ch == C39W_NONE ? '-' : r.ch, r.uncertain, r.err_x100, r.ratio_x100, r.unit_x10,
				r.ntied, r.ch2 == C39W_NONE ? '-' : r.ch2, r.evidence, r.narrow);
			if (r.ch != C39W_NONE && !r.uncertain) {
				if (got == 0) got = r.ch;
				else if (got != r.ch) *bad = 1;
			}
		}
	}
	return got;
}

/* ---- synthetic traces ---- */
static const unsigned short k_pat[26] = {
	0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C, 0x01C,
	0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106, 0x046, 0x016,
	0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0
};
#define PAT_STAR 0x094u
#define RES_MM   0.05

static unsigned long rng = 12345u;
static double g_wide_var = 0.0;   /* wide bars: darkness 1 - g_wide_var*rand(0..1) */
static double noise(double amp)
{
	rng = rng * 1103515245u + 12345u;
	return amp * ((double)((rng >> 8) & 0xFFFF) / 32768.0 - 1.0);
}

/* darkness profile (0..1) of "*X*" along the travel, RES_MM per cell */
static int profile(int letter, int rev, double narrow_mm, double wide_mm, double narrow_dark,
		   double quiet_mm, double *p, int cap)
{
	unsigned pats[3] = { PAT_STAR, k_pat[letter], PAT_STAR };
	double el[29 + 2];
	int nel = 0, c, k, i, n = 0;
	double darkn[31];

	for (c = 0; c < 3; c++) {
		for (k = 0; k < 9; k++) {
			int wide = (pats[c] >> (8 - k)) & 1;
			el[nel] = wide ? wide_mm : narrow_mm;
			darkn[nel] = (k % 2 == 0) ? (wide ? 1.0 - g_wide_var * (noise(0.5) + 0.5) : narrow_dark) : 0.0;
			nel++;
		}
		if (c < 2) { el[nel] = narrow_mm; darkn[nel] = 0.0; nel++; }
	}
	for (i = 0; i < (int)(quiet_mm / RES_MM) && n < cap; i++) p[n++] = 0.0;
	for (k = 0; k < nel; k++) {
		int e = rev ? nel - 1 - k : k;
		int cells = (int)(el[e] / RES_MM + 0.5);
		for (i = 0; i < cells && n < cap; i++) p[n++] = darkn[e];
	}
	for (i = 0; i < (int)(quiet_mm / RES_MM) && n < cap; i++) p[n++] = 0.0;
	return n;
}

/* blur with a Gaussian spot (sigma_mm), sample every 2 ms at speed_mms */
static int sample(const double *p, int n, double sigma_mm, double speed_mms, double noise_amp,
		  unsigned short *out, int cap)
{
	int m = 0, half = (int)(3.0 * sigma_mm / RES_MM) + 1;
	double step = speed_mms * 0.002 / RES_MM, x;

	for (x = 0.0; x < n && m < cap; x += step) {
		double s = 0.0, wsum = 0.0, v;
		int c = (int)x, j;
		for (j = -half; j <= half; j++) {
			double w = exp(-0.5 * pow(j * RES_MM / sigma_mm, 2.0));
			int q = c + j;
			double d = (q >= 0 && q < n) ? p[q] : 0.0;
			s += w * d;
			wsum += w;
		}
		v = WHITE + (BLACK - WHITE) * (s / wsum) + noise(noise_amp);
		if (v < 0) v = 0;
		if (v > 4095) v = 4095;
		out[m++] = (unsigned short)v;
	}
	return m;
}

static double g_prof[40000];
static unsigned short g_tr[20000];

/* one family of synthetic cases; returns failures */
static int run_family(const char *id, double sigma, double narrow_dark, double narrow_mm,
		      double wide_mm, unsigned allow_mask, const char *letters, int require_all)
{
	static const double speeds[3] = { 50.0, 100.0, 150.0 };
	int fails = 0, cases = 0, wrong = 0, s, rev;
	const char *lp;
	char msg[200];

	for (lp = letters; *lp; lp++) {
		int L = *lp - 'A';
		for (rev = 0; rev < 2; rev++) {
			for (s = 0; s < 3; s++) {
				int n = profile(L, rev, narrow_mm, wide_mm, narrow_dark, 40.0, g_prof, 40000);
				int m = sample(g_prof, n, sigma, speeds[s], 30.0, g_tr, 20000);
				unsigned allow = allow_mask ? allow_mask :
					((1u << L) | (1u << ((L + 1) % 26)) | (1u << ((L + 7) % 26)) | (1u << ((L + 13) % 26)));
				int tries, bad, got = decode_trace(g_tr, m, allow, &tries, &bad);
				cases++;
				if ((got != 0 && got != *lp) || bad) wrong++;
				if (got != *lp || bad) {
					fails++;
					if (fails <= 8) {
						printf("        %c %s %3.0f mm/s: got %c (%d tries%s)\n", *lp,
						       rev ? "rev" : "fwd", speeds[s], got ? got : '-', tries,
						       bad ? ", a wrong letter too" : "");
					}
				}
			}
		}
	}
	snprintf(msg, sizeof msg, "%d of %d read right, %d wrong letters (spot sigma %.1f mm, narrow bars %.0f %% dark%s)",
		 cases - fails, cases, wrong, sigma, narrow_dark * 100.0,
		 require_all ? "" : "; others not sent as unsure is allowed");
	check(wrong == 0 && (!require_all || fails == 0), id, msg);
	return fails;
}

static void no_false_letter(void)
{
	int i, m = 0, tries, bad, got;
	/* 20 cm white, a 20 mm crossbar, 20 cm white, 30 cm of black line, white */
	for (i = 0; i < 4000; i++) g_prof[i] = 0.0;
	for (i = 4000; i < 4400; i++) g_prof[i] = 1.0;
	for (i = 4400; i < 8400; i++) g_prof[i] = 0.0;
	for (i = 8400; i < 14400; i++) g_prof[i] = 1.0;
	for (i = 14400; i < 16000; i++) g_prof[i] = 0.0;
	m = sample(g_prof, 16000, 3.1, 100.0, 30.0, g_tr, 20000);
	got = decode_trace(g_tr, m, 0, &tries, &bad);
	check(got == 0, "S3-H05", "white floor, a junction crossbar and the black line: no letter");
}

int main(void)
{
	FILE *f = fopen("testing/subsystem3/host/cap_2026-11-22_A_118mms.txt", "r");
	static unsigned short cap[4000];
	char line[4096];
	int n = 0, tries, bad, got;
	unsigned mapped = (1u << ('C' - 'A')) | (1u << ('D' - 'A')) | (1u << 0) | (1u << 1);

	if (f == NULL) { printf("cannot open the capture (run from the repo root)\n"); return 2; }
	while (fgets(line, sizeof line, f)) {
		char *p = line, *e;
		if (line[0] == '#') continue;
		for (;;) {
			long v = strtol(p, &e, 10);
			if (e == p) break;
			cap[n++] = (unsigned short)v;
			p = e;
		}
	}
	fclose(f);
	for (; n < 1500; n++) cap[n] = 240;               /* the rest was white */

	got = decode_trace(cap, n, mapped, &tries, &bad);
	{
		char msg[120];
		snprintf(msg, sizeof msg, "real capture of *A* at ~118 mm/s, map C D A B: got %c (%d tries)",
			 got ? got : '-', tries);
		check(got == 'A' && !bad, "S3-H01a", msg);
		got = decode_trace(cap, n, 0, &tries, &bad);
		snprintf(msg, sizeof msg, "same capture, any of 26 letters allowed: got %c (%d tries), never a wrong letter",
			 got ? got : '-', tries);
		check((got == 0 || got == 'A') && !bad, "S3-H01b", msg);
	}

	run_family("S3-H02a", 1.2, 0.15, 3.3, 8.5, 0, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", 0);
	run_family("S3-H02b", 1.2, 0.15, 3.3, 8.5, mapped, "CDAB", 1);
	run_family("S3-H03", 3.1, 1.00, 3.0, 9.0, 0, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", 1);
	run_family("S3-H04", 3.1, 1.00, 3.0, 9.0, mapped, "CDAB", 1);
	/* S3-H06: wide bars of uneven darkness (the track print's wide bars peak
	   anywhere from ~1450 to ~2800), sharp and blurred, map C D A B */
	g_wide_var = 0.45;
	run_family("S3-H06a", 1.2, 0.15, 3.3, 8.5, mapped, "CDAB", 1);   /* track print, sharp, very uneven */
	g_wide_var = 0.20;
	run_family("S3-H06b", 3.1, 0.15, 3.3, 8.5, mapped, "CDAB", 1);   /* track print, today's blur */
	run_family("S3-H06c", 3.1, 1.00, 3.0, 9.0, mapped, "CDAB", 1);   /* printed sheet, today's blur */
	g_wide_var = 0.45;
	run_family("S3-H06d", 3.1, 0.15, 3.3, 8.5, mapped, "CDAB", 0);   /* blur AND very uneven: never wrong */
	g_wide_var = 0.0;
	no_false_letter();

	printf("[HOST] code39_wide SUMMARY %d passed, %d failed\n", n_pass, n_fail);
	return n_fail ? 1 : 0;
}
