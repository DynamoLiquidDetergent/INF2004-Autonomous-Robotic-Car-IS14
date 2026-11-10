/*
 *	obstacle_profile.c
 *	Subsystem 5 -- pure maths for the obstacle scan (see obstacle_profile.h).
 *
 *	No floating point (soft-float is slow on the Cortex-M0+) and no libm:
 *	cosines come from a table in 7.5 degree steps, square root is integer.
 */

#include "obstacle_profile.h"

/* cos(k * 7.5 degrees) * 1000, k = 0..24 (0..180 degrees) */
static const int16_t cos_tab[25] = {
	1000, 991, 966, 924, 866, 793, 707, 609, 500, 383, 259, 131, 0,
	-131, -259, -383, -500, -609, -707, -793, -866, -924, -966, -991, -1000
};

/* cos(k degrees) * 1000, k = 0..90 */
static const int16_t cos_deg_tab[91] = {
	1000, 1000, 999, 999, 998, 996, 995, 993, 990, 988, 985, 982, 978,
	974, 970, 966, 961, 956, 951, 946, 940, 934, 927, 921, 914, 906,
	899, 891, 883, 875, 866, 857, 848, 839, 829, 819, 809, 799, 788,
	777, 766, 755, 743, 731, 719, 707, 695, 682, 669, 656, 643, 629,
	616, 602, 588, 574, 559, 545, 530, 515, 500, 485, 469, 454, 438,
	423, 407, 391, 375, 358, 342, 326, 309, 292, 276, 259, 242, 225,
	208, 191, 174, 156, 139, 122, 105, 87, 70, 52, 35, 17, 0
};

/* cos and sin of a whole number of degrees, 0..180, times 1000 */
static int32_t cosd(int d)
{
	if (d < 0) {
		d = 0;
	}
	if (d > 180) {
		d = 180;
	}
	return (d <= 90) ? cos_deg_tab[d] : -cos_deg_tab[180 - d];
}

static int32_t sind(int d)
{
	return cosd((d > 90) ? d - 90 : 90 - d);
}

void obs_cfg_default(OBS_CFG *cfg)
{
	/* PLACEHOLDERS: measure on the real robot before trusting these */
	cfg->detect_cm = 50;
	cfg->stop_cm = 8;
	cfg->robot_half_width_cm = 8;
	cfg->margin_cm = 3;
	cfg->max_shift_cm = 40;
	cfg->beam_comp_deg = 15;	/* 15 degree HC-SR04 beam, assumption A5 */
	cfg->plan_width_cm = 20;	/* spec: obstacles 10..20 cm wide, plan for the widest */
	cfg->bg_enter_cm = 6;		/* 2 x the sensor's typical +/-3 cm scatter */
	cfg->bg_exit_cm = 3;
	cfg->detect_hyst_cm = 5;
	cfg->cluster_jump_cm = 10;	/* flat face at 17 cm: neighbours differ by at most 4 cm; a wall behind: 30+ */
	cfg->approach_cm = 17;		/* aim for the middle of the band */
	cfg->approach_near_cm = 15;	/* wanted gap before the bypass: 15..20 cm; closer: back away */
	cfg->approach_far_cm = 20;	/* farther: creep forward */
	cfg->approach_guard_cm = 13;	/* never closer than this while creeping */
	cfg->approach_back_max_cm = 15;	/* back away at most 15 cm if stopped too close */
	cfg->approach_min_cm = 3;	/* HC-SR04 minimum range is 2 cm: closer than 3 is not trusted */
}

void obs_scan_clear(OBS_SCAN *s)
{
	unsigned i;

	for (i = 0; i < OBS_NUM_ANGLES; i++) {
		s->dist_cm[i] = OBS_DIST_NONE;
	}
	s->measured = 0;
	s->fine_deg[0] = 0;
	s->fine_deg[1] = 0;
	s->fine_cm[0] = 0;
	s->fine_cm[1] = 0;
}

void obs_scan_init(OBS_SCAN *s)
{
	unsigned i;

	obs_scan_clear(s);
	for (i = 0; i < OBS_NUM_ANGLES; i++) {
		s->bg_cm[i] = OBS_DIST_NONE;
	}
	s->bg_known = 0;
}

void obs_scan_set_bg(OBS_SCAN *s, unsigned idx, uint16_t cm)
{
	if (idx >= OBS_NUM_ANGLES) {
		return;
	}
	s->bg_cm[idx] = cm;
	s->bg_known |= (uint16_t)(1u << idx);
}

/* a usable background: taken, and something was actually seen there */
static int bg_present(const OBS_SCAN *s, unsigned idx)
{
	return ((s->bg_known >> idx) & 1u) && s->bg_cm[idx] != OBS_DIST_NONE;
}

static int32_t sin_k(int k);

unsigned obs_bg_clear_ahead(OBS_SCAN *s, const OBS_CFG *cfg)
{
	unsigned i, n = 0;
	int off, ahead, side;

	for (i = 0; i < OBS_NUM_ANGLES; i++) {
		if (!bg_present(s, i)) {
			continue;
		}
		off = ((int)i > OBS_IDX_AHEAD) ? (int)i - OBS_IDX_AHEAD : OBS_IDX_AHEAD - (int)i;
		/* straight ahead (5..7): always. Elsewhere: only when it lies in the path */
		ahead = (int)(((int32_t)s->bg_cm[i] * cos_tab[2 * off]) / 1000);
		side  = (int)(((int32_t)s->bg_cm[i] * sin_k(2 * off)) / 1000);
		if (s->bg_cm[i] <= cfg->detect_cm + cfg->detect_hyst_cm &&
		    (off <= 1 ||
		     (ahead <= (int)(cfg->detect_cm + cfg->detect_hyst_cm) &&
		      side <= (int)(cfg->robot_half_width_cm + cfg->margin_cm)))) {
			s->bg_cm[i] = OBS_DIST_NONE;
			n++;
		}
	}
	return n;
}

uint16_t obs_median(const uint16_t *v, unsigned n)
{
	uint16_t t[5];
	unsigned i, j;

	if (n == 0) {
		return OBS_DIST_NONE;
	}
	if (n > 5) {
		n = 5;
	}
	for (i = 0; i < n; i++) {
		t[i] = v[i];
		for (j = i; j > 0 && t[j - 1] > t[j]; j--) {
			uint16_t x = t[j];
			t[j] = t[j - 1];
			t[j - 1] = x;
		}
	}
	return t[n / 2];	/* OBS_DIST_NONE = 0x1FF sorts last: "largest" */
}

void obs_scan_set(OBS_SCAN *s, unsigned idx, uint16_t cm)
{
	if (idx >= OBS_NUM_ANGLES) {
		return;
	}
	s->dist_cm[idx] = cm;
	s->measured |= (uint16_t)(1u << idx);
}

int obs_scan_measured(const OBS_SCAN *s, unsigned idx)
{
	return (idx < OBS_NUM_ANGLES) && ((s->measured >> idx) & 1u);
}

/* Hysteresis. ENTER (start an obstacle) is strict, STAY (extend it to a
   neighbouring angle) is looser, so an edge reading that wobbles by a few cm
   does not flip the width by a whole 15 degree step.
   With a background the test is "closer than the empty-track distance by
   enough"; without one it is the plain detect distance. */
int obs_dist_is_hit(const OBS_SCAN *s, unsigned idx, uint16_t cm, const OBS_CFG *cfg)
{
	if (idx >= OBS_NUM_ANGLES || cm == OBS_DIST_NONE || cm > cfg->detect_cm) {
		return 0;
	}
	if (bg_present(s, idx)) {
		return (int)s->bg_cm[idx] >= (int)cm + (int)cfg->bg_enter_cm;
	}
	return 1;
}

int obs_scan_is_hit(const OBS_SCAN *s, unsigned idx, const OBS_CFG *cfg)
{
	return obs_scan_measured(s, idx) && obs_dist_is_hit(s, idx, s->dist_cm[idx], cfg);
}

static int is_stay(const OBS_SCAN *s, unsigned idx, const OBS_CFG *cfg)
{
	uint16_t cm;

	if (!obs_scan_measured(s, idx)) {
		return 0;
	}
	cm = s->dist_cm[idx];
	if (cm == OBS_DIST_NONE || cm > cfg->detect_cm + cfg->detect_hyst_cm) {
		return 0;
	}
	if (bg_present(s, idx)) {
		return (int)s->bg_cm[idx] >= (int)cm + (int)cfg->bg_exit_cm;
	}
	return 1;
}

/* Two neighbouring angles belong to the same obstacle only if their distances are
   close: a wall far behind a box is not part of the box. */
static int same_surface(const OBS_SCAN *s, const OBS_CFG *cfg, int a, int b)
{
	int d = (int)s->dist_cm[a] - (int)s->dist_cm[b];

	return (d < 0 ? -d : d) <= (int)cfg->cluster_jump_cm;
}

/* Find the obstacle: the closest hit, grown left and right over neighbouring
   measured hits.  Returns 0 if nothing was hit. */
static int find_cluster(const OBS_SCAN *s, const OBS_CFG *cfg,
			int *lo, int *hi, int *closest)
{
	int i, best = -1;

	for (i = 0; i < OBS_NUM_ANGLES; i++) {
		if (obs_scan_is_hit(s, (unsigned)i, cfg) &&
		    (best < 0 || s->dist_cm[i] < s->dist_cm[best])) {
			best = i;
		}
	}
	if (best < 0) {
		return 0;
	}
	*closest = best;
	*lo = best;
	*hi = best;
	while (*lo > 0 && is_stay(s, (unsigned)(*lo - 1), cfg) &&
	       same_surface(s, cfg, *lo - 1, *lo)) {
		(*lo)--;
	}
	while (*hi < OBS_NUM_ANGLES - 1 && is_stay(s, (unsigned)(*hi + 1), cfg) &&
	       same_surface(s, cfg, *hi + 1, *hi)) {
		(*hi)++;
	}
	return 1;
}

int obs_next_edge_idx(const OBS_SCAN *s, const OBS_CFG *cfg, int dir)
{
	int lo, hi, closest, n;

	if (!find_cluster(s, cfg, &lo, &hi, &closest)) {
		return -1;
	}
	n = (dir < 0) ? lo - 1 : hi + 1;
	if (n < 0 || n >= OBS_NUM_ANGLES) {
		return -1;		/* scan limit reached */
	}
	if (obs_scan_measured(s, (unsigned)n)) {
		return -1;		/* neighbour measured and not a hit: edge found */
	}
	return n;
}

uint32_t obs_isqrt(uint32_t v)
{
	uint32_t res = 0, bit = 1uL << 30;

	while (bit > v) {
		bit >>= 2;
	}
	while (bit != 0) {
		if (v >= res + bit) {
			v -= res + bit;
			res = (res >> 1) + bit;
		} else {
			res >>= 1;
		}
		bit >>= 2;
	}
	return res;
}

static uint16_t cap_cm(int32_t v)
{
	if (v < 0) {
		return 0;
	}
	if (v > OBS_DIST_CAP) {
		return OBS_DIST_CAP;
	}
	return (uint16_t)v;
}

/* the neighbour of 'idx' in direction dir is unmeasured, or idx is at the scan limit */
static int edge_unresolved(const OBS_SCAN *s, int idx, int dir)
{
	int n = idx + dir;

	return (n < 0 || n >= OBS_NUM_ANGLES || !obs_scan_measured(s, (unsigned)n));
}

/* Sideways extent of the obstacle made of indices lo..hi (x: + = right of the
   centre line). Also returns the raw width. 'unresolved' is set when an edge
   was not measured. */
/* sin of k * 7.5 degrees, * 1000 */
static int32_t sin_k(int k)
{
	return (k <= 12) ? cos_tab[12 - k] : cos_tab[k - 12];
}

/* Outer bound of the RIGHT edge from a fine reading: the beam at 'deg' (degrees from the
   right) still saw the obstacle at r cm, the beam one fine step further out did not. A
   beam sees an edge up to half its width away, so the edge is at least a fine step
   plus half a beam further in than the miss, i.e. at an angle of at least
   deg - step + half. The face keeps its depth, not its range, so the edge is also
   at x = depth / tan(angle); take the larger of the two (the same rule as for the
   coarse bound). Whole centimetres, + = right of the centre line (a left edge uses
   180 - deg and negates the result). */
static int32_t fine_edge_cm(const OBS_CFG *cfg, int deg, int32_t r)
{
	int half = (int)cfg->beam_comp_deg / 2;
	int a_e = deg - OBS_FINE_STEP_DEG + half;
	int a_d = deg + half;
	int32_t xc, xd;

	if (a_e < 1) {
		a_e = 1;
	}
	if (a_d > 90) {
		a_d = 90;
	}
	xc = r * cosd(a_e) / 1000;
	xd = r * sind(a_d) * cosd(a_e) / (sind(a_e) * 1000);
	return (xd > xc) ? xd : xc;
}

static void cluster_geom(const OBS_SCAN *s, const OBS_CFG *cfg, int lo, int hi, int closest,
			 int32_t *x_r, int32_t *x_l, int32_t *w, int *unresolved,
			 int32_t *x_ro, int32_t *x_lo, int fine)
{
	int32_t r_r = s->dist_cm[lo], r_l = s->dist_cm[hi], chord2;
	int th_r, th_l, h;

	*unresolved = (edge_unresolved(s, lo, -1) || edge_unresolved(s, hi, +1));

	/* The 15 degree beam sees an edge up to half a beam early, so the true
	   edges lie half a beam INSIDE the outermost hit angles. Angles here
	   are in units of 7.5 degrees. */
	h = cfg->beam_comp_deg / 15;
	th_r = 2 * lo + h;
	th_l = 2 * hi - h;
	if (hi > lo && th_l <= th_r) {
		/* span no wider than the beam: do not compensate (keeps the
		   width estimate on the safe, wider side) */
		th_r = 2 * lo;
		th_l = 2 * hi;
	}

	if (hi == lo) {
		/* One sample: width is at most one beam footprint,
		   2 * tan(7.5 deg) * r = 0.263 * r. */
		*w = (int32_t)s->dist_cm[closest] * 263 / 1000;
		*x_r = (int32_t)s->dist_cm[closest] * cos_tab[2 * closest] / 1000 + *w / 2;
		*x_l = (int32_t)s->dist_cm[closest] * cos_tab[2 * closest] / 1000 - *w / 2;
		*unresolved = 1;
	} else {
		/* chord between the two edge points (law of cosines) */
		chord2 = r_r * r_r + r_l * r_l - 2 * r_r * r_l * cos_tab[th_l - th_r] / 1000;
		if (chord2 < 0) {
			chord2 = 0;
		}
		*w = (int32_t)obs_isqrt((uint32_t)chord2);
		*x_r = r_r * cos_tab[th_r] / 1000;	/* + = right of the centre line */
		*x_l = r_l * cos_tab[th_l] / 1000;	/* - = left of the centre line */
	}

	/* The estimate above is the middle one. For the bypass the true edge may
	   lie anywhere from half a beam inside to half a beam OUTSIDE the outermost
	   hit angle (the next angle saw nothing, so it cannot be further). Plan with
	   the outside one: with 15 degree steps that is up to 0.26 * range further out. */
	if (hi == lo) {
		*x_ro = *x_r;		/* already the full beam footprint */
		*x_lo = *x_l;
	} else {
		int tr = 2 * lo - cfg->beam_comp_deg / 15, tl = 2 * hi + cfg->beam_comp_deg / 15;

		if (tr < 0) {
			tr = 0;
		}
		if (tl > 24) {
			tl = 24;
		}
		*x_ro = r_r * cos_tab[tr] / 1000;
		*x_lo = r_l * cos_tab[tl] / 1000;
		/* A flat face keeps its DEPTH, not its range, so at the outer angle it is
		   further away than the hit reading. Depth based edge, never smaller than
		   the constant-range one: x = r * sin(hit angle) / tan(outer angle). */
		{
			int kr = (tr < 1) ? 1 : tr, kl = (tl > 23) ? 23 : tl;
			int32_t xr2 = r_r * sin_k(2 * lo) * cos_tab[kr] / (sin_k(kr) * 1000);
			int32_t xl2 = r_l * sin_k(2 * hi) * cos_tab[kl] / (sin_k(kl) * 1000) - 1;

			if (xr2 > *x_ro) {
				*x_ro = xr2;
			}
			if (xl2 < *x_lo) {
				*x_lo = xl2;
			}
		}
		if (*x_ro < *x_r) {
			*x_ro = *x_r;
		}
		if (*x_lo > *x_l) {
			*x_lo = *x_l;
		}
	}

	/* A finer look at an edge (obs_fine_edges) replaces the outer bound of that edge,
	   but never goes inside the middle estimate. */
	if (fine) {
		if (s->fine_deg[0] != 0) {
			int32_t x = fine_edge_cm(cfg, s->fine_deg[0], s->fine_cm[0]);

			*x_ro = (hi > lo && x < *x_r) ? *x_r : x;
		}
		if (s->fine_deg[1] != 0) {
			int32_t x = -fine_edge_cm(cfg, 180 - s->fine_deg[1], s->fine_cm[1]);

			*x_lo = (hi > lo && x > *x_l) ? *x_l : x;
		}
		if (*x_ro < *x_lo) {
			*x_ro = *x_lo = (*x_ro + *x_lo) / 2;
		}
	}
}

/* Plan for at least the widest obstacle the spec allows, centred where we
   measured it. A narrower obstacle is then simply passed with more room than
   needed; a mis-measured one is still cleared. */
static void plan_span(const OBS_CFG *cfg, int32_t *x_r, int32_t *x_l)
{
	if (cfg->plan_width_cm != 0) {
		int32_t c = (*x_r + *x_l) / 2;
		int32_t half = (int32_t)cfg->plan_width_cm / 2;

		if (c + half > *x_r) {
			*x_r = c + half;
		}
		if (c - half < *x_l) {
			*x_l = c - half;
		}
	}
}

/* Is there a run of "obstacle" indices (stay rule) starting at i that contains
   at least one real hit? Returns its last index in *last, or -1. */
static int run_at(const OBS_SCAN *s, const OBS_CFG *cfg, int i, int *last)
{
	int j = i, hit = 0;

	while (j < OBS_NUM_ANGLES && is_stay(s, (unsigned)j, cfg) &&
	       (j == i || same_surface(s, cfg, j, j - 1))) {
		hit |= obs_scan_is_hit(s, (unsigned)j, cfg);
		j++;
	}
	*last = j - 1;
	return hit;
}

int obs_next_fine_idx(const OBS_SCAN *s, const OBS_CFG *cfg, int lo, int hi, int from)
{
	int i = 0, last, best = -1, cand[2], k, n;

	if (lo < 0) {
		lo = 0;
	}
	if (hi > OBS_NUM_ANGLES - 1) {
		hi = OBS_NUM_ANGLES - 1;
	}
	while (i < OBS_NUM_ANGLES) {
		if (!is_stay(s, (unsigned)i, cfg)) {
			i++;
			continue;
		}
		if (run_at(s, cfg, i, &last)) {
			cand[0] = i - 1;
			cand[1] = last + 1;
			for (k = 0; k < 2; k++) {
				n = cand[k];
				if (n < lo || n > hi || obs_scan_measured(s, (unsigned)n)) {
					continue;
				}
				if (best < 0 ||
				    (n > from ? n - from : from - n) < (best > from ? best - from : from - best) ||
				    ((n > from ? n - from : from - n) == (best > from ? best - from : from - best) && n < best)) {
					best = n;
				}
			}
		}
		i = last + 1;
	}
	return best;
}

static int fine_is_hit(const OBS_SCAN *s, const OBS_CFG *cfg, unsigned deg, uint16_t cm, uint16_t ref)
{
	unsigned idx = (deg + OBS_STEP_DEG / 2) / OBS_STEP_DEG;
	int d = (int)cm - (int)ref;

	if (cm == OBS_DIST_NONE || !obs_dist_is_hit(s, idx, cm, cfg)) {
		return 0;
	}
	return (d < 0 ? -d : d) <= (int)cfg->cluster_jump_cm;	/* the same surface, not a wall behind */
}

/* One edge. dir = -1: the right edge (angles fall), +1: the left edge (angles rise). */
static unsigned fine_one_edge(OBS_SCAN *s, const OBS_CFG *cfg, int edge_idx, int dir,
			      unsigned min_deg, unsigned max_deg, OBS_READ_FN rd, void *ctx)
{
	int k, misses = 0, hit_deg = 0;
	unsigned n = 0;
	uint16_t ref = s->dist_cm[edge_idx], hit_cm = ref;
	int next = edge_idx + dir;

	/* the coarse neighbour must be measured and a miss (otherwise the edge is not
	   resolved, or lies outside the servo window): nothing to refine */
	if (next < 0 || next >= OBS_NUM_ANGLES || !obs_scan_measured(s, (unsigned)next)) {
		return 0;
	}
	for (k = 1; k < OBS_STEP_DEG / OBS_FINE_STEP_DEG; k++) {
		int deg = edge_idx * OBS_STEP_DEG + dir * k * OBS_FINE_STEP_DEG;
		uint16_t cm;

		if (deg < (int)min_deg || deg > (int)max_deg) {
			break;
		}
		cm = rd(ctx, (unsigned)deg);
		n++;
		if (fine_is_hit(s, cfg, (unsigned)deg, cm, hit_cm)) {
			hit_deg = deg;
			hit_cm = cm;
			misses = 0;
		} else {
			misses++;
			if (misses >= 2) {
				break;	/* a single dropped echo is forgiven, two in a row end the edge */
			}
		}
	}
	if (hit_deg != 0) {
		unsigned e = (dir < 0) ? 0u : 1u;

		s->fine_deg[e] = (uint8_t)hit_deg;
		s->fine_cm[e] = hit_cm;
	}
	return n;
}

unsigned obs_fine_edges(OBS_SCAN *s, const OBS_CFG *cfg, unsigned min_deg, unsigned max_deg,
			OBS_READ_FN rd, void *ctx)
{
	int lo, hi, closest;
	unsigned n = 0;

	s->fine_deg[0] = 0;
	s->fine_deg[1] = 0;
	if (!find_cluster(s, cfg, &lo, &hi, &closest)) {
		return 0;
	}
	n += fine_one_edge(s, cfg, lo, -1, min_deg, max_deg, rd, ctx);
	n += fine_one_edge(s, cfg, hi, +1, min_deg, max_deg, rd, ctx);
	return n;
}

void obs_profile(const OBS_SCAN *s, const OBS_CFG *cfg, OBS_PROFILE *out)
{
	int lo, hi, closest, i, last, unres;
	int32_t x_r, x_l, w, x_ro, x_lo;
	int32_t shift_r, shift_l, m, clear;
	int32_t oxr[OBS_NUM_ANGLES], oxl[OBS_NUM_ANGLES];	/* the OTHER obstacles, planned extents */
	int no = 0, k;

	out->detected = 0;
	out->closest_idx = 0;
	out->closest_cm = 0;
	out->width_cm = 0;
	out->left_cm = 0;
	out->right_cm = 0;
	out->action = OBS_ACT_CONTINUE;
	out->partial = 0;
	out->idx_right = 0;
	out->idx_left = 0;
	out->plan_width_cm = 0;
	out->unc_cm = 0;
	out->clusters = 0;

	if (!find_cluster(s, cfg, &lo, &hi, &closest)) {
		return;
	}

	out->detected = 1;
	out->closest_idx = (uint8_t)closest;
	out->closest_cm = s->dist_cm[closest];
	out->idx_right = (uint8_t)lo;
	out->idx_left = (uint8_t)hi;

	cluster_geom(s, cfg, lo, hi, closest, &x_r, &x_l, &w, &unres, &x_ro, &x_lo, 1);
	out->partial = (uint8_t)unres;
	out->width_cm = cap_cm(w);
	out->clusters = 1;

	/* Each edge is only known to +/- half a 15 degree step, i.e. about
	   0.131 * range. Report it; do not hide it. */
	out->unc_cm = (hi == lo) ? cap_cm((int32_t)s->dist_cm[lo] * 263 / 1000)
				 : cap_cm(((int32_t)s->dist_cm[lo] + s->dist_cm[hi]) * 131 / 1000);

	x_r = x_ro;
	x_l = x_lo;
	if (s->fine_deg[0] == 0 || s->fine_deg[1] == 0) {
		plan_span(cfg, &x_r, &x_l);	/* both edges known to a few degrees: no padding up to the widest spec box */
	}
	out->plan_width_cm = cap_cm(x_r - x_l);

	/* every OTHER obstacle in the scan (a second box beside the first) */
	for (i = 0; i < OBS_NUM_ANGLES; ) {
		if (!is_stay(s, (unsigned)i, cfg)) {
			i++;
			continue;
		}
		if (run_at(s, cfg, i, &last)) {
			if (i != lo) {
				int32_t a, b, ww, ao, bo;
				int u, cl = i, kk;

				for (kk = i; kk <= last; kk++) {	/* its closest index */
					if (s->dist_cm[kk] < s->dist_cm[cl]) {
						cl = kk;
					}
				}
				cluster_geom(s, cfg, i, last, cl, &a, &b, &ww, &u, &ao, &bo, 0);
				a = ao;
				b = bo;
				plan_span(cfg, &a, &b);
				oxr[no] = a;
				oxl[no] = b;
				no++;
				out->clusters++;
				out->partial |= (uint8_t)u;
			}
		}
		i = last + 1;
	}

	/* sideways move to clear the main obstacle on each side */
	clear = (int32_t)cfg->robot_half_width_cm + cfg->margin_cm;
	shift_r = x_r + clear;
	shift_l = -x_l + clear;
	if (shift_r < 0) {
		shift_r = 0;	/* the lane is already clear on that side: no move, but the lane itself */
	}
	if (shift_l < 0) {
		shift_l = 0;	/* must still be free of the other obstacles */
	}

	/* ...but a side is only possible if the corridor the robot would then
	   occupy, [c - clear, c + clear], does not touch any other obstacle */
	for (k = 0; k < no; k++) {
		if (oxr[k] > shift_r - clear && oxl[k] < shift_r + clear) {
			shift_r = OBS_DIST_CAP;
		}
		if (oxr[k] > -shift_l - clear && oxl[k] < -shift_l + clear) {
			shift_l = OBS_DIST_CAP;
		}
	}
	out->right_cm = cap_cm(shift_r);
	out->left_cm = cap_cm(shift_l);

	if (out->closest_cm < cfg->stop_cm) {
		out->action = OBS_ACT_STOP;
		return;
	}
	m = (out->right_cm <= out->left_cm) ? out->right_cm : out->left_cm;
	if (m > (int32_t)cfg->max_shift_cm) {
		out->action = OBS_ACT_REVERSE_RETRY;
	} else if (out->right_cm <= out->left_cm) {
		out->action = OBS_ACT_TURN_RIGHT;
	} else {
		out->action = OBS_ACT_TURN_LEFT;
	}
}

uint16_t obs_echo_us_to_cm(uint32_t echo_us)
{
	uint32_t cm = (echo_us + 29u) / 58u;	/* sound: 58 us per cm, round trip */

	return (cm > 0xFFFFu) ? 0xFFFFu : (uint16_t)cm;
}

uint32_t obs_angle_to_pulse_us(unsigned deg)
{
	/* SG90: about 0.5 ms (0 deg) to 2.4 ms (180 deg). Start conservative at
	   0.6 to 2.4 ms and calibrate on the bench so the servo never hits its stops. */
	if (deg > 180u) {
		deg = 180u;
	}
	return 600u + deg * 10u;
}

void obs_pack(const OBS_PROFILE *p, uint8_t seq, uint32_t w[OBS_MSG_WORDS])
{
	w[0] = ((uint32_t)(p->detected & 1u))
	     | ((uint32_t)(p->closest_idx & 0xFu) << 1)
	     | ((uint32_t)(p->closest_cm & 0x1FFu) << 5)
	     | ((uint32_t)(p->width_cm & 0x1FFu) << 14)
	     | ((uint32_t)(p->action & 0x7u) << 23)
	     | ((uint32_t)(p->partial & 1u) << 26)
	     | ((uint32_t)(seq & 0xFu) << 27);
	w[1] = ((uint32_t)(p->left_cm & 0x1FFu))
	     | ((uint32_t)(p->right_cm & 0x1FFu) << 9);
}

void obs_unpack(const uint32_t w[OBS_MSG_WORDS], OBS_PROFILE *p, uint8_t *seq)
{
	p->detected    = (uint8_t)(w[0] & 1u);
	p->closest_idx = (uint8_t)((w[0] >> 1) & 0xFu);
	p->closest_cm  = (uint16_t)((w[0] >> 5) & 0x1FFu);
	p->width_cm    = (uint16_t)((w[0] >> 14) & 0x1FFu);
	p->action      = (uint8_t)((w[0] >> 23) & 0x7u);
	p->partial     = (uint8_t)((w[0] >> 26) & 1u);
	if (seq) {
		*seq = (uint8_t)((w[0] >> 27) & 0xFu);
	}
	p->left_cm     = (uint16_t)(w[1] & 0x1FFu);
	p->right_cm    = (uint16_t)((w[1] >> 9) & 0x1FFu);
	p->idx_right   = 0;
	p->idx_left    = 0;
	p->plan_width_cm = 0;
	p->unc_cm      = 0;
}

int obs_approach_mm(const OBS_PROFILE *p, const OBS_CFG *cfg)
{
	int d = (int)p->closest_idx - OBS_IDX_AHEAD;
	int ahead_mm, travel_mm, back_max_mm;

	if (!p->detected ||
	    (p->action != OBS_ACT_TURN_RIGHT && p->action != OBS_ACT_TURN_LEFT &&
	     p->action != OBS_ACT_STOP && p->action != OBS_ACT_REVERSE_RETRY)) {
		return 0;
	}
	if (p->closest_cm < cfg->approach_min_cm) {
		return 0;			/* reading too close to trust: fault, do not move */
	}
	if (d < 0) {
		d = -d;
	}
	if (d > 6) {
		d = 6;
	}
	/* the closest point is d x 15 degrees off straight ahead; the table step is
	   7.5 degrees, so cos of that angle is entry 2 x d. Its cosine is the share
	   of the beam that points straight ahead. */
	ahead_mm  = (int)(((int32_t)p->closest_cm * 10 * cos_tab[2 * d]) / 1000);
	travel_mm = ahead_mm - (int)cfg->approach_cm * 10;
	if (ahead_mm < (int)cfg->approach_near_cm * 10) {
		if (p->action == OBS_ACT_REVERSE_RETRY) {
			return 0;		/* the retry itself backs away */
		}
		/* too close: back away, but only a short way (nothing watches behind) */
		back_max_mm = (int)cfg->approach_back_max_cm * 10;
		travel_mm = (-travel_mm > back_max_mm) ? -back_max_mm : travel_mm;
		return (travel_mm > -30) ? 0 : travel_mm;
	}
	if (p->action == OBS_ACT_STOP) {
		return 0;			/* STOP only ever backs away */
	}
	if (ahead_mm <= (int)cfg->approach_far_cm * 10 || travel_mm <= 0) {
		return 0;
	}
	return (travel_mm > 1023) ? 1023 : travel_mm;
}

void obs_bg_forget(OBS_SCAN *s)
{
	s->bg_known = 0;
}
