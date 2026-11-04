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
	cfg->approach_cm = 17;		/* aim for the middle of the band */
	cfg->approach_near_cm = 15;	/* wanted gap before the bypass: 15..20 cm; closer: back away */
	cfg->approach_far_cm = 20;	/* farther: creep forward */
	cfg->approach_guard_cm = 13;	/* never closer than this while creeping */
	cfg->approach_back_max_cm = 10;	/* back away at most 10 cm if stopped too close */
}

void obs_scan_clear(OBS_SCAN *s)
{
	unsigned i;

	for (i = 0; i < OBS_NUM_ANGLES; i++) {
		s->dist_cm[i] = OBS_DIST_NONE;
	}
	s->measured = 0;
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

unsigned obs_bg_clear_ahead(OBS_SCAN *s, const OBS_CFG *cfg)
{
	unsigned i, n = 0;

	for (i = OBS_IDX_AHEAD - 1; i <= OBS_IDX_AHEAD + 1; i++) {
		if (bg_present(s, i) && s->bg_cm[i] <= cfg->detect_cm + cfg->detect_hyst_cm) {
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
	while (*lo > 0 && is_stay(s, (unsigned)(*lo - 1), cfg)) {
		(*lo)--;
	}
	while (*hi < OBS_NUM_ANGLES - 1 && is_stay(s, (unsigned)(*hi + 1), cfg)) {
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

static void cluster_geom(const OBS_SCAN *s, const OBS_CFG *cfg, int lo, int hi, int closest,
			 int32_t *x_r, int32_t *x_l, int32_t *w, int *unresolved,
			 int32_t *x_ro, int32_t *x_lo)
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

	while (j < OBS_NUM_ANGLES && is_stay(s, (unsigned)j, cfg)) {
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

	cluster_geom(s, cfg, lo, hi, closest, &x_r, &x_l, &w, &unres, &x_ro, &x_lo);
	out->partial = (uint8_t)unres;
	out->width_cm = cap_cm(w);
	out->clusters = 1;

	/* Each edge is only known to +/- half a 15 degree step, i.e. about
	   0.131 * range. Report it; do not hide it. */
	out->unc_cm = (hi == lo) ? cap_cm((int32_t)s->dist_cm[lo] * 263 / 1000)
				 : cap_cm(((int32_t)s->dist_cm[lo] + s->dist_cm[hi]) * 131 / 1000);

	x_r = x_ro;
	x_l = x_lo;
	plan_span(cfg, &x_r, &x_l);
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
				cluster_geom(s, cfg, i, last, cl, &a, &b, &ww, &u, &ao, &bo);
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
	    (p->action != OBS_ACT_TURN_RIGHT && p->action != OBS_ACT_TURN_LEFT)) {
		return 0;
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
		/* too close: back away, but only a short way (nothing watches behind) */
		back_max_mm = (int)cfg->approach_back_max_cm * 10;
		travel_mm = (-travel_mm > back_max_mm) ? -back_max_mm : travel_mm;
		return (travel_mm > -30) ? 0 : travel_mm;
	}
	if (ahead_mm <= (int)cfg->approach_far_cm * 10 || travel_mm <= 0) {
		return 0;
	}
	return (travel_mm > 1023) ? 1023 : travel_mm;
}
