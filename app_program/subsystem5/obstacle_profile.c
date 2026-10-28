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
}

void obs_scan_init(OBS_SCAN *s)
{
	unsigned i;

	for (i = 0; i < OBS_NUM_ANGLES; i++) {
		s->dist_cm[i] = OBS_DIST_NONE;
	}
	s->measured = 0;
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

int obs_scan_is_hit(const OBS_SCAN *s, unsigned idx, const OBS_CFG *cfg)
{
	return obs_scan_measured(s, idx) &&
	       s->dist_cm[idx] != OBS_DIST_NONE &&
	       s->dist_cm[idx] <= cfg->detect_cm;
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
	while (*lo > 0 && obs_scan_is_hit(s, (unsigned)(*lo - 1), cfg)) {
		(*lo)--;
	}
	while (*hi < OBS_NUM_ANGLES - 1 && obs_scan_is_hit(s, (unsigned)(*hi + 1), cfg)) {
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

void obs_profile(const OBS_SCAN *s, const OBS_CFG *cfg, OBS_PROFILE *out)
{
	int lo, hi, closest;
	int32_t r_r, r_l, x_r, x_l, chord2, w;
	int th_r, th_l, h;
	int32_t shift_r, shift_l, m;

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

	if (!find_cluster(s, cfg, &lo, &hi, &closest)) {
		return;
	}

	out->detected = 1;
	out->closest_idx = (uint8_t)closest;
	out->closest_cm = s->dist_cm[closest];
	out->idx_right = (uint8_t)lo;
	out->idx_left = (uint8_t)hi;
	out->partial = (uint8_t)(edge_unresolved(s, lo, -1) || edge_unresolved(s, hi, +1));

	r_r = s->dist_cm[lo];
	r_l = s->dist_cm[hi];

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
		w = (int32_t)s->dist_cm[closest] * 263 / 1000;
		x_r = (int32_t)s->dist_cm[closest] * cos_tab[2 * closest] / 1000 + w / 2;
		x_l = (int32_t)s->dist_cm[closest] * cos_tab[2 * closest] / 1000 - w / 2;
		out->partial = 1;
	} else {
		/* chord between the two edge points (law of cosines) */
		chord2 = r_r * r_r + r_l * r_l - 2 * r_r * r_l * cos_tab[th_l - th_r] / 1000;
		if (chord2 < 0) {
			chord2 = 0;
		}
		w = (int32_t)obs_isqrt((uint32_t)chord2);
		x_r = r_r * cos_tab[th_r] / 1000;	/* + = right of the centre line */
		x_l = r_l * cos_tab[th_l] / 1000;	/* - = left of the centre line */
	}
	out->width_cm = cap_cm(w);

	/* sideways move to clear the obstacle on each side */
	shift_r = x_r + cfg->robot_half_width_cm + cfg->margin_cm;
	shift_l = -x_l + cfg->robot_half_width_cm + cfg->margin_cm;
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
}
