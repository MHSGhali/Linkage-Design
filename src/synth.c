#include "synth.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* An open stroke read as closed if its ends are within this fraction of the
 * stroke's overall size. */
#define CLOSE_FRACTION 0.22
/* How finely a candidate four-bar's coupler curve is sampled while searching.
 * Enough to catch the shape, few enough that tens of thousands of candidates
 * can be scored in a few seconds. */
#define CURVE_SAMPLES 120

/* ---------------------------------------------------------------------------
 * Stroke preparation
 * ------------------------------------------------------------------------ */

static Vec2 centroid_of(const Vec2 *pts, int n) {
    Vec2 c = { 0.0, 0.0 };
    for (int i = 0; i < n; i++) c = vec2_add(c, pts[i]);
    return vec2_scale(c, 1.0 / (double)n);
}

double synth_path_size(const Vec2 *pts, int count) {
    if (count < 1) return 0.0;
    Vec2 lo = pts[0], hi = pts[0];
    for (int i = 1; i < count; i++) {
        lo.x = fmin(lo.x, pts[i].x); lo.y = fmin(lo.y, pts[i].y);
        hi.x = fmax(hi.x, pts[i].x); hi.y = fmax(hi.y, pts[i].y);
    }
    return fmax(hi.x - lo.x, hi.y - lo.y);
}

bool synth_stroke_is_closed(const Vec2 *pts, int count) {
    if (count < 3) return false;
    Vec2 centre = centroid_of(pts, count);
    double span = 0.0;
    for (int i = 0; i < count; i++) span = fmax(span, vec2_dist(pts[i], centre));
    if (span < 1e-9) return false;
    return vec2_dist(pts[0], pts[count - 1]) < CLOSE_FRACTION * span;
}

void synth_resample(const Vec2 *pts, int count, bool closed, Vec2 *out, int out_count) {
    if (count < 2 || out_count < 1) return;

    int segments = closed ? count : count - 1;
    double total = 0.0;
    for (int i = 0; i < segments; i++) total += vec2_dist(pts[i], pts[(i + 1) % count]);
    if (total < 1e-9) {
        for (int i = 0; i < out_count; i++) out[i] = pts[0];
        return;
    }

    /* Walk the polyline at even arc-length intervals. */
    double spacing = total / (double)(closed ? out_count : (out_count - 1));
    int seg = 0;
    double seg_start = 0.0;
    double seg_len = vec2_dist(pts[0], pts[1 % count]);

    for (int i = 0; i < out_count; i++) {
        double want = spacing * (double)i;
        if (want > total) want = total;
        while (seg < segments - 1 && want > seg_start + seg_len) {
            seg_start += seg_len;
            seg++;
            seg_len = vec2_dist(pts[seg], pts[(seg + 1) % count]);
        }
        double t = (seg_len > 1e-12) ? (want - seg_start) / seg_len : 0.0;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        out[i] = vec2_add(pts[seg], vec2_scale(vec2_sub(pts[(seg + 1) % count], pts[seg]), t));
    }
}

/* ---------------------------------------------------------------------------
 * Fourier decomposition
 * ------------------------------------------------------------------------ */

Vec2 synth_fourier_point(Vec2 anchor, const FourierArm *arms, int count, double t) {
    Vec2 p = anchor;
    for (int i = 0; i < count; i++) {
        double angle = 2.0 * M_PI * (double)arms[i].harmonic * t;
        p = vec2_add(p, vec2_rotate(arms[i].amplitude, angle));
    }
    return p;
}

/* Builds the closed, evenly-sampled point sequence the transform runs on.
 *
 * An open stroke is mirrored -- traced out and then back -- rather than simply
 * joined end to start. Closing it with a straight jump would put a step in
 * the curve, and a step needs endless harmonics to represent; the mirrored
 * version is continuous, so it decomposes cleanly, and the pen sweeping there
 * and back is what an open path should do anyway. */
static void build_cycle(const Vec2 *stroke, int stroke_count, bool closed, Vec2 *out) {
    if (closed) {
        synth_resample(stroke, stroke_count, true, out, SYNTH_PATH_SAMPLES);
        return;
    }

    int half = SYNTH_PATH_SAMPLES / 2 + 1;          /* 129 */
    Vec2 forward[SYNTH_PATH_SAMPLES / 2 + 1];
    synth_resample(stroke, stroke_count, false, forward, half);

    for (int i = 0; i < half; i++) out[i] = forward[i];
    /* ...and back, skipping both endpoints so they aren't duplicated. */
    for (int i = 1; i < half - 1; i++) out[half - 1 + i] = forward[half - 1 - i];
}

typedef struct { int harmonic; Vec2 amplitude; double magnitude; } Coefficient;

static int by_magnitude_desc(const void *a, const void *b) {
    double ma = ((const Coefficient *)a)->magnitude;
    double mb = ((const Coefficient *)b)->magnitude;
    if (ma < mb) return 1;
    if (ma > mb) return -1;
    return 0;
}

int synth_fourier_fit(const Vec2 *stroke, int stroke_count, bool closed,
                       int max_arms, double target_rms,
                       Vec2 *out_anchor, FourierArm *out_arms, double *out_rms) {
    if (stroke_count < 3 || !out_anchor || !out_arms) return 0;
    if (max_arms < 1) return 0;
    if (max_arms > SYNTH_MAX_ARMS) max_arms = SYNTH_MAX_ARMS;

    Vec2 cycle[SYNTH_PATH_SAMPLES];
    build_cycle(stroke, stroke_count, closed, cycle);
    if (synth_path_size(cycle, SYNTH_PATH_SAMPLES) < 1e-6) return 0;

    const int n = SYNTH_PATH_SAMPLES;
    const int half = n / 2;

    /* The discrete transform, treating each point as a complex number. The
     * k = 0 term is the average position -- where the chain is anchored --
     * and every other term is one rotating arm. */
    Coefficient coeffs[SYNTH_PATH_SAMPLES];
    int coeff_count = 0;
    Vec2 anchor = { 0.0, 0.0 };

    for (int k = -half; k <= half; k++) {
        double re = 0.0, im = 0.0;
        for (int j = 0; j < n; j++) {
            double angle = -2.0 * M_PI * (double)k * (double)j / (double)n;
            double c = cos(angle), s = sin(angle);
            re += cycle[j].x * c - cycle[j].y * s;
            im += cycle[j].x * s + cycle[j].y * c;
        }
        Vec2 amp = { re / (double)n, im / (double)n };
        if (k == 0) {
            anchor = amp;
        } else if (coeff_count < SYNTH_PATH_SAMPLES) {
            coeffs[coeff_count].harmonic = k;
            coeffs[coeff_count].amplitude = amp;
            coeffs[coeff_count].magnitude = vec2_len(amp);
            coeff_count++;
        }
    }

    qsort(coeffs, (size_t)coeff_count, sizeof(Coefficient), by_magnitude_desc);

    /* Parseval: the energy left in the terms we drop IS the mean squared
     * deviation, so the error of every truncation is known without having to
     * reconstruct and compare. */
    double total_energy = 0.0;
    for (int i = 0; i < coeff_count; i++) {
        total_energy += coeffs[i].magnitude * coeffs[i].magnitude;
    }

    int used = 0;
    double kept_energy = 0.0;
    double rms = sqrt(total_energy);
    while (used < max_arms && used < coeff_count) {
        if (rms <= target_rms) break;
        kept_energy += coeffs[used].magnitude * coeffs[used].magnitude;
        used++;
        double left = total_energy - kept_energy;
        rms = sqrt(left > 0.0 ? left : 0.0);
    }
    if (used < 1) used = (coeff_count > 0) ? 1 : 0;

    for (int i = 0; i < used; i++) {
        out_arms[i].harmonic = coeffs[i].harmonic;
        out_arms[i].amplitude = coeffs[i].amplitude;
    }
    *out_anchor = anchor;
    if (out_rms) *out_rms = rms;
    return used;
}

/* ---------------------------------------------------------------------------
 * Four-bar kinematics
 * ------------------------------------------------------------------------ */

bool fourbar_pose(const FourBar *fb, double theta, Vec2 *a_out, Vec2 *b_out, Vec2 *point_out) {
    Vec2 a = vec2_add(fb->ground_a, vec2_scale(vec2_from_angle(theta), fb->crank));

    Vec2 d = vec2_sub(fb->ground_b, a);
    double dist = vec2_len(d);
    if (dist < 1e-9) return false;

    /* B is where the coupler circle about A meets the rocker circle about the
     * far ground pivot. If they don't meet, the linkage cannot be put
     * together at this crank angle. */
    double r3 = fb->coupler, r4 = fb->rocker;
    if (dist > r3 + r4) return false;
    if (dist < fabs(r3 - r4)) return false;

    double along = (r3 * r3 - r4 * r4 + dist * dist) / (2.0 * dist);
    double half2 = r3 * r3 - along * along;
    if (half2 < 0.0) return false;

    Vec2 unit = vec2_scale(d, 1.0 / dist);
    Vec2 mid = vec2_add(a, vec2_scale(unit, along));
    Vec2 b = vec2_add(mid, vec2_scale(vec2_perp(unit), (double)fb->branch * sqrt(half2)));

    if (a_out) *a_out = a;
    if (b_out) *b_out = b;

    if (point_out) {
        Vec2 along_ab = vec2_sub(b, a);
        double len = vec2_len(along_ab);
        if (len < 1e-9) return false;
        Vec2 u = vec2_scale(along_ab, 1.0 / len);
        *point_out = vec2_add(a, vec2_add(vec2_scale(u, fb->coupler_u),
                                           vec2_scale(vec2_perp(u), fb->coupler_v)));
    }
    return true;
}

bool fourbar_coupler_point(const FourBar *fb, double theta, Vec2 *out) {
    return fourbar_pose(fb, theta, NULL, NULL, out);
}

bool fourbar_crank_rotates(const FourBar *fb) {
    double ground = vec2_dist(fb->ground_a, fb->ground_b);
    double len[4] = { ground, fb->crank, fb->coupler, fb->rocker };
    for (int i = 0; i < 4; i++) {
        if (!(len[i] > 1e-6)) return false;
    }

    double shortest = len[0], longest = len[0], total = 0.0;
    for (int i = 0; i < 4; i++) {
        if (len[i] < shortest) shortest = len[i];
        if (len[i] > longest) longest = len[i];
        total += len[i];
    }

    /* Grashof: the shortest link turns fully only when s + l <= p + q, and it
     * is the crank that has to be that shortest link for a motor to drive it
     * round. Kept strictly inside the limit so the linkage never runs into a
     * change point, where it could flip branches mid-turn. */
    double margin = 1e-3 * (total > 0.0 ? total : 1.0);
    if (fb->crank > shortest + 1e-9) return false;
    return (shortest + longest) < (total - shortest - longest) - margin;
}

bool fourbar_coupler_curve(const FourBar *fb, Vec2 *out, int count) {
    for (int i = 0; i < count; i++) {
        double theta = 2.0 * M_PI * (double)i / (double)count;
        if (!fourbar_coupler_point(fb, theta, &out[i])) return false;
    }
    return true;
}

/* ---------------------------------------------------------------------------
 * Four-bar fitting
 * ------------------------------------------------------------------------ */

static double point_segment_dist2(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 ab = vec2_sub(b, a);
    double len2 = vec2_dot(ab, ab);
    double t = (len2 > 1e-12) ? vec2_dot(vec2_sub(p, a), ab) / len2 : 0.0;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    return vec2_dist2(p, vec2_add(a, vec2_scale(ab, t)));
}

/* Squared distance from p to the nearest point anywhere on a polyline. Using
 * segments rather than just the sampled vertices means a coarse sampling
 * still measures the true distance to the curve. */
static double dist2_to_polyline(Vec2 p, const Vec2 *poly, int n, bool closed) {
    double best = 1e300;
    int segments = closed ? n : n - 1;
    for (int i = 0; i < segments; i++) {
        double d2 = point_segment_dist2(p, poly[i], poly[(i + 1) % n]);
        if (d2 < best) best = d2;
    }
    return best;
}

static double rms_to_polyline(const Vec2 *pts, int n, const Vec2 *poly, int m, bool poly_closed) {
    double sum = 0.0;
    for (int i = 0; i < n; i++) sum += dist2_to_polyline(pts[i], poly, m, poly_closed);
    return sqrt(sum / (double)n);
}

static double spread_of(const Vec2 *pts, int n, Vec2 centre) {
    double worst = 0.0;
    for (int i = 0; i < n; i++) {
        double d = vec2_dist(pts[i], centre);
        if (d > worst) worst = d;
    }
    return worst;
}

double synth_fit_error(const FourBar *fb, const Vec2 *target, int count, bool closed) {
    if (!fourbar_crank_rotates(fb)) return 1e300;

    Vec2 curve[CURVE_SAMPLES];
    if (!fourbar_coupler_curve(fb, curve, CURVE_SAMPLES)) return 1e300;

    /* Every drawn point must lie on the coupler curve. */
    double forward = rms_to_polyline(target, count, curve, CURVE_SAMPLES, true);

    if (closed) {
        /* ...and, for a loop, the curve must not wander anywhere the drawing
         * doesn't go either, or a huge curve that happens to sweep past the
         * target would score well. */
        double backward = rms_to_polyline(curve, CURVE_SAMPLES, target, count, true);
        return forward + backward;
    }

    /* An open stroke is only part of the coupler curve -- the rest of the
     * loop has to close somewhere, so it can't be penalised for existing.
     * Instead just stop the curve ballooning far larger than the drawing. */
    double target_span = spread_of(target, count, centroid_of(target, count));
    double curve_span = spread_of(curve, CURVE_SAMPLES, centroid_of(curve, CURVE_SAMPLES));
    double excess = curve_span - 2.0 * target_span;
    return forward + (excess > 0.0 ? 0.3 * excess : 0.0);
}

/* Four-bar synthesis is badly multi-modal -- small changes in link lengths
 * swing the coupler curve into an entirely different shape -- so a single
 * local descent lands wherever it started. The approach here is the standard
 * one: draw many candidates from a prior scaled to the drawing, keep the
 * handful that score best, and polish each with a derivative-free pattern
 * search. The cost is piecewise-smooth (a nearest-point distance), which
 * rules out gradient methods but suits pattern search exactly. */

SynthParams synth_default_params(void) {
    SynthParams p;
    /* Tuned to land around six seconds on a modern laptop. Spending the
     * budget on more random starts beats spending it polishing more
     * candidates -- the landscape is rough enough that a mediocre start
     * rarely refines into a good answer. */
    p.random_starts = 300000;
    p.refine_candidates = 64;
    p.refine_sweeps = 200;
    p.seed = 0x9E3779B9u;
    return p;
}

static unsigned next_random(unsigned *state) {
    unsigned x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static double uniform(unsigned *state, double lo, double hi) {
    return lo + (hi - lo) * ((double)(next_random(state) >> 8) / 16777216.0);
}

#define PARAM_COUNT 9

static void fourbar_to_params(const FourBar *fb, double *p) {
    p[0] = fb->ground_a.x; p[1] = fb->ground_a.y;
    p[2] = fb->ground_b.x; p[3] = fb->ground_b.y;
    p[4] = fb->crank;      p[5] = fb->coupler;   p[6] = fb->rocker;
    p[7] = fb->coupler_u;  p[8] = fb->coupler_v;
}

static void params_to_fourbar(const double *p, int branch, FourBar *fb) {
    fb->ground_a = (Vec2){ p[0], p[1] };
    fb->ground_b = (Vec2){ p[2], p[3] };
    fb->crank = p[4]; fb->coupler = p[5]; fb->rocker = p[6];
    fb->coupler_u = p[7]; fb->coupler_v = p[8];
    fb->branch = branch;
}

typedef struct { FourBar fb; double error; } Candidate;

bool synth_fit_four_bar(const Vec2 *target, int count, bool closed,
                         SynthParams params, FourBar *out, double *out_error) {
    if (count < 4 || !out) return false;

    Vec2 centre = centroid_of(target, count);
    double scale = spread_of(target, count, centre);
    if (!(scale > 1e-6)) return false;

    int keep = params.refine_candidates;
    if (keep < 1) keep = 1;
    Candidate *best = malloc((size_t)keep * sizeof(Candidate));
    for (int i = 0; i < keep; i++) best[i].error = 1e300;

    unsigned rng = params.seed ? params.seed : 1u;

    for (int i = 0; i < params.random_starts; i++) {
        FourBar fb;
        fb.ground_a = (Vec2){ centre.x + uniform(&rng, -2.2, 2.2) * scale,
                               centre.y + uniform(&rng, -2.2, 2.2) * scale };
        fb.ground_b = (Vec2){ centre.x + uniform(&rng, -2.2, 2.2) * scale,
                               centre.y + uniform(&rng, -2.2, 2.2) * scale };
        fb.crank = uniform(&rng, 0.05, 1.1) * scale;
        fb.coupler = uniform(&rng, 0.2, 2.6) * scale;
        fb.rocker = uniform(&rng, 0.2, 2.6) * scale;
        fb.coupler_u = uniform(&rng, -1.0, 2.0) * scale;
        fb.coupler_v = uniform(&rng, -2.0, 2.0) * scale;
        fb.branch = (next_random(&rng) & 1u) ? 1 : -1;

        /* Most random draws don't even turn a full revolution; rejecting them
         * before sampling a curve is what makes this many starts affordable. */
        if (!fourbar_crank_rotates(&fb)) continue;

        double err = synth_fit_error(&fb, target, count, closed);
        if (err >= 1e299) continue;

        int worst = 0;
        for (int k = 1; k < keep; k++) {
            if (best[k].error > best[worst].error) worst = k;
        }
        if (err < best[worst].error) {
            best[worst].fb = fb;
            best[worst].error = err;
        }
    }

    /* Polish each survivor: try a step out and back along every parameter,
     * take any improvement, and halve the step once no direction helps. */
    FourBar champion;
    double champion_error = 1e300;
    bool found = false;

    for (int c = 0; c < keep; c++) {
        if (best[c].error >= 1e299) continue;

        double p[PARAM_COUNT];
        fourbar_to_params(&best[c].fb, p);
        int branch = best[c].fb.branch;
        double err = best[c].error;
        double step = 0.25 * scale;

        for (int sweep = 0; sweep < params.refine_sweeps && step > 1e-4 * scale; sweep++) {
            bool improved = false;
            for (int k = 0; k < PARAM_COUNT; k++) {
                for (int dir = -1; dir <= 1; dir += 2) {
                    double saved = p[k];
                    p[k] = saved + dir * step;
                    FourBar trial;
                    params_to_fourbar(p, branch, &trial);
                    double trial_err = synth_fit_error(&trial, target, count, closed);
                    if (trial_err < err) {
                        err = trial_err;
                        improved = true;
                        break;
                    }
                    p[k] = saved;
                }
            }
            if (!improved) step *= 0.5;
        }

        if (err < champion_error) {
            params_to_fourbar(p, branch, &champion);
            champion_error = err;
            found = true;
        }
    }

    free(best);
    if (!found) return false;

    *out = champion;
    if (out_error) *out_error = champion_error;
    return true;
}
