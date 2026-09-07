#include "cam.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "xalloc.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define UNDERCUT_SAMPLES 720
#define TWO_PI (2.0 * M_PI)
#define SAMPLE_STEP (TWO_PI / (double)CAM_PROFILE_SAMPLES)

/* Keeps the cached base_radius/lift in step with the profile table. Both are
 * only ever derived from it, never set independently. */
static void refresh_bounds(Cam *c) {
    double lo = c->pitch_r[0], hi = c->pitch_r[0];
    for (int i = 1; i < CAM_PROFILE_SAMPLES; i++) {
        if (c->pitch_r[i] < lo) lo = c->pitch_r[i];
        if (c->pitch_r[i] > hi) hi = c->pitch_r[i];
    }
    c->base_radius = lo;
    c->lift = hi - lo;
}

/* The cycloidal displacement fraction and its derivative over t in [0,1]:
 * s(t) = t - sin(2*pi*t)/(2*pi), s'(t) = 1 - cos(2*pi*t). Zero slope at both
 * ends, which is what keeps segment joins smooth. */
static double cycloidal(double t) {
    return t - sin(TWO_PI * t) / TWO_PI;
}

void cam_fill_motion_law(Cam *c, double base_radius, double lift,
                          double rise_deg, double high_dwell_deg, double fall_deg) {
    double a = fmax(0.0, rise_deg) * M_PI / 180.0;
    double b = fmax(0.0, high_dwell_deg) * M_PI / 180.0;
    double g = fmax(0.0, fall_deg) * M_PI / 180.0;
    double total = a + b + g;
    if (total > TWO_PI && total > 0.0) {
        double scale = TWO_PI / total;
        a *= scale; b *= scale; g *= scale;
    }

    for (int i = 0; i < CAM_PROFILE_SAMPLES; i++) {
        double phi = SAMPLE_STEP * (double)i;
        double r;
        if (a > 0.0 && phi < a) {
            r = base_radius + lift * cycloidal(phi / a);
        } else if (phi < a + b) {
            r = base_radius + lift;
        } else if (g > 0.0 && phi < a + b + g) {
            r = base_radius + lift * (1.0 - cycloidal((phi - a - b) / g));
        } else {
            r = base_radius;
        }
        c->pitch_r[i] = r;
    }
    refresh_bounds(c);
}

void cam_set_defaults(Cam *c, double base_radius) {
    c->body_link_id = -1;
    c->center_connector_id = -1;
    c->follower_connector_id = -1;
    c->roller_radius = fmax(2.0, base_radius * 0.12);
    /* Stiff enough that the follower tracks the profile at ordinary motor
     * speeds, but soft enough that a fast fall throws it off -- which is the
     * whole point of modelling contact as one-sided. */
    c->spring_k = 1200.0;
    c->spring_preload = base_radius * 0.25;
    c->axis_origin = (Vec2){ 0.0, 0.0 };
    c->axis_dir = (Vec2){ 1.0, 0.0 };
    c->ref_connector_id = -1;
    c->frozen_ref_angle = 0.0;
    c->last_angle = 0.0;
    c->in_contact = false;
    c->selected = false;
    c->alive = true;
    cam_fill_motion_law(c, base_radius, base_radius * 0.4, 90.0, 60.0, 90.0);
}

/* ---------------------------------------------------------------------------
 * Profile lookup
 *
 * The table is interpolated with a periodic Catmull-Rom spline rather than
 * linearly: a linear fit would give a piecewise-constant derivative, and the
 * derivative is what sets both the surface normal (so the physical offset)
 * and the contact velocity. Catmull-Rom is C1, so both come out smooth.
 * ------------------------------------------------------------------------ */

static double wrap_two_pi(double phi) {
    double t = fmod(phi, TWO_PI);
    if (t < 0.0) t += TWO_PI;
    return t;
}

static void spline_coeffs(const Cam *c, double phi, double *p0, double *p1,
                           double *p2, double *p3, double *t) {
    double u = wrap_two_pi(phi) / SAMPLE_STEP;
    int i = (int)floor(u);
    *t = u - (double)i;
    int n = CAM_PROFILE_SAMPLES;
    *p0 = c->pitch_r[((i - 1) % n + n) % n];
    *p1 = c->pitch_r[(i % n + n) % n];
    *p2 = c->pitch_r[((i + 1) % n + n) % n];
    *p3 = c->pitch_r[((i + 2) % n + n) % n];
}

double cam_pitch_radius(const Cam *c, double phi) {
    double p0, p1, p2, p3, t;
    spline_coeffs(c, phi, &p0, &p1, &p2, &p3, &t);
    return 0.5 * ((2.0 * p1) + (-p0 + p2) * t +
                   (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t * t +
                   (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t * t * t);
}

double cam_pitch_radius_deriv(const Cam *c, double phi) {
    double p0, p1, p2, p3, t;
    spline_coeffs(c, phi, &p0, &p1, &p2, &p3, &t);
    double d_dt = 0.5 * ((-p0 + p2) +
                          2.0 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t +
                          3.0 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t * t);
    return d_dt / SAMPLE_STEP;
}

void cam_scale_lift(Cam *c, double factor) {
    double base = c->base_radius;
    for (int i = 0; i < CAM_PROFILE_SAMPLES; i++) {
        c->pitch_r[i] = base + (c->pitch_r[i] - base) * factor;
    }
    refresh_bounds(c);
}

void cam_rotate_profile(Cam *c, double delta_rad) {
    double shifted[CAM_PROFILE_SAMPLES];
    for (int i = 0; i < CAM_PROFILE_SAMPLES; i++) {
        shifted[i] = cam_pitch_radius(c, SAMPLE_STEP * (double)i - delta_rad);
    }
    memcpy(c->pitch_r, shifted, sizeof shifted);
    refresh_bounds(c);
}

/* ---------------------------------------------------------------------------
 * Surface
 * ------------------------------------------------------------------------ */

/* The outward unit normal of the pitch curve at `phi`. For a polar curve
 * (r cos phi, r sin phi) the tangent is its derivative; turning that -90
 * degrees points outward (for a circle it reduces to (cos phi, sin phi)). */
static Vec2 outward_normal(const Cam *c, double phi) {
    double r = cam_pitch_radius(c, phi);
    double rp = cam_pitch_radius_deriv(c, phi);
    double cs = cos(phi), sn = sin(phi);
    Vec2 tangent = { rp * cs - r * sn, rp * sn + r * cs };
    Vec2 normal = { tangent.y, -tangent.x };
    double len = vec2_len(normal);
    if (len > 1e-12) return vec2_scale(normal, 1.0 / len);
    return (Vec2){ cs, sn };
}

void cam_sample_surface(const Cam *c, Vec2 *out, int count) {
    if (count < 3) return;
    for (int i = 0; i < count; i++) {
        double phi = TWO_PI * (double)i / (double)count;
        double r = cam_pitch_radius(c, phi);
        Vec2 pitch = { r * cos(phi), r * sin(phi) };
        out[i] = vec2_sub(pitch, vec2_scale(outward_normal(c, phi), c->roller_radius));
    }
}

bool cam_is_undercut(const Cam *c) {
    if (c->roller_radius <= 0.0) return false;

    Vec2 *pts = xmalloc((size_t)UNDERCUT_SAMPLES * sizeof(Vec2));
    if (!pts) return false;
    cam_sample_surface(c, pts, UNDERCUT_SAMPLES);

    bool undercut = false;
    for (int i = 0; i < UNDERCUT_SAMPLES && !undercut; i++) {
        /* The surface curling inside the centre is undercutting in the
         * extreme; catch it before the direction test. */
        if (vec2_len(pts[i]) <= 1e-9) undercut = true;
    }

    /* Where the offset folds, the surface doubles back: consecutive edges
     * point in opposing directions. On a well-formed cam every edge turns
     * only gradually from the last. */
    for (int i = 0; i < UNDERCUT_SAMPLES && !undercut; i++) {
        Vec2 a = pts[i];
        Vec2 b = pts[(i + 1) % UNDERCUT_SAMPLES];
        Vec2 d = pts[(i + 2) % UNDERCUT_SAMPLES];
        Vec2 e0 = vec2_sub(b, a);
        Vec2 e1 = vec2_sub(d, b);
        double l0 = vec2_len(e0), l1 = vec2_len(e1);
        if (l0 < 1e-12 || l1 < 1e-12) continue;
        if (vec2_dot(e0, e1) / (l0 * l1) < 0.0) undercut = true;
    }

    free(pts);
    return undercut;
}

/* ---------------------------------------------------------------------------
 * Reading a profile off a drawn outline
 * ------------------------------------------------------------------------ */

/* Farthest crossing of the ray from the origin along `dir` with the closed
 * polyline `pts`, or -1 if it misses. Taking the FARTHEST crossing is the
 * forgiving choice: a hand-drawn stroke that doubles back on itself still
 * yields the outermost silhouette rather than an inner fold. */
static double ray_polygon_radius(const Vec2 *pts, int n, Vec2 dir) {
    double best = -1.0;
    for (int k = 0; k < n; k++) {
        Vec2 a = pts[k];
        Vec2 b = pts[(k + 1) % n];
        Vec2 e = vec2_sub(b, a);
        /* Solve a + u*e = t*dir for u in [0,1], t >= 0. */
        double denom = dir.x * e.y - dir.y * e.x;
        if (fabs(denom) < 1e-12) continue;
        double u = (dir.x * a.y - dir.y * a.x) / denom;
        if (u < 0.0 || u > 1.0) continue;
        Vec2 hit = vec2_add(a, vec2_scale(e, u));
        double t = vec2_dot(hit, dir);
        if (t > 0.0 && t > best) best = t;
    }
    return best;
}

/* Fills gaps left where the ray missed the outline (an open or ragged
 * stroke), by interpolating circularly between the nearest samples that did
 * hit. */
static bool fill_gaps(double *r) {
    int first = -1;
    for (int i = 0; i < CAM_PROFILE_SAMPLES; i++) {
        if (r[i] > 0.0) { first = i; break; }
    }
    if (first < 0) return false; /* the ray missed everywhere: unusable */

    for (int step = 0; step < CAM_PROFILE_SAMPLES; step++) {
        int i = (first + step) % CAM_PROFILE_SAMPLES;
        if (r[i] > 0.0) continue;
        /* Walk out to the nearest valid sample each way and blend. */
        int back = 0, fwd = 0;
        while (r[((i - back - 1) % CAM_PROFILE_SAMPLES + CAM_PROFILE_SAMPLES) % CAM_PROFILE_SAMPLES] <= 0.0 &&
               back < CAM_PROFILE_SAMPLES) back++;
        while (r[(i + fwd + 1) % CAM_PROFILE_SAMPLES] <= 0.0 && fwd < CAM_PROFILE_SAMPLES) fwd++;
        double lo = r[((i - back - 1) % CAM_PROFILE_SAMPLES + CAM_PROFILE_SAMPLES) % CAM_PROFILE_SAMPLES];
        double hi = r[(i + fwd + 1) % CAM_PROFILE_SAMPLES];
        double w = (double)(back + 1) / (double)(back + fwd + 2);
        r[i] = lo * (1.0 - w) + hi * w;
    }
    return true;
}

/* A few passes of a 3-tap circular average: a freehand stroke is jittery at
 * the pixel level, and that jitter would otherwise become a spiky profile
 * with a wildly swinging derivative. */
static void smooth_ring(double *r, int passes) {
    double tmp[CAM_PROFILE_SAMPLES];
    for (int p = 0; p < passes; p++) {
        for (int i = 0; i < CAM_PROFILE_SAMPLES; i++) {
            double a = r[(i - 1 + CAM_PROFILE_SAMPLES) % CAM_PROFILE_SAMPLES];
            double b = r[i];
            double c = r[(i + 1) % CAM_PROFILE_SAMPLES];
            tmp[i] = 0.25 * a + 0.5 * b + 0.25 * c;
        }
        memcpy(r, tmp, sizeof tmp);
    }
}

/* The roller rides ON the drawn surface, so the pitch curve -- the path of
 * the roller's centre -- is that surface offset OUTWARD by the roller radius.
 * Offsetting along the surface normal, rather than just adding the radius to
 * each sample, is what keeps the follower exactly on the drawn shape where
 * the flanks are steep. Writes the resampled pitch table; false if the offset
 * came out unusable. */
static bool pitch_from_surface(const Cam *shape, const double *surface,
                                double roller, double *pitch_out) {
    Cam probe = *shape;
    memcpy(probe.pitch_r, surface, CAM_PROFILE_SAMPLES * sizeof(double));
    refresh_bounds(&probe);

    Vec2 offset[CAM_PROFILE_SAMPLES];
    for (int i = 0; i < CAM_PROFILE_SAMPLES; i++) {
        double phi = SAMPLE_STEP * (double)i;
        double r = cam_pitch_radius(&probe, phi);
        Vec2 p = { r * cos(phi), r * sin(phi) };
        offset[i] = vec2_add(p, vec2_scale(outward_normal(&probe, phi), roller));
    }

    /* The offset points no longer sit at even angles, so re-sample radially
     * back onto the table. */
    for (int i = 0; i < CAM_PROFILE_SAMPLES; i++) {
        double phi = SAMPLE_STEP * (double)i;
        pitch_out[i] = ray_polygon_radius(offset, CAM_PROFILE_SAMPLES, (Vec2){ cos(phi), sin(phi) });
    }
    if (!fill_gaps(pitch_out)) return false;

    /* Belt and braces: where the offset self-intersects, a stray crossing can
     * hand back a radius nothing like the drawn shape. Bound the result to
     * what the drawing could plausibly produce so one bad sample can never
     * fling the follower across the canvas. */
    double lo = surface[0], hi = surface[0];
    for (int i = 1; i < CAM_PROFILE_SAMPLES; i++) {
        if (surface[i] < lo) lo = surface[i];
        if (surface[i] > hi) hi = surface[i];
    }
    for (int i = 0; i < CAM_PROFILE_SAMPLES; i++) {
        if (!(pitch_out[i] > 0.0)) return false;
        if (pitch_out[i] < lo) pitch_out[i] = lo;
        if (pitch_out[i] > hi + roller * 1.5) pitch_out[i] = hi + roller * 1.5;
    }
    return true;
}

bool cam_set_from_drawn_outline(Cam *c, const Vec2 *pts, int count) {
    if (count < 3) return false;

    /* Read the drawn SURFACE as a radius per angle. */
    double surface[CAM_PROFILE_SAMPLES];
    for (int i = 0; i < CAM_PROFILE_SAMPLES; i++) {
        double phi = SAMPLE_STEP * (double)i;
        surface[i] = ray_polygon_radius(pts, count, (Vec2){ cos(phi), sin(phi) });
    }
    if (!fill_gaps(surface)) return false;
    smooth_ring(surface, 3);

    for (int i = 0; i < CAM_PROFILE_SAMPLES; i++) {
        if (!(surface[i] > 1.0)) return false; /* degenerate or inside-out */
    }

    /* A hand-drawn outline will often have a concave stretch tighter than the
     * roller that was guessed for it, which would undercut: the roller simply
     * cannot reach into the notch, and forcing it produces a profile that is
     * nothing like the drawing. Cam design solves this by choosing a smaller
     * roller, so that is what happens here -- shrink until it fits the shape
     * that was actually drawn. */
    double pitch[CAM_PROFILE_SAMPLES];
    double roller = c->roller_radius;
    bool have_fit = false;

    for (int attempt = 0; attempt < 10 && roller > 0.5; attempt++) {
        if (pitch_from_surface(c, surface, roller, pitch)) {
            Cam probe = *c;
            probe.roller_radius = roller;
            memcpy(probe.pitch_r, pitch, sizeof pitch);
            refresh_bounds(&probe);
            if (!cam_is_undercut(&probe)) { have_fit = true; break; }
        }
        roller *= 0.65;
    }

    if (!have_fit) {
        /* A genuinely sharp notch: keep the smallest roller we tried and let
         * the caller's undercut warning speak for itself. */
        roller = fmax(roller, 0.5);
        if (!pitch_from_surface(c, surface, roller, pitch)) return false;
    }

    c->roller_radius = roller;
    memcpy(c->pitch_r, pitch, sizeof pitch);
    refresh_bounds(c);
    return true;
}
