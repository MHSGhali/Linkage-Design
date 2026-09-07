#include "gearing.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Points spent on the arc across a tooth's tip and on the arc along the root
 * between two teeth. Both are shallow, so a handful reads as round. */
#define TIP_INTERIOR 3
#define ROOT_INTERIOR 4

/* The smallest flat we leave across the top of a tooth, as a multiple of the
 * module. A pinion whose flanks would meet before the addendum circle gets
 * truncated to this rather than printed as a knife edge that snaps off. */
#define MIN_TOP_LAND 0.2

static double involute(double angle) { return tan(angle) - angle; }

static Vec2 polar(double r, double a) { return (Vec2){ r * cos(a), r * sin(a) }; }

static bool spec_ok(const GearSpec *s) {
    return s && s->module > 1e-9 && s->teeth >= GEARING_MIN_TEETH &&
            s->teeth <= GEARING_MAX_TEETH && s->flank_samples >= 2 &&
            s->pressure_angle > 0.01 && s->pressure_angle < 1.5;
}

double gearing_pitch_radius(double module, int teeth) {
    return module * (double)teeth / 2.0;
}

int gearing_teeth_for_radius(double module, double radius) {
    if (!(module > 1e-9)) return GEARING_MIN_TEETH;
    long n = lround(2.0 * radius / module);
    if (n < GEARING_MIN_TEETH) n = GEARING_MIN_TEETH;
    if (n > GEARING_MAX_TEETH) n = GEARING_MAX_TEETH;
    return (int)n;
}

double gearing_center_distance(double module, int teeth_a, int teeth_b) {
    return module * (double)(teeth_a + teeth_b) / 2.0;
}

double gearing_base_radius(const GearSpec *s) {
    return gearing_pitch_radius(s->module, s->teeth) * cos(s->pressure_angle);
}

double gearing_tip_radius(const GearSpec *s) {
    return gearing_pitch_radius(s->module, s->teeth) + s->module * GEARING_ADDENDUM;
}

double gearing_root_radius(const GearSpec *s) {
    double rf = gearing_pitch_radius(s->module, s->teeth) -
                 s->module * (GEARING_ADDENDUM + GEARING_CLEARANCE);
    /* A gear small enough for the root circle to reach the shaft is not a gear;
     * keep it positive so the outline stays a simple closed curve. */
    return (rf > s->module * 0.5) ? rf : s->module * 0.5;
}

/* Half the tooth's angular width at the pitch circle, backlash already taken
 * off. This is the one place the pair's play is decided. */
static double pitch_half_angle(const GearSpec *s) {
    double rp = gearing_pitch_radius(s->module, s->teeth);
    return M_PI / (2.0 * (double)s->teeth) - s->backlash / (2.0 * rp);
}

/* The flank's polar angle at radius `r`, measured from the tooth's centreline.
 * Falls to zero as the flanks converge on the tip, and grows going inwards. */
static double flank_angle(const GearSpec *s, double r) {
    double rb = gearing_base_radius(s);
    double base = pitch_half_angle(s) + involute(s->pressure_angle);
    if (r <= rb) return base;   /* below the base circle there is no involute */
    return base - involute(acos(rb / r));
}

/* The radius in [lo, hi] where flank_angle equals `want`. flank_angle falls
 * monotonically with r, so a plain bisection is exact enough and cannot miss. */
static double radius_at_flank_angle(const GearSpec *s, double want, double lo, double hi) {
    for (int i = 0; i < 60; i++) {
        double mid = 0.5 * (lo + hi);
        if (flank_angle(s, mid) > want) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
}

/* Where the flanks start (at or above the root circle) and stop (at or below
 * the addendum circle), with both degenerate cases squeezed out: a tip that
 * would come to a point, and a root so wide the neighbouring tooth spaces
 * would run into each other. */
static void flank_span(const GearSpec *s, double *r_start_out, double *r_tip_out) {
    double rb = gearing_base_radius(s);
    double rf = gearing_root_radius(s);
    double rp = gearing_pitch_radius(s->module, s->teeth);
    double ra = gearing_tip_radius(s);
    double pitch_angle = 2.0 * M_PI / (double)s->teeth;

    double r_start = (rb > rf) ? rb : rf;
    /* Leave a sliver of root arc between teeth even on a coarse pinion. */
    double max_root_angle = pitch_angle * 0.5 - 0.02;
    if (flank_angle(s, r_start) > max_root_angle) {
        r_start = radius_at_flank_angle(s, max_root_angle, r_start, rp);
    }

    double r_tip = ra;
    double want_land = MIN_TOP_LAND * s->module * 0.5 / ra;   /* half the top land, as an angle */
    if (flank_angle(s, ra) < want_land) {
        r_tip = radius_at_flank_angle(s, want_land, rp, ra);
    }
    if (r_tip < r_start + 1e-9) r_tip = r_start + 1e-9;

    *r_start_out = r_start;
    *r_tip_out = r_tip;
}

int gearing_outline_capacity(const GearSpec *s) {
    if (!spec_ok(s)) return 0;
    return s->teeth * (2 * s->flank_samples + 2 + TIP_INTERIOR + ROOT_INTERIOR);
}

int gearing_tooth_outline(const GearSpec *s, Vec2 *out, int cap) {
    if (!spec_ok(s) || !out) return 0;
    int need = gearing_outline_capacity(s);
    if (cap < need) return 0;

    double rf = gearing_root_radius(s);
    double r_start, r_tip;
    flank_span(s, &r_start, &r_tip);

    /* Below the base circle a real generated flank is undercut away; we draw
     * the radial line a shaper would leave. When the root circle sits ABOVE
     * the base circle (which it does from about 42 teeth up) there is no such
     * step and the involute runs straight into the root arc. */
    bool has_drop = (r_start > rf + 1e-9);
    double root_angle = flank_angle(s, r_start);
    double tip_angle = flank_angle(s, r_tip);
    double pitch_angle = 2.0 * M_PI / (double)s->teeth;

    int n = 0;
    for (int k = 0; k < s->teeth; k++) {
        double base = pitch_angle * (double)k;

        if (has_drop) out[n++] = polar(rf, base - root_angle);

        for (int i = 0; i < s->flank_samples; i++) {
            double t = (double)i / (double)(s->flank_samples - 1);
            double r = r_start + (r_tip - r_start) * t;
            out[n++] = polar(r, base - flank_angle(s, r));
        }

        for (int i = 1; i <= TIP_INTERIOR; i++) {
            double t = (double)i / (double)(TIP_INTERIOR + 1);
            out[n++] = polar(r_tip, base - tip_angle + 2.0 * tip_angle * t);
        }

        for (int i = s->flank_samples - 1; i >= 0; i--) {
            double t = (double)i / (double)(s->flank_samples - 1);
            double r = r_start + (r_tip - r_start) * t;
            out[n++] = polar(r, base + flank_angle(s, r));
        }

        if (has_drop) out[n++] = polar(rf, base + root_angle);

        for (int i = 1; i <= ROOT_INTERIOR; i++) {
            double t = (double)i / (double)(ROOT_INTERIOR + 1);
            out[n++] = polar(rf, base + root_angle +
                                  (pitch_angle - 2.0 * root_angle) * t);
        }
    }
    return n;
}

/* --- Rack ---------------------------------------------------------------- */

int gearing_rack_teeth(double module, double length) {
    if (!(module > 1e-9)) return 0;
    int n = (int)floor(length / (M_PI * module));
    return (n < 1) ? 1 : n;
}

int gearing_rack_capacity(const GearSpec *s, double length) {
    if (!s || !(s->module > 1e-9)) return 0;
    return 4 * gearing_rack_teeth(s->module, length) + 6;
}

int gearing_rack_outline(const GearSpec *s, double length, double back, Vec2 *out, int cap) {
    if (!s || !(s->module > 1e-9) || !out) return 0;
    int teeth = gearing_rack_teeth(s->module, length);
    if (cap < gearing_rack_capacity(s, length)) return 0;

    double pitch = M_PI * s->module;
    double addendum = s->module * GEARING_ADDENDUM;
    double dedendum = s->module * (GEARING_ADDENDUM + GEARING_CLEARANCE);
    /* A rack flank is straight and leans at the pressure angle: the tooth
     * narrows going out and widens going in, by the same tangent the involute
     * approaches as its radius runs off to infinity. */
    double lean = tan(s->pressure_angle);
    double half_pitch_thick = pitch * 0.25 - s->backlash * 0.5;
    double half_tip = half_pitch_thick - addendum * lean;
    double half_root = half_pitch_thick + dedendum * lean;
    if (half_tip < s->module * 0.05) half_tip = s->module * 0.05;

    double half_w = pitch * (double)teeth * 0.5;
    double y_bottom = -dedendum - ((back > 0.0) ? back : s->module);

    int n = 0;
    out[n++] = (Vec2){ -half_w, y_bottom };
    out[n++] = (Vec2){  half_w, y_bottom };
    out[n++] = (Vec2){  half_w, -dedendum };
    /* Right to left along the toothed face, so the whole outline stays CCW. */
    for (int k = teeth - 1; k >= 0; k--) {
        double cx = (double)k * pitch - half_w + pitch * 0.5;
        out[n++] = (Vec2){ cx + half_root, -dedendum };
        out[n++] = (Vec2){ cx + half_tip,   addendum };
        out[n++] = (Vec2){ cx - half_tip,   addendum };
        out[n++] = (Vec2){ cx - half_root, -dedendum };
    }
    out[n++] = (Vec2){ -half_w, -dedendum };
    return n;
}
