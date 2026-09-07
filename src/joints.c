#include "joints.h"

#include <math.h>

/* ---------------------------------------------------------------------------
 * Gears
 * ------------------------------------------------------------------------ */

double gear_driven_rotation(const Gear *g, double driver_rotation) {
    if (g->kind == GEAR_RACK) return 0.0;
    if (!(g->driven_radius > 1e-9)) return 0.0;
    double ratio = g->driver_radius / g->driven_radius;
    /* Two external gears roll against each other, so they turn opposite ways;
     * an internal mesh has the pinion inside the ring, and they turn together. */
    return (g->kind == GEAR_INTERNAL) ? driver_rotation * ratio : -driver_rotation * ratio;
}

double gear_rack_travel(const Gear *g, double driver_rotation) {
    if (g->kind != GEAR_RACK) return 0.0;
    /* Rolling without slipping: arc length at the pitch radius. */
    return driver_rotation * g->driver_radius;
}

/* ---------------------------------------------------------------------------
 * Geneva wheel
 * ------------------------------------------------------------------------ */

double geneva_engagement_half_angle(int slots) {
    if (slots < 3) slots = 3;
    /* At entry the crank is perpendicular to the slot, which puts the pin,
     * the driver centre and the wheel centre on a right triangle with
     * cos(alpha) = crank/centre_distance = sin(pi/slots). */
    return M_PI / 2.0 - M_PI / (double)slots;
}

double geneva_center_distance(int slots, double crank_radius) {
    if (slots < 3) slots = 3;
    double s = sin(M_PI / (double)slots);
    return (s > 1e-9) ? crank_radius / s : crank_radius;
}

double geneva_crank_radius(int slots, double center_distance) {
    if (slots < 3) slots = 3;
    return center_distance * sin(M_PI / (double)slots);
}

double geneva_wheel_angle(int slots, double driver_rotation, bool *engaged_out) {
    if (slots < 3) slots = 3;

    double step = 2.0 * M_PI / (double)slots;   /* one index per driver turn */
    double alpha_e = geneva_engagement_half_angle(slots);
    double gamma_e = M_PI / (double)slots;      /* half the wheel's step */

    /* Split the driver's rotation into whole turns already indexed, plus where
     * it sits within the current turn. Zero is taken to be the moment the pin
     * ENTERS a slot, so that a freshly frozen mechanism starts with the wheel
     * exactly on an index and the pin exactly at the slot's mouth -- rather
     * than half way through a step, which is what centring engagement on zero
     * would mean. */
    double turns = floor(driver_rotation / (2.0 * M_PI));
    double local = driver_rotation - turns * 2.0 * M_PI;   /* in [0, 2*pi) */

    if (local > 2.0 * alpha_e) {
        if (engaged_out) *engaged_out = false;
        return -(turns * step + step);    /* indexed, locked until next time */
    }

    /* Engaged: the slot points straight at the pin, so the wheel's angle is
     * the direction from its centre to the pin, measured from the line of
     * centres. Distances are in units of the centre distance. */
    double inner = local - alpha_e;             /* -alpha_e at entry, +alpha_e at exit */
    double ratio = sin(M_PI / (double)slots);   /* crank / centre distance */
    double ux = ratio * cos(inner) - 1.0;
    double uy = ratio * sin(inner);
    double gamma = atan2(-uy, -ux);

    if (engaged_out) *engaged_out = true;
    /* An external Geneva turns its wheel the OPPOSITE way to the driver: as
     * the pin sweeps forward past the line of centres, the slot it sits in
     * swings backwards. gamma runs from +gamma_e down to -gamma_e across the
     * engagement, so (gamma - gamma_e) runs from 0 down to a full negative
     * step -- continuous with both locked cases either side. */
    return -(turns * step) + (gamma - gamma_e);
}
