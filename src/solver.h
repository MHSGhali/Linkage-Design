#ifndef SOLVER_H
#define SOLVER_H

#include <stdbool.h>
#include "mechanism.h"

typedef struct {
    int max_iters;
    double tol;           /* convergence threshold on the residual norm */
    double lambda_init;   /* initial Levenberg-Marquardt damping */
    double damping_floor; /* fixed additive damping term, see solver.c */
    Vec2 gravity;         /* world-units/s^2, applied to free connectors by
                            * solver_advance only; {0,0} (the default) disables it */
    /* How far off its rest length a link may be before it counts as
     * genuinely violated -- max(abs, rel * rest). Used both to decide
     * whether variable-length links had to give way and (via
     * solver_has_length_violation) whether the mechanism has bound up. */
    double length_tol_abs;
    double length_tol_rel;
} SolverParams;

SolverParams solver_default_params(void);

/* Call once when transitioning from editing to running: freezes every
 * link's rigid rest-shape (pairwise rest distances) from current connector
 * positions, and for driven links, captures each connector's offset from
 * the pivot at angle zero and resets the accumulated angle. */
void solver_freeze(Mechanism *m);

/* Poses every driven link from its CURRENT accumulated_angle_rad, then
 * Gauss-Newton solves every other (free) connector in place, warm-started
 * from its current position. Returns whether the solve converged within
 * tolerance; connectors are left at the last iterate regardless (never
 * left untouched or set to NaN) so a non-convergent frame degrades
 * gracefully instead of freezing or crashing the animation. */
bool solver_solve_at_current_angle(Mechanism *m, SolverParams params);

/* Reports whether any fixed-length (rigid, non-driven) link is currently
 * stretched or compressed away from its frozen rest length by more than
 * max(abs_tol, rel_tol * rest_length). Use this rather than
 * solver_solve_at_current_angle's convergence flag to decide "the mechanism
 * physically can't be in this position": that flag compares an absolute
 * least-squares cost against a fixed tolerance, which is far too strict once
 * coordinates are hundreds of units large (the residuals are in units of
 * length SQUARED), whereas this is scale-aware. */
bool solver_has_length_violation(const Mechanism *m, double abs_tol, double rel_tol);

/* Advances every driven link's accumulated angle by motor_speed_deg_s * dt.
 * If params.gravity is nonzero, every free (non-anchor, not part of a
 * driven link) connector is first given one Verlet integration step under
 * gravity (using its position on the previous call as an implicit
 * velocity), which solver_solve_at_current_angle then pulls back onto the
 * rigid-link constraint manifold -- so gravity acts as an external force on
 * otherwise-unconstrained degrees of freedom without changing how
 * constraints themselves are solved. Then calls solver_solve_at_current_angle. */
bool solver_advance(Mechanism *m, double dt, SolverParams params);

#endif
