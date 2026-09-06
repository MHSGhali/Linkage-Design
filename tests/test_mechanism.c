#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "../src/mechanism.h"
#include "../src/solver.h"
#include "../src/export.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int failures = 0;

static void check_close(const char *name, double actual, double expected, double tol) {
    if (fabs(actual - expected) > tol) {
        printf("FAIL %s: expected %.9f, got %.9f (diff %.3g)\n", name, expected, actual, fabs(actual - expected));
        failures++;
    } else {
        printf("PASS %s\n", name);
    }
}

static void check_true(const char *name, bool cond) {
    if (!cond) {
        printf("FAIL %s\n", name);
        failures++;
    } else {
        printf("PASS %s\n", name);
    }
}

/* Builds the same four-bar family used in the old linkage-specific tests:
 * O2=(0,0), O4=(4,0), crank length 1, coupler length 3, rocker length 2. B is
 * left at a geometrically valid theta=0 assembly (coupler length 3, rocker
 * length 2: circle-circle intersection of radius-3 around A and radius-2
 * around O4), NOT at a test's chosen seed -- solver_freeze() (called by the
 * caller right after this) captures rest_dist from whatever the CURRENT
 * positions are, so B must still be at the correct design geometry at that
 * moment. Callers should move B to their chosen (imperfect) seed only AFTER
 * calling solver_freeze(), so the seed serves purely as Newton's warm start
 * without retroactively corrupting the frozen rest lengths. */
static void build_four_bar(Mechanism *m, int *o2, int *o4, int *a, int *b) {
    mechanism_init(m);
    *o2 = mechanism_add_connector(m, (Vec2){ 0, 0 }, true);
    *o4 = mechanism_add_connector(m, (Vec2){ 4, 0 }, true);
    *a = mechanism_add_connector(m, (Vec2){ 1, 0 }, false); /* crank tip at angle 0 */
    *b = mechanism_add_connector(m, (Vec2){ 10.0 / 3.0, 4.0 * sqrt(2.0) / 3.0 }, false);

    int crank[2] = { *o2, *a };
    int coupler[2] = { *a, *b };
    int rocker[2] = { *b, *o4 };
    mechanism_add_link(m, crank, 2);
    mechanism_add_link(m, coupler, 2);
    mechanism_add_link(m, rocker, 2);

    mechanism_toggle_driven(m, 0, 0.0);
}

static void test_four_bar_reduces_to_closed_form(void) {
    Mechanism m;
    int o2, o4, a, b;
    build_four_bar(&m, &o2, &o4, &a, &b);

    solver_freeze(&m);
    m.connectors[b].pos = (Vec2){ 3, 1 }; /* imperfect seed, set only after freezing */
    m.links[0].accumulated_angle_rad = M_PI / 2.0;
    SolverParams params = solver_default_params();
    bool converged = solver_solve_at_current_angle(&m, params);

    check_true("four-bar solve converges", converged);
    check_close("four-bar A.x", m.connectors[a].pos.x, 0.0, 1e-6);
    check_close("four-bar A.y", m.connectors[a].pos.y, 1.0, 1e-6);
    check_close("four-bar B.x", m.connectors[b].pos.x, (44.0 + 4.0 * sqrt(2.0)) / 17.0, 1e-6);
    check_close("four-bar B.y", m.connectors[b].pos.y, (6.0 + 16.0 * sqrt(2.0)) / 17.0, 1e-6);

    mechanism_free(&m);
}

static void test_ternary_link_rigidity(void) {
    /* O2=(0,0) anchor, driven crank {O2,A}; ternary link {A,P2,P3} (A is
     * fixed for this link's purposes since the crank already poses it);
     * closing rocker {P3,O4} pins the remaining rotational freedom.
     *
     * The crank is driven by only 10 degrees here (see below), not a large
     * jump: solver_solve_at_current_angle is a single-shot solve from
     * whatever seed it's given, unlike real simulation where each frame
     * warm-starts from the previous frame's converged (and thus very close)
     * position. A large one-shot rotation can require the {P2,P3}
     * sub-assembly to swing through a full rigid-body rotation around A
     * to reach one of (generically) two valid rocker-closure solutions --
     * exactly the branch ambiguity four-bar linkages have -- which a local
     * solver started from the unrotated design pose isn't guaranteed to
     * find. A small rotation keeps the true solution within the seed's
     * basin of attraction, which is what this test is actually checking
     * (that ternary rigidity, including the non-consecutive P2-P3 pair,
     * holds after solving) rather than global reachability. */
    Mechanism m;
    mechanism_init(&m);
    int o2 = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int o4 = mechanism_add_connector(&m, (Vec2){ 6, 0 }, true);
    int a = mechanism_add_connector(&m, (Vec2){ 2, 0 }, false);
    int p2 = mechanism_add_connector(&m, (Vec2){ 4, 2 }, false);
    int p3 = mechanism_add_connector(&m, (Vec2){ 5, -1 }, false);

    int crank[2] = { o2, a };
    int ternary[3] = { a, p2, p3 };
    int rocker[2] = { p3, o4 };
    mechanism_add_link(&m, crank, 2);
    int ternary_link = mechanism_add_link(&m, ternary, 3);
    mechanism_add_link(&m, rocker, 2);
    mechanism_toggle_driven(&m, 0, 0.0);

    double rest_a_p2 = vec2_dist(m.connectors[a].pos, m.connectors[p2].pos);
    double rest_a_p3 = vec2_dist(m.connectors[a].pos, m.connectors[p3].pos);
    double rest_p2_p3 = vec2_dist(m.connectors[p2].pos, m.connectors[p3].pos);

    solver_freeze(&m);
    m.links[0].accumulated_angle_rad = M_PI / 18.0; /* 10 degrees */
    SolverParams params = solver_default_params();
    bool converged = solver_solve_at_current_angle(&m, params);

    check_true("ternary-link solve converges", converged);
    (void)ternary_link;
    check_close("ternary A-P2 stays rigid", vec2_dist(m.connectors[a].pos, m.connectors[p2].pos), rest_a_p2, 1e-5);
    check_close("ternary A-P3 stays rigid", vec2_dist(m.connectors[a].pos, m.connectors[p3].pos), rest_a_p3, 1e-5);
    check_close("ternary P2-P3 stays rigid (non-consecutive pair)",
                vec2_dist(m.connectors[p2].pos, m.connectors[p3].pos), rest_p2_p3, 1e-5);

    mechanism_free(&m);
}

static void test_damping_floor_prevents_singular_failure(void) {
    /* Same crank-rocker family as test 1, driven to the exact fully-extended
     * dead center (theta=180deg): A=(-1,0), O4=(4,0). At any point with
     * B.y==0 (not just the true root B=(2,0)), both the coupler and rocker
     * distance-constraint gradients point purely along x -- the Jacobian's
     * y-column is EXACTLY zero there, deterministically, regardless of
     * floating-point specifics. Seeding B at (1.5,0) (on that degenerate
     * line, but not yet at the true root) makes the bug reproduce
     * deterministically: with multiplicative-only damping, lambda*0 stays 0
     * at any lambda, so the y-unknown's row of the normal equations is
     * exactly singular on every attempt. */
    Mechanism m;
    int o2, o4, a, b;
    build_four_bar(&m, &o2, &o4, &a, &b);

    solver_freeze(&m);
    m.connectors[b].pos = (Vec2){ 1.5, 0.0 };
    m.links[0].accumulated_angle_rad = M_PI;
    SolverParams good_params = solver_default_params();
    bool converged = solver_solve_at_current_angle(&m, good_params);

    /* B's tolerance is deliberately loose: exactly at this dead center the
     * problem is genuinely singular in the y-direction (see comment above),
     * so even the tiny floating-point inexactness of M_PI (sin(M_PI) is
     * ~1.2e-16, not exactly 0) gets amplified by division against the tiny
     * damping_floor pivot into a small but real drift in B.y over the
     * iterations -- a real property of solving near a singularity, not a
     * logic bug. */
    check_true("dead-center solve converges with damping floor", converged);
    check_close("dead-center A.x", m.connectors[a].pos.x, -1.0, 1e-6);
    check_close("dead-center A.y", m.connectors[a].pos.y, 0.0, 1e-6);
    check_close("dead-center B.x", m.connectors[b].pos.x, 2.0, 1e-3);
    check_close("dead-center B.y", m.connectors[b].pos.y, 0.0, 1e-3);

    mechanism_free(&m);

    /* Same scenario, but with the damping floor disabled (regression-locks
     * the exact bug class found in the earlier four-bar synthesis tool: a
     * zero-sensitivity Jacobian column leaves multiplicative-only damping
     * singular no matter how far lambda is raised). */
    Mechanism m2;
    int o2b, o4b, ab, bb;
    build_four_bar(&m2, &o2b, &o4b, &ab, &bb);
    solver_freeze(&m2);
    m2.connectors[bb].pos = (Vec2){ 1.5, 0.0 };
    m2.links[0].accumulated_angle_rad = M_PI;
    SolverParams no_floor_params = good_params;
    no_floor_params.damping_floor = 0.0;
    bool converged_no_floor = solver_solve_at_current_angle(&m2, no_floor_params);
    check_true("dead-center solve fails without damping floor (regression lock)", !converged_no_floor);

    mechanism_free(&m2);
}

static void test_connector_tracing(void) {
    /* A single anchor-driven crank: A should trace a unit circle around O2
     * as the crank sweeps. Confirms mechanism_trace_step records one point
     * per call, mechanism_clear_traces resets without losing the flag, and
     * mechanism_set_traced(false) discards the recorded path. */
    Mechanism m;
    mechanism_init(&m);
    int o2 = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int a = mechanism_add_connector(&m, (Vec2){ 1, 0 }, false);
    int crank[2] = { o2, a };
    mechanism_add_link(&m, crank, 2);
    mechanism_toggle_driven(&m, 0, 0.0);
    mechanism_set_traced(&m, a, true);

    solver_freeze(&m);
    mechanism_clear_traces(&m);
    SolverParams params = solver_default_params();

    double angles[3] = { 0.0, M_PI / 2.0, M_PI };
    for (int i = 0; i < 3; i++) {
        m.links[0].accumulated_angle_rad = angles[i];
        solver_solve_at_current_angle(&m, params);
        mechanism_trace_step(&m);
    }

    check_true("traced connector recorded one point per step", m.connectors[a].path_count == 3);
    check_close("path point 0 x", m.connectors[a].path[0].x, 1.0, 1e-9);
    check_close("path point 0 y", m.connectors[a].path[0].y, 0.0, 1e-9);
    check_close("path point 1 x", m.connectors[a].path[1].x, 0.0, 1e-9);
    check_close("path point 1 y", m.connectors[a].path[1].y, 1.0, 1e-9);
    check_close("path point 2 x", m.connectors[a].path[2].x, -1.0, 1e-9);
    check_close("path point 2 y", m.connectors[a].path[2].y, 0.0, 1e-9);

    mechanism_clear_traces(&m);
    check_true("clear_traces resets count but keeps flag", m.connectors[a].path_count == 0 && m.connectors[a].traced);

    mechanism_set_traced(&m, a, false);
    check_true("untracing discards the path", m.connectors[a].path_count == 0 && m.connectors[a].path == NULL);

    mechanism_free(&m);
}

static void test_mechanism_clone_is_independent_deep_copy(void) {
    Mechanism m;
    mechanism_init(&m);
    int o2 = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int a = mechanism_add_connector(&m, (Vec2){ 1, 0 }, false);
    int b = mechanism_add_connector(&m, (Vec2){ 1, 1 }, false);
    int crank[2] = { o2, a };
    int coupler[2] = { a, b };
    mechanism_add_link(&m, crank, 2);
    mechanism_add_link(&m, coupler, 2);
    mechanism_toggle_driven(&m, 0, 45.0);
    mechanism_set_traced(&m, b, true);
    solver_freeze(&m);
    mechanism_trace_step(&m);

    Mechanism clone;
    mechanism_clone(&m, &clone);

    /* Mutate the original after cloning; the clone must be unaffected --
     * this is exactly the property undo relies on. */
    m.connectors[a].pos = (Vec2){ 99, 99 };
    mechanism_set_anchor(&m, b, true);
    mechanism_delete_link(&m, 1);

    check_true("clone connector count matches", clone.connector_count == 3);
    check_true("clone link count matches", clone.link_count == 2);
    check_close("clone A.x unaffected by later mutation", clone.connectors[a].pos.x, 1.0, 1e-12);
    check_true("clone B is_anchor unaffected by later mutation", clone.connectors[b].is_anchor == false);
    check_true("clone link 1 still alive after original's was deleted", clone.links[1].alive);
    check_true("clone connector B traced flag preserved", clone.connectors[b].traced);
    check_true("clone connector B path preserved", clone.connectors[b].path_count == 1);
    check_true("clone link 0 is_driven preserved", clone.links[0].is_driven);
    check_close("clone link 0 rest_dist preserved", clone.links[0].rest_dist[0], 1.0, 1e-12);

    mechanism_free(&m);
    mechanism_free(&clone);
}

static void test_export_blender_script(void) {
    Mechanism m;
    mechanism_init(&m);
    int o2 = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int o4 = mechanism_add_connector(&m, (Vec2){ 100, 0 }, true);
    int a = mechanism_add_connector(&m, (Vec2){ 20, 0 }, false);
    int crank[2] = { o2, a };
    int rocker[2] = { a, o4 };
    mechanism_add_link(&m, crank, 2);
    mechanism_add_link(&m, rocker, 2);

    const char *path = "test_export_output.py";
    check_true("export_blender_script succeeds", export_blender_script(&m, path));

    FILE *f = fopen(path, "r");
    check_true("exported file can be reopened", f != NULL);
    if (f) {
        char buf[8192];
        size_t n = fread(buf, 1, sizeof(buf) - 1, f);
        buf[n] = '\0';
        fclose(f);

        check_true("script sets metric units", strstr(buf, "unit_settings.system = 'METRIC'") != NULL);
        check_true("script defines add_rod", strstr(buf, "def add_rod(") != NULL);
        check_true("script defines add_joint", strstr(buf, "def add_joint(") != NULL);
        check_true("script includes an anchor joint call", strstr(buf, "add_joint(\"Anchor_0\"") != NULL);
        check_true("script includes a non-anchor joint call", strstr(buf, "add_joint(\"Joint_2\"") != NULL);
        check_true("script includes a rod for the crank link", strstr(buf, "add_rod(\"Link0_c0c2\"") != NULL);
        check_true("script includes a rod for the rocker link", strstr(buf, "add_rod(\"Link1_c2c1\"") != NULL);
        check_true("script encodes the crank endpoint coordinates", strstr(buf, "20.000000, 0.000000") != NULL);
    }

    remove(path);
    mechanism_free(&m);
}

static void test_gravity_moves_free_unconstrained_connector(void) {
    /* A single free connector with no links at all: solver_advance's
     * Gauss-Newton stage has nothing to project against, so this isolates
     * the Verlet gravity step itself. Starting at rest (prev_pos == pos,
     * set by solver_freeze), one step of dt should move it by
     * gravity * dt^2 exactly. */
    Mechanism m;
    mechanism_init(&m);
    int a = mechanism_add_connector(&m, (Vec2){ 0, 0 }, false);

    solver_freeze(&m);
    SolverParams params = solver_default_params();
    check_true("gravity is off by default", params.gravity.x == 0.0 && params.gravity.y == 0.0);
    params.gravity = (Vec2){ 0.0, 100.0 };

    solver_advance(&m, 1.0, params);
    check_close("gravity displaces an unconstrained connector by g*dt^2", m.connectors[a].pos.y, 100.0, 1e-9);
    check_close("gravity does not introduce sideways drift", m.connectors[a].pos.x, 0.0, 1e-9);

    mechanism_free(&m);
}

static void test_gravity_preserves_rigid_constraint(void) {
    /* A pendulum: O anchored, P free, joined by a rigid rod. Gravity should
     * pull P generally downward over time, but the Gauss-Newton projection
     * every frame must keep the rod's length exactly rigid regardless. */
    Mechanism m;
    mechanism_init(&m);
    int o = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int p = mechanism_add_connector(&m, (Vec2){ 50, 0 }, false);
    int rod[2] = { o, p };
    mechanism_add_link(&m, rod, 2);

    solver_freeze(&m);
    SolverParams params = solver_default_params();
    params.gravity = (Vec2){ 0.0, 500.0 };

    for (int i = 0; i < 30; i++) {
        solver_advance(&m, 1.0 / 60.0, params);
    }

    check_close("gravity-driven pendulum keeps its rod length rigid",
                vec2_dist(m.connectors[o].pos, m.connectors[p].pos), 50.0, 1e-3);
    check_true("gravity pulls the pendulum bob downward over time", m.connectors[p].pos.y > 1.0);

    mechanism_free(&m);
}

static void test_variable_length_link_is_not_enforced(void) {
    Mechanism m;
    mechanism_init(&m);
    int o = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int a = mechanism_add_connector(&m, (Vec2){ 50, 0 }, false);
    int link[2] = { o, a };
    int lid = mechanism_add_link(&m, link, 2);

    check_true("links default to rigid", m.links[lid].rigid);
    mechanism_set_rigid(&m, lid, false);
    check_true("mechanism_set_rigid can turn rigidity off", !m.links[lid].rigid);

    solver_freeze(&m); /* rest_dist still recomputed (=50) but unused while non-rigid */
    SolverParams params = solver_default_params();

    /* Simulate having dragged A far away in edit mode, then run one frame. */
    m.connectors[a].pos = (Vec2){ 500, 0 };
    bool converged = solver_solve_at_current_angle(&m, params);

    check_true("solve still reports converged (nothing left to enforce)", converged);
    check_close("non-rigid link's distance is left wherever it was, not restored",
                vec2_dist(m.connectors[o].pos, m.connectors[a].pos), 500.0, 1e-6);

    mechanism_free(&m);
}

static void test_toggling_back_to_rigid_reenforces_constraint(void) {
    Mechanism m;
    mechanism_init(&m);
    int o = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int a = mechanism_add_connector(&m, (Vec2){ 50, 0 }, false);
    int link[2] = { o, a };
    int lid = mechanism_add_link(&m, link, 2);

    mechanism_set_rigid(&m, lid, false);
    solver_freeze(&m);
    m.connectors[a].pos = (Vec2){ 50, 500 };
    SolverParams params = solver_default_params();
    solver_solve_at_current_angle(&m, params);
    check_close("moved freely while non-rigid", m.connectors[a].pos.y, 500.0, 1e-6);

    /* Toggling back to rigid and re-freezing captures whatever the CURRENT
     * (stretched) distance is as the new fixed length -- consistent with
     * how solver_freeze always recomputes rest_dist from current positions. */
    mechanism_set_rigid(&m, lid, true);
    solver_freeze(&m);
    double expected_len = vec2_dist(m.connectors[o].pos, m.connectors[a].pos);
    m.connectors[a].pos = vec2_add(m.connectors[a].pos, (Vec2){ 10, 10 });
    solver_solve_at_current_angle(&m, params);
    check_close("re-enabled rigidity restores the (new) fixed length",
                vec2_dist(m.connectors[o].pos, m.connectors[a].pos), expected_len, 1e-3);

    mechanism_free(&m);
}

int main(void) {
    test_four_bar_reduces_to_closed_form();
    test_ternary_link_rigidity();
    test_damping_floor_prevents_singular_failure();
    test_connector_tracing();
    test_mechanism_clone_is_independent_deep_copy();
    test_export_blender_script();
    test_gravity_moves_free_unconstrained_connector();
    test_gravity_preserves_rigid_constraint();
    test_variable_length_link_is_not_enforced();
    test_toggling_back_to_rigid_reenforces_constraint();

    if (failures == 0) {
        printf("\nAll tests passed.\n");
        return 0;
    }
    printf("\n%d test(s) FAILED.\n", failures);
    return 1;
}
