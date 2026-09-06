#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "../src/mechanism.h"
#include "../src/solver.h"
#include "../src/export.h"
#include "../src/ui.h"

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

static void test_dead_center_singularity_is_solved(void) {
    /* Same crank-rocker family as test 1, driven to the exact fully-extended
     * dead center (theta=180deg): A=(-1,0), O4=(4,0). At any point with
     * B.y==0 (not just the true root B=(2,0)), both the coupler and rocker
     * distance-constraint gradients point purely along x -- the Jacobian's
     * y-column is EXACTLY zero there, deterministically, regardless of
     * floating-point specifics. Seeding B at (1.5,0) (on that degenerate
     * line, but not yet at the true root) exercises that singular direction
     * head-on: damping has to restrain it, or the solve either divides
     * through a singular matrix or proposes a runaway step along it. */
    Mechanism m;
    int o2, o4, a, b;
    build_four_bar(&m, &o2, &o4, &a, &b);

    solver_freeze(&m);
    m.connectors[b].pos = (Vec2){ 1.5, 0.0 };
    m.links[0].accumulated_angle_rad = M_PI;
    SolverParams params = solver_default_params();
    solver_solve_at_current_angle(&m, params);

    check_close("dead-center A.x", m.connectors[a].pos.x, -1.0, 1e-6);
    check_close("dead-center A.y", m.connectors[a].pos.y, 0.0, 1e-6);
    check_close("dead-center B.x", m.connectors[b].pos.x, 2.0, 1e-3);
    check_close("dead-center B.y", m.connectors[b].pos.y, 0.0, 1e-3);
    check_true("dead-center solve leaves no length violation",
               !solver_has_length_violation(&m, params.length_tol_abs, params.length_tol_rel));

    mechanism_free(&m);
}

static void test_near_null_direction_does_not_stall_the_solve(void) {
    /* A pendulum hanging straight down: the bob is directly below its
     * anchor, so dx==0 and moving sideways changes the rod's length only to
     * second order. Damping scaled per-diagonal (lambda * JtJ[d][d]) barely
     * restrains that direction at all, so Gauss-Newton proposes an enormous
     * sideways step, every step gets rejected, the solve gives up, and the
     * rod silently stretches further every frame. Damping scaled to the
     * problem's overall magnitude keeps it in hand. This is the shape of a
     * motorless mechanism swinging under gravity, so it must stay solid. */
    Mechanism m;
    mechanism_init(&m);
    int o = mechanism_add_connector(&m, (Vec2){ 400, 200 }, true);
    int bob = mechanism_add_connector(&m, (Vec2){ 400, 400 }, false); /* straight down */
    int rod[2] = { o, bob };
    mechanism_add_link(&m, rod, 2);

    solver_freeze(&m); /* rest length 200 */
    SolverParams params = solver_default_params();

    /* Displace it the way a fast-moving gravity step would: mostly
     * sideways, dropping it off the constraint circle. */
    m.connectors[bob].pos = (Vec2){ 388.0, 401.0 };
    solver_solve_at_current_angle(&m, params);

    check_close("the rod is pulled back to its rest length",
                vec2_dist(m.connectors[o].pos, m.connectors[bob].pos), 200.0, 1e-6);
    check_true("no length violation is reported after the correction",
               !solver_has_length_violation(&m, params.length_tol_abs, params.length_tol_rel));

    mechanism_free(&m);
}

static void test_motorless_mechanism_falls_under_gravity(void) {
    /* The whole point of the no-motor case: with gravity on and nothing
     * driving it, a pendulum must actually swing, and its rod must stay
     * exactly rigid the entire time -- including through the bottom of the
     * swing, where it moves fastest and passes through the singular
     * straight-down configuration. */
    Mechanism m;
    mechanism_init(&m);
    int o = mechanism_add_connector(&m, (Vec2){ 400, 200 }, true);
    int bob = mechanism_add_connector(&m, (Vec2){ 600, 200 }, false);
    int rod[2] = { o, bob };
    mechanism_add_link(&m, rod, 2);
    check_true("mechanism reports having no driven link", !mechanism_has_driven_link(&m));

    solver_freeze(&m);
    SolverParams params = solver_default_params();
    params.gravity = (Vec2){ 0.0, 400.0 };

    double start_y = m.connectors[bob].pos.y;
    bool ever_violated = false;
    for (int f = 0; f < 600; f++) {
        solver_advance(&m, 1.0 / 60.0, params);
        if (solver_has_length_violation(&m, params.length_tol_abs, params.length_tol_rel)) {
            ever_violated = true;
            break;
        }
    }

    check_true("a motorless pendulum never spuriously binds while swinging", !ever_violated);
    check_true("gravity actually swung the bob downward", m.connectors[bob].pos.y > start_y + 50.0);
    check_close("the rod held its length for the whole swing",
                vec2_dist(m.connectors[o].pos, m.connectors[bob].pos), 200.0, 1e-3);

    mechanism_free(&m);
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

/* Reads a whole file into a heap buffer, or NULL. Caller frees. */
static char *read_file(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len < 0) { fclose(f); return NULL; }
    char *buf = malloc((size_t)len + 1);
    size_t n = fread(buf, 1, (size_t)len, f);
    buf[n] = '\0';
    fclose(f);
    return buf;
}

/* Counts non-overlapping occurrences of `needle` in `haystack`. */
static int count_occurrences(const char *haystack, const char *needle) {
    int count = 0;
    size_t nlen = strlen(needle);
    for (const char *p = strstr(haystack, needle); p; p = strstr(p + nlen, needle)) count++;
    return count;
}

static void test_export_blender_script(void) {
    /* A Grashof crank-rocker (ground 400, crank 100, coupler 350, rocker
     * 300: s+l = 500 <= p+q = 650, with the crank shortest), so the crank
     * turns all the way round and the export has a full cycle of real
     * motion to record rather than binding partway. B is placed at the
     * circle-circle intersection that makes those rest lengths exact. */
    Mechanism m;
    mechanism_init(&m);
    double bx = 100.0 + (300.0 * 300.0 - 300.0 * 300.0 + 350.0 * 350.0) / (2.0 * 300.0);
    double by = sqrt(350.0 * 350.0 - (bx - 100.0) * (bx - 100.0));
    int o2 = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int o4 = mechanism_add_connector(&m, (Vec2){ 400, 0 }, true);
    int a = mechanism_add_connector(&m, (Vec2){ 100, 0 }, false);
    int b = mechanism_add_connector(&m, (Vec2){ bx, by }, false);
    int crank[2] = { o2, a };
    int coupler[2] = { a, b };
    int rocker[2] = { b, o4 };
    mechanism_add_link(&m, crank, 2);
    mechanism_add_link(&m, coupler, 2);
    mechanism_add_link(&m, rocker, 2);
    mechanism_toggle_driven(&m, 0, 90.0);

    const char *path = "test_export_output.py";
    SolverParams params = solver_default_params();
    check_true("export_blender_script succeeds", export_blender_script(&m, params, path));

    char *buf = read_file(path);
    check_true("exported file can be reopened", buf != NULL);
    if (buf) {
        check_true("script sets metric units", strstr(buf, "unit_settings.system = 'METRIC'") != NULL);
        check_true("script builds joints", strstr(buf, "def make_joint(") != NULL);
        check_true("script builds rods", strstr(buf, "def make_rod(") != NULL);
        check_true("script names the anchor joints", strstr(buf, "(\"Anchor_0\", True)") != NULL);
        check_true("script names the moving joints", strstr(buf, "(\"Joint_2\", False)") != NULL);
        check_true("script includes a rod for the crank link", strstr(buf, "(\"Link0_c0c2\"") != NULL);
        check_true("script includes a rod for the coupler link", strstr(buf, "(\"Link1_c2c3\"") != NULL);

        /* The animation itself: a frame table, keyframes, and a frame range. */
        check_true("script emits a per-frame position table", strstr(buf, "FRAMES = [") != NULL);
        check_true("script keyframes rod position", strstr(buf, "keyframe_insert('location'") != NULL);
        check_true("script keyframes rod orientation", strstr(buf, "keyframe_insert('rotation_quaternion'") != NULL);
        check_true("script keyframes rod length", strstr(buf, "keyframe_insert('scale'") != NULL);
        check_true("script sets the scene frame range", strstr(buf, "scene.frame_end") != NULL);
        check_true("script uses linear interpolation between samples",
                   strstr(buf, "'LINEAR'") != NULL);

        int frame_rows = count_occurrences(buf, "    [(");
        check_true("a fully rotating mechanism records every animation frame",
                   frame_rows == EXPORT_FRAMES);

        /* A static export would repeat one pose. Compare the first row
         * against one a quarter of the way through rather than the last:
         * the crank completes exactly one revolution, so the final frame
         * lands back on the starting pose (the animation loops cleanly). */
        const char *first_row = strstr(buf, "    [(");
        const char *quarter_row = first_row;
        for (int i = 0; i < EXPORT_FRAMES / 4 && quarter_row; i++) {
            quarter_row = strstr(quarter_row + 6, "    [(");
        }
        /* Compare the WHOLE row: the anchors are written first and never
         * move, so a short prefix would match no matter what the mechanism
         * is doing. */
        bool rows_differ = false;
        if (first_row && quarter_row) {
            size_t len_a = strcspn(first_row, "\n");
            size_t len_b = strcspn(quarter_row, "\n");
            rows_differ = (len_a != len_b) || (memcmp(first_row, quarter_row, len_a) != 0);
        }
        check_true("the recorded frames are not all the same pose", rows_differ);

        free(buf);
    }

    remove(path);
    mechanism_free(&m);
}

static void test_export_animation_actually_moves(void) {
    /* Guard against exporting a table of identical frames (which is what a
     * static export looks like): the driven crank's tip must trace out a
     * genuinely varying path across the recorded frames. */
    Mechanism m;
    mechanism_init(&m);
    int o2 = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int a = mechanism_add_connector(&m, (Vec2){ 100, 0 }, false);
    int crank[2] = { o2, a };
    mechanism_add_link(&m, crank, 2);
    mechanism_toggle_driven(&m, 0, 90.0);

    const char *path = "test_export_anim.py";
    SolverParams params = solver_default_params();
    check_true("animated export succeeds", export_blender_script(&m, params, path));

    char *buf = read_file(path);
    check_true("animated export can be reopened", buf != NULL);
    if (buf) {
        /* The crank tip starts at (100,0); a quarter turn later it should be
         * near (0,100), and the table should contain both extremes. */
        char *frames = strstr(buf, "FRAMES = [");
        check_true("frame table present", frames != NULL);
        if (frames) {
            check_true("first frame holds the starting pose", strstr(frames, "(100.0000,0.0000)") != NULL);
            /* Somewhere in the revolution the tip passes near the far side. */
            check_true("the crank tip swings to the opposite side",
                       strstr(frames, "(-99.") != NULL || strstr(frames, "(-100.") != NULL);
        }
        free(buf);
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

static void test_variable_link_holds_its_length_when_nothing_forces_it(void) {
    /* A variable-length link is not a free-floating connection: it should
     * still hold its rest length whenever the rest of the mechanism lets
     * it, and only give way when the geometry leaves no choice. Here
     * nothing else touches A, so the link has no excuse to be stretched. */
    Mechanism m;
    mechanism_init(&m);
    int o = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int a = mechanism_add_connector(&m, (Vec2){ 50, 0 }, false);
    int link[2] = { o, a };
    int lid = mechanism_add_link(&m, link, 2);

    check_true("links default to rigid", m.links[lid].rigid);
    mechanism_set_rigid(&m, lid, false);
    check_true("mechanism_set_rigid can turn rigidity off", !m.links[lid].rigid);

    solver_freeze(&m); /* rest = 50 */
    SolverParams params = solver_default_params();

    /* Simulate having dragged A far away in edit mode, then run one frame. */
    m.connectors[a].pos = (Vec2){ 500, 0 };
    solver_solve_at_current_angle(&m, params);

    check_close("a variable link still pulls back to its rest length when unforced",
                vec2_dist(m.connectors[o].pos, m.connectors[a].pos), 50.0, 1e-3);

    mechanism_free(&m);
}

static void test_variable_link_stretches_only_as_far_as_forced(void) {
    /* O2 and O4 anchored 400 apart. B is held exactly 150 from O4 by a
     * RIGID link, so B can never come closer than 250 to O2 -- yet the
     * VARIABLE link joining O2 to B only "wants" to be 100 long. It must
     * therefore stretch, but only to 250: the least the rigid geometry
     * leaves it. */
    Mechanism m;
    mechanism_init(&m);
    int o2 = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int o4 = mechanism_add_connector(&m, (Vec2){ 400, 0 }, true);
    int b = mechanism_add_connector(&m, (Vec2){ 250, 0 }, false);
    int rigid_link[2] = { b, o4 };
    int variable_link[2] = { o2, b };
    int rigid_id = mechanism_add_link(&m, rigid_link, 2);
    int variable_id = mechanism_add_link(&m, variable_link, 2);
    mechanism_set_rigid(&m, variable_id, false);

    solver_freeze(&m);
    /* Freeze captures rest lengths from the current layout (150 and 250);
     * shorten what the variable link wants so that it is genuinely forced
     * to stretch. */
    m.links[variable_id].rest_dist[0] = 100.0;

    SolverParams params = solver_default_params();
    solver_solve_at_current_angle(&m, params);

    check_close("the rigid link is held exactly at its rest length",
                vec2_dist(m.connectors[b].pos, m.connectors[o4].pos), 150.0, 1e-3);
    check_close("the variable link stretches only as far as the rigid geometry forces",
                vec2_dist(m.connectors[o2].pos, m.connectors[b].pos), 250.0, 1e-3);
    check_true("a forced variable link does not count as the mechanism binding",
               !solver_has_length_violation(&m, params.length_tol_abs, params.length_tol_rel));
    (void)rigid_id;

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
    SolverParams params = solver_default_params();

    /* Re-freezing after toggling back to rigid captures whatever the
     * CURRENT distance is as the new fixed length -- consistent with how
     * solver_freeze always recomputes rest_dist from current positions. */
    mechanism_set_rigid(&m, lid, true);
    m.connectors[a].pos = (Vec2){ 50, 500 };
    solver_freeze(&m);
    double expected_len = vec2_dist(m.connectors[o].pos, m.connectors[a].pos);
    m.connectors[a].pos = vec2_add(m.connectors[a].pos, (Vec2){ 10, 10 });
    solver_solve_at_current_angle(&m, params);
    check_close("re-enabled rigidity restores the (new) fixed length",
                vec2_dist(m.connectors[o].pos, m.connectors[a].pos), expected_len, 1e-3);

    mechanism_free(&m);
}

static void test_jam_detection(void) {
    /* A four-bar that binds as soon as the crank turns: ground 400, crank
     * 100, coupler 150, rocker 150. B must be 150 from A and 150 from O4,
     * which needs |A - O4| <= 300. At crank angle 0 that distance is
     * exactly 300 (fully extended, just assemblable); at ANY other angle A
     * swings away from O4 and the distance exceeds 300, so the loop cannot
     * close without stretching a fixed-length link. */
    Mechanism m;
    mechanism_init(&m);
    int o2 = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int o4 = mechanism_add_connector(&m, (Vec2){ 400, 0 }, true);
    int a = mechanism_add_connector(&m, (Vec2){ 100, 0 }, false);
    int b = mechanism_add_connector(&m, (Vec2){ 250, 0 }, false);
    int crank[2] = { o2, a };
    int coupler[2] = { a, b };
    int rocker[2] = { b, o4 };
    mechanism_add_link(&m, crank, 2);
    mechanism_add_link(&m, coupler, 2);
    mechanism_add_link(&m, rocker, 2);
    mechanism_toggle_driven(&m, 0, 0.0);

    solver_freeze(&m);
    SolverParams params = solver_default_params();

    solver_solve_at_current_angle(&m, params);
    check_true("the assemblable starting position reports no violation",
               !solver_has_length_violation(&m, 0.02, 0.0001));

    /* Now drive it past what the fixed lengths allow. */
    m.links[0].accumulated_angle_rad = M_PI / 4.0;
    solver_solve_at_current_angle(&m, params);

    check_true("driving past the linkage's limit is reported as a length violation",
               solver_has_length_violation(&m, 0.02, 0.0001));

    /* Confirm the report reflects a real stretch, not solver noise: the
     * coupler and rocker together can't span the gap they're asked to. */
    double span = vec2_dist(m.connectors[a].pos, m.connectors[o4].pos);
    check_true("the required span really exceeds coupler + rocker", span > 300.0 + 0.25);
    double coupler_len = vec2_dist(m.connectors[a].pos, m.connectors[b].pos);
    double rocker_len = vec2_dist(m.connectors[b].pos, m.connectors[o4].pos);
    check_true("at least one fixed link is visibly off its rest length",
               fabs(coupler_len - 150.0) > 0.25 || fabs(rocker_len - 150.0) > 0.25);

    mechanism_free(&m);
}

static void test_working_mechanism_reports_no_jam(void) {
    /* The same four-bar as test 1, at a solvable angle: the jam check must
     * NOT fire, or a perfectly good simulation would freeze. Uses realistic
     * app-scale coordinates (hundreds of units) to confirm the check is
     * scale-aware rather than tied to an absolute residual tolerance. */
    Mechanism m;
    mechanism_init(&m);
    int o2 = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int o4 = mechanism_add_connector(&m, (Vec2){ 400, 0 }, true);
    int a = mechanism_add_connector(&m, (Vec2){ 100, 0 }, false);
    int b = mechanism_add_connector(&m, (Vec2){ 1000.0 / 3.0, 400.0 * sqrt(2.0) / 3.0 }, false);
    int crank[2] = { o2, a };
    int coupler[2] = { a, b };
    int rocker[2] = { b, o4 };
    mechanism_add_link(&m, crank, 2);
    mechanism_add_link(&m, coupler, 2);
    mechanism_add_link(&m, rocker, 2);
    mechanism_toggle_driven(&m, 0, 0.0);

    solver_freeze(&m);
    SolverParams params = solver_default_params();

    /* Drive it through a range of angles it can actually reach. */
    for (int step = 1; step <= 10; step++) {
        m.links[0].accumulated_angle_rad = (M_PI / 4.0) * ((double)step / 10.0);
        solver_solve_at_current_angle(&m, params);
        check_true("a solvable four-bar never reports a length violation",
                   !solver_has_length_violation(&m, 0.02, 0.0001));
    }

    mechanism_free(&m);
}

/* --------------------------------------------------------------------------
 * Toolbar (src/ui.c) -- layout, hit-testing and the enable/active rules.
 * ----------------------------------------------------------------------- */

static const UiButton *button_for(const Toolbar *t, UiAction action) {
    for (int i = 0; i < t->count; i++) {
        if (t->buttons[i].action == action) return &t->buttons[i];
    }
    return NULL;
}

/* A plain editing state with nothing selected -- the baseline each test
 * tweaks one field of. */
static UiState blank_ui_state(void) {
    UiState s;
    s.editing = true;
    s.selected_connector_count = 0;
    s.selected_link = -1;
    s.selected_link_driven = false;
    s.selected_link_rigid = true;
    s.selected_link_can_drive = false;
    s.all_selected_traced = false;
    s.has_selection = false;
    s.can_undo = false;
    s.can_redo = false;
    s.gravity_on = false;
    s.running = false;
    return s;
}

static void test_toolbar_layout_is_well_formed(void) {
    Toolbar t;
    ui_init(&t);

    check_true("toolbar has every requested button", t.count == 13);

    bool all_inside = true, no_overlap = true, all_have_labels = true;
    for (int i = 0; i < t.count; i++) {
        const UiRect *r = &t.buttons[i].rect;
        if (r->x < 0 || r->x + r->w > UI_TOOLBAR_W || r->y < 0) all_inside = false;
        if (!t.buttons[i].label || !t.buttons[i].label[0]) all_have_labels = false;
        for (int j = i + 1; j < t.count; j++) {
            const UiRect *o = &t.buttons[j].rect;
            if (r->y < o->y + o->h && o->y < r->y + r->h) no_overlap = false;
        }
    }
    check_true("every button sits inside the toolbar strip", all_inside);
    check_true("no two buttons overlap", no_overlap);
    check_true("every button has a label", all_have_labels);

    const UiButton *last = &t.buttons[t.count - 1];
    check_true("the whole toolbar fits in the window height", last->rect.y + last->rect.h < 700);
}

static void test_toolbar_hit_testing(void) {
    Toolbar t;
    ui_init(&t);

    bool centers_hit = true;
    for (int i = 0; i < t.count; i++) {
        const UiRect *r = &t.buttons[i].rect;
        if (ui_hit_test(&t, r->x + r->w / 2, r->y + r->h / 2) != i) centers_hit = false;
    }
    check_true("each button's centre hit-tests to that button", centers_hit);

    /* The gap between the first two buttons belongs to no button. */
    const UiRect *first = &t.buttons[0].rect;
    check_true("the gap between buttons hits nothing",
                ui_hit_test(&t, first->x + first->w / 2, first->y + first->h + 1) == -1);

    check_true("a point right of the strip hits nothing",
                ui_hit_test(&t, UI_TOOLBAR_W + 40, first->y + 5) == -1);
    check_true("the canvas is not inside the toolbar", !ui_contains(&t, UI_TOOLBAR_W, 300));
    check_true("the strip is inside the toolbar", ui_contains(&t, UI_TOOLBAR_W - 1, 300));
}

static void test_toolbar_enablement_rules(void) {
    Toolbar t;
    ui_init(&t);

    /* Nothing selected: the actions that need a selection are all off. */
    UiState s = blank_ui_state();
    ui_apply_state(&t, s);
    check_true("ANCHOR is disabled with nothing selected", !button_for(&t, UI_ANCHOR)->enabled);
    check_true("LINK is disabled with nothing selected", !button_for(&t, UI_LINK)->enabled);
    check_true("DELETE is disabled with nothing selected", !button_for(&t, UI_DELETE)->enabled);
    check_true("MOTOR is disabled with no link selected", !button_for(&t, UI_MOTOR)->enabled);
    check_true("UNDO is disabled with empty history", !button_for(&t, UI_UNDO)->enabled);
    check_true("REDO is disabled with empty history", !button_for(&t, UI_REDO)->enabled);
    check_true("RUN is always available", button_for(&t, UI_RUN)->enabled);
    check_true("GRAVITY is always available", button_for(&t, UI_GRAVITY)->enabled);

    /* One connector is enough to anchor or trace, but not to link. */
    s = blank_ui_state();
    s.selected_connector_count = 1;
    s.has_selection = true;
    ui_apply_state(&t, s);
    check_true("ANCHOR is enabled with one connector selected", button_for(&t, UI_ANCHOR)->enabled);
    check_true("TRACE is enabled with one connector selected", button_for(&t, UI_TRACE)->enabled);
    check_true("LINK still needs a second connector", !button_for(&t, UI_LINK)->enabled);

    s.selected_connector_count = 2;
    ui_apply_state(&t, s);
    check_true("LINK is enabled with two connectors selected", button_for(&t, UI_LINK)->enabled);

    /* TRACE lights up only when everything selected is already traced. */
    s.all_selected_traced = true;
    ui_apply_state(&t, s);
    check_true("TRACE is lit when the selection is traced", button_for(&t, UI_TRACE)->active);

    /* A selected link with one anchor can be driven; without one it cannot. */
    s = blank_ui_state();
    s.selected_link = 0;
    s.has_selection = true;
    s.selected_link_can_drive = true;
    ui_apply_state(&t, s);
    check_true("MOTOR is enabled on a link with exactly one anchor", button_for(&t, UI_MOTOR)->enabled);
    check_true("MOTOR is unlit on an undriven link", !button_for(&t, UI_MOTOR)->active);
    check_true("VARY is enabled on an undriven link", button_for(&t, UI_VARY)->enabled);

    s.selected_link_can_drive = false;
    ui_apply_state(&t, s);
    check_true("MOTOR is disabled on a link with no anchor", !button_for(&t, UI_MOTOR)->enabled);

    /* A driven link: MOTOR lit (so it can be turned off), VARY unavailable. */
    s = blank_ui_state();
    s.selected_link = 0;
    s.has_selection = true;
    s.selected_link_driven = true;
    ui_apply_state(&t, s);
    check_true("MOTOR stays enabled on a driven link so it can be undriven",
                button_for(&t, UI_MOTOR)->enabled);
    check_true("MOTOR is lit on a driven link", button_for(&t, UI_MOTOR)->active);
    check_true("VARY is disabled on a driven link", !button_for(&t, UI_VARY)->enabled);

    /* A variable-length link lights VARY. */
    s = blank_ui_state();
    s.selected_link = 0;
    s.has_selection = true;
    s.selected_link_rigid = false;
    ui_apply_state(&t, s);
    check_true("VARY is lit on a variable-length link", button_for(&t, UI_VARY)->active);

    /* History depth drives UNDO/REDO independently. */
    s = blank_ui_state();
    s.can_undo = true;
    ui_apply_state(&t, s);
    check_true("UNDO is enabled once there is history", button_for(&t, UI_UNDO)->enabled);
    check_true("REDO stays disabled until something is undone", !button_for(&t, UI_REDO)->enabled);
    s.can_redo = true;
    ui_apply_state(&t, s);
    check_true("REDO is enabled once something has been undone", button_for(&t, UI_REDO)->enabled);

    check_true("GRAVITY is unlit when gravity is off", !button_for(&t, UI_GRAVITY)->active);
    s.gravity_on = true;
    ui_apply_state(&t, s);
    check_true("GRAVITY is lit when gravity is in force", button_for(&t, UI_GRAVITY)->active);
}

static void test_toolbar_disables_editing_while_running(void) {
    Toolbar t;
    ui_init(&t);

    UiState s = blank_ui_state();
    s.editing = false;
    s.running = true;
    /* Plenty selected, plenty of history -- running is what must gate these. */
    s.selected_connector_count = 3;
    s.has_selection = true;
    s.selected_link = 0;
    s.selected_link_can_drive = true;
    s.can_undo = true;
    s.can_redo = true;
    ui_apply_state(&t, s);

    check_true("JOINT is disabled while running", !button_for(&t, UI_JOINT)->enabled);
    check_true("LINK is disabled while running", !button_for(&t, UI_LINK)->enabled);
    check_true("ANCHOR is disabled while running", !button_for(&t, UI_ANCHOR)->enabled);
    check_true("MOTOR is disabled while running", !button_for(&t, UI_MOTOR)->enabled);
    check_true("DELETE is disabled while running", !button_for(&t, UI_DELETE)->enabled);
    check_true("UNDO is disabled while running", !button_for(&t, UI_UNDO)->enabled);
    check_true("REDO is disabled while running", !button_for(&t, UI_REDO)->enabled);
    check_true("EXPORT is disabled while running", !button_for(&t, UI_EXPORT)->enabled);

    check_true("GRAVITY still works while running", button_for(&t, UI_GRAVITY)->enabled);
    check_true("CLEAR still works while running", button_for(&t, UI_CLEAR)->enabled);
    check_true("RUN still works while running", button_for(&t, UI_RUN)->enabled);
    check_true("RUN is lit while running", button_for(&t, UI_RUN)->active);
    check_true("RUN reads STOP while running", strcmp(button_for(&t, UI_RUN)->label, "STOP") == 0);

    s.running = false;
    s.editing = true;
    ui_apply_state(&t, s);
    check_true("RUN reads RUN again when stopped", strcmp(button_for(&t, UI_RUN)->label, "RUN") == 0);
}

int main(void) {
    test_four_bar_reduces_to_closed_form();
    test_ternary_link_rigidity();
    test_dead_center_singularity_is_solved();
    test_near_null_direction_does_not_stall_the_solve();
    test_motorless_mechanism_falls_under_gravity();
    test_connector_tracing();
    test_mechanism_clone_is_independent_deep_copy();
    test_export_blender_script();
    test_export_animation_actually_moves();
    test_gravity_moves_free_unconstrained_connector();
    test_gravity_preserves_rigid_constraint();
    test_variable_link_holds_its_length_when_nothing_forces_it();
    test_variable_link_stretches_only_as_far_as_forced();
    test_toggling_back_to_rigid_reenforces_constraint();
    test_jam_detection();
    test_working_mechanism_reports_no_jam();
    test_toolbar_layout_is_well_formed();
    test_toolbar_hit_testing();
    test_toolbar_enablement_rules();
    test_toolbar_disables_editing_while_running();

    if (failures == 0) {
        printf("\nAll tests passed.\n");
        return 0;
    }
    printf("\n%d test(s) FAILED.\n", failures);
    return 1;
}
