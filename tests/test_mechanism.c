#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "../src/mechanism.h"
#include "../src/solver.h"
#include "../src/export.h"
#include "../src/ui.h"
#include "../src/cam.h"
#include "../src/synth.h"
#include "../src/joints.h"
#include "../src/templates.h"
#include "../src/status.h"
#include "../src/scene.h"

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

/* For assertions inside a loop, where one line per iteration would drown the
 * log: only says anything when it fails. */
static void check_true_quiet(bool cond) {
    if (!cond) {
        printf("FAIL (in loop)\n");
        failures++;
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
        mechanism_trace_step(&m, 0.1 * (double)i);
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
    mechanism_trace_step(&m, 0.5);

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
    s.drawing_cam = false;
    s.drawing_linkage = false;
    s.drawing_arms = false;
    s.gallery_open = false;
    s.can_make_slider = false;
    s.can_make_gear = false;
    s.can_make_geneva = false;
    s.has_selection = false;
    s.can_undo = false;
    s.can_redo = false;
    s.gravity_on = false;
    s.running = false;
    s.jammed = false;
    s.paused = false;
    return s;
}

static void test_toolbar_layout_is_well_formed(void) {
    Toolbar t;
    ui_init(&t, 900);

    /* One button per action, so a command can never be added to the enum
     * without a button (and a tooltip, and a hotkey) to go with it. */
    check_true("the toolbar has a button for every action", t.count == UI_ACTION_COUNT - 1);

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

    /* The strip runs the full height of the window (canvas plus plot panel),
     * not just the canvas. */
    const UiButton *last = &t.buttons[t.count - 1];
    check_true("the whole toolbar fits in the window height", last->rect.y + last->rect.h < 900);
}

static void test_toolbar_hit_testing(void) {
    Toolbar t;
    ui_init(&t, 900);

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
    ui_init(&t, 900);

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
    ui_init(&t, 900);

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


/* --------------------------------------------------------------------------
 * Cams (src/cam.c) -- the profile, and the follower it drives.
 * ----------------------------------------------------------------------- */

static Cam make_test_cam(void) {
    Cam c;
    cam_set_defaults(&c, 60.0);
    c.roller_radius = 6.0;
    cam_fill_motion_law(&c, 60.0, 40.0, 90.0, 60.0, 90.0);
    return c;
}

static void test_cam_profile_is_continuous_and_smooth(void) {
    Cam c = make_test_cam();
    double rise = 90.0 * M_PI / 180.0;
    double high = 60.0 * M_PI / 180.0;
    double fall = 90.0 * M_PI / 180.0;
    double eps = 1e-7;

    check_close("cam starts at the base radius", cam_pitch_radius(&c, 0.0), 60.0, 1e-9);
    check_close("cam base radius is reported", c.base_radius, 60.0, 1e-9);
    check_close("cam lift is reported", c.lift, 40.0, 0.05);
    check_close("cam reaches base+lift at the top of the rise",
                cam_pitch_radius(&c, rise), 100.0, 0.05);
    check_close("cam holds base+lift through the high dwell",
                cam_pitch_radius(&c, rise + high / 2.0), 100.0, 1e-9);
    check_close("cam returns to the base radius after the fall",
                cam_pitch_radius(&c, rise + high + fall), 60.0, 0.05);
    check_close("cam holds the base radius through the low dwell",
                cam_pitch_radius(&c, rise + high + fall + 0.1), 60.0, 1e-9);

    /* Continuity across every segment join. */
    double joins[3] = { rise, rise + high, rise + high + fall };
    for (int i = 0; i < 3; i++) {
        double before = cam_pitch_radius(&c, joins[i] - eps);
        double after = cam_pitch_radius(&c, joins[i] + eps);
        check_close("cam radius is continuous across a segment join", after, before, 1e-4);
    }

    /* Cycloidal motion is chosen so the follower's velocity vanishes at both
     * ends of every segment: no step in velocity, so no jerk spike. */
    check_close("follower velocity is zero at the start of the rise",
                cam_pitch_radius_deriv(&c, 0.0), 0.0, 0.5);
    check_close("follower velocity is zero at the top of the rise",
                cam_pitch_radius_deriv(&c, rise - eps), 0.0, 0.5);
    check_close("follower velocity is zero at the start of the fall",
                cam_pitch_radius_deriv(&c, rise + high + eps), 0.0, 0.5);
    check_close("follower velocity is zero at the end of the fall",
                cam_pitch_radius_deriv(&c, rise + high + fall - eps), 0.0, 0.5);

    check_true("cam radius never dips below the base radius",
               cam_pitch_radius(&c, 1.234) >= 60.0 - 1e-6);

    /* phi wraps, so a full turn lands back on the same profile. */
    check_close("cam profile is periodic over one turn",
                cam_pitch_radius(&c, 0.7 + 2.0 * M_PI), cam_pitch_radius(&c, 0.7), 1e-12);
}

static void test_zero_lift_cam_is_a_circle(void) {
    Cam c = make_test_cam();
    cam_fill_motion_law(&c, 60.0, 0.0, 90.0, 60.0, 90.0);

    bool constant = true;
    for (int i = 0; i < 64; i++) {
        double phi = 2.0 * M_PI * (double)i / 64.0;
        if (fabs(cam_pitch_radius(&c, phi) - 60.0) > 1e-9) constant = false;
    }
    check_true("a zero-lift cam's pitch curve is a circle of the base radius", constant);

    /* The physical surface is the pitch curve inset by the roller radius, so
     * for a circle it is a smaller concentric circle -- the cleanest check
     * that the offset direction and magnitude are right. */
    Vec2 pts[128];
    cam_sample_surface(&c, pts, 128);
    bool inset_ok = true;
    for (int i = 0; i < 128; i++) {
        if (fabs(vec2_len(pts[i]) - (60.0 - 6.0)) > 1e-6) inset_ok = false;
    }
    check_true("its surface is inset by exactly the roller radius", inset_ok);
    check_true("a gentle cam does not undercut", !cam_is_undercut(&c));
}

static void test_undercut_detection(void) {
    Cam sane = make_test_cam();
    check_true("a well-proportioned cam is not flagged as undercut", !cam_is_undercut(&sane));

    /* A big lift crammed into a narrow rise makes a tight concave flank; a
     * roller larger than that radius of curvature cannot reach into it, and
     * the inward offset folds back on itself. */
    Cam sharp = make_test_cam();
    sharp.roller_radius = 25.0;
    cam_fill_motion_law(&sharp, 20.0, 80.0, 15.0, 60.0, 15.0);
    check_true("an oversized roller on a steep flank is flagged as undercut",
               cam_is_undercut(&sharp));
}

/* Cam turning about O with a translating follower on the radial axis. */
static void build_cam_rig(Mechanism *m, int *centre, int *follower, double speed_deg_s) {
    mechanism_init(m);
    *centre = mechanism_add_connector(m, (Vec2){ 0, 0 }, true);
    int tip = mechanism_add_connector(m, (Vec2){ 100, 0 }, false);
    *follower = mechanism_add_connector(m, (Vec2){ 0, -60 }, false);

    int body[2] = { *centre, tip };
    mechanism_add_link(m, body, 2);
    mechanism_toggle_driven(m, 0, speed_deg_s);
    mechanism_add_cam(m, 0, *centre, *follower);
}

static void test_follower_tracks_the_cam_profile(void) {
    /* The strongest statement the cam feature can make: driven at a sane
     * speed with its default return spring, the follower's position along its
     * axis IS the profile the motion law describes, all the way round. */
    Mechanism m;
    int centre, follower;
    build_cam_rig(&m, &centre, &follower, 90.0);

    SolverParams params = solver_default_params();
    solver_freeze(&m);

    double axis_angle = atan2(m.cams[0].axis_dir.y, m.cams[0].axis_dir.x);
    double worst = 0.0;
    bool ever_off_axis = false;
    bool ever_lifted = false;

    /* 90 deg/s, so four seconds is one full revolution. */
    double dt = 1.0 / 240.0;
    for (int step = 0; step < 960; step++) {
        solver_advance(&m, dt, params);

        const Cam *c = &m.cams[0];
        Vec2 d = vec2_sub(m.connectors[follower].pos, m.connectors[centre].pos);
        double s = vec2_dot(d, c->axis_dir);
        double expected = cam_pitch_radius(c, axis_angle - mechanism_cam_angle(&m, 0));
        double err = fabs(s - expected);
        if (err > worst) worst = err;
        if (fabs(vec2_dot(d, vec2_perp(c->axis_dir))) > 1e-6) ever_off_axis = true;
        if (!c->in_contact) ever_lifted = true;
    }

    check_true("follower position matches the cam profile through a full turn", worst < 0.5);
    check_true("follower never leaves its guide axis", !ever_off_axis);
    check_true("a stiff spring keeps the follower on the cam throughout", !ever_lifted);

    mechanism_free(&m);
}

static void test_follower_lifts_off_when_the_spring_cannot_keep_up(void) {
    /* Contact is one-sided: the cam pushes but never pulls. Spin it fast
     * enough, with a weak enough return spring, and the profile falls away
     * faster than the spring can push the follower down -- real cam float.
     * This is what makes the liftoff model more than decoration. */
    Mechanism m;
    int centre, follower;
    build_cam_rig(&m, &centre, &follower, 720.0);
    m.cams[0].spring_k = 50.0;

    SolverParams params = solver_default_params();
    solver_freeze(&m);

    bool lifted = false, recontacted_after_lift = false;
    bool ever_inside_profile = false;
    double axis_angle = atan2(m.cams[0].axis_dir.y, m.cams[0].axis_dir.x);

    double dt = 1.0 / 480.0;
    for (int step = 0; step < 1440; step++) {
        solver_advance(&m, dt, params);
        const Cam *c = &m.cams[0];
        if (!c->in_contact) lifted = true;
        else if (lifted) recontacted_after_lift = true;

        /* Whatever else happens, the follower must never sink into the cam. */
        Vec2 d = vec2_sub(m.connectors[follower].pos, m.connectors[centre].pos);
        double s = vec2_dot(d, c->axis_dir);
        double surface = cam_pitch_radius(c, axis_angle - mechanism_cam_angle(&m, 0));
        if (s < surface - 0.5) ever_inside_profile = true;
    }

    check_true("a weak spring lets the follower leave the cam on a fast fall", lifted);
    check_true("the follower comes back down onto the cam", recontacted_after_lift);
    check_true("the follower never penetrates the cam surface", !ever_inside_profile);

    mechanism_free(&m);
}

static void test_cam_cascades_on_delete(void) {
    Mechanism m;
    int centre, follower;
    build_cam_rig(&m, &centre, &follower, 90.0);
    check_true("cam was created", m.cam_count == 1 && m.cams[0].alive);

    mechanism_delete_connector(&m, follower);
    check_true("deleting the follower deletes the cam", !m.cams[0].alive);

    mechanism_free(&m);
}

/* --------------------------------------------------------------------------
 * Trace timestamps -- what the x/y-versus-time plot is drawn from.
 * ----------------------------------------------------------------------- */

static void test_trace_records_sample_times(void) {
    Mechanism m;
    int o2, o4, a, b;
    build_four_bar(&m, &o2, &o4, &a, &b);
    mechanism_set_traced(&m, b, true);
    solver_freeze(&m);

    double t = 0.0, dt = 0.05;
    for (int i = 0; i < 5; i++) {
        t += dt;
        mechanism_trace_step(&m, t);
    }

    check_true("a timestamp is recorded per trace sample", m.connectors[b].path_count == 5);

    bool increasing = true, matches = true;
    for (int i = 0; i < m.connectors[b].path_count; i++) {
        if (fabs(m.connectors[b].path_time[i] - dt * (double)(i + 1)) > 1e-9) matches = false;
        if (i > 0 && m.connectors[b].path_time[i] <= m.connectors[b].path_time[i - 1]) increasing = false;
    }
    check_true("trace timestamps are strictly increasing", increasing);
    check_true("trace timestamps match the accumulated simulation time", matches);

    /* Undo snapshots the whole mechanism, so the timestamps have to survive
     * a clone as their own allocation. */
    Mechanism clone;
    mechanism_clone(&m, &clone);
    mechanism_trace_step(&m, 99.0);
    check_true("clone keeps its own timestamp array", clone.connectors[b].path_count == 5);
    check_close("clone's timestamps are unaffected by later tracing",
                clone.connectors[b].path_time[4], 0.25, 1e-9);

    mechanism_free(&clone);
    mechanism_free(&m);
}


static void test_drawn_outline_becomes_the_profile(void) {
    /* Draw a plain circle of radius 80. The roller rides ON that surface, so
     * the pitch curve -- the roller centre's path -- must come back as a
     * circle one roller radius larger, and the surface the cam draws and
     * exports must be the circle that was drawn. */
    Cam c;
    cam_set_defaults(&c, 60.0);
    c.roller_radius = 10.0;

    Vec2 drawn[120];
    for (int i = 0; i < 120; i++) {
        double a = 2.0 * M_PI * (double)i / 120.0;
        drawn[i] = (Vec2){ 80.0 * cos(a), 80.0 * sin(a) };
    }
    check_true("a drawn circle is accepted as a profile",
               cam_set_from_drawn_outline(&c, drawn, 120));

    bool pitch_ok = true, surface_ok = true;
    for (int i = 0; i < 64; i++) {
        double phi = 2.0 * M_PI * (double)i / 64.0;
        if (fabs(cam_pitch_radius(&c, phi) - 90.0) > 0.6) pitch_ok = false;
    }
    check_true("the pitch curve sits one roller radius outside what was drawn", pitch_ok);

    Vec2 surface[128];
    cam_sample_surface(&c, surface, 128);
    for (int i = 0; i < 128; i++) {
        if (fabs(vec2_len(surface[i]) - 80.0) > 0.6) surface_ok = false;
    }
    check_true("the cam's surface comes back as the circle that was drawn", surface_ok);
}

static void test_drawn_lobe_is_reproduced(void) {
    /* A genuinely non-circular outline: a base circle with one bump. The
     * profile has to follow it, not average it away. */
    Cam c;
    cam_set_defaults(&c, 60.0);
    c.roller_radius = 4.0;

    Vec2 drawn[240];
    for (int i = 0; i < 240; i++) {
        double a = 2.0 * M_PI * (double)i / 240.0;
        /* A smooth bump centred on a = pi/2, 60 wide at the base. */
        double bump = 0.0;
        double da = a - M_PI / 2.0;
        if (fabs(da) < 0.5) bump = 30.0 * (1.0 + cos(da * M_PI / 0.5)) / 2.0;
        double r = 70.0 + bump;
        drawn[i] = (Vec2){ r * cos(a), r * sin(a) };
    }
    check_true("a lobed outline is accepted", cam_set_from_drawn_outline(&c, drawn, 240));

    double at_bump = cam_pitch_radius(&c, M_PI / 2.0);
    double away = cam_pitch_radius(&c, -M_PI / 2.0);
    check_close("the drawn lobe's height is reproduced", at_bump - away, 30.0, 2.0);
    check_true("the lobe is where it was drawn", at_bump > away + 20.0);
    check_close("the base circle away from the lobe is reproduced", away, 74.0, 1.5);
}

static void test_degenerate_drawings_are_rejected(void) {
    Cam c;
    cam_set_defaults(&c, 60.0);
    double before = c.base_radius;

    Vec2 tiny[3] = { { 0, 0 }, { 0.4, 0 }, { 0, 0.4 } };
    check_true("a scribble too small to read is rejected", !cam_set_from_drawn_outline(&c, tiny, 3));

    Vec2 two[2] = { { 0, 0 }, { 50, 0 } };
    check_true("a stroke with too few points is rejected", !cam_set_from_drawn_outline(&c, two, 2));

    check_close("a rejected drawing leaves the existing profile alone", c.base_radius, before, 1e-9);
}

static void test_roller_shrinks_to_fit_a_drawn_shape(void) {
    /* A hand-drawn outline routinely has concave stretches tighter than the
     * roller that was guessed for it. Rather than producing a profile that is
     * nothing like the drawing, the roller is reduced until it fits -- which
     * is what a cam designer would do. */
    Cam c;
    cam_set_defaults(&c, 60.0);
    c.roller_radius = 20.0;

    Vec2 drawn[240];
    for (int i = 0; i < 240; i++) {
        double a = 2.0 * M_PI * (double)i / 240.0;
        double r = 55.0 + 18.0 * sin(3.0 * a); /* deep valleys, tight radii */
        drawn[i] = (Vec2){ r * cos(a), r * sin(a) };
    }
    check_true("a deeply lobed outline is accepted", cam_set_from_drawn_outline(&c, drawn, 240));
    check_true("the roller was shrunk to fit the drawn shape", c.roller_radius < 20.0);
    check_true("the shrunken cam no longer undercuts", !cam_is_undercut(&c));
    check_true("the roller stays a usable size", c.roller_radius > 0.5);

    /* A gentle outline should leave the roller alone. */
    Cam gentle;
    cam_set_defaults(&gentle, 60.0);
    gentle.roller_radius = 5.0;
    Vec2 circle[120];
    for (int i = 0; i < 120; i++) {
        double a = 2.0 * M_PI * (double)i / 120.0;
        circle[i] = (Vec2){ 80.0 * cos(a), 80.0 * sin(a) };
    }
    check_true("a circle is accepted", cam_set_from_drawn_outline(&gentle, circle, 120));
    check_close("a shape the roller already fits keeps its roller",
                gentle.roller_radius, 5.0, 1e-9);
}

static void test_cam_angle_wrap_does_not_fling_the_follower(void) {
    /* The cam's rotation is read from an atan2, so it wraps by a full turn
     * once per revolution. Read literally that looks like the cam spinning
     * 360 degrees in one frame, which used to hurl the follower off the cam;
     * this pins the fix. Three lobes put a steep flank at the wrap point,
     * which is what makes the bug bite. */
    Mechanism m;
    int centre, follower;
    build_cam_rig(&m, &centre, &follower, 90.0);

    Vec2 drawn[240];
    for (int i = 0; i < 240; i++) {
        double a = 2.0 * M_PI * (double)i / 240.0;
        double r = 55.0 + 18.0 * sin(3.0 * a);
        drawn[i] = (Vec2){ r * cos(a), r * sin(a) };
    }
    cam_set_from_drawn_outline(&m.cams[0], drawn, 240);
    double axis_angle = atan2(m.cams[0].axis_dir.y, m.cams[0].axis_dir.x);
    m.connectors[follower].pos =
        vec2_add(m.connectors[centre].pos,
                  vec2_scale(m.cams[0].axis_dir, cam_pitch_radius(&m.cams[0], axis_angle)));

    SolverParams params = solver_default_params();
    solver_freeze(&m);

    double ceiling = m.cams[0].base_radius + m.cams[0].lift + 5.0;
    double highest = 0.0;
    /* Three full revolutions, so the wrap is crossed several times. */
    for (int step = 0; step < 2880; step++) {
        solver_advance(&m, 1.0 / 240.0, params);
        double s_pos = vec2_dot(vec2_sub(m.connectors[follower].pos, m.connectors[centre].pos),
                                 m.cams[0].axis_dir);
        if (s_pos > highest) highest = s_pos;
    }
    check_true("the follower never gets flung past the cam's outer radius",
               highest < ceiling);

    mechanism_free(&m);
}

static void test_lift_and_timing_adjustments(void) {
    Cam c = make_test_cam();  /* base 60, lift 40 */
    double base = c.base_radius;

    cam_scale_lift(&c, 2.0);
    check_close("scaling the lift leaves the base circle put", c.base_radius, base, 1e-6);
    check_close("scaling the lift doubles it", c.lift, 80.0, 0.1);

    cam_scale_lift(&c, 0.5);
    check_close("scaling back restores the lift", c.lift, 40.0, 0.1);

    /* Timing: the same profile, rotated. The peak must move with it. */
    double peak_before = -1.0, peak_at = 0.0;
    for (int i = 0; i < 360; i++) {
        double phi = 2.0 * M_PI * (double)i / 360.0;
        double r = cam_pitch_radius(&c, phi);
        if (r > peak_before) { peak_before = r; peak_at = phi; }
    }
    cam_rotate_profile(&c, M_PI / 2.0);
    double peak_after = -1.0, peak_at_after = 0.0;
    for (int i = 0; i < 360; i++) {
        double phi = 2.0 * M_PI * (double)i / 360.0;
        double r = cam_pitch_radius(&c, phi);
        if (r > peak_after) { peak_after = r; peak_at_after = phi; }
    }
    check_close("shifting the timing keeps the profile's shape", peak_after, peak_before, 0.2);
    check_close("shifting the timing moves the lobe round by that much",
                peak_at_after - peak_at, M_PI / 2.0, 0.1);
}

static void test_follower_tracks_a_drawn_cam(void) {
    /* The end-to-end statement for drawn cams: whatever outline you draw, the
     * follower rides exactly on it. */
    Mechanism m;
    int centre, follower;
    build_cam_rig(&m, &centre, &follower, 90.0);

    Vec2 drawn[240];
    for (int i = 0; i < 240; i++) {
        double a = 2.0 * M_PI * (double)i / 240.0;
        double r = 55.0 + 18.0 * sin(3.0 * a); /* a three-lobed cam */
        drawn[i] = (Vec2){ r * cos(a), r * sin(a) };
    }
    check_true("three-lobed drawing accepted",
               cam_set_from_drawn_outline(&m.cams[0], drawn, 240));

    /* Seat the follower on the profile, as the app does after drawing. */
    double axis_angle = atan2(m.cams[0].axis_dir.y, m.cams[0].axis_dir.x);
    m.connectors[follower].pos =
        vec2_add(m.connectors[centre].pos,
                  vec2_scale(m.cams[0].axis_dir, cam_pitch_radius(&m.cams[0], axis_angle)));

    SolverParams params = solver_default_params();
    solver_freeze(&m);

    /* Count how many times the follower rises past the profile's midpoint:
     * for a three-lobed cam that is exactly three per revolution. Counting
     * level crossings rather than local maxima keeps the check immune to
     * sample-level wobble. */
    double mid = m.cams[0].base_radius + m.cams[0].lift * 0.5;
    double worst = 0.0;
    int lobes_seen = 0;
    /* Seed the crossing state from where the follower actually starts (on a
     * lobe, as it happens), so the first sample isn't miscounted as a rise. */
    bool above = vec2_dot(vec2_sub(m.connectors[follower].pos, m.connectors[centre].pos),
                           m.cams[0].axis_dir) > mid;
    for (int step = 0; step < 960; step++) {   /* 90 deg/s -> exactly one turn */
        solver_advance(&m, 1.0 / 240.0, params);
        const Cam *c = &m.cams[0];
        double s_pos = vec2_dot(vec2_sub(m.connectors[follower].pos, m.connectors[centre].pos), c->axis_dir);
        double expected = cam_pitch_radius(c, axis_angle - mechanism_cam_angle(&m, 0));
        double err = fabs(s_pos - expected);
        if (err > worst) worst = err;
        if (!above && s_pos > mid) { lobes_seen++; above = true; }
        else if (above && s_pos < mid) { above = false; }
    }
    check_true("follower rides the drawn profile through a full turn", worst < 0.6);
    check_true("a three-lobed cam lifts the follower three times a turn",
               lobes_seen == 3);

    mechanism_free(&m);
}


/* --------------------------------------------------------------------------
 * Path synthesis (src/synth.c): draw any curve, get a machine that redraws it.
 * ----------------------------------------------------------------------- */

/* A five-pointed star -- sharp corners, which is exactly what a four-bar
 * could never trace and what the arm chain is meant to handle. */
static int build_star(Vec2 *out, double radius)  {
    Vec2 vertex[10];
    for (int i = 0; i < 10; i++) {
        double a = M_PI / 2.0 + 2.0 * M_PI * (double)i / 10.0;
        double r = (i % 2 == 0) ? radius : radius * 0.41;
        vertex[i] = (Vec2){ r * cos(a), r * sin(a) };
    }
    int n = 0;
    for (int i = 0; i < 10; i++) {
        Vec2 a = vertex[i], b = vertex[(i + 1) % 10];
        for (int k = 0; k < 24; k++) {
            double t = (double)k / 24.0;
            out[n++] = vec2_add(a, vec2_scale(vec2_sub(b, a), t));
        }
    }
    return n;
}

static void test_stroke_preparation(void) {
    /* A freehand stroke bunches up wherever the hand slowed; resampling to
     * even arc length is what stops those stretches dominating. */
    Vec2 uneven[5] = { { 0, 0 }, { 1, 0 }, { 2, 0 }, { 3, 0 }, { 100, 0 } };
    Vec2 even[11];
    synth_resample(uneven, 5, false, even, 11);

    check_close("resampling starts at the stroke's start", vec2_dist(even[0], uneven[0]), 0.0, 1e-9);
    check_close("resampling ends at the stroke's end", vec2_dist(even[10], uneven[4]), 0.0, 1e-6);

    bool spacing_even = true;
    double step = vec2_dist(even[0], even[1]);
    for (int i = 1; i < 10; i++) {
        if (fabs(vec2_dist(even[i], even[i + 1]) - step) > 1e-6) spacing_even = false;
    }
    check_true("resampled points are evenly spaced along the stroke", spacing_even);
    check_close("...at the expected spacing", step, 10.0, 1e-6);

    Vec2 loop[17];
    for (int i = 0; i < 17; i++) {
        double a = 2.0 * M_PI * (double)i / 16.0;
        loop[i] = (Vec2){ 50.0 * cos(a), 50.0 * sin(a) };
    }
    check_true("a stroke ending where it began reads as closed", synth_stroke_is_closed(loop, 17));

    Vec2 open_stroke[9];
    for (int i = 0; i < 9; i++) open_stroke[i] = (Vec2){ 10.0 * (double)i, 0.0 };
    check_true("a stroke ending far from its start reads as open",
               !synth_stroke_is_closed(open_stroke, 9));

    check_close("path size is its largest extent", synth_path_size(loop, 17), 100.0, 1e-6);
}

/* ---- The four-bar tool ------------------------------------------------- */

/* The same four-bar family the solver tests use: O2=(0,0), O4=(4,0), crank 1,
 * coupler 3, rocker 2. At a 90-degree crank angle the closed-form answer for
 * B is known exactly, so it pins the kinematics independently of the solver. */
static void test_fourbar_pose_matches_closed_form(void) {
    FourBar fb = { { 0, 0 }, { 4, 0 }, 1.0, 3.0, 2.0, 3.0, 0.0, 1 };
    Vec2 a, b, point;
    check_true("four-bar assembles at 90 degrees",
               fourbar_pose(&fb, M_PI / 2.0, &a, &b, &point));

    check_close("synth A.x", a.x, 0.0, 1e-12);
    check_close("synth A.y", a.y, 1.0, 1e-12);
    check_close("synth B.x", b.x, (44.0 + 4.0 * sqrt(2.0)) / 17.0, 1e-9);
    check_close("synth B.y", b.y, (6.0 + 16.0 * sqrt(2.0)) / 17.0, 1e-9);
    check_close("traced point at u=coupler,v=0 lands on B", vec2_dist(point, b), 0.0, 1e-9);

    fb.branch = -1;
    fourbar_pose(&fb, M_PI / 2.0, &a, &b, &point);
    check_close("the other branch mirrors B.x", b.x, (44.0 - 4.0 * sqrt(2.0)) / 17.0, 1e-9);
    check_close("the other branch mirrors B.y", b.y, (6.0 - 16.0 * sqrt(2.0)) / 17.0, 1e-9);

    FourBar broken = { { 0, 0 }, { 4, 0 }, 1.0, 0.2, 0.2, 0.1, 0.0, 1 };
    check_true("an unassemblable linkage is reported as such",
               !fourbar_coupler_point(&broken, 0.0, &point));
}

static void test_grashof_classification(void) {
    FourBar good = { { 0, 0 }, { 400, 0 }, 100.0, 350.0, 300.0, 180.0, 120.0, 1 };
    check_true("a Grashof crank-rocker turns fully", fourbar_crank_rotates(&good));

    FourBar not_shortest = { { 0, 0 }, { 450, 0 }, 250.0, 452.0, 200.0, 100.0, 50.0, 1 };
    check_true("a crank that isn't the shortest link doesn't turn fully",
               !fourbar_crank_rotates(&not_shortest));

    /* s + l == p + q exactly: a change point, where the linkage can flip
     * branches. Excluded deliberately. */
    FourBar change_point = { { 0, 0 }, { 4, 0 }, 1.0, 3.0, 2.0, 3.0, 0.0, 1 };
    check_true("a change-point linkage is excluded", !fourbar_crank_rotates(&change_point));

    FourBar degenerate = { { 0, 0 }, { 0, 0 }, 1.0, 3.0, 2.0, 1.0, 0.0, 1 };
    check_true("coincident ground pivots are rejected", !fourbar_crank_rotates(&degenerate));
}

/* Small enough to keep the suite quick, large enough to find a good answer. */
static SynthParams test_synth_params(void) {
    SynthParams p = synth_default_params();
    p.random_starts = 60000;
    p.refine_candidates = 16;
    p.refine_sweeps = 80;
    return p;
}

static void test_four_bar_recovers_an_achievable_curve(void) {
    /* Take a curve a four-bar CAN trace, hand the fitter nothing but the
     * points, and check it finds a linkage tracing essentially that curve.
     * The answer need not be the original -- different four-bars share coupler
     * curves -- so the check is on the curve, not the parameters. */
    FourBar truth = { { 0, 0 }, { 400, 0 }, 100.0, 350.0, 300.0, 180.0, 120.0, 1 };
    Vec2 target[64];
    bool traceable = true;
    for (int i = 0; i < 64; i++) {
        if (!fourbar_coupler_point(&truth, 2.0 * M_PI * (double)i / 64.0, &target[i])) traceable = false;
    }
    check_true("the truth curve is traceable all the way round", traceable);
    double span = synth_path_size(target, 64);

    FourBar got;
    double err = 0.0;
    check_true("the four-bar tool finds a linkage for an achievable curve",
               synth_fit_four_bar(target, 64, true, test_synth_params(), &got, &err));
    check_true("the fitted linkage's crank turns fully", fourbar_crank_rotates(&got));
    check_true("the fit is within a few percent of the curve's size", err < 0.06 * span);
    check_close("reported error matches a fresh evaluation",
                synth_fit_error(&got, target, 64, true), err, 1e-9);
    check_true("the original linkage scores near-zero on its own curve",
               synth_fit_error(&truth, target, 64, true) < 0.01 * span);
}

static void test_four_bar_is_deterministic(void) {
    FourBar truth = { { 0, 0 }, { 400, 0 }, 100.0, 350.0, 300.0, 180.0, 120.0, 1 };
    Vec2 target[48];
    for (int i = 0; i < 48; i++) {
        fourbar_coupler_point(&truth, 2.0 * M_PI * (double)i / 48.0, &target[i]);
    }
    SynthParams p = test_synth_params();
    p.random_starts = 20000;
    FourBar a, b;
    double ea = 0.0, eb = 0.0;
    bool ok_a = synth_fit_four_bar(target, 48, true, p, &a, &ea);
    bool ok_b = synth_fit_four_bar(target, 48, true, p, &b, &eb);
    check_true("both runs succeed", ok_a && ok_b);
    check_close("the same drawing gives the same error", eb, ea, 1e-12);
    check_close("...and the same crank", b.crank, a.crank, 1e-12);
}

/* The whole reason both tools exist: a star is outside what a four-bar can
 * trace, and the tool should return a visibly poor fit rather than pretend --
 * which is exactly the case where the app points you at ARMS. */
static void test_four_bar_cannot_manage_a_star(void) {
    Vec2 star[256];
    int n = build_star(star, 110.0);
    Vec2 target[64];
    synth_resample(star, n, true, target, 64);
    double size = synth_path_size(target, 64);

    FourBar got;
    double err = 0.0;
    bool ok = synth_fit_four_bar(target, 64, true, test_synth_params(), &got, &err);
    check_true("the four-bar tool still returns its best effort on a star", ok);
    check_true("but a star is well outside what a four-bar can trace",
               err > 0.05 * size);

    /* The arm chain, on the same star, gets nowhere near that badly wrong. */
    Vec2 anchor;
    FourierArm arms[SYNTH_MAX_ARMS];
    double rms = 0.0;
    int arm_count = synth_fourier_fit(star, n, true, 32, 0.0, &anchor, arms, &rms);
    check_true("the arm chain handles the same star", arm_count == 32);
    check_true("and does so far more accurately than the four-bar", rms < err * 0.2);
}

/* ---- The arm-chain tool ------------------------------------------------ */

static void test_fourier_reproduces_a_circle_exactly(void) {
    /* A circle is a single rotating arm, and it is the one shape that stays a
     * single arm under arc-length resampling, so it pins the decomposition
     * exactly. (Sampled finely: the stroke is a polygon, and its chords sag
     * very slightly inside the true circle.) */
    Vec2 circle[512];
    for (int i = 0; i < 512; i++) {
        double a = 2.0 * M_PI * (double)i / 512.0;
        circle[i] = (Vec2){ 200.0 + 80.0 * cos(a), 150.0 + 80.0 * sin(a) };
    }

    Vec2 anchor;
    FourierArm arms[SYNTH_MAX_ARMS];
    double rms = 0.0;
    int count = synth_fourier_fit(circle, 512, true, SYNTH_MAX_ARMS, 0.05, &anchor, arms, &rms);

    check_true("a circle needs exactly one arm", count == 1);
    check_close("the chain hangs from the circle's centre.x", anchor.x, 200.0, 1e-4);
    check_close("the chain hangs from the circle's centre.y", anchor.y, 150.0, 1e-4);
    check_close("the arm is the circle's radius", vec2_len(arms[0].amplitude), 80.0, 0.01);
    check_true("the arm turns once per cycle", abs(arms[0].harmonic) == 1);
    check_true("the deviation is negligible", rms < 0.05);
}

static void test_fourier_traces_a_star(void) {
    /* The shape the user actually asked for. Sharp corners need many terms,
     * so this checks both that the error falls as arms are added and that the
     * final machine really does follow the drawing. */
    Vec2 star[256];
    int n = build_star(star, 110.0);
    double size = synth_path_size(star, n);

    Vec2 anchor;
    FourierArm arms[SYNTH_MAX_ARMS];
    double coarse = 0.0, fine = 0.0;
    int few = synth_fourier_fit(star, n, true, 6, 0.0, &anchor, arms, &coarse);
    int many = synth_fourier_fit(star, n, true, 32, 0.0, &anchor, arms, &fine);

    check_true("a small budget uses every arm it is given", few == 6);
    check_true("a larger budget uses more arms", many == 32);
    check_true("more arms means less deviation", fine < coarse);
    check_true("32 arms trace a star to well under a percent", fine < 0.01 * size);

    /* Every point of the drawing should lie on the curve the machine draws. */
    double worst = 0.0;
    for (int i = 0; i < n; i++) {
        double best = 1e300;
        for (int k = 0; k < 720; k++) {
            Vec2 p = synth_fourier_point(anchor, arms, many, (double)k / 720.0);
            best = fmin(best, vec2_dist(star[i], p));
        }
        if (best > worst) worst = best;
    }
    check_true("every point of the star lands on the traced curve", worst < 0.02 * size);
}

static void test_fourier_stops_early_when_accurate_enough(void) {
    /* A gentle shape should not spend the whole budget. Note an ellipse is
     * NOT two arms here: the stroke is resampled to even arc length, which is
     * right for a hand drawing but is a different parameterisation from the
     * even-angle one that makes an ellipse two harmonics. */
    Vec2 ellipse[256];
    for (int i = 0; i < 256; i++) {
        double a = 2.0 * M_PI * (double)i / 256.0;
        ellipse[i] = (Vec2){ 120.0 * cos(a), 60.0 * sin(a) };
    }
    double size = synth_path_size(ellipse, 256);

    Vec2 anchor;
    FourierArm arms[SYNTH_MAX_ARMS];
    double rms = 0.0;
    int count = synth_fourier_fit(ellipse, 256, true, SYNTH_MAX_ARMS, size * 0.002, &anchor, arms, &rms);
    check_true("a gentle shape stops well inside the arm budget", count < SYNTH_MAX_ARMS / 2);
    check_true("having met the tolerance asked for", rms <= size * 0.002);

    bool descending = true;
    for (int i = 1; i < count; i++) {
        if (vec2_len(arms[i].amplitude) > vec2_len(arms[i - 1].amplitude) + 1e-12) descending = false;
    }
    check_true("arms come out longest first", descending);
}

static void test_fourier_handles_an_open_stroke(void) {
    /* An open stroke is mirrored into a closed cycle, so the pen sweeps out
     * along it and back. Closing it with a straight jump instead would put a
     * step in the curve that no sane number of arms could represent. */
    Vec2 wave[64];
    for (int i = 0; i < 64; i++) {
        double t = (double)i / 63.0;
        wave[i] = (Vec2){ 300.0 * t, 40.0 * sin(3.0 * M_PI * t) };
    }
    double size = synth_path_size(wave, 64);

    Vec2 anchor;
    FourierArm arms[SYNTH_MAX_ARMS];
    double rms = 0.0;
    int count = synth_fourier_fit(wave, 64, false, SYNTH_MAX_ARMS, size * 0.002, &anchor, arms, &rms);
    check_true("an open stroke decomposes", count > 0);

    double worst = 0.0;
    for (int i = 0; i < 64; i++) {
        double best = 1e300;
        for (int k = 0; k < 720; k++) {
            Vec2 p = synth_fourier_point(anchor, arms, count, (double)k / 720.0);
            best = fmin(best, vec2_dist(wave[i], p));
        }
        if (best > worst) worst = best;
    }
    check_true("the pen passes along every point of an open stroke", worst < 0.02 * size);
}

static void test_fourier_rejects_useless_input(void) {
    Vec2 anchor;
    FourierArm arms[SYNTH_MAX_ARMS];
    double rms = 0.0;
    Vec2 two[2] = { { 0, 0 }, { 10, 0 } };
    check_true("too few points is rejected",
               synth_fourier_fit(two, 2, true, SYNTH_MAX_ARMS, 0.1, &anchor, arms, &rms) == 0);

    Vec2 dot[8];
    for (int i = 0; i < 8; i++) dot[i] = (Vec2){ 5.0, 5.0 };
    check_true("a path with no extent is rejected",
               synth_fourier_fit(dot, 8, true, SYNTH_MAX_ARMS, 0.1, &anchor, arms, &rms) == 0);
}

/* The end-to-end statement: build the machine out of real mechanism parts,
 * simulate it, and check the traced point follows the drawing. This is what
 * exercises the solver's chained motors -- each arm pivots on the tip of the
 * one before, not on ground. */
static void test_arm_chain_simulates_and_traces_the_path(void) {
    Vec2 star[256];
    int n = build_star(star, 110.0);
    double size = synth_path_size(star, n);

    Vec2 anchor;
    FourierArm arms[SYNTH_MAX_ARMS];
    double rms = 0.0;
    int count = synth_fourier_fit(star, n, true, 24, 0.0, &anchor, arms, &rms);
    check_true("the star decomposes", count == 24);

    Mechanism m;
    mechanism_init(&m);
    int previous = mechanism_add_connector(&m, anchor, true);
    Vec2 tip = anchor;
    double base_speed = 45.0;
    for (int i = 0; i < count; i++) {
        tip = vec2_add(tip, arms[i].amplitude);
        int t = mechanism_add_connector(&m, tip, false);
        int ids[2] = { previous, t };
        int link = mechanism_add_link(&m, ids, 2);
        check_true_quiet(mechanism_set_driven_about(&m, link, previous,
                                                     (double)arms[i].harmonic * base_speed));
        previous = t;
    }
    int pen = previous;

    SolverParams params = solver_default_params();
    solver_freeze(&m);

    /* One cycle is one turn of the slowest arm: 360/45 = 8 seconds. */
    const double dt = 1.0 / 240.0;
    const int steps = (int)(8.0 / dt);
    Vec2 drawn[2000];
    int drawn_count = 0;
    for (int step = 0; step < steps; step++) {
        solver_advance(&m, dt, params);
        if (step % 2 == 0 && drawn_count < 2000) drawn[drawn_count++] = m.connectors[pen].pos;
    }

    /* Every point of the star should have been passed through. */
    double worst = 0.0;
    for (int i = 0; i < n; i++) {
        double best = 1e300;
        for (int k = 0; k < drawn_count; k++) best = fmin(best, vec2_dist(star[i], drawn[k]));
        if (best > worst) worst = best;
    }
    check_true("the simulated machine draws the star", worst < 0.03 * size);

    /* And it must go all the way round once -- a machine that merely wobbled
     * near the figure could pass a sloppier version of the check above. The
     * pen's travel in one cycle should be the star's perimeter. */
    double perimeter = 0.0;
    for (int i = 0; i < n; i++) perimeter += vec2_dist(star[i], star[(i + 1) % n]);
    double travel = 0.0;
    for (int k = 1; k < drawn_count; k++) travel += vec2_dist(drawn[k - 1], drawn[k]);
    check_true("the pen travels one full circuit of the figure",
               travel > 0.9 * perimeter && travel < 1.15 * perimeter);

    mechanism_free(&m);
}


/* --------------------------------------------------------------------------
 * Sliders, gears, Geneva wheels (src/joints.c) and the template catalogue.
 * ----------------------------------------------------------------------- */

static void test_geneva_indexes_exactly(void) {
    /* The whole point of a Geneva: each turn of the driver advances the wheel
     * by exactly one slot, and it sits perfectly still in between. */
    for (int slots = 3; slots <= 8; slots++) {
        double step = 2.0 * M_PI / (double)slots;
        bool exact = true, still = true, smooth = true;
        double prev = geneva_wheel_angle(slots, -2.0 * M_PI, NULL);

        for (int k = 1; k <= 5; k++) {
            /* Mid-dwell, where the wheel must be exactly k steps along. */
            double w = geneva_wheel_angle(slots, (double)(k - 1) * 2.0 * M_PI + M_PI, NULL);
            /* Negative: an external Geneva turns its wheel the other way. */
            if (fabs(w + (double)k * step) > 1e-9) exact = false;
        }
        for (double a = -2.0 * M_PI; a <= 4.0 * M_PI; a += 0.001) {
            bool eng;
            double w = geneva_wheel_angle(slots, a, &eng);
            if (fabs(w - prev) > 0.02) smooth = false;   /* no jumps */
            if (!eng) {
                /* Locked means parked exactly on an index position, which is
                 * the property that matters (and doesn't depend on where the
                 * sample happens to fall relative to the engagement edge). */
                double indexes = w / step;
                if (fabs(indexes - floor(indexes + 0.5)) > 1e-9) still = false;
            }
            prev = w;
        }
        check_true("a Geneva advances exactly one slot per driver turn", exact);
        check_true("...and does so opposite to the driver",
                   geneva_wheel_angle(slots, 2.0 * M_PI + M_PI, NULL) < 0.0);
        check_true("reversing the driver reverses the wheel",
                   geneva_wheel_angle(slots, -(2.0 * M_PI + M_PI), NULL) > 0.0);
        check_true("...and is perfectly still while locked", still);
        check_true("...with no jump anywhere in the cycle", smooth);
    }

    /* The shock-free proportions: the pin enters and leaves along the slot. */
    check_close("a 6-slot Geneva's crank is half its centre distance",
                geneva_crank_radius(6, 100.0), 50.0, 1e-9);
    check_close("...and the two are inverses",
                geneva_center_distance(6, geneva_crank_radius(6, 100.0)), 100.0, 1e-9);
    check_close("a 4-slot Geneva engages for 90 degrees either side",
                geneva_engagement_half_angle(4), M_PI / 4.0, 1e-12);
}

static void test_crank_slider_gives_exactly_twice_the_crank(void) {
    /* A slider on a rail through the crank centre travels exactly one crank
     * diameter, and never leaves its rail. */
    Mechanism m;
    mechanism_init(&m);
    int o    = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int a    = mechanism_add_connector(&m, (Vec2){ 60, 0 }, false);
    int pist = mechanism_add_connector(&m, (Vec2){ 260, 0 }, false);
    int r1   = mechanism_add_connector(&m, (Vec2){ -400, 0 }, true);
    int r2   = mechanism_add_connector(&m, (Vec2){ 400, 0 }, true);
    int crank_ids[2] = { o, a }, rod_ids[2] = { a, pist };
    mechanism_add_link(&m, crank_ids, 2);
    mechanism_add_link(&m, rod_ids, 2);
    mechanism_toggle_driven(&m, 0, 90.0);
    check_true("the slider is accepted", mechanism_add_slider(&m, pist, r1, r2) >= 0);

    SolverParams p = solver_default_params();
    solver_freeze(&m);
    double lo = 1e30, hi = -1e30, off = 0.0;
    for (int k = 0; k < 960; k++) {
        solver_advance(&m, 1.0 / 240.0, p);
        Vec2 q = m.connectors[pist].pos;
        lo = fmin(lo, q.x); hi = fmax(hi, q.x);
        off = fmax(off, fabs(q.y));
    }
    check_close("the piston's stroke is twice the crank", hi - lo, 120.0, 1e-3);
    check_true("the piston never leaves its rail", off < 1e-6);

    /* Degenerate rails are refused rather than dividing by zero. */
    check_true("a zero-length rail is rejected", mechanism_add_slider(&m, pist, r1, r1) < 0);
    mechanism_free(&m);
}

/* A rack travels the arc length rolled off its pinion. */
static void test_rack_travels_the_arc_it_rolls(void) {
    Mechanism m;
    mechanism_init(&m);
    int pc = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int pt = mechanism_add_connector(&m, (Vec2){ 50, 0 }, false);
    int ra = mechanism_add_connector(&m, (Vec2){ -150, 50 }, false);
    int rb = mechanism_add_connector(&m, (Vec2){ 150, 50 }, false);
    int pin_ids[2] = { pc, pt }, rack_ids[2] = { ra, rb };
    mechanism_add_link(&m, pin_ids, 2);
    mechanism_add_link(&m, rack_ids, 2);
    mechanism_toggle_driven(&m, 0, 90.0);
    check_true("the rack is accepted",
               mechanism_add_rack(&m, 0, pc, 1, ra, (Vec2){ 1, 0 }) >= 0);

    SolverParams p = solver_default_params();
    solver_freeze(&m);
    Vec2 start = m.connectors[ra].pos;
    for (int k = 0; k < 240; k++) solver_advance(&m, 1.0 / 240.0, p);
    Vec2 now = m.connectors[ra].pos;
    check_close("the rack travels the arc rolled off the pitch circle",
                now.x - start.x, 50.0 * M_PI / 2.0, 1e-6);
    check_close("...and doesn't drift off its axis", now.y - start.y, 0.0, 1e-9);
    mechanism_free(&m);
}

/* A wheel is a body with a size of its own: put down on its own, resized on
 * its own, meshed only when asked. That is what stops one button press from
 * dropping a wheel on top of the last, and what lets one wheel drive several. */
static void test_wheels_are_placed_and_sized_on_their_own(void) {
    Mechanism m;
    mechanism_init(&m);
    int w = mechanism_add_wheel(&m, (Vec2){ 0, 0 }, 80.0);
    check_true("a wheel can be put down on its own", w >= 0);
    check_true("it is a wheel, not a bar", mechanism_is_gear_body(&m, w));
    check_true("nothing is meshed with it", m.gear_count == 0);
    check_close("it is the size it was asked for",
                mechanism_gear_wheel_radius(&m, w, NULL), 80.0, 1e-12);

    int centre = -1;
    (void)mechanism_gear_wheel_radius(&m, w, &centre);
    check_true("its hub is grounded", centre >= 0 && m.connectors[centre].is_anchor);
    int mark = mechanism_wheel_mark(&m, w);
    check_true("it carries a mark on its rim", mark >= 0 && mark != centre);
    check_close("...which sits on the pitch circle, so a trace is the wheel itself",
                vec2_dist(m.connectors[mark].pos, m.connectors[centre].pos), 80.0, 1e-9);
    mechanism_set_traced(&m, mark, true);
    check_true("...which can be traced like any other point", m.connectors[mark].traced);

    /* Resizing moves the mark with the face rather than stranding it. */
    mechanism_set_wheel_radius(&m, w, 160.0);
    check_close("resizing changes the radius", m.links[w].wheel_radius, 160.0, 1e-12);
    check_close("...and the rim mark rides out with it, still on the rim",
                vec2_dist(m.connectors[mark].pos, m.connectors[centre].pos), 160.0, 1e-9);
    check_true("...and the body is still rigid at its new size",
               fabs(m.links[w].rest_dist[0] - 160.0) < 1e-9);

    mechanism_set_wheel_radius(&m, w, 1.0);
    check_close("a wheel cannot be shrunk to nothing",
                m.links[w].wheel_radius, MECHANISM_WHEEL_MIN_RADIUS, 1e-12);
    mechanism_free(&m);
}

/* Meshing is a separate act, and it makes the mesh real: two wheels of fixed
 * size touch at exactly one distance, so the driven one slides into contact
 * however far apart they were drawn. */
static void test_meshing_slides_wheels_into_contact(void) {
    Mechanism m;
    mechanism_init(&m);
    int a = mechanism_add_wheel(&m, (Vec2){ 0, 0 }, 80.0);
    int b = mechanism_add_wheel(&m, (Vec2){ 400, 0 }, 120.0);   /* far too far apart */
    mechanism_toggle_driven(&m, a, 90.0);

    check_true("the two mesh", mechanism_mesh_wheels(&m, a, b) >= 0);
    int ca = -1, cb = -1;
    (void)mechanism_gear_wheel_radius(&m, a, &ca);
    (void)mechanism_gear_wheel_radius(&m, b, &cb);
    check_close("the driven wheel slid in until they touch",
                vec2_dist(m.connectors[ca].pos, m.connectors[cb].pos), 200.0, 1e-9);
    check_close("each wheel kept its own size", m.gears[0].driver_radius, 80.0, 1e-12);
    check_close("...both of them", m.gears[0].driven_radius, 120.0, 1e-12);
    check_true("meshing the same two again is refused", mechanism_mesh_wheels(&m, a, b) < 0);
    check_true("...in either order", mechanism_mesh_wheels(&m, b, a) < 0);

    /* Resizing a meshed wheel closes the gap again by itself. */
    mechanism_set_wheel_radius(&m, b, 60.0);
    mechanism_refresh_joint_sizes(&m);
    check_close("resizing a meshed wheel keeps the mesh",
                vec2_dist(m.connectors[ca].pos, m.connectors[cb].pos), 140.0, 1e-9);

    /* And it turns at the ratio its size dictates. */
    SolverParams p = solver_default_params();
    solver_freeze(&m);
    int mark = mechanism_wheel_mark(&m, b);
    Vec2 d0 = vec2_sub(m.connectors[mark].pos, m.connectors[cb].pos);
    double start = atan2(d0.y, d0.x);
    for (int k = 0; k < 240; k++) solver_advance(&m, 1.0 / 240.0, p);
    Vec2 d1 = vec2_sub(m.connectors[mark].pos, m.connectors[cb].pos);
    double turned = (atan2(d1.y, d1.x) - start) * 180.0 / M_PI;
    while (turned > 180.0) turned -= 360.0;      /* the same angle, read once round */
    while (turned < -180.0) turned += 360.0;
    check_close("an 80 driving a 60 turns it 4/3 as far, reversed", turned, -120.0, 1e-6);
    mechanism_free(&m);
}

/* One wheel can drive several, because meshing names the wheels rather than
 * guessing; and naming any wheel the driver turns the train round to suit. */
static void test_one_wheel_drives_several(void) {
    Mechanism m;
    mechanism_init(&m);
    int hub = mechanism_add_wheel(&m, (Vec2){ 0, 0 }, 120.0);
    int left = mechanism_add_wheel(&m, (Vec2){ -260, 0 }, 80.0);
    int right = mechanism_add_wheel(&m, (Vec2){ 260, 0 }, 80.0);
    int below = mechanism_add_wheel(&m, (Vec2){ 0, 260 }, 80.0);
    mechanism_toggle_driven(&m, hub, 90.0);

    check_true("the hub drives one wheel", mechanism_mesh_wheels(&m, hub, left) >= 0);
    check_true("...and a second", mechanism_mesh_wheels(&m, hub, right) >= 0);
    check_true("...and a third", mechanism_mesh_wheels(&m, hub, below) >= 0);
    check_true("three wheels share it", m.gear_count == 3);
    check_true("a wheel already turned by something else is refused",
               mechanism_mesh_wheels(&m, left, right) < 0);

    mechanism_refresh_joint_sizes(&m);
    SolverParams p = solver_default_params();
    solver_freeze(&m);
    int marks[3] = { mechanism_wheel_mark(&m, left), mechanism_wheel_mark(&m, right),
                     mechanism_wheel_mark(&m, below) };
    Vec2 was[3];
    for (int k = 0; k < 3; k++) was[k] = m.connectors[marks[k]].pos;
    for (int k = 0; k < 240; k++) solver_advance(&m, 1.0 / 240.0, p);
    bool all_turned = true;
    for (int k = 0; k < 3; k++) {
        if (vec2_dist(m.connectors[marks[k]].pos, was[k]) < 1.0) all_turned = false;
    }
    check_true("all three are driven from the one hub", all_turned);

    /* Now name an outer wheel the driver: the mesh it sits in must turn round. */
    check_true("the train can be re-rooted", mechanism_orient_train_from(&m, left));
    check_true("the chosen wheel is no longer driven by another",
               !m.links[left].driven_externally);
    check_true("...and the old hub now takes its motion from it",
               m.links[hub].driven_externally);
    bool from_left = false;
    for (int i = 0; i < m.gear_count; i++) {
        if (m.gears[i].driver_link_id == left && m.gears[i].driven_link_id == hub) from_left = true;
    }
    check_true("the mesh between them points outwards from it", from_left);
    mechanism_free(&m);
}

/* A Geneva's crank radius is fixed by its slot count and centre distance, so
 * both follow the geometry -- change either and it stays shock-free. */
static void test_geneva_sizes_follow_the_geometry(void) {
    Mechanism m;
    mechanism_init(&m);
    int dc = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int pin = mechanism_add_connector(&m, (Vec2){ 0, -60 }, false);
    int wc = mechanism_add_connector(&m, (Vec2){ 200, 0 }, true);
    int mk = mechanism_add_connector(&m, (Vec2){ 280, 0 }, false);
    int da[2] = { dc, pin }, wa[2] = { wc, mk };
    mechanism_add_link(&m, da, 2);
    mechanism_add_link(&m, wa, 2);
    mechanism_toggle_driven(&m, 0, 90.0);
    int vi = mechanism_add_geneva(&m, 0, dc, 1, wc, 6);

    check_close("the centre distance is read off the two centres",
                m.genevas[vi].center_distance, 200.0, 1e-9);
    check_close("the crank radius is the shock-free one for six slots",
                m.genevas[vi].crank_radius, geneva_crank_radius(6, 200.0), 1e-9);

    m.connectors[wc].pos = (Vec2){ 300, 0 };
    mechanism_refresh_joint_sizes(&m);
    check_close("moving the wheel restretches the centre distance",
                m.genevas[vi].center_distance, 300.0, 1e-9);
    check_close("...and the crank with it",
                m.genevas[vi].crank_radius, geneva_crank_radius(6, 300.0), 1e-9);

    m.genevas[vi].slot_count = 4;
    mechanism_refresh_joint_sizes(&m);
    check_close("fewer slots means a longer crank",
                m.genevas[vi].crank_radius, geneva_crank_radius(4, 300.0), 1e-9);
    check_true("...which is indeed longer",
               m.genevas[vi].crank_radius > geneva_crank_radius(6, 300.0));
    mechanism_free(&m);
}

/* Each joint can be clicked in its own right -- a wheel anywhere on its face,
 * a slider on its rail, a Geneva on its rim -- which is how it is reached. */
static void test_joints_can_be_picked(void) {
    Mechanism m;
    mechanism_init(&m);
    int wa = mechanism_add_wheel(&m, (Vec2){ 0, 0 }, 40.0);
    int wb = mechanism_add_wheel(&m, (Vec2){ 120, 0 }, 80.0);
    check_true("a click on a wheel's rim finds that wheel",
               mechanism_pick_wheel(&m, (Vec2){ 0, 40 }, 6.0) == wa);
    check_true("a click on the other finds the other one",
               mechanism_pick_wheel(&m, (Vec2){ 120, -80 }, 6.0) == wb);
    check_true("a click well inside a wheel still finds it -- it is a disc",
               mechanism_pick_wheel(&m, (Vec2){ 150, 30 }, 6.0) == wb);
    check_true("a click in open space finds nothing",
               mechanism_pick_wheel(&m, (Vec2){ 0, 400 }, 6.0) < 0);
    mechanism_free(&m);

    mechanism_init(&m);
    int pist = mechanism_add_connector(&m, (Vec2){ 0, 0 }, false);
    int ra = mechanism_add_connector(&m, (Vec2){ -100, 0 }, true);
    int rb = mechanism_add_connector(&m, (Vec2){ 100, 0 }, true);
    int sid = mechanism_add_slider(&m, pist, ra, rb);
    check_true("a click along the rail finds the slider",
               mechanism_pick_slider(&m, (Vec2){ 40, 3 }, 7.0) == sid);
    check_true("a click well off it doesn't",
               mechanism_pick_slider(&m, (Vec2){ 40, 60 }, 7.0) < 0);
    mechanism_free(&m);

    mechanism_init(&m);
    int dc = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int pin = mechanism_add_connector(&m, (Vec2){ 0, -60 }, false);
    int wc = mechanism_add_connector(&m, (Vec2){ 200, 0 }, true);
    int mk = mechanism_add_connector(&m, (Vec2){ 280, 0 }, false);
    int da[2] = { dc, pin }, wl[2] = { wc, mk };
    mechanism_add_link(&m, da, 2);
    mechanism_add_link(&m, wl, 2);
    mechanism_toggle_driven(&m, 0, 90.0);
    int vi = mechanism_add_geneva(&m, 0, dc, 1, wc, 6);
    double wr = mechanism_geneva_wheel_radius(&m.genevas[vi]);
    check_true("a click on the Geneva wheel's rim finds it",
               mechanism_pick_geneva(&m, (Vec2){ 200, wr }, 7.0) == vi);
    check_true("a click between the two centres finds nothing",
               mechanism_pick_geneva(&m, (Vec2){ 120, 0 }, 7.0) < 0);
    mechanism_free(&m);
}

/* The gear train, crank-slider and Geneva used by the tests below. These used
 * to be lifted from the template catalogue; they are built here by hand now
 * that those three are made by their toolbar buttons instead -- which is also
 * a truer test, since it is the same sequence of calls the buttons make. */
static int build_test_gear_train(Mechanism *m) {
    double r1 = 70, r2 = 45, r3 = 90;
    int c1 = mechanism_add_connector(m, (Vec2){ -160, 0 }, true);
    int t1 = mechanism_add_connector(m, (Vec2){ -160 + r1, 0 }, false);
    int c2 = mechanism_add_connector(m, (Vec2){ -160 + r1 + r2, 0 }, true);
    int t2 = mechanism_add_connector(m, (Vec2){ -160 + r1 + 2 * r2, 0 }, false);
    int c3 = mechanism_add_connector(m, (Vec2){ -160 + r1 + 2 * r2 + r3, 0 }, true);
    int t3 = mechanism_add_connector(m, (Vec2){ -160 + r1 + 2 * r2 + 2 * r3, 0 }, false);
    int a[2] = { c1, t1 }, b[2] = { c2, t2 }, c[2] = { c3, t3 };
    int g1 = mechanism_add_link(m, a, 2);
    int g2 = mechanism_add_link(m, b, 2);
    int g3 = mechanism_add_link(m, c, 2);
    mechanism_toggle_driven(m, g1, 90.0);
    mechanism_add_gear(m, g1, c1, g2, c2, r2 / r1, false);
    mechanism_add_gear(m, g2, c2, g3, c3, r3 / r2, false);
    mechanism_set_traced(m, t3, true);
    return t3;
}

static void build_test_crank_slider(Mechanism *m) {
    int o    = mechanism_add_connector(m, (Vec2){ -140, 0 }, true);
    int a    = mechanism_add_connector(m, (Vec2){ -80, 0 }, false);
    int pist = mechanism_add_connector(m, (Vec2){ 120, 0 }, false);
    int r1   = mechanism_add_connector(m, (Vec2){ -20, 0 }, true);
    int r2   = mechanism_add_connector(m, (Vec2){ 260, 0 }, true);
    int crank_ids[2] = { o, a }, rod_ids[2] = { a, pist };
    int crank = mechanism_add_link(m, crank_ids, 2);
    mechanism_add_link(m, rod_ids, 2);
    mechanism_toggle_driven(m, crank, 90.0);
    mechanism_add_slider(m, pist, r1, r2);
    mechanism_set_traced(m, pist, true);
}

static void build_test_geneva(Mechanism *m) {
    int slots = 6;
    double c_dist = 150.0;
    double a = geneva_crank_radius(slots, c_dist);
    double entry = geneva_engagement_half_angle(slots);
    int dc  = mechanism_add_connector(m, (Vec2){ -c_dist / 2, 0 }, true);
    int pin = mechanism_add_connector(m, (Vec2){ -c_dist / 2 + a * cos(entry),
                                                 -a * sin(entry) }, false);
    int wc  = mechanism_add_connector(m, (Vec2){ c_dist / 2, 0 }, true);
    int mk  = mechanism_add_connector(m, (Vec2){ c_dist / 2 - c_dist * 0.55, 0 }, false);
    int d[2] = { dc, pin }, w[2] = { wc, mk };
    int driver = mechanism_add_link(m, d, 2);
    int wheel = mechanism_add_link(m, w, 2);
    mechanism_toggle_driven(m, driver, 90.0);
    mechanism_add_geneva(m, driver, dc, wheel, wc, slots);
    mechanism_set_traced(m, mk, true);
}

/* Pressing GEAR again adds another wheel to the end of the train. Each one has
 * to mesh with the last, keep the chain in a line, and carry the drive on
 * through -- a train that merely looks right but doesn't turn is no use. */
static void test_gear_chain_runs_smoothly(void) {
    Mechanism m;
    mechanism_init(&m);
    build_test_gear_train(&m);
    check_true("the train has two meshes", m.gear_count == 2);

    int rim = -1;
    for (int c = 0; c < m.connector_count; c++) {
        if (m.connectors[c].alive && m.connectors[c].traced) rim = c;
    }
    check_true("the last gear's rim is traced", rim >= 0);

    SolverParams p = solver_default_params();
    solver_freeze(&m);

    /* Ten seconds, long enough for the middle gear to wrap several times. */
    const double dt = 1.0 / 240.0;
    Vec2 prev = m.connectors[rim].pos;
    double worst = 0.0, total = 0.0;
    for (int k = 0; k < 2400; k++) {
        solver_advance(&m, dt, p);
        double step = vec2_dist(prev, m.connectors[rim].pos);
        if (step > worst) worst = step;
        total += step;
        prev = m.connectors[rim].pos;
    }
    double mean = total / 2400.0;

    check_true("the last gear moves at all", total > 100.0);
    /* A smooth constant rotation moves the same distance every frame; a wrap
     * would show up as one frame moving many times the rest. */
    check_true("the last gear never jumps", worst < mean * 1.5);

    mechanism_free(&m);
}

static void test_geneva_drives_a_real_wheel(void) {
    /* End to end through the solver, not just the closed form: build the
     * driver and wheel out of parts and check the wheel steps and dwells. */
    Mechanism m;
    mechanism_init(&m);
    int slots = 6;
    double c = 150.0, a = geneva_crank_radius(slots, c);
    double entry = geneva_engagement_half_angle(slots);
    int dc = mechanism_add_connector(&m, (Vec2){ 0, 0 }, true);
    int pin = mechanism_add_connector(&m, (Vec2){ a * cos(entry), -a * sin(entry) }, false);
    int wc = mechanism_add_connector(&m, (Vec2){ c, 0 }, true);
    int mark = mechanism_add_connector(&m, (Vec2){ c + 80, 0 }, false);  /* starts at angle 0 */
    int drv[2] = { dc, pin }, whl[2] = { wc, mark };
    mechanism_add_link(&m, drv, 2);
    mechanism_add_link(&m, whl, 2);
    mechanism_toggle_driven(&m, 0, 90.0);
    check_true("the Geneva is accepted", mechanism_add_geneva(&m, 0, dc, 1, wc, slots) >= 0);

    SolverParams p = solver_default_params();
    solver_freeze(&m);

    /* 90 deg/s, so four seconds is one driver turn and one index. */
    double dt = 1.0 / 240.0;
    int still_frames = 0, moving_frames = 0;
    Vec2 prev = m.connectors[mark].pos;
    for (int k = 0; k < 960; k++) {
        solver_advance(&m, dt, p);
        double moved = vec2_dist(prev, m.connectors[mark].pos);
        if (moved < 1e-9) still_frames++; else moving_frames++;
        prev = m.connectors[mark].pos;
    }
    Vec2 d = vec2_sub(m.connectors[mark].pos, m.connectors[wc].pos);
    /* The driver runs forwards, so the wheel must have gone BACK one sixth --
     * an external Geneva reverses, which is what the pin sweeping past the
     * line of centres does to the slot it sits in. */
    check_close("one driver turn indexes the wheel exactly one sixth, the other way",
                atan2(d.y, d.x), -2.0 * M_PI / 6.0, 1e-6);
    check_true("the wheel keeps its radius", fabs(vec2_len(d) - 80.0) < 1e-9);
    check_true("the wheel is locked still for most of the turn",
               still_frames > moving_frames);
    check_true("...but does move for part of it", moving_frames > 100);
    mechanism_free(&m);
}

/* Every template must build, run for six seconds without binding, and
 * actually move the point it traces. A template that jams is worse than no
 * template at all. */
static void test_every_template_runs(void) {
    check_true("the catalogue holds nine mechanisms", templates_count() == 9);

    bool all_run = true, all_move = true, all_named = true;
    for (int i = 0; i < templates_count(); i++) {
        const Template *t = templates_get(i);
        if (!t || !t->name || !t->name[0] || !t->blurb || !t->blurb[0]) { all_named = false; continue; }

        Mechanism m;
        mechanism_init(&m);
        t->build(&m, (Vec2){ 0, 0 }, 1.0);

        int traced = -1;
        for (int c = 0; c < m.connector_count; c++) {
            if (m.connectors[c].alive && m.connectors[c].traced) traced = c;
        }
        if (traced < 0) all_move = false;

        SolverParams p = solver_default_params();
        solver_freeze(&m);
        double travel = 0.0;
        Vec2 prev = (traced >= 0) ? m.connectors[traced].pos : (Vec2){ 0, 0 };
        for (int k = 0; k < 1440; k++) {
            solver_advance(&m, 1.0 / 240.0, p);
            if (solver_has_length_violation(&m, p.length_tol_abs, p.length_tol_rel)) {
                all_run = false;
                break;
            }
            if (traced >= 0) {
                travel += vec2_dist(prev, m.connectors[traced].pos);
                prev = m.connectors[traced].pos;
            }
        }
        if (travel < 10.0) all_move = false;
        mechanism_free(&m);
    }
    check_true("every template has a name and a description", all_named);
    check_true("every template runs six seconds without binding", all_run);
    check_true("every template's traced point actually moves", all_move);
}

static void test_joint_deletion_cascades(void) {
    /* The new elements are defined by links and connectors, so removing one
     * has to take the element with it or the mechanism is left referring to
     * something that no longer exists. */
    Mechanism m;
    mechanism_init(&m);
    build_test_gear_train(&m);
    check_true("the gear train has gears", m.gear_count == 2);
    mechanism_delete_link(&m, 1);                        /* the idler */
    check_true("deleting a gear's link deletes the gear pairs that used it",
               !m.gears[0].alive && !m.gears[1].alive);
    mechanism_free(&m);

    mechanism_init(&m);
    build_test_crank_slider(&m);
    check_true("the crank-slider has a slider", m.slider_count == 1);
    int rail = m.sliders[0].rail_a_id;
    mechanism_delete_connector(&m, rail);
    check_true("deleting a rail point deletes the slider", !m.sliders[0].alive);
    mechanism_free(&m);

    /* Undo snapshots everything, so a clone must carry the new arrays. */
    mechanism_init(&m);
    build_test_geneva(&m);
    Mechanism clone;
    mechanism_clone(&m, &clone);
    check_true("a clone carries the Geneva", clone.geneva_count == m.geneva_count);
    mechanism_delete_geneva(&m, 0);
    check_true("...independently of the original", clone.genevas[0].alive && !m.genevas[0].alive);
    mechanism_free(&clone);
    mechanism_free(&m);
}

/* Every button says what it does on hover. A missing or shouty tip is a button
 * nobody can learn from, and the stroke font only draws what it knows. */
static void test_every_button_has_a_tooltip(void) {
    Toolbar t;
    ui_init(&t, 900);
    bool all_present = true, all_sane = true, all_drawable = true;
    for (int i = 0; i < t.count; i++) {
        const char *tip = t.buttons[i].tip;
        if (!tip || !tip[0]) { all_present = false; continue; }
        size_t len = strlen(tip);
        if (len < 20 || len > 220) all_sane = false;
        for (const char *c = tip; *c; c++) {
            bool ok = (*c >= 'A' && *c <= 'Z') || (*c >= 'a' && *c <= 'z') ||
                       (*c >= '0' && *c <= '9') ||
                       *c == ' ' || *c == '.' || *c == ',' || *c == '-' ||
                       *c == '/' || *c == '+' || *c == ':' || *c == ';' ||
                       *c == '(' || *c == ')' || *c == '!' || *c == '?' ||
                       *c == '%' || *c == '\'' || *c == '<' || *c == '>';
            if (!ok) all_drawable = false;
        }
    }
    check_true("every button carries a tooltip", all_present);
    check_true("each one is a sentence, not a word or an essay", all_sane);
    check_true("and uses only characters the stroke font can draw", all_drawable);

    /* The one that needed explaining most: GEAR does two different things. */
    const char *gear = NULL;
    for (int i = 0; i < t.count; i++) {
        if (t.buttons[i].action == UI_GEAR) gear = t.buttons[i].tip;
    }
    check_true("the GEAR tip exists", gear != NULL);
    check_true("...and says how to mesh: select the wheels first",
               gear && strstr(gear, "mesh") && strstr(gear, "selected"));
}

static void test_cam_button_enablement(void) {
    Toolbar t;
    ui_init(&t, 900);

    /* The cam tool takes no selection at all -- that was the whole problem
     * with the old version -- so it is offered whenever you can edit. */
    UiState s = blank_ui_state();
    ui_apply_state(&t, s);
    check_true("CAM needs no selection to be available", button_for(&t, UI_CAM)->enabled);
    check_true("CAM is unlit until the tool is armed", !button_for(&t, UI_CAM)->active);

    s.drawing_cam = true;
    ui_apply_state(&t, s);
    check_true("CAM is lit while the drawing tool is armed", button_for(&t, UI_CAM)->active);

    /* The joint-making buttons need the right selection, and say so by
     * greying out rather than by failing when pressed. */
    s = blank_ui_state();
    ui_apply_state(&t, s);
    check_true("SLIDER is off without a selection", !button_for(&t, UI_SLIDER)->enabled);
    check_true("GEAR is off without a selection", !button_for(&t, UI_GEAR)->enabled);
    check_true("GENEVA is off without a selection", !button_for(&t, UI_GENEVA)->enabled);

    s.can_make_slider = true;
    ui_apply_state(&t, s);
    check_true("SLIDER lights up for three connectors", button_for(&t, UI_SLIDER)->enabled);
    check_true("...without enabling GEAR", !button_for(&t, UI_GEAR)->enabled);

    s = blank_ui_state();
    s.can_make_gear = true;
    ui_apply_state(&t, s);
    check_true("GEAR lights up for two centres", button_for(&t, UI_GEAR)->enabled);

    s = blank_ui_state();
    s.can_make_geneva = true;
    ui_apply_state(&t, s);
    check_true("GENEVA lights up for a motor and a wheel", button_for(&t, UI_GENEVA)->enabled);
    s.editing = false;
    s.running = true;
    ui_apply_state(&t, s);
    check_true("GENEVA is off while running", !button_for(&t, UI_GENEVA)->enabled);

    /* Both path tools are the same shape of tool: no selection, lit while
     * armed, and only one of them armed at a time. */
    s = blank_ui_state();
    ui_apply_state(&t, s);
    check_true("LINKAGE needs no selection to be available", button_for(&t, UI_LINKAGE)->enabled);
    check_true("ARMS needs no selection to be available", button_for(&t, UI_ARMS)->enabled);
    check_true("LINKAGE is unlit until armed", !button_for(&t, UI_LINKAGE)->active);
    check_true("ARMS is unlit until armed", !button_for(&t, UI_ARMS)->active);

    s.drawing_linkage = true;
    ui_apply_state(&t, s);
    check_true("LINKAGE lights up when armed", button_for(&t, UI_LINKAGE)->active);
    check_true("...and ARMS stays unlit", !button_for(&t, UI_ARMS)->active);

    s = blank_ui_state();
    s.drawing_arms = true;
    ui_apply_state(&t, s);
    check_true("ARMS lights up when armed", button_for(&t, UI_ARMS)->active);
    check_true("...and LINKAGE stays unlit", !button_for(&t, UI_LINKAGE)->active);

    s = blank_ui_state();
    s.editing = false;
    s.running = true;
    ui_apply_state(&t, s);
    check_true("CAM is disabled while running", !button_for(&t, UI_CAM)->enabled);
    check_true("LINKAGE is disabled while running", !button_for(&t, UI_LINKAGE)->enabled);
    check_true("ARMS is disabled while running", !button_for(&t, UI_ARMS)->enabled);
}

/* --------------------------------------------------------------------------
 * The status log -- the app's message channel
 * ----------------------------------------------------------------------- */

static void test_status_log_keeps_the_newest_messages(void) {
    StatusLog log;
    status_init(&log);
    check_true("a fresh log has nothing to show", log.count == 0 && !log.sticky_set);

    status_push(&log, STATUS_INFO, 1000, "one");
    status_push(&log, STATUS_WARN, 1000, "two");
    status_push(&log, STATUS_INFO, 1000, "three");
    status_push(&log, STATUS_INFO, 1000, "four");

    const StatusMessage *shown[STATUS_HISTORY];
    int n = status_visible(&log, 1000, shown, STATUS_HISTORY);
    check_true("the log holds its last three messages", n == STATUS_HISTORY);
    check_true("the newest message comes first", strcmp(shown[0]->text, "four") == 0);
    check_true("the oldest one was dropped", strcmp(shown[2]->text, "two") == 0);
}

static void test_repeating_a_message_restamps_it(void) {
    StatusLog log;
    status_init(&log);
    status_push(&log, STATUS_WARN, 1000, "select a pin first");
    status_push(&log, STATUS_WARN, 4000, "select a pin first");

    check_true("saying the same thing twice does not scroll the log", log.count == 1);
    check_true("but it does restamp it", log.recent[0].stamp_ms == 4000);
}

static void test_messages_fade_and_stickies_do_not(void) {
    StatusLog log;
    status_init(&log);
    status_push(&log, STATUS_INFO, 1000, "wheel radius 92");

    const StatusMessage *shown[STATUS_HISTORY];
    check_true("a fresh message is solid", status_alpha(&log.recent[0], 1000) == 255);
    check_true("it is still up part-way through",
                status_visible(&log, 1000 + STATUS_FADE_MS / 2, shown, STATUS_HISTORY) == 1);
    check_true("and gone after its time",
                status_visible(&log, 1000 + STATUS_FADE_MS + 1, shown, STATUS_HISTORY) == 0);

    /* A jam is a condition, not a moment: it holds until it is cleared. */
    status_set_sticky(&log, STATUS_ERROR, "JAMMED");
    check_true("a sticky message stays set", log.sticky_set);
    status_clear_sticky(&log);
    check_true("until it is cleared", !log.sticky_set);
}

/* The hotkeys go through ui_action_enabled and the toolbar draws from
 * ui_apply_state. If those two ever disagreed, a key would work where its
 * button was greyed out -- which is how the keyboard used to fail silently. */
static void test_keys_and_buttons_agree_about_what_is_allowed(void) {
    /* Three states worth checking: nothing selected, a full selection while
     * editing, and the same while running. */
    UiState states[3];
    states[0] = blank_ui_state();

    states[1] = blank_ui_state();
    states[1].selected_connector_count = 3;
    states[1].selected_link = 0;
    states[1].selected_link_driven = true;
    states[1].selected_link_can_drive = true;
    states[1].has_selection = true;
    states[1].can_undo = true;
    states[1].can_redo = true;

    states[2] = states[1];
    states[2].editing = false;
    states[2].running = true;
    states[2].paused = true;

    bool agree = true;
    for (int si = 0; si < 3; si++) {
        Toolbar t;
        ui_init(&t, 900);
        ui_apply_state(&t, states[si]);
        for (int i = 0; i < t.count; i++) {
            if (ui_action_enabled(&t, t.buttons[i].action) != t.buttons[i].enabled) agree = false;
            if (strcmp(ui_action_label(&t, t.buttons[i].action), t.buttons[i].label) != 0) agree = false;
            if (ui_action_tip(&t, t.buttons[i].action) != t.buttons[i].tip) agree = false;
        }
        /* An action with no button reads as not allowed, rather than as a key
         * that quietly does whatever it likes. */
        if (ui_action_enabled(&t, UI_NONE)) agree = false;
    }
    check_true("every hotkey obeys the same rule its button does", agree);
}

static void test_run_button_says_when_it_jammed(void) {
    Toolbar t;
    ui_init(&t, 900);
    UiState s = blank_ui_state();
    s.editing = false;
    s.running = true;
    ui_apply_state(&t, s);
    check_true("RUN reads STOP while it is running",
                strcmp(button_for(&t, UI_RUN)->label, "STOP") == 0);

    s.jammed = true;
    ui_apply_state(&t, s);
    check_true("and JAMMED once it has stopped dead",
                strcmp(button_for(&t, UI_RUN)->label, "JAMMED") == 0);
    check_true("RUN is still pressable when jammed", button_for(&t, UI_RUN)->enabled);
}

/* --------------------------------------------------------------------------
 * Saving and reopening a mechanism
 * ----------------------------------------------------------------------- */

/* Something with one of everything in it, including a deleted part, so the
 * round trip has to cope with tombstones as well as live entities. */
static void build_kitchen_sink(Mechanism *m) {
    mechanism_init(m);

    /* A crank on ground, driven by a motor, with a traced coupler point. */
    int ground = mechanism_add_connector(m, (Vec2){ 100, 300 }, true);
    int crank_pin = mechanism_add_connector(m, (Vec2){ 160, 300 }, false);
    int coupler_tip = mechanism_add_connector(m, (Vec2){ 260, 260 }, false);
    int crank_ids[2] = { ground, crank_pin };
    int coupler_ids[2] = { crank_pin, coupler_tip };
    int crank = mechanism_add_link(m, crank_ids, 2);
    int coupler = mechanism_add_link(m, coupler_ids, 2);
    mechanism_toggle_driven(m, crank, 123.5);
    mechanism_set_traced(m, coupler_tip, true);
    mechanism_set_rigid(m, coupler, false);

    /* A pin sliding on a rail between two anchors. */
    int rail_a = mechanism_add_connector(m, (Vec2){ 300, 400 }, true);
    int rail_b = mechanism_add_connector(m, (Vec2){ 500, 400 }, true);
    int slide = mechanism_add_connector(m, (Vec2){ 400, 400 }, false);
    mechanism_add_slider(m, slide, rail_a, rail_b);

    /* A meshed pair of wheels. */
    int wheel_a = mechanism_add_wheel(m, (Vec2){ 700, 300 }, 80.0);
    int wheel_b = mechanism_add_wheel(m, (Vec2){ 860, 300 }, 50.0);
    mechanism_mesh_wheels(m, wheel_a, wheel_b);

    /* A cam with a profile that is not the default one. */
    int cam_centre = mechanism_add_connector(m, (Vec2){ 200, 700 }, true);
    int shaft = mechanism_add_connector(m, (Vec2){ 230, 700 }, false);
    int follower = mechanism_add_connector(m, (Vec2){ 200, 620 }, false);
    int cam_ids[2] = { cam_centre, shaft };
    int cam_body = mechanism_add_link(m, cam_ids, 2);
    mechanism_toggle_driven(m, cam_body, 90.0);
    int cam_id = mechanism_add_cam(m, cam_body, cam_centre, follower);
    cam_fill_motion_law(&m->cams[cam_id], 60.0, 25.0, 90.0, 60.0, 120.0);

    /* And a part that has been deleted, so ids in memory have a hole in them. */
    int spare = mechanism_add_connector(m, (Vec2){ 999, 999 }, false);
    mechanism_delete_connector(m, spare);
}

static void test_scene_round_trip_preserves_the_mechanism(void) {
    Mechanism original;
    build_kitchen_sink(&original);
    SolverParams params = solver_default_params();
    params.gravity = (Vec2){ 0.0, 250.0 };

    const char *path = "tests/tmp_round_trip.linkage";
    char err[256] = { 0 };
    check_true("the mechanism saves", scene_save(&original, &params, path, err, sizeof err));

    Mechanism reopened;
    mechanism_init(&reopened);
    SolverParams reopened_params = solver_default_params();
    check_true("and reopens", scene_load(&reopened, &reopened_params, path, err, sizeof err));

    /* Tombstones are not written, so the file is dense: what must match is
     * every LIVE part, in order. */
    int live_connectors = 0, live_links = 0;
    for (int i = 0; i < original.connector_count; i++) {
        if (original.connectors[i].alive) live_connectors++;
    }
    for (int i = 0; i < original.link_count; i++) if (original.links[i].alive) live_links++;

    check_true("every pin came back", reopened.connector_count == live_connectors);
    check_true("every body came back", reopened.link_count == live_links);
    check_true("the slider came back", reopened.slider_count == 1);
    check_true("the mesh came back", reopened.gear_count == 1);
    check_true("the cam came back", reopened.cam_count == 1);
    check_true("gravity came back", reopened_params.gravity.y == params.gravity.y);

    int seen = 0;
    for (int i = 0; i < original.connector_count; i++) {
        const Connector *a = &original.connectors[i];
        if (!a->alive) continue;
        const Connector *b = &reopened.connectors[seen++];
        check_true_quiet(a->pos.x == b->pos.x && a->pos.y == b->pos.y);
        check_true_quiet(a->is_anchor == b->is_anchor);
        check_true_quiet(a->traced == b->traced);
    }
    check_true("pins came back where they were, anchored and traced as they were",
                seen == live_connectors);

    seen = 0;
    for (int i = 0; i < original.link_count; i++) {
        const Link *a = &original.links[i];
        if (!a->alive) continue;
        const Link *b = &reopened.links[seen++];
        check_true_quiet(a->connector_count == b->connector_count);
        check_true_quiet(a->rigid == b->rigid);
        check_true_quiet(a->is_driven == b->is_driven);
        check_true_quiet(a->motor_speed_deg_s == b->motor_speed_deg_s);
        check_true_quiet(a->wheel_radius == b->wheel_radius);
        check_true_quiet(a->driven_externally == b->driven_externally);
    }
    check_true("bodies kept their shape, their motor and their size", seen == live_links);

    bool profile_matches = true;
    for (int k = 0; k < CAM_PROFILE_SAMPLES; k++) {
        if (original.cams[0].pitch_r[k] != reopened.cams[0].pitch_r[k]) profile_matches = false;
    }
    check_true("the drawn cam profile came back exactly", profile_matches);
    check_close("and its base radius with it",
                 reopened.cams[0].base_radius, original.cams[0].base_radius, 1e-12);

    /* The real test of a reopened file: does it MOVE the same way? */
    SolverParams sim = solver_default_params();
    solver_freeze(&original);
    solver_freeze(&reopened);
    for (int step = 0; step < 60; step++) {
        solver_advance(&original, 1.0 / 60.0, sim);
        solver_advance(&reopened, 1.0 / 60.0, sim);
    }
    bool same_motion = true;
    seen = 0;
    for (int i = 0; i < original.connector_count; i++) {
        if (!original.connectors[i].alive) continue;
        Vec2 a = original.connectors[i].pos, b = reopened.connectors[seen++].pos;
        if (fabs(a.x - b.x) > 1e-9 || fabs(a.y - b.y) > 1e-9) same_motion = false;
    }
    check_true("and a second of simulation runs identically from both", same_motion);

    mechanism_free(&original);
    mechanism_free(&reopened);
    remove(path);
}

static void test_a_damaged_file_costs_nothing(void) {
    const char *path = "tests/tmp_damaged.linkage";
    FILE *f = fopen(path, "w");
    /* The body names pin 9, which does not exist. */
    fprintf(f, "LINKAGE 1\n" "C 10 20 0 0\n" "C 30 40 0 0\n" "L 2 0 9 1 0 -1 0 0\n");
    fclose(f);

    /* Something worth keeping is already open. */
    Mechanism m;
    mechanism_init(&m);
    mechanism_add_connector(&m, (Vec2){ 5, 5 }, true);
    SolverParams params = solver_default_params();

    char err[256] = { 0 };
    check_true("a damaged file is refused", !scene_load(&m, &params, path, err, sizeof err));
    check_true("and it says so", err[0] != '\0');
    check_true("and what was open is untouched", m.connector_count == 1 && m.connectors[0].alive);
    mechanism_free(&m);
    remove(path);

    check_true("so is a file that is not a mechanism at all",
                !scene_load(&m, &params, "Makefile", err, sizeof err));
}

int main(void) {
    test_scene_round_trip_preserves_the_mechanism();
    test_a_damaged_file_costs_nothing();
    test_status_log_keeps_the_newest_messages();
    test_repeating_a_message_restamps_it();
    test_messages_fade_and_stickies_do_not();
    test_keys_and_buttons_agree_about_what_is_allowed();
    test_run_button_says_when_it_jammed();
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
    test_cam_profile_is_continuous_and_smooth();
    test_zero_lift_cam_is_a_circle();
    test_undercut_detection();
    test_follower_tracks_the_cam_profile();
    test_follower_lifts_off_when_the_spring_cannot_keep_up();
    test_cam_cascades_on_delete();
    test_drawn_outline_becomes_the_profile();
    test_drawn_lobe_is_reproduced();
    test_degenerate_drawings_are_rejected();
    test_roller_shrinks_to_fit_a_drawn_shape();
    test_cam_angle_wrap_does_not_fling_the_follower();
    test_lift_and_timing_adjustments();
    test_follower_tracks_a_drawn_cam();
    test_trace_records_sample_times();
    test_every_button_has_a_tooltip();
    test_cam_button_enablement();
    test_geneva_indexes_exactly();
    test_crank_slider_gives_exactly_twice_the_crank();
    test_rack_travels_the_arc_it_rolls();
    test_wheels_are_placed_and_sized_on_their_own();
    test_meshing_slides_wheels_into_contact();
    test_one_wheel_drives_several();
    test_geneva_sizes_follow_the_geometry();
    test_joints_can_be_picked();
    test_gear_chain_runs_smoothly();
    test_geneva_drives_a_real_wheel();
    test_every_template_runs();
    test_joint_deletion_cascades();
    test_stroke_preparation();
    test_fourbar_pose_matches_closed_form();
    test_grashof_classification();
    test_four_bar_recovers_an_achievable_curve();
    test_four_bar_is_deterministic();
    test_four_bar_cannot_manage_a_star();
    test_fourier_reproduces_a_circle_exactly();
    test_fourier_traces_a_star();
    test_fourier_stops_early_when_accurate_enough();
    test_fourier_handles_an_open_stroke();
    test_fourier_rejects_useless_input();
    test_arm_chain_simulates_and_traces_the_path();

    if (failures == 0) {
        printf("\nAll tests passed.\n");
        return 0;
    }
    printf("\n%d test(s) FAILED.\n", failures);
    return 1;
}
