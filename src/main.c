#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#include "mechanism.h"
#include "solver.h"
#include "render.h"
#include "export.h"
#include "ui.h"

#define CANVAS_W 900
#define CANVAS_H 700
#define PLOT_H 200
#define WIN_H (CANVAS_H + PLOT_H)
#define WIN_W (UI_TOOLBAR_W + CANVAS_W)

#define CONNECTOR_HIT_RADIUS 10.0
#define LINK_EDGE_HIT_DIST 6.0
#define DRAG_THRESHOLD 4.0
#define DEFAULT_MOTOR_SPEED_DEG_S 90.0
#define MOTOR_SPEED_STEP_DEG_S 10.0
#define ZOOM_MIN 0.1
#define ZOOM_MAX 10.0
#define ZOOM_STEP 1.1
#define DEFAULT_GRAVITY_MAGNITUDE 400.0 /* world-units/s^2; a qualitative default, not physically calibrated */
#define CAM_HIT_DIST 6.0
#define CAM_LIFT_SCALE 1.12          /* per +/- press */
#define CAM_TIMING_STEP (5.0 * M_PI / 180.0)
#define CAM_STROKE_MIN_SPACING 3.0   /* world units between recorded points */
#define CAM_DEFAULT_RADIUS 90.0      /* a click, rather than a drawn outline */

/* World<->screen mapping: screen = world*zoom + pan. All Mechanism/mouse
 * positions are handled in world space; only rendering and raw SDL mouse
 * coordinates cross this boundary. The pan starts at the canvas's left edge
 * so world (0,0) is the top-left of the drawing area, not of the window. */
static Vec2 screen_to_world(Vec2 screen, Vec2 pan, double zoom) {
    return (Vec2){ (screen.x - pan.x) / zoom, (screen.y - pan.y) / zoom };
}

static Vec2 world_to_screen(Vec2 world, Vec2 pan, double zoom) {
    return (Vec2){ world.x * zoom + pan.x, world.y * zoom + pan.y };
}

typedef enum { APP_EDIT, APP_RUNNING } AppState;

typedef enum {
    DRAG_NONE,
    DRAG_MOVE_CONNECTORS,
    DRAG_PENDING_EMPTY,
    DRAG_BOX_SELECT,
    DRAG_DRAW_CAM
} DragMode;

/* Bounded stacks of full mechanism snapshots (mechanism_clone). Simple and
 * correct for the small mechanisms this tool targets. */
#define UNDO_MAX 50

typedef struct {
    Mechanism mech;
    SolverParams params;
    /* Gravity is applied automatically to a mechanism with no motor -- there
     * would otherwise be nothing to make it move at all. Once the user
     * toggles gravity themselves, their choice wins from then on. */
    bool gravity_set_by_user;

    Mechanism undo_stack[UNDO_MAX];
    int undo_count;
    Mechanism redo_stack[UNDO_MAX];
    int redo_count;

    AppState state;

    DragMode drag_mode;
    Vec2 drag_start, drag_last, drag_current;

    /* CAM is a drawing tool: arm it, then drag out the cam's outline. The
     * stroke is collected in world space and handed to the cam as its
     * profile on mouse-up. */
    bool cam_draw_armed;
    Vec2 *cam_stroke;
    int cam_stroke_count, cam_stroke_capacity;

    Vec2 *pre_run_positions;
    int pre_run_count;

    /* Start-of-frame snapshot, so a step that binds up can be rolled back
     * rather than leaving fixed-length links visibly stretched. */
    Vec2 *frame_positions;
    double *frame_angles;
    int frame_link_count;
    bool jammed;

    Vec2 view_pan;
    double view_zoom;

    /* Seconds since the run started, stamped onto every trace sample so the
     * plot has a real time axis (frames are not uniform in length). */
    double sim_time;

    Toolbar toolbar;
} App;

static void clear_selection(Mechanism *m) {
    for (int i = 0; i < m->connector_count; i++) m->connectors[i].selected = false;
    for (int i = 0; i < m->link_count; i++) m->links[i].selected = false;
    for (int i = 0; i < m->cam_count; i++) m->cams[i].selected = false;
}

static int find_single_selected_cam(const Mechanism *m) {
    int found = -1;
    for (int i = 0; i < m->cam_count; i++) {
        if (m->cams[i].alive && m->cams[i].selected) {
            if (found >= 0) return -1;
            found = i;
        }
    }
    return found;
}

static int find_single_selected_link(const Mechanism *m) {
    int found = -1;
    for (int i = 0; i < m->link_count; i++) {
        if (m->links[i].alive && m->links[i].selected) {
            if (found >= 0) return -1;
            found = i;
        }
    }
    return found;
}

static int gather_selected_connectors(const Mechanism *m, int *out, int max_out) {
    int n = 0;
    for (int i = 0; i < m->connector_count; i++) {
        if (m->connectors[i].alive && m->connectors[i].selected) {
            if (n < max_out) out[n] = i;
            n++;
        }
    }
    return n;
}

/* How many of a link's own connectors are anchors -- a link can only be
 * driven when exactly one is (it becomes the motor's pivot). Checked before
 * taking an undo snapshot so a rejected toggle leaves no history behind. */
static int link_anchor_count(const Mechanism *m, int link_id) {
    const Link *l = &m->links[link_id];
    int n = 0;
    for (int i = 0; i < l->connector_count; i++) {
        int cid = l->connector_ids[i];
        if (m->connectors[cid].alive && m->connectors[cid].is_anchor) n++;
    }
    return n;
}

static bool has_selection(const Mechanism *m) {
    for (int i = 0; i < m->connector_count; i++) {
        if (m->connectors[i].alive && m->connectors[i].selected) return true;
    }
    for (int i = 0; i < m->link_count; i++) {
        if (m->links[i].alive && m->links[i].selected) return true;
    }
    for (int i = 0; i < m->cam_count; i++) {
        if (m->cams[i].alive && m->cams[i].selected) return true;
    }
    return false;
}

/* The gravity actually in force: whatever the user set, or the automatic
 * downward pull a motorless mechanism gets so it does something at all. */
static Vec2 effective_gravity(const App *a) {
    if (!a->gravity_set_by_user && !mechanism_has_driven_link(&a->mech)) {
        return (Vec2){ 0.0, DEFAULT_GRAVITY_MAGNITUDE };
    }
    return a->params.gravity;
}

/* --------------------------------------------------------------------------
 * Undo / redo
 * ----------------------------------------------------------------------- */

static void push_snapshot(Mechanism *stack, int *count, const Mechanism *mech) {
    if (*count >= UNDO_MAX) {
        mechanism_free(&stack[0]);
        for (int i = 1; i < UNDO_MAX; i++) stack[i - 1] = stack[i];
        *count = UNDO_MAX - 1;
    }
    mechanism_clone(mech, &stack[*count]);
    (*count)++;
}

static bool pop_snapshot(Mechanism *stack, int *count, Mechanism *out) {
    if (*count <= 0) return false;
    (*count)--;
    *out = stack[*count];
    return true;
}

static void clear_stack(Mechanism *stack, int *count) {
    for (int i = 0; i < *count; i++) mechanism_free(&stack[i]);
    *count = 0;
}

/* Call right before an edit. A fresh edit invalidates any redo history. */
static void push_undo(App *a) {
    push_snapshot(a->undo_stack, &a->undo_count, &a->mech);
    clear_stack(a->redo_stack, &a->redo_count);
}

/* Drops the most recent undo snapshot without restoring it -- for the
 * defensive case where a push_undo() turned out to precede a no-op. */
static void discard_last_undo(App *a) {
    if (a->undo_count <= 0) return;
    a->undo_count--;
    mechanism_free(&a->undo_stack[a->undo_count]);
}

static void app_undo(App *a) {
    if (a->undo_count <= 0) {
        printf("Nothing to undo.\n");
        return;
    }
    push_snapshot(a->redo_stack, &a->redo_count, &a->mech);
    Mechanism restored;
    pop_snapshot(a->undo_stack, &a->undo_count, &restored);
    mechanism_free(&a->mech);
    a->mech = restored;
    printf("Undo.\n");
}

static void app_redo(App *a) {
    if (a->redo_count <= 0) {
        printf("Nothing to redo.\n");
        return;
    }
    /* Deliberately not push_undo(): redoing must not clear the redo stack. */
    push_snapshot(a->undo_stack, &a->undo_count, &a->mech);
    Mechanism restored;
    pop_snapshot(a->redo_stack, &a->redo_count, &restored);
    mechanism_free(&a->mech);
    a->mech = restored;
    printf("Redo.\n");
}

/* --------------------------------------------------------------------------
 * Actions -- one per command, shared by the hotkeys and the toolbar so there
 * is exactly one implementation of each behaviour.
 * ----------------------------------------------------------------------- */

static void app_add_joint(App *a) {
    Vec2 center = { UI_TOOLBAR_W + CANVAS_W / 2.0, WIN_H / 2.0 };
    Vec2 p = screen_to_world(center, a->view_pan, a->view_zoom);
    push_undo(a);
    clear_selection(&a->mech);
    int id = mechanism_add_connector(&a->mech, p, false);
    a->mech.connectors[id].selected = true;
}

static void app_link_selected(App *a) {
    int ids[256];
    int n = gather_selected_connectors(&a->mech, ids, 256);
    if (n < 2) {
        printf("Select at least 2 connectors before linking.\n");
        return;
    }
    push_undo(a);
    int lid = mechanism_add_link(&a->mech, ids, n);
    if (lid >= 0) {
        clear_selection(&a->mech);
        a->mech.links[lid].selected = true;
    } else {
        discard_last_undo(a);
    }
}

static void app_toggle_anchor(App *a) {
    if (!has_selection(&a->mech)) return;
    push_undo(a);
    for (int i = 0; i < a->mech.connector_count; i++) {
        if (a->mech.connectors[i].alive && a->mech.connectors[i].selected) {
            mechanism_set_anchor(&a->mech, i, !a->mech.connectors[i].is_anchor);
        }
    }
}

static void app_toggle_motor(App *a) {
    int lid = find_single_selected_link(&a->mech);
    if (lid < 0) {
        printf("Select exactly one link before toggling a motor.\n");
        return;
    }
    if (!a->mech.links[lid].is_driven && link_anchor_count(&a->mech, lid) != 1) {
        printf("A driven link needs exactly one anchor connector.\n");
        return;
    }
    push_undo(a);
    if (!mechanism_toggle_driven(&a->mech, lid, DEFAULT_MOTOR_SPEED_DEG_S)) {
        discard_last_undo(a);
        printf("A driven link needs exactly one anchor connector.\n");
    }
}

/* Records a point of the cam outline being drawn, thinning out samples that
 * are too close together to matter. */
static void stroke_push(App *a, Vec2 p) {
    if (a->cam_stroke_count > 0 &&
        vec2_dist(a->cam_stroke[a->cam_stroke_count - 1], p) < CAM_STROKE_MIN_SPACING) {
        return;
    }
    if (a->cam_stroke_count >= a->cam_stroke_capacity) {
        int cap = (a->cam_stroke_capacity == 0) ? 64 : a->cam_stroke_capacity * 2;
        a->cam_stroke = realloc(a->cam_stroke, (size_t)cap * sizeof(Vec2));
        a->cam_stroke_capacity = cap;
    }
    a->cam_stroke[a->cam_stroke_count++] = p;
}

/* Area centroid of the drawn outline (the shoelace centroid, which is far
 * steadier than the mean of the points when the stroke is drawn unevenly).
 * Falls back to the mean for a degenerate, zero-area scribble. */
static Vec2 outline_centroid(const Vec2 *pts, int n) {
    double area = 0.0, cx = 0.0, cy = 0.0;
    for (int i = 0; i < n; i++) {
        Vec2 a = pts[i], b = pts[(i + 1) % n];
        double cross = a.x * b.y - b.x * a.y;
        area += cross;
        cx += (a.x + b.x) * cross;
        cy += (a.y + b.y) * cross;
    }
    if (fabs(area) > 1e-9) {
        return (Vec2){ cx / (3.0 * area), cy / (3.0 * area) };
    }
    Vec2 mean = { 0.0, 0.0 };
    for (int i = 0; i < n; i++) mean = vec2_add(mean, pts[i]);
    return vec2_scale(mean, 1.0 / (double)n);
}

/* Builds a complete, running cam in one go: the centre (grounded), a short
 * shaft marker driven by a motor so the disc actually turns, the roller
 * follower on its guide axis, and the cam itself. Creating the whole rig at
 * once is the point -- there is nothing to select and nothing to assemble by
 * hand. `outline` may be NULL, in which case a default profile is used. */
static void app_create_cam(App *a, const Vec2 *outline, int n, Vec2 fallback_centre) {
    bool drawn = (outline != NULL && n >= 3);
    Vec2 centre = drawn ? outline_centroid(outline, n) : fallback_centre;

    double radius_hint = CAM_DEFAULT_RADIUS;
    if (drawn) {
        double sum = 0.0;
        for (int i = 0; i < n; i++) sum += vec2_dist(outline[i], centre);
        radius_hint = sum / (double)n;
        if (radius_hint < 10.0) drawn = false; /* too small to read a profile from */
    }

    push_undo(a);
    clear_selection(&a->mech);

    int centre_id = mechanism_add_connector(&a->mech, centre, true);
    /* The shaft marker sits inside the disc, so it reads as a keyway showing
     * the cam's rotation rather than as stray geometry. */
    double shaft = fmax(8.0, radius_hint * 0.45);
    int shaft_id = mechanism_add_connector(&a->mech, vec2_add(centre, (Vec2){ shaft, 0.0 }), false);
    int ids[2] = { centre_id, shaft_id };
    int link_id = mechanism_add_link(&a->mech, ids, 2);
    mechanism_toggle_driven(&a->mech, link_id, DEFAULT_MOTOR_SPEED_DEG_S);

    /* The follower starts straight above the centre; its guide axis is the
     * line through the two, so dragging it elsewhere before running aims the
     * cam at a different angle. */
    int follower_id = mechanism_add_connector(&a->mech,
                                               vec2_add(centre, (Vec2){ 0.0, -radius_hint }), false);

    int cam_id = mechanism_add_cam(&a->mech, link_id, centre_id, follower_id);
    if (cam_id < 0) {
        discard_last_undo(a);
        printf("Could not create a cam there.\n");
        return;
    }
    Cam *cam = &a->mech.cams[cam_id];

    if (drawn) {
        Vec2 *local = malloc((size_t)n * sizeof(Vec2));
        for (int i = 0; i < n; i++) local[i] = vec2_sub(outline[i], centre);
        if (!cam_set_from_drawn_outline(cam, local, n)) {
            printf("Couldn't read a profile from that outline -- using a default cam. "
                    "Try drawing a single loop right around the centre.\n");
        }
        free(local);
    }

    /* Seat the follower exactly on the profile so it starts in contact. */
    cam->spring_preload = cam->base_radius * 0.25;
    double phi = atan2(cam->axis_dir.y, cam->axis_dir.x);
    a->mech.connectors[follower_id].pos =
        vec2_add(centre, vec2_scale(cam->axis_dir, cam_pitch_radius(cam, phi)));

    /* It is seated on the profile, so show it as touching straight away
     * rather than flashing the airborne colour until the first frame. */
    cam->in_contact = true;
    cam->selected = true;
    printf("Cam added: base radius %.1f, lift %.1f. Press R to run. "
            "Select the cam to reshape it: +/- lift, [ and ] timing.\n",
            cam->base_radius, cam->lift);
    if (cam_is_undercut(cam)) {
        printf("Warning: this cam undercuts -- it has a concave notch tighter than "
                "the roller, so it could not be cut to give this motion.\n");
    }
}

static void app_toggle_cam_draw(App *a) {
    a->cam_draw_armed = !a->cam_draw_armed;
    a->cam_stroke_count = 0;
    if (a->cam_draw_armed) {
        printf("Cam tool armed: drag on the canvas to draw the cam's outline "
                "(or click once for a default cam). Escape cancels.\n");
    }
}

static void app_adjust_cam_lift(App *a, double factor) {
    int cid = find_single_selected_cam(&a->mech);
    if (cid < 0) return;
    push_undo(a);
    Cam *c = &a->mech.cams[cid];
    cam_scale_lift(c, factor);
    printf("Cam lift: %.1f\n", c->lift);
    if (cam_is_undercut(c)) printf("Warning: this cam now undercuts.\n");
}

/* Rotating the profile against the shaft is cam timing: same motion, earlier
 * or later in the turn. */
static void app_adjust_cam_timing(App *a, double delta_rad) {
    int cid = find_single_selected_cam(&a->mech);
    if (cid < 0) return;
    push_undo(a);
    cam_rotate_profile(&a->mech.cams[cid], delta_rad);
    printf("Cam timing shifted by %.0f deg.\n", delta_rad * 180.0 / M_PI);
}

static void app_toggle_vary(App *a) {
    int lid = find_single_selected_link(&a->mech);
    if (lid < 0) {
        printf("Select exactly one link before toggling its length.\n");
        return;
    }
    if (a->mech.links[lid].is_driven) {
        printf("A driven link's shape is always rigid; turn its motor off first.\n");
        return;
    }
    push_undo(a);
    bool now_rigid = !a->mech.links[lid].rigid;
    mechanism_set_rigid(&a->mech, lid, now_rigid);
    printf("Link length is now %s.\n", now_rigid ? "fixed" : "variable");
}

static void app_toggle_trace(App *a) {
    if (!has_selection(&a->mech)) return;
    push_undo(a);
    for (int i = 0; i < a->mech.connector_count; i++) {
        if (a->mech.connectors[i].alive && a->mech.connectors[i].selected) {
            mechanism_set_traced(&a->mech, i, !a->mech.connectors[i].traced);
        }
    }
}

static void app_adjust_motor_speed(App *a, double step) {
    int lid = find_single_selected_link(&a->mech);
    if (lid < 0 || !a->mech.links[lid].is_driven) return;
    push_undo(a);
    a->mech.links[lid].motor_speed_deg_s += step;
    printf("Motor speed: %.1f deg/s\n", a->mech.links[lid].motor_speed_deg_s);
}

static void app_delete_selection(App *a) {
    if (!has_selection(&a->mech)) return;
    push_undo(a);
    for (int i = 0; i < a->mech.link_count; i++) {
        if (a->mech.links[i].alive && a->mech.links[i].selected) mechanism_delete_link(&a->mech, i);
    }
    for (int i = 0; i < a->mech.cam_count; i++) {
        if (a->mech.cams[i].alive && a->mech.cams[i].selected) mechanism_delete_cam(&a->mech, i);
    }
    for (int i = 0; i < a->mech.connector_count; i++) {
        if (a->mech.connectors[i].alive && a->mech.connectors[i].selected) {
            mechanism_delete_connector(&a->mech, i);
        }
    }
}

static void app_toggle_gravity(App *a) {
    a->gravity_set_by_user = true;
    if (a->params.gravity.x == 0.0 && a->params.gravity.y == 0.0) {
        a->params.gravity = (Vec2){ 0.0, DEFAULT_GRAVITY_MAGNITUDE };
        printf("Gravity ON.\n");
    } else {
        a->params.gravity = (Vec2){ 0.0, 0.0 };
        printf("Gravity OFF.\n");
    }
}

static void app_export(App *a) {
    const char *path = "linkage_export.py";
    /* Export the motion the app would actually show, including gravity
     * applying automatically to a motorless mechanism. */
    SolverParams export_params = a->params;
    export_params.gravity = effective_gravity(a);
    if (export_blender_script(&a->mech, export_params, path)) {
        printf("Exported %d animation frames to %s -- run it inside Blender's Scripting tab "
                "(or `blender --python %s`), then press Space to play.\n",
                EXPORT_FRAMES, path, path);
    } else {
        printf("Failed to write %s\n", path);
    }
}

static void app_start_run(App *a) {
    free(a->pre_run_positions);
    a->pre_run_count = a->mech.connector_count;
    a->pre_run_positions = malloc((size_t)a->pre_run_count * sizeof(Vec2));
    for (int i = 0; i < a->pre_run_count; i++) a->pre_run_positions[i] = a->mech.connectors[i].pos;

    free(a->frame_positions);
    a->frame_positions = malloc((size_t)a->pre_run_count * sizeof(Vec2));
    free(a->frame_angles);
    a->frame_link_count = a->mech.link_count;
    a->frame_angles = (a->frame_link_count > 0) ? malloc((size_t)a->frame_link_count * sizeof(double)) : NULL;
    a->jammed = false;

    if (!a->gravity_set_by_user && !mechanism_has_driven_link(&a->mech)) {
        printf("No motor in this mechanism -- running it under gravity. "
                "Use the GRAVITY button (or G) to control gravity yourself.\n");
    }

    a->sim_time = 0.0;
    solver_freeze(&a->mech);
    mechanism_clear_traces(&a->mech);
    a->drag_mode = DRAG_NONE;
    a->state = APP_RUNNING;
}

static void app_stop_run(App *a) {
    for (int i = 0; i < a->pre_run_count && i < a->mech.connector_count; i++) {
        a->mech.connectors[i].pos = a->pre_run_positions[i];
    }
    free(a->pre_run_positions);
    a->pre_run_positions = NULL;
    a->pre_run_count = 0;
    free(a->frame_positions);
    a->frame_positions = NULL;
    free(a->frame_angles);
    a->frame_angles = NULL;
    a->frame_link_count = 0;
    a->jammed = false;
    a->state = APP_EDIT;
}

static void app_toggle_run(App *a) {
    if (a->state == APP_EDIT) app_start_run(a);
    else app_stop_run(a);
}

static void app_dispatch(App *a, UiAction action) {
    switch (action) {
    case UI_JOINT:   app_add_joint(a); break;
    case UI_ANCHOR:  app_toggle_anchor(a); break;
    case UI_LINK:    app_link_selected(a); break;
    case UI_MOTOR:   app_toggle_motor(a); break;
    case UI_CAM:     app_toggle_cam_draw(a); break;
    case UI_VARY:    app_toggle_vary(a); break;
    case UI_TRACE:   app_toggle_trace(a); break;
    case UI_DELETE:  app_delete_selection(a); break;
    case UI_UNDO:    app_undo(a); break;
    case UI_REDO:    app_redo(a); break;
    case UI_GRAVITY: app_toggle_gravity(a); break;
    case UI_CLEAR:   mechanism_clear_traces(&a->mech); break;
    case UI_EXPORT:  app_export(a); break;
    case UI_RUN:     app_toggle_run(a); break;
    case UI_NONE:
    case UI_ACTION_COUNT:
        break;
    }
}

/* Everything the toolbar needs to decide what is enabled and lit right now. */
static UiState app_ui_state(const App *a) {
    UiState s;
    s.editing = (a->state == APP_EDIT);
    s.running = (a->state == APP_RUNNING);
    s.selected_connector_count = gather_selected_connectors(&a->mech, NULL, 0);
    s.has_selection = has_selection(&a->mech);

    s.all_selected_traced = true;
    for (int i = 0; i < a->mech.connector_count; i++) {
        const Connector *c = &a->mech.connectors[i];
        if (c->alive && c->selected && !c->traced) { s.all_selected_traced = false; break; }
    }

    s.selected_link = find_single_selected_link(&a->mech);
    if (s.selected_link >= 0) {
        const Link *l = &a->mech.links[s.selected_link];
        s.selected_link_driven = l->is_driven;
        s.selected_link_rigid = l->rigid;
        s.selected_link_can_drive = (link_anchor_count(&a->mech, s.selected_link) == 1);
    } else {
        s.selected_link_driven = false;
        s.selected_link_rigid = true;
        s.selected_link_can_drive = false;
    }

    s.drawing_cam = a->cam_draw_armed;
    s.can_undo = (a->undo_count > 0);
    s.can_redo = (a->redo_count > 0);
    Vec2 g = effective_gravity(a);
    s.gravity_on = (g.x != 0.0 || g.y != 0.0);
    return s;
}

static void draw_mechanism(SDL_Renderer *ren, const Mechanism *m, DragMode drag_mode, Vec2 drag_start, Vec2 drag_current,
                            Vec2 view_pan, double view_zoom) {
    for (int i = 0; i < m->connector_count; i++) {
        const Connector *c = &m->connectors[i];
        if (!c->alive || !c->traced || c->path_count < 2) continue;
        Uint8 tr, tg, tb;
        render_trace_color(i, &tr, &tg, &tb);
        for (int k = 1; k < c->path_count; k++) {
            render_line(ren, world_to_screen(c->path[k - 1], view_pan, view_zoom),
                        world_to_screen(c->path[k], view_pan, view_zoom), tr, tg, tb, 255);
        }
    }

    for (int ci = 0; ci < m->cam_count; ci++) {
        const Cam *cam = &m->cams[ci];
        if (!cam->alive || cam->center_connector_id < 0) continue;
        if (!m->connectors[cam->center_connector_id].alive) continue;

        Vec2 centre = world_to_screen(m->connectors[cam->center_connector_id].pos, view_pan, view_zoom);
        double angle = mechanism_cam_angle(m, ci);

        Uint8 r, g, b;
        if (cam->selected) { r = 255; g = 225; b = 70; }
        else { r = 175; g = 150; b = 200; }
        render_cam(ren, cam, centre, angle, view_zoom, r, g, b, 255);

        /* The guide axis the follower slides along, and the roller riding the
         * profile. The roller changes colour the moment it leaves the cam, so
         * float is visible rather than something you have to infer. */
        Vec2 axis_far = vec2_add(m->connectors[cam->center_connector_id].pos,
                                  vec2_scale(cam->axis_dir, cam->base_radius + cam->lift * 2.0 + 40.0));
        render_line(ren, centre, world_to_screen(axis_far, view_pan, view_zoom), 90, 85, 105, 255);

        if (cam->follower_connector_id >= 0 && m->connectors[cam->follower_connector_id].alive) {
            Vec2 f = world_to_screen(m->connectors[cam->follower_connector_id].pos, view_pan, view_zoom);
            if (cam->in_contact) render_circle(ren, f, cam->roller_radius * view_zoom, 120, 215, 140, 255);
            else render_circle(ren, f, cam->roller_radius * view_zoom, 235, 110, 90, 255);
        }

        /* Base radius and lift, in the same seven-segment numerals the link
         * lengths use. */
        double dh = 10.0 * view_zoom;
        render_number(ren, (Vec2){ centre.x + 6.0, centre.y + 8.0 }, 0.0, dh,
                      cam->base_radius, 160, 150, 185, 255);
        render_number(ren, (Vec2){ centre.x + 6.0, centre.y + 8.0 + dh * 1.5 }, 0.0, dh,
                      cam->lift, 160, 150, 185, 255);
    }

    for (int li = 0; li < m->link_count; li++) {
        const Link *l = &m->links[li];
        if (!l->alive) continue;
        Uint8 r, g, b;
        if (l->selected) { r = 255; g = 225; b = 70; }
        else if (l->is_driven) { r = 230; g = 90; b = 70; }
        else if (!l->rigid) { r = 110; g = 210; b = 140; }
        else { r = 140; g = 150; b = 165; }
        for (int i = 0; i < l->connector_count; i++) {
            for (int j = i + 1; j < l->connector_count; j++) {
                Vec2 world_a = m->connectors[l->connector_ids[i]].pos;
                Vec2 world_b = m->connectors[l->connector_ids[j]].pos;
                Vec2 a = world_to_screen(world_a, view_pan, view_zoom);
                Vec2 bpt = world_to_screen(world_b, view_pan, view_zoom);
                render_line(ren, a, bpt, r, g, b, 255);

                double length = vec2_dist(world_a, world_b);
                Vec2 mid = { (a.x + bpt.x) / 2.0, (a.y + bpt.y) / 2.0 };
                Vec2 dir = vec2_sub(bpt, a);
                double dir_len = vec2_len(dir);
                if (dir_len > 1e-6) {
                    Vec2 dir_unit = vec2_scale(dir, 1.0 / dir_len);
                    double angle = atan2(dir_unit.y, dir_unit.x);
                    /* Keep text reading left-to-right rather than upside down
                     * when the edge points leftward. */
                    if (dir_unit.x < 0.0) {
                        angle += M_PI;
                        dir_unit = vec2_scale(dir_unit, -1.0);
                    }
                    Vec2 perp_unit = vec2_perp(dir_unit); /* consistent side once dir_unit is normalized above */
                    double digit_height = 10.0 * view_zoom;
                    double gap = 6.0 * view_zoom;
                    double text_width = render_number_width(digit_height, length);
                    Vec2 start = {
                        mid.x - dir_unit.x * (text_width / 2.0) + perp_unit.x * gap,
                        mid.y - dir_unit.y * (text_width / 2.0) + perp_unit.y * gap,
                    };
                    render_number(ren, start, angle, digit_height, length, 190, 195, 205, 255);
                }
            }
        }
    }

    for (int i = 0; i < m->connector_count; i++) {
        const Connector *c = &m->connectors[i];
        if (!c->alive) continue;
        Uint8 r, g, b;
        if (c->selected) { r = 255; g = 225; b = 70; }
        else if (c->is_anchor) { r = 230; g = 160; b = 60; }
        else if (c->traced) { render_trace_color(i, &r, &g, &b); }
        else { r = 220; g = 220; b = 220; }
        Vec2 sp = world_to_screen(c->pos, view_pan, view_zoom);
        render_circle(ren, sp, 5.0 * view_zoom, r, g, b, 255);
        if (c->is_anchor) render_ground_hatch(ren, sp, 8.0 * view_zoom, r, g, b, 255);
    }

    if (drag_mode == DRAG_BOX_SELECT) {
        Vec2 p0 = world_to_screen((Vec2){ fmin(drag_start.x, drag_current.x), fmin(drag_start.y, drag_current.y) }, view_pan, view_zoom);
        Vec2 p1 = world_to_screen((Vec2){ fmax(drag_start.x, drag_current.x), fmax(drag_start.y, drag_current.y) }, view_pan, view_zoom);
        Vec2 tl = { p0.x, p0.y }, tr = { p1.x, p0.y }, br = { p1.x, p1.y }, bl = { p0.x, p1.y };
        render_line(ren, tl, tr, 100, 160, 255, 255);
        render_line(ren, tr, br, 100, 160, 255, 255);
        render_line(ren, br, bl, 100, 160, 255, 255);
        render_line(ren, bl, tl, 100, 160, 255, 255);
    }
}

int main(void) {
    /* Line-buffer stdout so the running commentary still appears in order
     * when it is redirected to a file, not just when it goes to a terminal. */
    setvbuf(stdout, NULL, _IOLBF, 0);

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *win = SDL_CreateWindow("Linkage Design",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_W, WIN_H, SDL_WINDOW_SHOWN);
    if (!win) {
        fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
    if (!ren) {
        fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(win);
        SDL_Quit();
        return 1;
    }

    printf("Linkage Design - mechanism editor\n");
    printf("  Every command below is also a button in the toolbar down the left edge.\n");
    printf("  Edit mode:\n");
    printf("    Click empty space: place a connector      Drag empty space: box-select\n");
    printf("    Click a connector: select it (Shift: add/remove)   Drag a selected connector: move selection\n");
    printf("    Click a link's edge: select that link\n");
    printf("    J: place a connector at the centre of the view\n");
    printf("    L: link selected connectors     A: toggle anchor on selected connectors\n");
    printf("    M: toggle motor on selected link (needs exactly one anchor)   +/-: motor speed\n");
    printf("    K: cam tool -- then DRAG on the canvas to draw the cam's outline (or\n");
    printf("       click once for a default cam). The centre, motor and roller follower\n");
    printf("       are all created for you. Click a cam's outline to select it:\n");
    printf("       +/- resize its lift, [ and ] shift its timing.\n");
    printf("    V: toggle selected link's length between fixed and variable (green = variable)\n");
    printf("    T: toggle path tracing on selected connectors\n");
    printf("    E: export mechanism to a Blender Python script\n");
    printf("    Delete/Backspace: delete selection     Escape: clear selection\n");
    printf("    Cmd/Ctrl+Z: undo     Shift+Cmd/Ctrl+Z or Cmd/Ctrl+Y: redo\n");
    printf("  Scroll wheel: zoom in/out (centered on cursor)\n");
    printf("  C: clear all traces (works while running too)\n");
    printf("  G: toggle gravity (on automatically when the mechanism has no motor)\n");
    printf("  R: run/stop simulation\n");

    App app;
    mechanism_init(&app.mech);
    app.params = solver_default_params();
    app.gravity_set_by_user = false;
    app.undo_count = 0;
    app.redo_count = 0;
    app.state = APP_EDIT;
    app.drag_mode = DRAG_NONE;
    app.drag_start = app.drag_last = app.drag_current = (Vec2){ 0, 0 };
    app.pre_run_positions = NULL;
    app.pre_run_count = 0;
    app.frame_positions = NULL;
    app.frame_angles = NULL;
    app.frame_link_count = 0;
    app.jammed = false;
    app.view_pan = (Vec2){ UI_TOOLBAR_W, 0 };
    app.view_zoom = 1.0;
    app.sim_time = 0.0;
    app.cam_draw_armed = false;
    app.cam_stroke = NULL;
    app.cam_stroke_count = 0;
    app.cam_stroke_capacity = 0;
    ui_init(&app.toolbar);

    Uint32 last_ticks = SDL_GetTicks();
    bool running = true;

    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) {
                running = false;
            } else if (ev.type == SDL_MOUSEMOTION) {
                app.toolbar.hover = ui_hit_test(&app.toolbar, ev.motion.x, ev.motion.y);
                /* A drag can only have started on the canvas (the toolbar
                 * consumes its own press), so let it continue even if the
                 * cursor strays over the strip. */
                if (app.state == APP_EDIT) {
                    Vec2 p = screen_to_world((Vec2){ (double)ev.motion.x, (double)ev.motion.y }, app.view_pan, app.view_zoom);
                    if (app.drag_mode == DRAG_MOVE_CONNECTORS) {
                        Vec2 delta = vec2_sub(p, app.drag_last);
                        for (int i = 0; i < app.mech.connector_count; i++) {
                            if (app.mech.connectors[i].alive && app.mech.connectors[i].selected) {
                                app.mech.connectors[i].pos = vec2_add(app.mech.connectors[i].pos, delta);
                            }
                        }
                        app.drag_last = p;
                    } else if (app.drag_mode == DRAG_PENDING_EMPTY) {
                        if (vec2_dist(p, app.drag_start) > DRAG_THRESHOLD / app.view_zoom) app.drag_mode = DRAG_BOX_SELECT;
                        app.drag_current = p;
                    } else if (app.drag_mode == DRAG_BOX_SELECT) {
                        app.drag_current = p;
                    } else if (app.drag_mode == DRAG_DRAW_CAM) {
                        stroke_push(&app, p);
                    }
                }
            } else if (ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_LEFT &&
                        ui_contains(&app.toolbar, ev.button.x, ev.button.y)) {
                /* Toolbar presses are handled in both edit and running mode. */
                app.toolbar.pressed = ui_hit_test(&app.toolbar, ev.button.x, ev.button.y);
            } else if (ev.type == SDL_MOUSEBUTTONUP && ev.button.button == SDL_BUTTON_LEFT &&
                        app.toolbar.pressed >= 0) {
                int released_on = ui_hit_test(&app.toolbar, ev.button.x, ev.button.y);
                const UiButton *b = &app.toolbar.buttons[app.toolbar.pressed];
                if (released_on == app.toolbar.pressed && b->enabled) app_dispatch(&app, b->action);
                app.toolbar.pressed = -1;
            } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_c) {
                mechanism_clear_traces(&app.mech); /* works in both edit and running mode */
            } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_g) {
                app_toggle_gravity(&app);
            } else if (ev.type == SDL_KEYDOWN && app.state == APP_EDIT) {
                SDL_Keycode k = ev.key.keysym.sym;
                SDL_Keymod mod = SDL_GetModState();
                bool cmd = (mod & (KMOD_CTRL | KMOD_GUI)) != 0;
                if (k == SDLK_z && cmd) {
                    if (mod & KMOD_SHIFT) app_redo(&app);
                    else app_undo(&app);
                } else if (k == SDLK_y && cmd) {
                    app_redo(&app);
                } else if (k == SDLK_j) {
                    app_add_joint(&app);
                } else if (k == SDLK_l) {
                    app_link_selected(&app);
                } else if (k == SDLK_a) {
                    app_toggle_anchor(&app);
                } else if (k == SDLK_m) {
                    app_toggle_motor(&app);
                } else if (k == SDLK_k) {
                    app_toggle_cam_draw(&app);
                } else if (k == SDLK_LEFTBRACKET) {
                    app_adjust_cam_timing(&app, -CAM_TIMING_STEP);
                } else if (k == SDLK_RIGHTBRACKET) {
                    app_adjust_cam_timing(&app, CAM_TIMING_STEP);
                } else if (k == SDLK_v) {
                    app_toggle_vary(&app);
                } else if (k == SDLK_EQUALS || k == SDLK_KP_PLUS) {
                    if (find_single_selected_cam(&app.mech) >= 0) app_adjust_cam_lift(&app, CAM_LIFT_SCALE);
                    else app_adjust_motor_speed(&app, MOTOR_SPEED_STEP_DEG_S);
                } else if (k == SDLK_MINUS || k == SDLK_KP_MINUS) {
                    if (find_single_selected_cam(&app.mech) >= 0) app_adjust_cam_lift(&app, 1.0 / CAM_LIFT_SCALE);
                    else app_adjust_motor_speed(&app, -MOTOR_SPEED_STEP_DEG_S);
                } else if (k == SDLK_t) {
                    app_toggle_trace(&app);
                } else if (k == SDLK_e) {
                    app_export(&app);
                } else if (k == SDLK_DELETE || k == SDLK_BACKSPACE) {
                    app_delete_selection(&app);
                } else if (k == SDLK_ESCAPE) {
                    if (app.cam_draw_armed) {
                        app.cam_draw_armed = false;
                        app.cam_stroke_count = 0;
                        printf("Cam tool cancelled.\n");
                    } else {
                        clear_selection(&app.mech);
                    }
                } else if (k == SDLK_r) {
                    app_start_run(&app);
                }
            } else if (ev.type == SDL_KEYDOWN && app.state == APP_RUNNING) {
                if (ev.key.keysym.sym == SDLK_r) app_stop_run(&app);
            } else if (ev.type == SDL_MOUSEWHEEL) {
                int mx, my;
                SDL_GetMouseState(&mx, &my);
                if (!ui_contains(&app.toolbar, mx, my)) {
                    Vec2 screen_mouse = { (double)mx, (double)my };
                    Vec2 world_before = screen_to_world(screen_mouse, app.view_pan, app.view_zoom);

                    double factor = (ev.wheel.y > 0) ? ZOOM_STEP : 1.0 / ZOOM_STEP;
                    double new_zoom = app.view_zoom * factor;
                    if (new_zoom < ZOOM_MIN) new_zoom = ZOOM_MIN;
                    if (new_zoom > ZOOM_MAX) new_zoom = ZOOM_MAX;

                    app.view_pan.x = screen_mouse.x - world_before.x * new_zoom;
                    app.view_pan.y = screen_mouse.y - world_before.y * new_zoom;
                    app.view_zoom = new_zoom;
                }
            } else if (app.state == APP_EDIT && ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_LEFT &&
                        app.cam_draw_armed) {
                Vec2 p = screen_to_world((Vec2){ (double)ev.button.x, (double)ev.button.y }, app.view_pan, app.view_zoom);
                app.drag_mode = DRAG_DRAW_CAM;
                app.drag_start = p;
                app.cam_stroke_count = 0;
                stroke_push(&app, p);
            } else if (app.state == APP_EDIT && ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_LEFT) {
                Vec2 p = screen_to_world((Vec2){ (double)ev.button.x, (double)ev.button.y }, app.view_pan, app.view_zoom);
                bool shift = (SDL_GetModState() & KMOD_SHIFT) != 0;
                int hit_conn = mechanism_pick_connector(&app.mech, p, CONNECTOR_HIT_RADIUS / app.view_zoom);
                if (hit_conn >= 0) {
                    if (shift) {
                        app.mech.connectors[hit_conn].selected = !app.mech.connectors[hit_conn].selected;
                        app.drag_mode = DRAG_NONE;
                    } else {
                        if (!app.mech.connectors[hit_conn].selected) {
                            clear_selection(&app.mech);
                            app.mech.connectors[hit_conn].selected = true;
                        }
                        push_undo(&app);
                        app.drag_mode = DRAG_MOVE_CONNECTORS;
                        app.drag_last = p;
                    }
                } else {
                    int hit_link = mechanism_pick_link_edge(&app.mech, p, LINK_EDGE_HIT_DIST / app.view_zoom);
                    int hit_cam = (hit_link >= 0) ? -1
                                    : mechanism_pick_cam(&app.mech, p, CAM_HIT_DIST / app.view_zoom);
                    if (hit_link >= 0) {
                        clear_selection(&app.mech);
                        app.mech.links[hit_link].selected = true;
                        app.drag_mode = DRAG_NONE;
                    } else if (hit_cam >= 0) {
                        clear_selection(&app.mech);
                        app.mech.cams[hit_cam].selected = true;
                        app.drag_mode = DRAG_NONE;
                    } else {
                        app.drag_mode = DRAG_PENDING_EMPTY;
                        app.drag_start = p;
                        app.drag_current = p;
                    }
                }
            } else if (app.state == APP_EDIT && ev.type == SDL_MOUSEBUTTONUP && ev.button.button == SDL_BUTTON_LEFT &&
                        app.drag_mode == DRAG_DRAW_CAM) {
                if (app.cam_stroke_count >= 3) {
                    app_create_cam(&app, app.cam_stroke, app.cam_stroke_count, app.drag_start);
                } else {
                    /* A click rather than a drag: give them a working default
                     * cam right there instead of nothing. */
                    app_create_cam(&app, NULL, 0, app.drag_start);
                }
                app.cam_stroke_count = 0;
                app.cam_draw_armed = false;
                app.drag_mode = DRAG_NONE;
            } else if (app.state == APP_EDIT && ev.type == SDL_MOUSEBUTTONUP && ev.button.button == SDL_BUTTON_LEFT) {
                if (app.drag_mode == DRAG_PENDING_EMPTY) {
                    push_undo(&app);
                    clear_selection(&app.mech);
                    int id = mechanism_add_connector(&app.mech, app.drag_start, false);
                    app.mech.connectors[id].selected = true;
                } else if (app.drag_mode == DRAG_BOX_SELECT) {
                    double x0 = fmin(app.drag_start.x, app.drag_current.x), x1 = fmax(app.drag_start.x, app.drag_current.x);
                    double y0 = fmin(app.drag_start.y, app.drag_current.y), y1 = fmax(app.drag_start.y, app.drag_current.y);
                    clear_selection(&app.mech);
                    for (int i = 0; i < app.mech.connector_count; i++) {
                        Connector *c = &app.mech.connectors[i];
                        if (c->alive && c->pos.x >= x0 && c->pos.x <= x1 && c->pos.y >= y0 && c->pos.y <= y1) {
                            c->selected = true;
                        }
                    }
                }
                app.drag_mode = DRAG_NONE;
            }
        }

        Uint32 now = SDL_GetTicks();
        double dt = (double)(now - last_ticks) / 1000.0;
        last_ticks = now;

        if (app.state == APP_RUNNING && !app.jammed) {
            if (dt > 0.05) dt = 0.05; /* clamp huge stalls (e.g. window drag) */

            for (int i = 0; i < app.pre_run_count; i++) app.frame_positions[i] = app.mech.connectors[i].pos;
            for (int i = 0; i < app.frame_link_count; i++) app.frame_angles[i] = app.mech.links[i].accumulated_angle_rad;

            SolverParams frame_params = app.params;
            frame_params.gravity = effective_gravity(&app);
            solver_advance(&app.mech, dt, frame_params);

            if (solver_has_length_violation(&app.mech, app.params.length_tol_abs, app.params.length_tol_rel)) {
                /* This step would only be reachable by stretching a
                 * fixed-length link, so roll it back entirely and stop:
                 * a rigid link is rigid, so the mechanism binds instead. */
                for (int i = 0; i < app.pre_run_count; i++) app.mech.connectors[i].pos = app.frame_positions[i];
                for (int i = 0; i < app.frame_link_count; i++) app.mech.links[i].accumulated_angle_rad = app.frame_angles[i];
                app.jammed = true;
                printf("Mechanism jammed: a fixed-length link would have to change length here. "
                        "Press STOP (or R), then adjust the geometry (or press VARY to let a link's length vary).\n");
            } else {
                app.sim_time += dt;
                mechanism_trace_step(&app.mech, app.sim_time);
            }
        }

        ui_apply_state(&app.toolbar, app_ui_state(&app));

        SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
        SDL_RenderClear(ren);

        /* Keep the mechanism inside the canvas so long links can't draw over
         * the toolbar. */
        SDL_Rect canvas_clip = { UI_TOOLBAR_W, 0, CANVAS_W, CANVAS_H };
        SDL_RenderSetClipRect(ren, &canvas_clip);
        draw_mechanism(ren, &app.mech, app.drag_mode, app.drag_start, app.drag_current, app.view_pan, app.view_zoom);

        /* The cam outline being drawn, closed back to its start so what you
         * see is the shape that will actually be read. */
        if (app.cam_stroke_count >= 2) {
            for (int i = 1; i <= app.cam_stroke_count; i++) {
                Vec2 a0 = world_to_screen(app.cam_stroke[i - 1], app.view_pan, app.view_zoom);
                Vec2 a1 = world_to_screen(app.cam_stroke[i % app.cam_stroke_count], app.view_pan, app.view_zoom);
                bool closing = (i == app.cam_stroke_count);
                if (closing) render_line(ren, a0, a1, 120, 100, 150, 255);
                else render_line(ren, a0, a1, 200, 175, 235, 255);
            }
        }
        if (app.cam_draw_armed) {
            const char *hint = "DRAG TO DRAW THE CAM OUTLINE   CLICK FOR A DEFAULT CAM   ESC TO CANCEL";
            double w = render_text_width(9.0, hint);
            render_text(ren, (Vec2){ UI_TOOLBAR_W + (CANVAS_W - w) / 2.0, 16.0 }, 9.0, hint,
                        200, 175, 235, 255);
        }
        SDL_RenderSetClipRect(ren, NULL);

        render_plot(ren, &app.mech, (UiRect){ UI_TOOLBAR_W, CANVAS_H, CANVAS_W, PLOT_H });
        render_toolbar(ren, &app.toolbar, WIN_H);
        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }

    free(app.cam_stroke);
    free(app.pre_run_positions);
    free(app.frame_positions);
    free(app.frame_angles);
    mechanism_free(&app.mech);
    clear_stack(app.undo_stack, &app.undo_count);
    clear_stack(app.redo_stack, &app.redo_count);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
