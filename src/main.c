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
#include "synth.h"
#include "templates.h"

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
#define JOINT_HIT_DIST 7.0       /* pitch circles, Geneva rims, slider rails */
#define GENEVA_DEFAULT_SLOTS 6
#define GENEVA_MIN_SLOTS 3       /* below three the slots overlap */
#define GENEVA_MAX_SLOTS 12
#define TOOLTIP_DELAY_MS 350       /* dwell before a tooltip appears */
#define DEFAULT_WHEEL_RADIUS 80.0
#define WHEEL_RESIZE_STEP 1.15       /* per +/- press */
#define CAM_LIFT_SCALE 1.12          /* per +/- press */
#define CAM_TIMING_STEP (5.0 * M_PI / 180.0)
#define CAM_STROKE_MIN_SPACING 3.0   /* world units between recorded points */
#define CAM_DEFAULT_RADIUS 90.0      /* a click, rather than a drawn outline */
#define PATH_GHOST_POINTS 220        /* how finely the target is kept for drawing */
#define PATH_BASE_SPEED_DEG_S 45.0   /* the slowest arm: one cycle every 8 seconds */
#define GALLERY_COLS 3
#define GALLERY_TILE_W 190
#define GALLERY_TILE_H 150
#define GALLERY_PAD 10
#define PATH_TARGET_POINTS 64        /* the drawn path, resampled for four-bar fitting */
#define PATH_POOR_FIT_FRACTION 0.05  /* above this, say the linkage isn't up to the path */
#define PATH_TARGET_FRACTION 0.0025  /* stop adding arms below this share of the path's size */

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

/* The two machines a drawn path can be turned into. A four-bar is five parts
 * and one motor but can only trace the curves four-bars trace; a chain of
 * arms will follow anything at the cost of dozens of parts and a motor each. */
typedef enum { PATH_TOOL_NONE, PATH_TOOL_LINKAGE, PATH_TOOL_ARMS } PathTool;

typedef enum {
    DRAG_NONE,
    DRAG_MOVE_CONNECTORS,
    DRAG_PENDING_EMPTY,
    DRAG_BOX_SELECT,
    DRAG_DRAW_CAM,
    DRAG_DRAW_PATH
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

    /* PATH works the same way, but what it does with the stroke is decompose
     * it into a chain of rotating arms that redraws it. */
    PathTool path_tool;          /* PATH_TOOL_NONE unless one is armed */
    Vec2 *path_stroke;
    int path_stroke_count, path_stroke_capacity;

    /* The drawn path is kept after synthesis and drawn faintly, so the
     * generated curve can be compared against what was asked for. */
    Vec2 path_ghost[PATH_GHOST_POINTS];
    int path_ghost_count;
    bool path_ghost_closed;

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

    bool gallery_open;

    Toolbar toolbar;
} App;

static void clear_selection(Mechanism *m) {
    for (int i = 0; i < m->connector_count; i++) m->connectors[i].selected = false;
    for (int i = 0; i < m->link_count; i++) m->links[i].selected = false;
    for (int i = 0; i < m->cam_count; i++) m->cams[i].selected = false;
    for (int i = 0; i < m->gear_count; i++) m->gears[i].selected = false;
    for (int i = 0; i < m->geneva_count; i++) m->genevas[i].selected = false;
    for (int i = 0; i < m->slider_count; i++) m->sliders[i].selected = false;
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

static int find_single_selected_geneva(const Mechanism *m) {
    int found = -1;
    for (int i = 0; i < m->geneva_count; i++) {
        if (m->genevas[i].alive && m->genevas[i].selected) {
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
    for (int i = 0; i < m->gear_count; i++) {
        if (m->gears[i].alive && m->gears[i].selected) return true;
    }
    for (int i = 0; i < m->geneva_count; i++) {
        if (m->genevas[i].alive && m->genevas[i].selected) return true;
    }
    for (int i = 0; i < m->slider_count; i++) {
        if (m->sliders[i].alive && m->sliders[i].selected) return true;
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
    /* Naming a wheel as the driver turns its whole train round to suit, so it
     * works on any wheel rather than only the one the train was meshed from. */
    bool wheel = mechanism_is_gear_body(&a->mech, lid);
    if (wheel && !a->mech.links[lid].is_driven) mechanism_orient_train_from(&a->mech, lid);
    if (!mechanism_toggle_driven(&a->mech, lid, DEFAULT_MOTOR_SPEED_DEG_S)) {
        discard_last_undo(a);
        printf("A driven link needs exactly one anchor connector.\n");
        return;
    }
    if (wheel && a->mech.links[lid].is_driven) {
        printf("This wheel now drives; the rest of its train follows from it.\n");
    }
}

/* Records a point of a stroke being drawn, thinning out samples that are too
 * close together to matter. Shared by the cam and path tools. */
static void stroke_push(Vec2 **pts, int *count, int *capacity, Vec2 p) {
    if (*count > 0 && vec2_dist((*pts)[*count - 1], p) < CAM_STROKE_MIN_SPACING) return;
    if (*count >= *capacity) {
        int cap = (*capacity == 0) ? 64 : *capacity * 2;
        *pts = realloc(*pts, (size_t)cap * sizeof(Vec2));
        *capacity = cap;
    }
    (*pts)[(*count)++] = p;
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

/* ---- Making the sliding, gear and Geneva joints by hand -----------------
 *
 * Each of these is inferred from a plain connector selection rather than
 * asking you to nominate roles in some order: with only pins to click, there
 * is nothing to get wrong and nothing to remember.
 * ---------------------------------------------------------------------- */

/* The live link containing `cid`, or -1. */
static int link_containing(const Mechanism *m, int cid) {
    for (int li = 0; li < m->link_count; li++) {
        const Link *l = &m->links[li];
        if (!l->alive) continue;
        for (int i = 0; i < l->connector_count; i++) {
            if (l->connector_ids[i] == cid) return li;
        }
    }
    return -1;
}

/* Of three selected connectors, the two farthest apart are the rail and the
 * remaining one is the pin. A slider's rail is the long guide and the pin sits
 * on it, so this is what the selection means every time. */
static bool slider_roles(const Mechanism *m, int *pin, int *rail_a, int *rail_b) {
    int ids[4];
    if (gather_selected_connectors(m, ids, 4) != 3) return false;
    double best = -1.0;
    int ba = 0, bb = 1;
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 3; j++) {
            double d = vec2_dist(m->connectors[ids[i]].pos, m->connectors[ids[j]].pos);
            if (d > best) { best = d; ba = i; bb = j; }
        }
    }
    if (best < 1e-6) return false;
    *rail_a = ids[ba];
    *rail_b = ids[bb];
    *pin = ids[3 - ba - bb];   /* the remaining index of {0,1,2} */
    return true;
}

/* Two selected centres mesh. Two grounded ones are a gear pair -- the motor,
 * if there is one, is the driver; one grounded and one on a free bar is a
 * rack and pinion, the grounded one being the pinion. */
static bool gear_roles(const Mechanism *m, int *pinion_link, int *pinion_centre,
                        int *other_link, int *other_ref, bool *is_rack) {
    int ids[3];
    if (gather_selected_connectors(m, ids, 3) != 2) return false;
    int la = link_containing(m, ids[0]), lb = link_containing(m, ids[1]);
    if (la < 0 || lb < 0 || la == lb) return false;

    bool anchor_a = m->connectors[ids[0]].is_anchor, anchor_b = m->connectors[ids[1]].is_anchor;
    if (anchor_a && anchor_b) {
        /* Two grounded bodies: a gear pair. The motor, if there is one, drives. */
        bool driven_a = m->links[la].is_driven;
        int first = driven_a ? 0 : (m->links[lb].is_driven ? 1 : 0);
        *pinion_link = first ? lb : la;
        *pinion_centre = ids[first];
        *other_link = first ? la : lb;
        *other_ref = ids[1 - first];
        *is_rack = false;
        return true;
    }
    if (anchor_a != anchor_b) {
        int p = anchor_a ? 0 : 1;               /* the grounded one is the pinion */
        *pinion_link = p ? lb : la;
        *pinion_centre = ids[p];
        *other_link = p ? la : lb;
        *other_ref = ids[1 - p];
        *is_rack = true;
        return true;
    }
    return false;
}

/* Where a freshly inserted assembly should land. */
static Vec2 app_view_centre(const App *a) {
    return screen_to_world((Vec2){ UI_TOOLBAR_W + CANVAS_W / 2.0, CANVAS_H / 2.0 },
                            a->view_pan, a->view_zoom);
}

/* Each of these three buttons builds a complete, running assembly if you have
 * nothing useful selected -- you should not have to lay out the right
 * arrangement of parts before the program will let you name a joint. If you DO
 * have the right selection, it uses that instead, so a joint can still be
 * added to something you built yourself. Either way every dimension afterwards
 * is just geometry: drag a part and the sizes follow. */

static void app_add_slider(App *a) {
    int pin, ra, rb;
    if (slider_roles(&a->mech, &pin, &ra, &rb)) {
        push_undo(a);
        if (mechanism_add_slider(&a->mech, pin, ra, rb) < 0) {
            discard_last_undo(a);
            printf("Those three don't make a slider.\n");
            return;
        }
        printf("Slider added: that pin now runs along the line through the other two.\n");
        return;
    }

    /* Nothing suitable selected: draw out a whole crank-slider. */
    Vec2 c = app_view_centre(a);
    push_undo(a);
    clear_selection(&a->mech);
    int o    = mechanism_add_connector(&a->mech, (Vec2){ c.x - 220, c.y }, true);
    int crk  = mechanism_add_connector(&a->mech, (Vec2){ c.x - 160, c.y }, false);
    int pist = mechanism_add_connector(&a->mech, (Vec2){ c.x + 40,  c.y }, false);
    int r1   = mechanism_add_connector(&a->mech, (Vec2){ c.x - 100, c.y }, true);
    int r2   = mechanism_add_connector(&a->mech, (Vec2){ c.x + 180, c.y }, true);
    int crank_ids[2] = { o, crk }, rod_ids[2] = { crk, pist };
    int crank = mechanism_add_link(&a->mech, crank_ids, 2);
    mechanism_add_link(&a->mech, rod_ids, 2);
    mechanism_toggle_driven(&a->mech, crank, DEFAULT_MOTOR_SPEED_DEG_S);
    mechanism_add_slider(&a->mech, pist, r1, r2);
    mechanism_set_traced(&a->mech, pist, true);
    a->mech.connectors[crk].selected = true;
    printf("Crank-slider added. Drag the crank pin to change the stroke, or either "
            "rail anchor to aim the slide.\n");
}

/* Pans and zooms so the whole mechanism is on screen. Only ever zooms out --
 * a train that has grown past the edge should come into view without the rest
 * of the canvas suddenly changing scale under a mechanism that already fitted. */
static void app_fit_view(App *a) {
    const Mechanism *m = &a->mech;
    double x0 = 1e30, y0 = 1e30, x1 = -1e30, y1 = -1e30;
    bool any = false;
    for (int i = 0; i < m->connector_count; i++) {
        if (!m->connectors[i].alive) continue;
        Vec2 p = m->connectors[i].pos;
        if (p.x < x0) x0 = p.x;
        if (p.y < y0) y0 = p.y;
        if (p.x > x1) x1 = p.x;
        if (p.y > y1) y1 = p.y;
        any = true;
    }
    /* Pitch circles and Geneva wheels reach beyond their centres. */
    for (int i = 0; i < m->gear_count; i++) {
        const Gear *g = &m->gears[i];
        if (!g->alive) continue;
        const int cs[2] = { g->driver_center_id, g->driven_center_id };
        const double rs[2] = { g->driver_radius, g->driven_radius };
        for (int k = 0; k < 2; k++) {
            if (cs[k] < 0 || !m->connectors[cs[k]].alive) continue;
            Vec2 c = m->connectors[cs[k]].pos;
            if (c.x - rs[k] < x0) x0 = c.x - rs[k];
            if (c.y - rs[k] < y0) y0 = c.y - rs[k];
            if (c.x + rs[k] > x1) x1 = c.x + rs[k];
            if (c.y + rs[k] > y1) y1 = c.y + rs[k];
        }
    }
    if (!any) return;

    const double margin = 40.0;
    double w = (x1 - x0) + 2 * margin, h = (y1 - y0) + 2 * margin;
    if (w < 1.0) w = 1.0;
    if (h < 1.0) h = 1.0;
    double zoom = fmin((double)CANVAS_W / w, (double)CANVAS_H / h);
    if (zoom > a->view_zoom) zoom = a->view_zoom;   /* never zoom in */
    if (zoom < ZOOM_MIN) zoom = ZOOM_MIN;
    a->view_zoom = zoom;
    Vec2 mid = { (x0 + x1) / 2.0, (y0 + y1) / 2.0 };
    a->view_pan.x = UI_TOOLBAR_W + CANVAS_W / 2.0 - mid.x * zoom;
    a->view_pan.y = CANVAS_H / 2.0 - mid.y * zoom;
}

/* Every alive wheel that is currently selected, in link order. */
static int gather_selected_wheels(const Mechanism *m, int *out, int max_out) {
    int n = 0;
    for (int li = 0; li < m->link_count; li++) {
        if (!m->links[li].alive || !m->links[li].selected) continue;
        if (!mechanism_is_gear_body(m, li)) continue;
        if (out && n < max_out) out[n] = li;
        n++;
    }
    return n;
}

/* Somewhere clear to put a new wheel: near the middle of the view, but stepped
 * outwards until it lands on nothing. Dropping a wheel on top of one already
 * there is never what was wanted. */
static Vec2 free_wheel_spot(const App *a, double radius) {
    Vec2 home = app_view_centre(a);
    const double gap = 24.0;
    for (int ring = 0; ring < 24; ring++) {
        int steps = (ring == 0) ? 1 : 8;
        for (int k = 0; k < steps; k++) {
            double ang = (2.0 * M_PI * k) / (double)steps;
            double reach = ring * (radius * 1.6 + gap);
            Vec2 p = { home.x + reach * cos(ang), home.y + reach * sin(ang) };
            bool clash = false;
            for (int li = 0; li < a->mech.link_count && !clash; li++) {
                int centre = -1;
                double r = mechanism_gear_wheel_radius(&a->mech, li, &centre);
                if (!(r > 0.0) || centre < 0) continue;
                if (vec2_dist(p, a->mech.connectors[centre].pos) < r + radius + gap) clash = true;
            }
            if (!clash) return p;
        }
    }
    return home;
}

/* GEAR does one of two things, and which one is plain from what is selected.
 *
 *   nothing (or one wheel):  put down a single new wheel, meshed with nothing
 *   two or more wheels:      mesh them, in the order they were made
 *
 * So wheels are placed and sized first and connected afterwards -- one press,
 * one wheel, never a pile of them on the same spot. */
static void app_add_gear(App *a) {
    int wheels[16];
    int n = gather_selected_wheels(&a->mech, wheels, 16);
    if (n > 16) n = 16;

    if (n >= 2) {
        push_undo(a);
        int meshed = 0, refused = 0;
        for (int k = 0; k + 1 < n; k++) {
            if (mechanism_mesh_wheels(&a->mech, wheels[k], wheels[k + 1]) >= 0) meshed++;
            else refused++;
        }
        if (meshed == 0) {
            discard_last_undo(a);
            printf("Those wheels are meshed already, or the second one is turned by "
                    "something else -- a wheel takes its motion from one place.\n");
            return;
        }
        app_fit_view(a);
        printf("Meshed %d pair%s; each driven wheel slid into contact with its driver.%s "
                "Press M on a wheel to make it the one that drives.\n",
                meshed, meshed == 1 ? "" : "s",
                refused ? " Some were already meshed and were left alone." : "");
        return;
    }

    /* A rack still comes from picking a pinion and a bar, which is a different
     * thing entirely and has nowhere else to live. */
    int pl, pc, ol, oref;
    bool rack = false;
    if (gear_roles(&a->mech, &pl, &pc, &ol, &oref, &rack) && rack) {
        push_undo(a);
        const Link *bar = &a->mech.links[ol];
        Vec2 b0 = a->mech.connectors[bar->connector_ids[0]].pos;
        Vec2 b1 = a->mech.connectors[bar->connector_ids[1]].pos;
        if (mechanism_add_rack(&a->mech, pl, pc, ol, oref, vec2_sub(b1, b0)) < 0) {
            discard_last_undo(a);
            printf("Those two don't make a rack and pinion.\n");
            return;
        }
        printf("Rack and pinion made. The pinion's pitch radius is how far it stands "
                "off the bar, so sliding the bar retimes it.\n");
        return;
    }

    /* Otherwise: one wheel, on its own, somewhere clear. */
    push_undo(a);
    double r = DEFAULT_WHEEL_RADIUS;
    int link = mechanism_add_wheel(&a->mech, free_wheel_spot(a, r), r);
    if (link < 0) { discard_last_undo(a); printf("Couldn't place a wheel.\n"); return; }
    clear_selection(&a->mech);
    a->mech.links[link].selected = true;
    int mark = mechanism_wheel_mark(&a->mech, link);
    if (mark >= 0) mechanism_set_traced(&a->mech, mark, true);
    app_fit_view(a);   /* wheels step outwards to find space; keep them in view */
    printf("Wheel added, radius %.0f. +/- resizes it, M makes it the driver, and "
            "selecting two or more wheels and pressing GEAR meshes them.\n", r);
}

static void app_add_geneva(App *a) {
    int ids[3];
    if (gather_selected_connectors(&a->mech, ids, 3) == 2 &&
        a->mech.connectors[ids[0]].is_anchor && a->mech.connectors[ids[1]].is_anchor) {
        int la = link_containing(&a->mech, ids[0]), lb = link_containing(&a->mech, ids[1]);
        if (la >= 0 && lb >= 0 && la != lb &&
            a->mech.links[la].is_driven != a->mech.links[lb].is_driven) {
            bool driven_a = a->mech.links[la].is_driven;
            int drv = driven_a ? la : lb, drv_c = driven_a ? ids[0] : ids[1];
            int whl = driven_a ? lb : la, whl_c = driven_a ? ids[1] : ids[0];
            push_undo(a);
            int gid = mechanism_add_geneva(&a->mech, drv, drv_c, whl, whl_c, GENEVA_DEFAULT_SLOTS);
            if (gid < 0) { discard_last_undo(a); printf("Those two don't make a Geneva.\n"); return; }
            a->mech.genevas[gid].selected = true;
            printf("Geneva added, %d slots. Select the wheel and use +/- to change the "
                    "slot count.\n", a->mech.genevas[gid].slot_count);
            return;
        }
    }

    /* Nothing suitable selected: draw out the whole thing. */
    Vec2 c = app_view_centre(a);
    push_undo(a);
    clear_selection(&a->mech);
    int slots = GENEVA_DEFAULT_SLOTS;
    double dist = 170.0;
    double a_r = geneva_crank_radius(slots, dist);
    double entry = geneva_engagement_half_angle(slots);

    int dc = mechanism_add_connector(&a->mech, (Vec2){ c.x - dist / 2, c.y }, true);
    int pin = mechanism_add_connector(&a->mech,
                  (Vec2){ c.x - dist / 2 + a_r * cos(entry), c.y - a_r * sin(entry) }, false);
    int wc = mechanism_add_connector(&a->mech, (Vec2){ c.x + dist / 2, c.y }, true);
    int mark = mechanism_add_connector(&a->mech, (Vec2){ c.x + dist / 2 + dist * 0.45, c.y }, false);
    int drv_ids[2] = { dc, pin }, whl_ids[2] = { wc, mark };
    int drv = mechanism_add_link(&a->mech, drv_ids, 2);
    int whl = mechanism_add_link(&a->mech, whl_ids, 2);
    mechanism_toggle_driven(&a->mech, drv, DEFAULT_MOTOR_SPEED_DEG_S);
    int gid = mechanism_add_geneva(&a->mech, drv, dc, whl, wc, slots);
    mechanism_set_traced(&a->mech, mark, true);
    if (gid >= 0) a->mech.genevas[gid].selected = true;
    printf("Geneva added, %d slots -- one turn of the motor indexes it %.0f degrees. "
            "Use +/- to change the slot count, or drag a centre to resize it.\n",
            slots, 360.0 / slots);
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

/* +/- on a selected wheel changes its own size. Nothing else moves except a
 * wheel meshed to it, which slides along to stay in contact. */
static void app_adjust_wheel_radius(App *a, double factor) {
    int wheels[2];
    if (gather_selected_wheels(&a->mech, wheels, 2) != 1) return;
    int centre = -1;
    double r = mechanism_gear_wheel_radius(&a->mech, wheels[0], &centre);
    if (!(r > 0.0)) return;
    push_undo(a);
    mechanism_set_wheel_radius(&a->mech, wheels[0], r * factor);
    mechanism_refresh_joint_sizes(&a->mech);
    printf("Wheel radius %.0f.\n", a->mech.links[wheels[0]].wheel_radius);
}

/* +/- on a selected Geneva changes the number of slots, which is the whole
 * character of the mechanism: fewer slots means a bigger, faster index. */
static void app_adjust_geneva_slots(App *a, int delta) {
    int vi = find_single_selected_geneva(&a->mech);
    if (vi < 0) return;
    Geneva *gv = &a->mech.genevas[vi];
    int n = gv->slot_count + delta;
    if (n < GENEVA_MIN_SLOTS) n = GENEVA_MIN_SLOTS;
    if (n > GENEVA_MAX_SLOTS) n = GENEVA_MAX_SLOTS;
    if (n == gv->slot_count) return;
    push_undo(a);
    a->mech.genevas[vi].slot_count = n;
    mechanism_refresh_joint_sizes(&a->mech);
    printf("Geneva: %d slots, indexing %.0f degrees a turn.\n", n, 360.0 / n);
}

/* The gallery is a grid of tiles laid over the canvas. */
static UiRect gallery_panel(void) {
    int rows = (templates_count() + GALLERY_COLS - 1) / GALLERY_COLS;
    int w = GALLERY_COLS * GALLERY_TILE_W + (GALLERY_COLS + 1) * GALLERY_PAD;
    int h = rows * GALLERY_TILE_H + (rows + 1) * GALLERY_PAD + 26;
    return (UiRect){ UI_TOOLBAR_W + (CANVAS_W - w) / 2, (CANVAS_H - h) / 2, w, h };
}

static UiRect gallery_tile(int index) {
    UiRect panel = gallery_panel();
    int col = index % GALLERY_COLS, row = index / GALLERY_COLS;
    return (UiRect){ panel.x + GALLERY_PAD + col * (GALLERY_TILE_W + GALLERY_PAD),
                      panel.y + 26 + GALLERY_PAD + row * (GALLERY_TILE_H + GALLERY_PAD),
                      GALLERY_TILE_W, GALLERY_TILE_H };
}

static int gallery_hit(int x, int y) {
    for (int i = 0; i < templates_count(); i++) {
        UiRect r = gallery_tile(i);
        if (x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h) return i;
    }
    return -1;
}

/* Draws a real miniature of a template by building it, measuring it, and
 * scaling it into the tile -- so a tile can never disagree with what the
 * button actually inserts. */
static void draw_template_preview(SDL_Renderer *ren, int index, UiRect tile, bool hot) {
    const Template *t = templates_get(index);
    if (!t) return;

    Mechanism preview;
    mechanism_init(&preview);
    t->build(&preview, (Vec2){ 0, 0 }, 1.0);

    Vec2 lo = { 1e30, 1e30 }, hi = { -1e30, -1e30 };
    bool any = false;
    for (int i = 0; i < preview.connector_count; i++) {
        if (!preview.connectors[i].alive) continue;
        Vec2 q = preview.connectors[i].pos;
        lo.x = fmin(lo.x, q.x); lo.y = fmin(lo.y, q.y);
        hi.x = fmax(hi.x, q.x); hi.y = fmax(hi.y, q.y);
        any = true;
    }

    if (any) {
        double pad = 16.0;
        double art_h = tile.h - 26.0;
        double sx = (hi.x - lo.x > 1e-6) ? (tile.w - 2 * pad) / (hi.x - lo.x) : 1.0;
        double sy = (hi.y - lo.y > 1e-6) ? (art_h - 2 * pad) / (hi.y - lo.y) : 1.0;
        double sc = fmin(sx, sy);
        Vec2 mid = { (lo.x + hi.x) / 2.0, (lo.y + hi.y) / 2.0 };
        Vec2 org = { tile.x + tile.w / 2.0, tile.y + art_h / 2.0 };

        for (int li = 0; li < preview.link_count; li++) {
            const Link *l = &preview.links[li];
            if (!l->alive) continue;
            Uint8 cr = l->is_driven ? 230 : 150, cg = l->is_driven ? 110 : 158, cb = l->is_driven ? 90 : 172;
            for (int i = 0; i < l->connector_count; i++) {
                for (int j = i + 1; j < l->connector_count; j++) {
                    Vec2 pa = preview.connectors[l->connector_ids[i]].pos;
                    Vec2 pb = preview.connectors[l->connector_ids[j]].pos;
                    Vec2 sa = { org.x + (pa.x - mid.x) * sc, org.y + (pa.y - mid.y) * sc };
                    Vec2 sb = { org.x + (pb.x - mid.x) * sc, org.y + (pb.y - mid.y) * sc };
                    render_line(ren, sa, sb, cr, cg, cb, 255);
                }
            }
        }
        for (int i = 0; i < preview.connector_count; i++) {
            const Connector *c = &preview.connectors[i];
            if (!c->alive) continue;
            Vec2 sp = { org.x + (c->pos.x - mid.x) * sc, org.y + (c->pos.y - mid.y) * sc };
            if (c->is_anchor) render_circle(ren, sp, 3.0, 230, 160, 60, 255);
            else render_circle(ren, sp, 2.0, 210, 210, 215, 255);
        }
        /* Sliders show their rail, so a tile reads as "this one slides". */
        for (int si = 0; si < preview.slider_count; si++) {
            const Slider *sl = &preview.sliders[si];
            if (!sl->alive) continue;
            Vec2 pa = preview.connectors[sl->rail_a_id].pos, pb = preview.connectors[sl->rail_b_id].pos;
            Vec2 sa = { org.x + (pa.x - mid.x) * sc, org.y + (pa.y - mid.y) * sc };
            Vec2 sb = { org.x + (pb.x - mid.x) * sc, org.y + (pb.y - mid.y) * sc };
            render_dashed_line(ren, sa, sb, 4.0, 120, 190, 235, 255);
        }
    }
    mechanism_free(&preview);

    double w = render_text_width(9.0, t->name);
    render_text(ren, (Vec2){ tile.x + (tile.w - w) / 2.0, tile.y + tile.h - 16.0 }, 9.0, t->name,
                hot ? 255 : 205, hot ? 225 : 210, hot ? 70 : 220, 255);
}

static void app_toggle_gallery(App *a) {
    a->gallery_open = !a->gallery_open;
    if (a->gallery_open) {
        printf("Template gallery: click a mechanism to drop it on the canvas. Escape closes.\n");
    }
}

static void app_insert_template(App *a, int index) {
    const Template *t = templates_get(index);
    if (!t) return;
    push_undo(a);
    clear_selection(&a->mech);

    Vec2 centre = screen_to_world((Vec2){ UI_TOOLBAR_W + CANVAS_W / 2.0, CANVAS_H / 2.0 },
                                   a->view_pan, a->view_zoom);
    t->build(&a->mech, centre, 1.0);
    a->gallery_open = false;
    printf("Inserted %s -- %s. Press R to run it.\n", t->name, t->blurb);
}

static void app_arm_path_tool(App *a, PathTool tool) {
    a->path_tool = (a->path_tool == tool) ? PATH_TOOL_NONE : tool;
    a->path_stroke_count = 0;
    if (a->path_tool == PATH_TOOL_LINKAGE) {
        printf("Linkage tool armed: draw a curve and a four-bar will be fitted to it -- "
                "five parts and one motor, but only the curves a four-bar can trace. "
                "Escape cancels.\n");
    } else if (a->path_tool == PATH_TOOL_ARMS) {
        printf("Arms tool armed: draw any curve at all and a chain of rotating arms "
                "will be built to redraw it exactly. Escape cancels.\n");
    }
}

/* Turns a decomposition into real, editable mechanism parts: a grounded point
 * for the average position, then one motorised arm per term, each hung off the
 * tip of the last and turning at its own multiple of the base speed. The pen
 * is the final tip, traced. Nothing here is special-cased -- once built it is
 * an ordinary mechanism you can drag, retime and export. */
static void app_build_fourier_chain(App *a, Vec2 anchor, const FourierArm *arms, int count,
                                     double rms, double size, bool closed) {
    push_undo(a);
    clear_selection(&a->mech);

    int previous = mechanism_add_connector(&a->mech, anchor, true);
    Vec2 tip_pos = anchor;
    int built = 0;

    for (int i = 0; i < count; i++) {
        /* An arm far shorter than a pixel contributes nothing but clutter. */
        if (vec2_len(arms[i].amplitude) < size * 1e-4) continue;

        tip_pos = vec2_add(tip_pos, arms[i].amplitude);
        int tip = mechanism_add_connector(&a->mech, tip_pos, false);
        int ids[2] = { previous, tip };
        int link = mechanism_add_link(&a->mech, ids, 2);
        mechanism_set_driven_about(&a->mech, link, previous,
                                    (double)arms[i].harmonic * PATH_BASE_SPEED_DEG_S);
        previous = tip;
        built++;
    }

    if (built < 1) {
        discard_last_undo(a);
        printf("That path decomposed to nothing usable.\n");
        return;
    }

    mechanism_set_traced(&a->mech, previous, true);
    a->mech.connectors[previous].selected = true;

    printf("Built a %d-arm drawing machine for that %s path. Average miss %.2f units "
            "(%.2f%% of its size). Press R to watch it draw.\n",
            built, closed ? "closed" : "open", rms, size > 0.0 ? 100.0 * rms / size : 0.0);
}

/* Turns a fitted four-bar into real, editable mechanism parts: two grounded
 * pivots, the crank driven by a motor, a ternary coupler carrying the traced
 * point, and the rocker closing the loop. */
static void app_build_four_bar(App *a, const FourBar *fb, double error, double size) {
    Vec2 crank_end, coupler_end, traced;
    if (!fourbar_pose(fb, 0.0, &crank_end, &coupler_end, &traced)) {
        printf("The fitted linkage could not be assembled; nothing was added.\n");
        return;
    }

    push_undo(a);
    clear_selection(&a->mech);

    int o2 = mechanism_add_connector(&a->mech, fb->ground_a, true);
    int o4 = mechanism_add_connector(&a->mech, fb->ground_b, true);
    int pa = mechanism_add_connector(&a->mech, crank_end, false);
    int pb = mechanism_add_connector(&a->mech, coupler_end, false);
    int tip = mechanism_add_connector(&a->mech, traced, false);

    int crank_ids[2] = { o2, pa };
    int coupler_ids[3] = { pa, pb, tip };   /* ternary: the traced point rides the coupler */
    int rocker_ids[2] = { pb, o4 };
    int crank_link = mechanism_add_link(&a->mech, crank_ids, 2);
    mechanism_add_link(&a->mech, coupler_ids, 3);
    mechanism_add_link(&a->mech, rocker_ids, 2);
    mechanism_toggle_driven(&a->mech, crank_link, DEFAULT_MOTOR_SPEED_DEG_S);

    mechanism_set_traced(&a->mech, tip, true);
    a->mech.connectors[tip].selected = true;

    double percent = (size > 0.0) ? 100.0 * error / size : 0.0;
    printf("Fitted a four-bar: crank %.1f, coupler %.1f, rocker %.1f, ground %.1f. "
            "Average miss %.2f units (%.2f%% of the path). Press R to watch it trace.\n",
            fb->crank, fb->coupler, fb->rocker, vec2_dist(fb->ground_a, fb->ground_b),
            error, percent);
    if (error > size * PATH_POOR_FIT_FRACTION) {
        printf("That path is outside what a four-bar can trace. Undo and use ARMS "
                "for a machine that will follow it exactly.\n");
    }
}

static void app_synthesize_linkage(App *a) {
    int n = a->path_stroke_count;
    if (n < 4) {
        printf("That stroke is too short to fit a linkage to.\n");
        return;
    }
    bool closed = synth_stroke_is_closed(a->path_stroke, n);

    a->path_ghost_count = PATH_GHOST_POINTS;
    a->path_ghost_closed = closed;
    synth_resample(a->path_stroke, n, closed, a->path_ghost, PATH_GHOST_POINTS);

    Vec2 target[PATH_TARGET_POINTS];
    synth_resample(a->path_stroke, n, closed, target, PATH_TARGET_POINTS);
    double size = synth_path_size(a->path_stroke, n);

    printf("Searching for a four-bar that traces that %s path...\n", closed ? "closed" : "open");
    FourBar fb;
    double error = 0.0;
    if (!synth_fit_four_bar(target, PATH_TARGET_POINTS, closed, synth_default_params(), &fb, &error)) {
        printf("No four-bar linkage could be fitted to that path. Try ARMS instead.\n");
        return;
    }
    app_build_four_bar(a, &fb, error, size);
}

static void app_synthesize_arms(App *a) {
    int n = a->path_stroke_count;
    if (n < 4) {
        printf("That stroke is too short to build a mechanism from.\n");
        return;
    }

    bool closed = synth_stroke_is_closed(a->path_stroke, n);

    /* Keep the drawing to display behind the result. */
    a->path_ghost_count = PATH_GHOST_POINTS;
    a->path_ghost_closed = closed;
    synth_resample(a->path_stroke, n, closed, a->path_ghost, PATH_GHOST_POINTS);

    double size = synth_path_size(a->path_stroke, n);
    Vec2 anchor;
    FourierArm arms[SYNTH_MAX_ARMS];
    double rms = 0.0;
    int count = synth_fourier_fit(a->path_stroke, n, closed, SYNTH_MAX_ARMS,
                                   size * PATH_TARGET_FRACTION, &anchor, arms, &rms);
    if (count < 1) {
        printf("Couldn't read a usable path from that stroke.\n");
        return;
    }
    app_build_fourier_chain(a, anchor, arms, count, rms, size, closed);
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
    /* A joint can be selected in its own right, so it can be removed on its
     * own -- taking the mesh or the rail away and leaving the bars standing. */
    for (int i = 0; i < a->mech.gear_count; i++) {
        if (a->mech.gears[i].alive && a->mech.gears[i].selected) mechanism_delete_gear(&a->mech, i);
    }
    for (int i = 0; i < a->mech.geneva_count; i++) {
        if (a->mech.genevas[i].alive && a->mech.genevas[i].selected) mechanism_delete_geneva(&a->mech, i);
    }
    for (int i = 0; i < a->mech.slider_count; i++) {
        if (a->mech.sliders[i].alive && a->mech.sliders[i].selected) mechanism_delete_slider(&a->mech, i);
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
    case UI_TEMPLATE: app_toggle_gallery(a); break;
    case UI_JOINT:   app_add_joint(a); break;
    case UI_ANCHOR:  app_toggle_anchor(a); break;
    case UI_LINK:    app_link_selected(a); break;
    case UI_MOTOR:   app_toggle_motor(a); break;
    case UI_SLIDER:  app_add_slider(a); break;
    case UI_GEAR:    app_add_gear(a); break;
    case UI_GENEVA:  app_add_geneva(a); break;
    case UI_CAM:     app_toggle_cam_draw(a); break;
    case UI_LINKAGE: app_arm_path_tool(a, PATH_TOOL_LINKAGE); break;
    case UI_ARMS:    app_arm_path_tool(a, PATH_TOOL_ARMS); break;
    case UI_VARY:    app_toggle_vary(a); break;
    case UI_TRACE:   app_toggle_trace(a); break;
    case UI_DELETE:  app_delete_selection(a); break;
    case UI_UNDO:    app_undo(a); break;
    case UI_REDO:    app_redo(a); break;
    case UI_GRAVITY: app_toggle_gravity(a); break;
    case UI_CLEAR:
        mechanism_clear_traces(&a->mech);
        a->path_ghost_count = 0; /* the drawn target is a guide like any trace */
        break;
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
    s.drawing_linkage = (a->path_tool == PATH_TOOL_LINKAGE);
    s.drawing_arms = (a->path_tool == PATH_TOOL_ARMS);
    s.can_undo = (a->undo_count > 0);
    s.can_redo = (a->redo_count > 0);
    Vec2 g = effective_gravity(a);
    s.gravity_on = (g.x != 0.0 || g.y != 0.0);
    s.gallery_open = a->gallery_open;
    {
        int pin, ra, rb, pl, pc, ol, oref;
        bool rack;
        /* Always offered: with nothing selected they draw a whole assembly. */
        (void)slider_roles(&a->mech, &pin, &ra, &rb);
        (void)gear_roles(&a->mech, &pl, &pc, &ol, &oref, &rack);
        s.can_make_slider = true;
        s.can_make_gear = true;
        s.can_make_geneva = true;
    }
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

    /* Rails, pitch circles and Geneva wheels: guides that show what kind of
     * joint is at work, drawn behind the parts themselves. */
    for (int si = 0; si < m->slider_count; si++) {
        const Slider *sl = &m->sliders[si];
        if (!sl->alive) continue;
        if (!m->connectors[sl->rail_a_id].alive || !m->connectors[sl->rail_b_id].alive) continue;
        Vec2 a = m->connectors[sl->rail_a_id].pos, b = m->connectors[sl->rail_b_id].pos;
        Vec2 d = vec2_sub(b, a);
        double len = vec2_len(d);
        if (len < 1e-9) continue;
        /* Extend the rail past its ends: the constraint is the whole line. */
        Vec2 u = vec2_scale(d, 1.0 / len);
        Vec2 e0 = vec2_sub(a, vec2_scale(u, len * 0.15));
        Vec2 e1 = vec2_add(b, vec2_scale(u, len * 0.15));
        Uint8 rr = sl->selected ? 255 : 105, rg = sl->selected ? 225 : 150,
              rb = sl->selected ? 70 : 190;
        render_dashed_line(ren, world_to_screen(e0, view_pan, view_zoom),
                           world_to_screen(e1, view_pan, view_zoom), 6.0, rr, rg, rb, 255);
        if (m->connectors[sl->pin_connector_id].alive) {
            Vec2 pin = world_to_screen(m->connectors[sl->pin_connector_id].pos, view_pan, view_zoom);
            render_circle_outline(ren, pin, 7.0 * view_zoom, 130, 190, 235, 255);
        }
    }

    /* Wheels are drawn from the bodies themselves, not from the meshes, so a
     * wheel put down on its own shows up straight away and is there to be
     * sized and moved before anything is connected to it. */
    for (int li = 0; li < m->link_count; li++) {
        int centre = -1;
        double r = mechanism_gear_wheel_radius(m, li, &centre);
        if (!(r > 0.0) || centre < 0 || !m->connectors[centre].alive) continue;
        const Link *l = &m->links[li];

        Uint8 cr, cg, cb;
        if (l->selected) { cr = 255; cg = 225; cb = 70; }
        else if (l->is_driven) { cr = 230; cg = 90; cb = 70; }   /* the one that drives */
        else if (l->driven_externally) { cr = 165; cg = 185; cb = 130; }  /* meshed */
        else { cr = 120; cg = 130; cb = 145; }                   /* loose, not meshed yet */

        Vec2 c0 = world_to_screen(m->connectors[centre].pos, view_pan, view_zoom);
        render_circle_outline(ren, c0, r * view_zoom, cr, cg, cb, 255);
        double dh = 10.0 * view_zoom;
        render_number(ren, (Vec2){ c0.x + 5.0, c0.y - r * view_zoom - dh - 4.0 },
                      0.0, dh, r, cr, cg, cb, 255);
    }

    /* A rack's pitch line, which belongs to the mesh rather than to a body. */
    for (int gi = 0; gi < m->gear_count; gi++) {
        const Gear *gr = &m->gears[gi];
        if (!gr->alive || gr->kind != GEAR_RACK) continue;
        const Link *rack = &m->links[gr->driven_link_id];
        if (!rack->alive || rack->connector_count < 2) continue;
        Vec2 a = m->connectors[rack->connector_ids[0]].pos;
        Vec2 b = m->connectors[rack->connector_ids[1]].pos;
        render_dashed_line(ren, world_to_screen(a, view_pan, view_zoom),
                           world_to_screen(b, view_pan, view_zoom), 5.0, 165, 185, 130, 255);
    }

    for (int vi = 0; vi < m->geneva_count; vi++) {
        const Geneva *gv = &m->genevas[vi];
        if (!gv->alive) continue;
        if (!m->connectors[gv->wheel_center_id].alive) continue;
        double wheel_r = mechanism_geneva_wheel_radius(gv);
        double angle = 0.0;
        const Link *wheel = &m->links[gv->wheel_link_id];
        if (wheel->alive && wheel->connector_count >= 2) {
            /* Read the wheel's current rotation off one of its own points. */
            int ref = -1;
            for (int i = 0; i < wheel->connector_count; i++) {
                if (wheel->connector_ids[i] != gv->wheel_center_id) { ref = wheel->connector_ids[i]; break; }
            }
            if (ref >= 0) {
                Vec2 d = vec2_sub(m->connectors[ref].pos, m->connectors[gv->wheel_center_id].pos);
                if (vec2_len(d) > 1e-9) angle = atan2(d.y, d.x);
            }
        }
        Uint8 cr, cg, cb;
        if (gv->selected) { cr = 255; cg = 225; cb = 70; }
        else if (gv->engaged) { cr = 120; cg = 215; cb = 140; }
        else { cr = 175; cg = 150; cb = 200; }
        render_geneva_wheel(ren, world_to_screen(m->connectors[gv->wheel_center_id].pos, view_pan, view_zoom),
                            wheel_r * view_zoom, gv->slot_count, angle, cr, cg, cb, 255);
        /* The driver's locking disc, which holds the wheel between indexes. */
        if (m->connectors[gv->driver_center_id].alive) {
            render_circle_outline(ren, world_to_screen(m->connectors[gv->driver_center_id].pos, view_pan, view_zoom),
                                  (gv->center_distance - wheel_r) * view_zoom, 150, 140, 180, 255);
        }
        /* The slot count, which is the one number that decides how it indexes. */
        Vec2 wc = world_to_screen(m->connectors[gv->wheel_center_id].pos, view_pan, view_zoom);
        double dh = 10.0 * view_zoom;
        render_number(ren, (Vec2){ wc.x + 5.0, wc.y - wheel_r * view_zoom - dh - 4.0 },
                      0.0, dh, (double)gv->slot_count, cr, cg, cb, 255);
    }

    for (int li = 0; li < m->link_count; li++) {
        const Link *l = &m->links[li];
        if (!l->alive) continue;
        /* A gear wheel is a disc, not a bar. Its pitch circle is already drawn
         * above and the mark on its rim shows which way it is pointing, so
         * drawing a rod out to that mark would only be a spoke nobody asked
         * for -- and a length label on it means nothing. */
        if (mechanism_is_gear_body(m, li)) continue;
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
                /* Only label a link the number actually fits alongside. A
                 * drawing machine has dozens of short arms, and labelling
                 * every one buries the mechanism in numerals. */
                if (dir_len > 1e-6 && dir_len > render_number_width(10.0 * view_zoom, length) + 8.0) {
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
    printf("    N: TEMPLATE gallery -- pick a famous mechanism and drop it in\n");
    printf("    J: place a connector at the centre of the view\n");
    printf("    L: link selected connectors     A: toggle anchor on selected connectors\n");
    printf("    M: toggle motor on selected link (needs exactly one anchor)   +/-: motor speed\n");
    printf("    S/O/W: SLIDER, GEAR, GENEVA. With nothing selected each one draws a\n");
    printf("       whole working assembly -- motor and all -- into the middle of the\n");
    printf("       view; drag any part of it to change its proportions. With the right\n");
    printf("       parts already selected it joins those instead:\n");
    printf("         SLIDER: 3 connectors -- the two farthest apart are the rail, the\n");
    printf("                 third is the pin that runs along it\n");
    printf("         GEAR:   2 grounded centres for a gear pair, or a grounded centre\n");
    printf("                 plus a point on a free bar for a rack and pinion\n");
    printf("         GENEVA: the motor's centre and the wheel's centre\n");
    printf("       GEAR works differently from the other two, because gears are\n");
    printf("       built rather than dropped in whole:\n");
    printf("         press it with nothing selected: ONE wheel, on its own, clear of\n");
    printf("           whatever is already there and meshed with nothing\n");
    printf("         select two or more wheels and press it: they mesh, in order\n");
    printf("       A wheel is a disc, not a bar -- a hub and a mark on its RIM, and\n");
    printf("       that mark is the point TRACE plots. Click anywhere on a wheel to\n");
    printf("       select it, and SHIFT-CLICK to add a second -- that is how two wheels\n");
    printf("       get chosen for meshing. +/- resize a wheel alone, and M makes it the one\n");
    printf("       that drives (its whole train turns round to suit). Meshed wheels\n");
    printf("       slide along until they touch, so dragging one swings it round its\n");
    printf("       partner rather than pulling the teeth apart.\n");
    printf("       Click a Geneva wheel or a slider rail to select that joint, and\n");
    printf("       +/- changes a Geneva's slot count.\n");
    printf("    K: cam tool -- then DRAG on the canvas to draw the cam's outline (or\n");
    printf("       click once for a default cam). The centre, motor and roller follower\n");
    printf("       are all created for you. Click a cam's outline to select it:\n");
    printf("       +/- resize its lift, [ and ] shift its timing.\n");
    printf("    Two path tools: draw a curve and get a machine that traces it.\n");
    printf("      P: LINKAGE -- fits a four-bar. Five parts and one motor, but only\n");
    printf("         the curves four-bars can trace (beans, ellipses, figure-eights).\n");
    printf("      B: ARMS -- a chain of rotating arms. Traces ANY curve exactly,\n");
    printf("         at the cost of dozens of parts. Use it for stars, hearts, etc.\n");
    printf("       Close the loop for a repeating cycle; leave it open and the pen\n");
    printf("       sweeps out along it and back.\n");
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
    app.path_tool = PATH_TOOL_NONE;
    app.gallery_open = false;
    app.path_stroke = NULL;
    app.path_stroke_count = 0;
    app.path_stroke_capacity = 0;
    app.path_ghost_count = 0;
    app.path_ghost_closed = false;
    ui_init(&app.toolbar);

    Uint32 hover_since = 0;   /* when the cursor settled on the hovered button */
    int mouse_x = 0, mouse_y = 0;
    Uint32 last_ticks = SDL_GetTicks();
    bool running = true;

    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) {
                running = false;
            } else if (ev.type == SDL_MOUSEMOTION) {
                mouse_x = ev.motion.x;
                mouse_y = ev.motion.y;
                {
                    /* A tooltip waits for the cursor to settle, so sweeping
                     * down the strip doesn't flash a panel per button. */
                    int was = app.toolbar.hover;
                    app.toolbar.hover = ui_hit_test(&app.toolbar, ev.motion.x, ev.motion.y);
                    if (app.toolbar.hover != was) hover_since = SDL_GetTicks();
                }
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
                        stroke_push(&app.cam_stroke, &app.cam_stroke_count, &app.cam_stroke_capacity, p);
                    } else if (app.drag_mode == DRAG_DRAW_PATH) {
                        stroke_push(&app.path_stroke, &app.path_stroke_count, &app.path_stroke_capacity, p);
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
                app.path_ghost_count = 0;
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
                } else if (k == SDLK_n) {
                    app_toggle_gallery(&app);
                } else if (k == SDLK_j) {
                    app_add_joint(&app);
                } else if (k == SDLK_l) {
                    app_link_selected(&app);
                } else if (k == SDLK_a) {
                    app_toggle_anchor(&app);
                } else if (k == SDLK_m) {
                    app_toggle_motor(&app);
                } else if (k == SDLK_s) {
                    app_add_slider(&app);
                } else if (k == SDLK_o) {
                    app_add_gear(&app);
                } else if (k == SDLK_w) {
                    app_add_geneva(&app);
                } else if (k == SDLK_k) {
                    app_toggle_cam_draw(&app);
                } else if (k == SDLK_p) {
                    app_arm_path_tool(&app, PATH_TOOL_LINKAGE);
                } else if (k == SDLK_b) {
                    app_arm_path_tool(&app, PATH_TOOL_ARMS);
                } else if (k == SDLK_LEFTBRACKET) {
                    app_adjust_cam_timing(&app, -CAM_TIMING_STEP);
                } else if (k == SDLK_RIGHTBRACKET) {
                    app_adjust_cam_timing(&app, CAM_TIMING_STEP);
                } else if (k == SDLK_v) {
                    app_toggle_vary(&app);
                } else if (k == SDLK_EQUALS || k == SDLK_KP_PLUS) {
                    /* +/- means "more of whatever is selected". */
                    if (find_single_selected_cam(&app.mech) >= 0) app_adjust_cam_lift(&app, CAM_LIFT_SCALE);
                    else if (gather_selected_wheels(&app.mech, NULL, 0) == 1) app_adjust_wheel_radius(&app, WHEEL_RESIZE_STEP);
                    else if (find_single_selected_geneva(&app.mech) >= 0) app_adjust_geneva_slots(&app, 1);
                    else app_adjust_motor_speed(&app, MOTOR_SPEED_STEP_DEG_S);
                } else if (k == SDLK_MINUS || k == SDLK_KP_MINUS) {
                    if (find_single_selected_cam(&app.mech) >= 0) app_adjust_cam_lift(&app, 1.0 / CAM_LIFT_SCALE);
                    else if (gather_selected_wheels(&app.mech, NULL, 0) == 1) app_adjust_wheel_radius(&app, 1.0 / WHEEL_RESIZE_STEP);
                    else if (find_single_selected_geneva(&app.mech) >= 0) app_adjust_geneva_slots(&app, -1);
                    else app_adjust_motor_speed(&app, -MOTOR_SPEED_STEP_DEG_S);
                } else if (k == SDLK_t) {
                    app_toggle_trace(&app);
                } else if (k == SDLK_e) {
                    app_export(&app);
                } else if (k == SDLK_DELETE || k == SDLK_BACKSPACE) {
                    app_delete_selection(&app);
                } else if (k == SDLK_ESCAPE) {
                    if (app.gallery_open) {
                        app.gallery_open = false;
                    } else if (app.cam_draw_armed) {
                        app.cam_draw_armed = false;
                        app.cam_stroke_count = 0;
                        printf("Cam tool cancelled.\n");
                    } else if (app.path_tool != PATH_TOOL_NONE) {
                        app.path_tool = PATH_TOOL_NONE;
                        app.path_stroke_count = 0;
                        printf("Path tool cancelled.\n");
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
                        app.gallery_open) {
                int tile = gallery_hit(ev.button.x, ev.button.y);
                if (tile >= 0) app_insert_template(&app, tile);
                else app.gallery_open = false;   /* a click outside dismisses it */
            } else if (app.state == APP_EDIT && ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_LEFT &&
                        app.path_tool != PATH_TOOL_NONE) {
                Vec2 p = screen_to_world((Vec2){ (double)ev.button.x, (double)ev.button.y }, app.view_pan, app.view_zoom);
                app.drag_mode = DRAG_DRAW_PATH;
                app.drag_start = p;
                app.path_stroke_count = 0;
                stroke_push(&app.path_stroke, &app.path_stroke_count, &app.path_stroke_capacity, p);
            } else if (app.state == APP_EDIT && ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_LEFT &&
                        app.cam_draw_armed) {
                Vec2 p = screen_to_world((Vec2){ (double)ev.button.x, (double)ev.button.y }, app.view_pan, app.view_zoom);
                app.drag_mode = DRAG_DRAW_CAM;
                app.drag_start = p;
                app.cam_stroke_count = 0;
                stroke_push(&app.cam_stroke, &app.cam_stroke_count, &app.cam_stroke_capacity, p);
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
                    /* Bars first, then the joints drawn behind them: a cam
                     * outline, a pitch circle, a Geneva wheel, a slider rail.
                     * Picking one is how its proportions get edited. */
                    double jt = JOINT_HIT_DIST / app.view_zoom;
                    int hit_link = mechanism_pick_link_edge(&app.mech, p, LINK_EDGE_HIT_DIST / app.view_zoom);
                    int hit_cam = (hit_link >= 0) ? -1
                                    : mechanism_pick_cam(&app.mech, p, CAM_HIT_DIST / app.view_zoom);
                    /* A wheel is picked before a bar, since a bar crossing a
                     * wheel's face is the rarer thing to want. */
                    int hit_wheel = (hit_cam >= 0) ? -1 : mechanism_pick_wheel(&app.mech, p, jt);
                    if (hit_wheel >= 0) hit_link = -1;
                    int hit_gear = (hit_link >= 0 || hit_cam >= 0 || hit_wheel >= 0)
                                    ? -1 : mechanism_pick_gear(&app.mech, p, jt);
                    int hit_gen = (hit_link >= 0 || hit_cam >= 0 || hit_gear >= 0)
                                    ? -1 : mechanism_pick_geneva(&app.mech, p, jt);
                    int hit_slid = (hit_link >= 0 || hit_cam >= 0 || hit_gear >= 0 || hit_gen >= 0)
                                    ? -1 : mechanism_pick_slider(&app.mech, p, jt);
                    if (hit_link >= 0) {
                        clear_selection(&app.mech);
                        app.mech.links[hit_link].selected = true;
                        app.drag_mode = DRAG_NONE;
                    } else if (hit_cam >= 0) {
                        clear_selection(&app.mech);
                        app.mech.cams[hit_cam].selected = true;
                        app.drag_mode = DRAG_NONE;
                    } else if (hit_wheel >= 0) {
                        /* Shift adds to the selection, which is how two or
                         * more wheels get chosen for meshing. */
                        if (shift) {
                            app.mech.links[hit_wheel].selected = !app.mech.links[hit_wheel].selected;
                        } else {
                            clear_selection(&app.mech);
                            app.mech.links[hit_wheel].selected = true;
                        }
                        app.drag_mode = DRAG_NONE;
                    } else if (hit_gear >= 0) {
                        clear_selection(&app.mech);
                        app.mech.gears[hit_gear].selected = true;
                        app.drag_mode = DRAG_NONE;
                    } else if (hit_gen >= 0) {
                        clear_selection(&app.mech);
                        app.mech.genevas[hit_gen].selected = true;
                        app.drag_mode = DRAG_NONE;
                    } else if (hit_slid >= 0) {
                        clear_selection(&app.mech);
                        app.mech.sliders[hit_slid].selected = true;
                        app.drag_mode = DRAG_NONE;
                    } else {
                        app.drag_mode = DRAG_PENDING_EMPTY;
                        app.drag_start = p;
                        app.drag_current = p;
                    }
                }
            } else if (app.state == APP_EDIT && ev.type == SDL_MOUSEBUTTONUP && ev.button.button == SDL_BUTTON_LEFT &&
                        app.drag_mode == DRAG_DRAW_PATH) {
                if (app.path_tool == PATH_TOOL_LINKAGE) app_synthesize_linkage(&app);
                else app_synthesize_arms(&app);
                app.path_stroke_count = 0;
                app.path_tool = PATH_TOOL_NONE;
                app.drag_mode = DRAG_NONE;
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

        if (app.state == APP_EDIT) {
            /* A joint's proportions are read off the geometry, so dragging a
             * part while editing resizes it live -- the pitch circles you see
             * are always the ones the solver will use. */
            mechanism_refresh_joint_sizes(&app.mech);
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
        /* The path that was asked for, kept behind the result so the fit can
         * be judged by eye. */
        for (int i = 1; i < app.path_ghost_count; i++) {
            render_line(ren, world_to_screen(app.path_ghost[i - 1], app.view_pan, app.view_zoom),
                        world_to_screen(app.path_ghost[i], app.view_pan, app.view_zoom),
                        95, 90, 120, 255);
        }
        if (app.path_ghost_closed && app.path_ghost_count > 2) {
            render_line(ren, world_to_screen(app.path_ghost[app.path_ghost_count - 1], app.view_pan, app.view_zoom),
                        world_to_screen(app.path_ghost[0], app.view_pan, app.view_zoom), 95, 90, 120, 255);
        }

        for (int i = 1; i < app.path_stroke_count; i++) {
            render_line(ren, world_to_screen(app.path_stroke[i - 1], app.view_pan, app.view_zoom),
                        world_to_screen(app.path_stroke[i], app.view_pan, app.view_zoom),
                        150, 200, 255, 255);
        }

        if (app.path_tool != PATH_TOOL_NONE) {
            const char *hint = (app.path_tool == PATH_TOOL_LINKAGE)
                ? "DRAW A PATH FOR A FOUR-BAR LINKAGE TO TRACE   ESC TO CANCEL"
                : "DRAW ANY PATH FOR A CHAIN OF ARMS TO REDRAW   ESC TO CANCEL";
            double w = render_text_width(9.0, hint);
            render_text(ren, (Vec2){ UI_TOOLBAR_W + (CANVAS_W - w) / 2.0, 16.0 }, 9.0, hint,
                        150, 200, 255, 255);
        }
        if (app.cam_draw_armed) {
            const char *hint = "DRAG TO DRAW THE CAM OUTLINE   CLICK FOR A DEFAULT CAM   ESC TO CANCEL";
            double w = render_text_width(9.0, hint);
            render_text(ren, (Vec2){ UI_TOOLBAR_W + (CANVAS_W - w) / 2.0, 16.0 }, 9.0, hint,
                        200, 175, 235, 255);
        }
        if (app.gallery_open) {
            UiRect panel = gallery_panel();
            render_rect_filled(ren, panel, 24, 25, 30, 255);
            render_rect_outline(ren, panel, 90, 95, 110, 255);
            const char *title = "PICK A MECHANISM";
            double tw = render_text_width(10.0, title);
            render_text(ren, (Vec2){ panel.x + (panel.w - tw) / 2.0, panel.y + 8.0 }, 10.0, title,
                        205, 210, 220, 255);
            for (int i = 0; i < templates_count(); i++) {
                UiRect tile = gallery_tile(i);
                bool hot = (gallery_hit(mouse_x, mouse_y) == i);
                render_rect_filled(ren, tile, hot ? 44 : 34, hot ? 46 : 36, hot ? 54 : 42, 255);
                render_rect_outline(ren, tile, hot ? 255 : 66, hot ? 225 : 70, hot ? 70 : 80, 255);
                draw_template_preview(ren, i, tile, hot);
            }
        }

        SDL_RenderSetClipRect(ren, NULL);

        render_plot(ren, &app.mech, (UiRect){ UI_TOOLBAR_W, CANVAS_H, CANVAS_W, PLOT_H });
        render_toolbar(ren, &app.toolbar, WIN_H);
        /* Last of all, so it sits over the canvas and the panels. */
        if (app.toolbar.hover >= 0 && app.toolbar.hover < app.toolbar.count &&
            SDL_GetTicks() - hover_since > TOOLTIP_DELAY_MS) {
            const UiButton *hb = &app.toolbar.buttons[app.toolbar.hover];
            render_tooltip(ren, hb->rect, hb->tip, WIN_W, WIN_H);
        }

        SDL_RenderPresent(ren);

        SDL_Delay(16);
    }

    free(app.cam_stroke);
    free(app.path_stroke);
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
