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
#define WIN_H 700
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
    DRAG_BOX_SELECT
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

    Toolbar toolbar;
} App;

static void clear_selection(Mechanism *m) {
    for (int i = 0; i < m->connector_count; i++) m->connectors[i].selected = false;
    for (int i = 0; i < m->link_count; i++) m->links[i].selected = false;
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
        for (int k = 1; k < c->path_count; k++) {
            render_line(ren, world_to_screen(c->path[k - 1], view_pan, view_zoom),
                        world_to_screen(c->path[k], view_pan, view_zoom), 80, 200, 220, 255);
        }
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
        else if (c->traced) { r = 80; g = 200; b = 220; }
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
                } else if (k == SDLK_v) {
                    app_toggle_vary(&app);
                } else if (k == SDLK_EQUALS || k == SDLK_KP_PLUS) {
                    app_adjust_motor_speed(&app, MOTOR_SPEED_STEP_DEG_S);
                } else if (k == SDLK_MINUS || k == SDLK_KP_MINUS) {
                    app_adjust_motor_speed(&app, -MOTOR_SPEED_STEP_DEG_S);
                } else if (k == SDLK_t) {
                    app_toggle_trace(&app);
                } else if (k == SDLK_e) {
                    app_export(&app);
                } else if (k == SDLK_DELETE || k == SDLK_BACKSPACE) {
                    app_delete_selection(&app);
                } else if (k == SDLK_ESCAPE) {
                    clear_selection(&app.mech);
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
                    if (hit_link >= 0) {
                        clear_selection(&app.mech);
                        app.mech.links[hit_link].selected = true;
                        app.drag_mode = DRAG_NONE;
                    } else {
                        app.drag_mode = DRAG_PENDING_EMPTY;
                        app.drag_start = p;
                        app.drag_current = p;
                    }
                }
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
                mechanism_trace_step(&app.mech);
            }
        }

        ui_apply_state(&app.toolbar, app_ui_state(&app));

        SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
        SDL_RenderClear(ren);

        /* Keep the mechanism inside the canvas so long links can't draw over
         * the toolbar. */
        SDL_Rect canvas_clip = { UI_TOOLBAR_W, 0, CANVAS_W, WIN_H };
        SDL_RenderSetClipRect(ren, &canvas_clip);
        draw_mechanism(ren, &app.mech, app.drag_mode, app.drag_start, app.drag_current, app.view_pan, app.view_zoom);
        SDL_RenderSetClipRect(ren, NULL);

        render_toolbar(ren, &app.toolbar, WIN_H);
        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }

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
