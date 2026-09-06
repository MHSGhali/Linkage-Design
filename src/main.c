#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#include "mechanism.h"
#include "solver.h"
#include "render.h"
#include "export.h"

#define WIN_W 900
#define WIN_H 700

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
 * coordinates cross this boundary. */
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

static void delete_selected(Mechanism *m) {
    for (int i = 0; i < m->link_count; i++) {
        if (m->links[i].alive && m->links[i].selected) mechanism_delete_link(m, i);
    }
    for (int i = 0; i < m->connector_count; i++) {
        if (m->connectors[i].alive && m->connectors[i].selected) mechanism_delete_connector(m, i);
    }
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

/* Bounded undo stack of full mechanism snapshots (mechanism_clone), pushed
 * right before an edit that is about to happen. Simple and correct for the
 * small mechanisms this tool targets; no redo (not asked for). */
#define UNDO_MAX 50

static void push_undo(Mechanism *stack, int *count, const Mechanism *mech) {
    if (*count >= UNDO_MAX) {
        mechanism_free(&stack[0]);
        for (int i = 1; i < UNDO_MAX; i++) stack[i - 1] = stack[i];
        *count = UNDO_MAX - 1;
    }
    mechanism_clone(mech, &stack[*count]);
    (*count)++;
}

/* Removes the most recently pushed snapshot without restoring it -- call
 * when a push_undo() turned out to precede a no-op (e.g. a toggle that
 * failed its precondition), so undo history doesn't fill with duplicates. */
static void discard_last_undo(Mechanism *stack, int *count) {
    if (*count <= 0) return;
    (*count)--;
    mechanism_free(&stack[*count]);
}

static bool pop_undo(Mechanism *stack, int *count, Mechanism *out) {
    if (*count <= 0) return false;
    (*count)--;
    *out = stack[*count];
    return true;
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
    printf("  Edit mode:\n");
    printf("    Click empty space: place a connector      Drag empty space: box-select\n");
    printf("    Click a connector: select it (Shift: add/remove)   Drag a selected connector: move selection\n");
    printf("    Click a link's edge: select that link\n");
    printf("    L: link selected connectors     A: toggle anchor on selected connectors\n");
    printf("    M: toggle motor on selected link (needs exactly one anchor)   +/-: motor speed\n");
    printf("    V: toggle selected link's length between fixed and variable (green = variable)\n");
    printf("    T: toggle path tracing on selected connectors\n");
    printf("    E: export mechanism to a Blender Python script\n");
    printf("    Delete/Backspace: delete selection     Escape: clear selection\n");
    printf("    Cmd/Ctrl+Z: undo\n");
    printf("  Scroll wheel: zoom in/out (centered on cursor)\n");
    printf("  C: clear all traces (works while running too)\n");
    printf("  G: toggle gravity (on automatically when the mechanism has no motor)\n");
    printf("  R: run/stop simulation\n");

    Mechanism mech;
    mechanism_init(&mech);
    SolverParams params = solver_default_params();
    /* Gravity is applied automatically to a mechanism with no motor -- there
     * would otherwise be nothing to make it move at all. Once the user
     * presses G, their choice wins from then on. */
    bool gravity_set_by_user = false;

    Mechanism undo_stack[UNDO_MAX];
    int undo_count = 0;

    AppState state = APP_EDIT;
    DragMode drag_mode = DRAG_NONE;
    Vec2 drag_start = { 0, 0 };
    Vec2 drag_last = { 0, 0 };
    Vec2 drag_current = { 0, 0 };

    Vec2 *pre_run_positions = NULL;
    int pre_run_count = 0;

    /* Start-of-frame snapshot, so a step that binds up can be rolled back
     * rather than leaving fixed-length links visibly stretched. */
    Vec2 *frame_positions = NULL;
    double *frame_angles = NULL;
    int frame_link_count = 0;
    bool jammed = false;

    Vec2 view_pan = { 0, 0 };
    double view_zoom = 1.0;

    Uint32 last_ticks = SDL_GetTicks();
    bool running = true;

    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) {
                running = false;
            } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_c) {
                mechanism_clear_traces(&mech); /* works in both edit and running mode */
            } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_g) {
                gravity_set_by_user = true;
                if (params.gravity.x == 0.0 && params.gravity.y == 0.0) {
                    params.gravity = (Vec2){ 0.0, DEFAULT_GRAVITY_MAGNITUDE };
                    printf("Gravity ON.\n");
                } else {
                    params.gravity = (Vec2){ 0.0, 0.0 };
                    printf("Gravity OFF.\n");
                }
            } else if (ev.type == SDL_KEYDOWN && state == APP_EDIT) {
                SDL_Keycode k = ev.key.keysym.sym;
                if (k == SDLK_z && (SDL_GetModState() & (KMOD_CTRL | KMOD_GUI))) {
                    Mechanism restored;
                    if (pop_undo(undo_stack, &undo_count, &restored)) {
                        mechanism_free(&mech);
                        mech = restored;
                        printf("Undo.\n");
                    } else {
                        printf("Nothing to undo.\n");
                    }
                } else if (k == SDLK_l) {
                    int ids[256];
                    int n = gather_selected_connectors(&mech, ids, 256);
                    if (n >= 2) {
                        push_undo(undo_stack, &undo_count, &mech);
                        int lid = mechanism_add_link(&mech, ids, n);
                        if (lid >= 0) {
                            clear_selection(&mech);
                            mech.links[lid].selected = true;
                        } else {
                            discard_last_undo(undo_stack, &undo_count);
                        }
                    } else {
                        printf("Select at least 2 connectors before pressing L.\n");
                    }
                } else if (k == SDLK_a) {
                    if (has_selection(&mech)) {
                        push_undo(undo_stack, &undo_count, &mech);
                        for (int i = 0; i < mech.connector_count; i++) {
                            if (mech.connectors[i].alive && mech.connectors[i].selected) {
                                mechanism_set_anchor(&mech, i, !mech.connectors[i].is_anchor);
                            }
                        }
                    }
                } else if (k == SDLK_m) {
                    int lid = find_single_selected_link(&mech);
                    if (lid < 0) {
                        printf("Select exactly one link before pressing M.\n");
                    } else {
                        push_undo(undo_stack, &undo_count, &mech);
                        if (!mechanism_toggle_driven(&mech, lid, DEFAULT_MOTOR_SPEED_DEG_S)) {
                            discard_last_undo(undo_stack, &undo_count);
                            printf("A driven link needs exactly one anchor connector.\n");
                        }
                    }
                } else if (k == SDLK_v) {
                    int lid = find_single_selected_link(&mech);
                    if (lid < 0) {
                        printf("Select exactly one link before pressing V.\n");
                    } else if (mech.links[lid].is_driven) {
                        printf("A driven link's shape is always rigid; toggle M off first.\n");
                    } else {
                        push_undo(undo_stack, &undo_count, &mech);
                        bool now_rigid = !mech.links[lid].rigid;
                        mechanism_set_rigid(&mech, lid, now_rigid);
                        printf("Link length is now %s.\n", now_rigid ? "fixed" : "variable");
                    }
                } else if (k == SDLK_EQUALS || k == SDLK_KP_PLUS || k == SDLK_MINUS || k == SDLK_KP_MINUS) {
                    int lid = find_single_selected_link(&mech);
                    if (lid >= 0 && mech.links[lid].is_driven) {
                        push_undo(undo_stack, &undo_count, &mech);
                        double step = (k == SDLK_MINUS || k == SDLK_KP_MINUS) ? -MOTOR_SPEED_STEP_DEG_S : MOTOR_SPEED_STEP_DEG_S;
                        mech.links[lid].motor_speed_deg_s += step;
                        printf("Motor speed: %.1f deg/s\n", mech.links[lid].motor_speed_deg_s);
                    }
                } else if (k == SDLK_t) {
                    if (has_selection(&mech)) {
                        push_undo(undo_stack, &undo_count, &mech);
                        for (int i = 0; i < mech.connector_count; i++) {
                            if (mech.connectors[i].alive && mech.connectors[i].selected) {
                                mechanism_set_traced(&mech, i, !mech.connectors[i].traced);
                            }
                        }
                    }
                } else if (k == SDLK_e) {
                    const char *path = "linkage_export.py";
                    if (export_blender_script(&mech, path)) {
                        printf("Exported to %s -- run it inside Blender's Scripting tab, or `blender --python %s`.\n", path, path);
                    } else {
                        printf("Failed to write %s\n", path);
                    }
                } else if (k == SDLK_DELETE || k == SDLK_BACKSPACE) {
                    if (has_selection(&mech)) {
                        push_undo(undo_stack, &undo_count, &mech);
                        delete_selected(&mech);
                    }
                } else if (k == SDLK_ESCAPE) {
                    clear_selection(&mech);
                } else if (k == SDLK_r) {
                    free(pre_run_positions);
                    pre_run_count = mech.connector_count;
                    pre_run_positions = malloc((size_t)pre_run_count * sizeof(Vec2));
                    for (int i = 0; i < pre_run_count; i++) pre_run_positions[i] = mech.connectors[i].pos;

                    free(frame_positions);
                    frame_positions = malloc((size_t)pre_run_count * sizeof(Vec2));
                    free(frame_angles);
                    frame_link_count = mech.link_count;
                    frame_angles = (frame_link_count > 0) ? malloc((size_t)frame_link_count * sizeof(double)) : NULL;
                    jammed = false;

                    if (!gravity_set_by_user && !mechanism_has_driven_link(&mech)) {
                        printf("No motor in this mechanism -- running it under gravity. "
                                "Press G to control gravity yourself.\n");
                    }

                    solver_freeze(&mech);
                    mechanism_clear_traces(&mech);
                    drag_mode = DRAG_NONE;
                    state = APP_RUNNING;
                }
            } else if (ev.type == SDL_KEYDOWN && state == APP_RUNNING) {
                if (ev.key.keysym.sym == SDLK_r) {
                    for (int i = 0; i < pre_run_count && i < mech.connector_count; i++) {
                        mech.connectors[i].pos = pre_run_positions[i];
                    }
                    free(pre_run_positions);
                    pre_run_positions = NULL;
                    pre_run_count = 0;
                    free(frame_positions);
                    frame_positions = NULL;
                    free(frame_angles);
                    frame_angles = NULL;
                    frame_link_count = 0;
                    jammed = false;
                    state = APP_EDIT;
                }
            } else if (ev.type == SDL_MOUSEWHEEL) {
                int mx, my;
                SDL_GetMouseState(&mx, &my);
                Vec2 screen_mouse = { (double)mx, (double)my };
                Vec2 world_before = screen_to_world(screen_mouse, view_pan, view_zoom);

                double factor = (ev.wheel.y > 0) ? ZOOM_STEP : 1.0 / ZOOM_STEP;
                double new_zoom = view_zoom * factor;
                if (new_zoom < ZOOM_MIN) new_zoom = ZOOM_MIN;
                if (new_zoom > ZOOM_MAX) new_zoom = ZOOM_MAX;

                view_pan.x = screen_mouse.x - world_before.x * new_zoom;
                view_pan.y = screen_mouse.y - world_before.y * new_zoom;
                view_zoom = new_zoom;
            } else if (state == APP_EDIT && ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_LEFT) {
                Vec2 p = screen_to_world((Vec2){ (double)ev.button.x, (double)ev.button.y }, view_pan, view_zoom);
                bool shift = (SDL_GetModState() & KMOD_SHIFT) != 0;
                int hit_conn = mechanism_pick_connector(&mech, p, CONNECTOR_HIT_RADIUS / view_zoom);
                if (hit_conn >= 0) {
                    if (shift) {
                        mech.connectors[hit_conn].selected = !mech.connectors[hit_conn].selected;
                        drag_mode = DRAG_NONE;
                    } else {
                        if (!mech.connectors[hit_conn].selected) {
                            clear_selection(&mech);
                            mech.connectors[hit_conn].selected = true;
                        }
                        push_undo(undo_stack, &undo_count, &mech);
                        drag_mode = DRAG_MOVE_CONNECTORS;
                        drag_last = p;
                    }
                } else {
                    int hit_link = mechanism_pick_link_edge(&mech, p, LINK_EDGE_HIT_DIST / view_zoom);
                    if (hit_link >= 0) {
                        clear_selection(&mech);
                        mech.links[hit_link].selected = true;
                        drag_mode = DRAG_NONE;
                    } else {
                        drag_mode = DRAG_PENDING_EMPTY;
                        drag_start = p;
                        drag_current = p;
                    }
                }
            } else if (state == APP_EDIT && ev.type == SDL_MOUSEMOTION) {
                Vec2 p = screen_to_world((Vec2){ (double)ev.motion.x, (double)ev.motion.y }, view_pan, view_zoom);
                if (drag_mode == DRAG_MOVE_CONNECTORS) {
                    Vec2 delta = vec2_sub(p, drag_last);
                    for (int i = 0; i < mech.connector_count; i++) {
                        if (mech.connectors[i].alive && mech.connectors[i].selected) {
                            mech.connectors[i].pos = vec2_add(mech.connectors[i].pos, delta);
                        }
                    }
                    drag_last = p;
                } else if (drag_mode == DRAG_PENDING_EMPTY) {
                    if (vec2_dist(p, drag_start) > DRAG_THRESHOLD / view_zoom) drag_mode = DRAG_BOX_SELECT;
                    drag_current = p;
                } else if (drag_mode == DRAG_BOX_SELECT) {
                    drag_current = p;
                }
            } else if (state == APP_EDIT && ev.type == SDL_MOUSEBUTTONUP && ev.button.button == SDL_BUTTON_LEFT) {
                if (drag_mode == DRAG_PENDING_EMPTY) {
                    push_undo(undo_stack, &undo_count, &mech);
                    clear_selection(&mech);
                    int id = mechanism_add_connector(&mech, drag_start, false);
                    mech.connectors[id].selected = true;
                } else if (drag_mode == DRAG_BOX_SELECT) {
                    double x0 = fmin(drag_start.x, drag_current.x), x1 = fmax(drag_start.x, drag_current.x);
                    double y0 = fmin(drag_start.y, drag_current.y), y1 = fmax(drag_start.y, drag_current.y);
                    clear_selection(&mech);
                    for (int i = 0; i < mech.connector_count; i++) {
                        Connector *c = &mech.connectors[i];
                        if (c->alive && c->pos.x >= x0 && c->pos.x <= x1 && c->pos.y >= y0 && c->pos.y <= y1) {
                            c->selected = true;
                        }
                    }
                }
                drag_mode = DRAG_NONE;
            }
        }

        Uint32 now = SDL_GetTicks();
        double dt = (double)(now - last_ticks) / 1000.0;
        last_ticks = now;

        if (state == APP_RUNNING && !jammed) {
            if (dt > 0.05) dt = 0.05; /* clamp huge stalls (e.g. window drag) */

            for (int i = 0; i < pre_run_count; i++) frame_positions[i] = mech.connectors[i].pos;
            for (int i = 0; i < frame_link_count; i++) frame_angles[i] = mech.links[i].accumulated_angle_rad;

            SolverParams frame_params = params;
            if (!gravity_set_by_user && !mechanism_has_driven_link(&mech)) {
                frame_params.gravity = (Vec2){ 0.0, DEFAULT_GRAVITY_MAGNITUDE };
            }
            solver_advance(&mech, dt, frame_params);

            if (solver_has_length_violation(&mech, params.length_tol_abs, params.length_tol_rel)) {
                /* This step would only be reachable by stretching a
                 * fixed-length link, so roll it back entirely and stop:
                 * a rigid link is rigid, so the mechanism binds instead. */
                for (int i = 0; i < pre_run_count; i++) mech.connectors[i].pos = frame_positions[i];
                for (int i = 0; i < frame_link_count; i++) mech.links[i].accumulated_angle_rad = frame_angles[i];
                jammed = true;
                printf("Mechanism jammed: a fixed-length link would have to change length here. "
                        "Press R to stop, then adjust the geometry (or press V to let a link's length vary).\n");
            } else {
                mechanism_trace_step(&mech);
            }
        }

        SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
        SDL_RenderClear(ren);
        draw_mechanism(ren, &mech, drag_mode, drag_start, drag_current, view_pan, view_zoom);
        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }

    free(pre_run_positions);
    free(frame_positions);
    free(frame_angles);
    mechanism_free(&mech);
    for (int i = 0; i < undo_count; i++) mechanism_free(&undo_stack[i]);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
