#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>

#include "mechanism.h"
#include "solver.h"
#include "render.h"
#include "export.h"
#include "print3d.h"
#include "gearing.h"
#include "ui.h"
#include "synth.h"
#include "templates.h"
#include "status.h"
#include "scene.h"

/* The window opens at this size and can be resized freely from there. It used
 * to be fixed, which meant a laptop screen shorter than WIN_H simply lost the
 * plot panel off the bottom with no way to get it back. */
#define WIN_DEFAULT_W (UI_TOOLBAR_W + 900)
#define WIN_DEFAULT_H 900
#define WIN_MIN_W 700
#define WIN_MIN_H 520
#define PLOT_FRACTION 0.22       /* of the window height ... */
#define PLOT_MIN_H 120           /* ... between these */
#define PLOT_MAX_H 240

#define CONNECTOR_HIT_RADIUS 10.0     /* screen pixels; never smaller than the dot drawn */
#define LINK_EDGE_HIT_DIST 6.0
#define DRAG_THRESHOLD 4.0
#define DEFAULT_MOTOR_SPEED_DEG_S 90.0
#define MOTOR_SPEED_STEP_DEG_S 10.0
#define ZOOM_MIN 0.1
#define ZOOM_MAX 10.0
#define ZOOM_STEP 1.1
#define VIEW_PAN_STEP 60.0        /* screen pixels per arrow-key press */
#define SIM_RATE_MIN 0.1          /* slow enough to watch a linkage pass through a dead centre */
#define SIM_RATE_MAX 4.0
#define SIM_RATE_STEP 1.25
#define SIM_STEP_DT (1.0 / 60.0)  /* one frame, when stepping by hand */
#define DEFAULT_GRAVITY_MAGNITUDE 400.0 /* world-units/s^2; a qualitative default, not physically calibrated */
#define CAM_HIT_DIST 6.0
#define JOINT_HIT_DIST 7.0       /* pitch circles, Geneva rims, slider rails */
#define GENEVA_DEFAULT_SLOTS 6
#define GENEVA_MIN_SLOTS 3       /* below three the slots overlap */
#define GENEVA_MAX_SLOTS 12
#define TOOLTIP_DELAY_MS 350       /* dwell before a tooltip appears */
#define DEFAULT_WHEEL_RADIUS 80.0
/* Involute points per flank when drawing a wheel on the canvas. */
#define CANVAS_FLANK_SAMPLES 3
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
#define GALLERY_TITLE_H 26
#define GALLERY_FOOTER_H 22
#define PATH_TARGET_POINTS 64        /* the drawn path, resampled for four-bar fitting */
#define PATH_POOR_FIT_FRACTION 0.05  /* above this, say the linkage isn't up to the path */
#define PATH_TARGET_FRACTION 0.0025  /* stop adding arms below this share of the path's size */

/* World<->screen mapping: screen = world*zoom + pan. All Mechanism/mouse
 * positions are handled in world space; only rendering and raw SDL mouse
 * coordinates cross this boundary. The pan starts at the canvas's left edge
 * so world (0,0) is the top-left of the drawing area, not of the window. */
static double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Screen sizes for the things you point at and the numbers you read.
 *
 * These used to be plain multiples of the zoom, which is right for a pitch
 * circle or a cam profile -- those ARE sizes in the world -- but wrong for a
 * pin, a ground symbol or a dimension label. At ZOOM_MIN a pin was half a
 * pixel across; at ZOOM_MAX it was a fifty-pixel blob five times wider than
 * the ten-pixel target you actually had to hit. Clamping keeps a handle
 * handle-sized and a numeral readable at every zoom. */
static double handle_radius(double zoom)  { return clampd(5.0 * zoom, 3.0, 9.0); }
static double hatch_size(double zoom)     { return clampd(8.0 * zoom, 6.0, 14.0); }
static double pin_ring_radius(double zoom){ return clampd(7.0 * zoom, 5.0, 12.0); }
static double label_height(double zoom)   { return clampd(10.0 * zoom, 7.0, 14.0); }

static Vec2 screen_to_world(Vec2 screen, Vec2 pan, double zoom) {
    return (Vec2){ (screen.x - pan.x) / zoom, (screen.y - pan.y) / zoom };
}

static Vec2 world_to_screen(Vec2 world, Vec2 pan, double zoom) {
    return (Vec2){ world.x * zoom + pan.x, world.y * zoom + pan.y };
}

/* Where everything is, recomputed whenever the window changes size. Every
 * position that used to be written in terms of compile-time canvas constants
 * comes from here instead, so one resize moves the lot together. */
typedef struct {
    int win_w, win_h;
    UiRect canvas;   /* the drawing area, right of the toolbar */
    UiRect plot;     /* the time-series panel below it */
} Layout;

static Layout layout_for(int win_w, int win_h) {
    Layout l;
    l.win_w = win_w;
    l.win_h = win_h;

    int plot_h = (int)clampd(PLOT_FRACTION * win_h, PLOT_MIN_H, PLOT_MAX_H);
    if (plot_h > win_h / 2) plot_h = win_h / 2;       /* never more than half */
    int canvas_w = win_w - UI_TOOLBAR_W;
    if (canvas_w < 1) canvas_w = 1;
    int canvas_h = win_h - plot_h;
    if (canvas_h < 1) canvas_h = 1;

    l.canvas = (UiRect){ UI_TOOLBAR_W, 0, canvas_w, canvas_h };
    l.plot   = (UiRect){ UI_TOOLBAR_W, canvas_h, canvas_w, plot_h };
    return l;
}

static Vec2 rect_centre(UiRect r) {
    return (Vec2){ r.x + r.w / 2.0, r.y + r.h / 2.0 };
}

#define APP_PATH_MAX 1024

/* Asking for a file name.
 *
 * SDL has no file dialog, and a design tool with five numbered slots would be
 * worse than one where files have names. So: a one-line prompt drawn on the
 * canvas, in the same stroke font as everything else. The confirmations
 * (overwrite, quit with unsaved work) are the same overlay with nothing to
 * type into. */
typedef enum {
    PROMPT_NONE,
    PROMPT_SAVE,
    PROMPT_OPEN,
    PROMPT_EXPORT,
    PROMPT_PRINT,       /* a folder of STL parts, plus optional key=value settings */
    PROMPT_OVERWRITE,   /* "that file exists" */
    PROMPT_QUIT         /* "you have unsaved changes" */
} PromptKind;

typedef struct {
    PromptKind kind;
    char title[128];
    char hint[160];
    char text[APP_PATH_MAX];      /* what has been typed so far */
    char pending[APP_PATH_MAX];   /* the path a confirmation is about */
} Prompt;

static bool prompt_takes_typing(PromptKind k) {
    return k == PROMPT_SAVE || k == PROMPT_OPEN || k == PROMPT_EXPORT ||
            k == PROMPT_PRINT;
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
    DRAG_DRAW_PATH,
    DRAG_PAN
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
    /* Panning is screen-space: the mechanism does not move, the window does. */
    Vec2 pan_last_screen;
    /* Whether this drag has taken its undo snapshot yet. A plain click on an
     * already-selected pin used to snapshot the whole mechanism before knowing
     * anything would move, so clicking about filled the undo history with
     * steps that undo to the same picture. */
    bool drag_pushed_undo;

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

    Layout layout;

    Vec2 view_pan;
    double view_zoom;

    /* Seconds since the run started, stamped onto every trace sample so the
     * plot has a real time axis (frames are not uniform in length). */
    double sim_time;

    /* Watching, as opposed to just running it. RUN/STOP alone meant the only
     * way to look closely at a fast mechanism was to stop it, which throws it
     * back to its starting pose. */
    bool paused;
    bool step_once;       /* advance exactly one frame, then pause again */
    double sim_rate;      /* 1.0 is real time */

    bool gallery_open;
    int gallery_page;
    bool help_open;

    /* Everything the app has to say, shown on the canvas. */
    StatusLog status;

    /* The document. `current_path` is empty until the mechanism has been
     * saved or opened; `dirty` is whether it has been edited since. */
    char current_path[APP_PATH_MAX];
    bool dirty;
    Prompt prompt;
    SDL_Window *window;   /* only so the title bar can show the file name */

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
 * Talking to the user
 * ----------------------------------------------------------------------- */

/* Says something, once, in both places it could be read: the canvas, where
 * the user is actually looking, and stdout, where all of this used to go on
 * its own -- invisible to anyone who launched the app from a file manager or
 * whose window covers the terminal it was started from. */
static void app_message(App *a, StatusLevel level, const char *fmt, ...) {
    char text[STATUS_MAX_TEXT];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof text, fmt, args);
    va_end(args);

    status_push(&a->status, level, SDL_GetTicks(), text);
    printf("%s\n", text);
}

/* --------------------------------------------------------------------------
 * The document: naming it, saving it, opening it
 * ----------------------------------------------------------------------- */

static const char *path_basename(const char *path) {
    const char *slash = strrchr(path, '/');
    return slash ? slash + 1 : path;
}

/* Where a typed name actually lands.
 *
 * An absolute path is taken as given. A bare name goes in the working
 * directory -- unless that is not writable, which is exactly what happens when
 * the app is launched from a file manager and inherits "/" as its working
 * directory. Then it goes in the home directory instead, and the message says
 * where, rather than the write simply failing somewhere unseen. */
static void resolve_path(const char *typed, char *out, size_t out_size) {
    if (typed[0] == '/') {
        snprintf(out, out_size, "%s", typed);
        return;
    }
    const char *home = getenv("HOME");
    if (typed[0] == '~' && typed[1] == '/' && home) {
        snprintf(out, out_size, "%s/%s", home, typed + 2);
        return;
    }
    if (access(".", W_OK) == 0 || !home) {
        snprintf(out, out_size, "%s", typed);
        return;
    }
    snprintf(out, out_size, "%s/%s", home, typed);
}

/* Adds `.linkage` unless the name already carries an extension of its own. */
static void ensure_extension(char *path, size_t size, const char *ext) {
    const char *base = path_basename(path);
    if (strrchr(base, '.')) return;
    size_t len = strlen(path);
    snprintf(path + len, size - len, "%s", ext);
}

static void app_update_title(App *a) {
    if (!a->window) return;
    char title[APP_PATH_MAX + 64];
    const char *name = a->current_path[0] ? path_basename(a->current_path) : "untitled";
    snprintf(title, sizeof title, "Linkage Design - %s%s", name, a->dirty ? " *" : "");
    SDL_SetWindowTitle(a->window, title);
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
    a->dirty = true;
    app_update_title(a);
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
        app_message(a, STATUS_WARN, "Nothing to undo.");
        return;
    }
    push_snapshot(a->redo_stack, &a->redo_count, &a->mech);
    Mechanism restored;
    pop_snapshot(a->undo_stack, &a->undo_count, &restored);
    mechanism_free(&a->mech);
    a->mech = restored;
    a->dirty = true;
    app_update_title(a);
    app_message(a, STATUS_INFO, "Undo.");
}

static void app_redo(App *a) {
    if (a->redo_count <= 0) {
        app_message(a, STATUS_WARN, "Nothing to redo.");
        return;
    }
    /* Deliberately not push_undo(): redoing must not clear the redo stack. */
    push_snapshot(a->undo_stack, &a->undo_count, &a->mech);
    Mechanism restored;
    pop_snapshot(a->redo_stack, &a->redo_count, &restored);
    mechanism_free(&a->mech);
    a->mech = restored;
    a->dirty = true;
    app_update_title(a);
    app_message(a, STATUS_INFO, "Redo.");
}

/* --------------------------------------------------------------------------
 * The file prompt
 * ----------------------------------------------------------------------- */

static void prompt_close(App *a) {
    if (prompt_takes_typing(a->prompt.kind)) SDL_StopTextInput();
    a->prompt.kind = PROMPT_NONE;
    a->prompt.text[0] = '\0';
}

static void prompt_open(App *a, PromptKind kind, const char *title, const char *hint,
                         const char *initial) {
    prompt_close(a);
    a->prompt.kind = kind;
    snprintf(a->prompt.title, sizeof a->prompt.title, "%s", title);
    snprintf(a->prompt.hint, sizeof a->prompt.hint, "%s", hint);
    snprintf(a->prompt.text, sizeof a->prompt.text, "%s", initial ? initial : "");
    if (prompt_takes_typing(kind)) SDL_StartTextInput();
}

static void app_save_to(App *a, const char *typed) {
    char path[APP_PATH_MAX];
    resolve_path(typed, path, sizeof path);
    ensure_extension(path, sizeof path, SCENE_EXTENSION);

    char err[256] = { 0 };
    if (!scene_save(&a->mech, &a->params, path, err, sizeof err)) {
        app_message(a, STATUS_ERROR, "%s", err);
        return;
    }
    snprintf(a->current_path, sizeof a->current_path, "%s", path);
    a->dirty = false;
    app_update_title(a);
    app_message(a, STATUS_INFO, "Saved to %s", path);
}

static void app_open_from(App *a, const char *typed) {
    char path[APP_PATH_MAX];
    resolve_path(typed, path, sizeof path);
    ensure_extension(path, sizeof path, SCENE_EXTENSION);

    char err[256] = { 0 };
    if (!scene_load(&a->mech, &a->params, path, err, sizeof err)) {
        app_message(a, STATUS_ERROR, "%s", err);
        return;
    }
    /* A file just opened is a different mechanism, not an edit of this one:
     * the history of what was here before no longer applies to it. */
    clear_stack(a->undo_stack, &a->undo_count);
    clear_stack(a->redo_stack, &a->redo_count);
    a->gravity_set_by_user = (a->params.gravity.x != 0.0 || a->params.gravity.y != 0.0);
    a->path_ghost_count = 0;
    snprintf(a->current_path, sizeof a->current_path, "%s", path);
    a->dirty = false;
    app_update_title(a);
    app_message(a, STATUS_INFO, "Opened %s", path);
}

static void app_export_to(App *a, const char *path) {
    SolverParams export_params = a->params;
    export_params.gravity = effective_gravity(a);
    if (export_blender_script(&a->mech, export_params, path)) {
        app_message(a, STATUS_INFO,
                     "Exported %d animation frames to %s -- run it inside Blender's Scripting "
                     "tab (or blender --python %s), then press Space to play.",
                     EXPORT_FRAMES, path, path);
    } else {
        app_message(a, STATUS_ERROR, "Could not write %s.", path);
    }
}

/* The typed line is a folder name followed by any number of key=value
 * settings -- "gearbox m=1.5 t=4 fit=m3" -- so print settings can be tuned
 * without a panel of their own. */
static void app_print3d_to(App *a, const char *typed) {
    char line[APP_PATH_MAX];
    snprintf(line, sizeof line, "%s", typed);

    char *opts = line;
    while (*opts && *opts != ' ' && *opts != '\t') opts++;
    if (*opts) { *opts = '\0'; opts++; }

    PrintParams params = print_default_params();
    /* The mechanism was drawn at a particular module; print it at that unless
     * asked otherwise, so the teeth on the bed are the teeth on the canvas. */
    params.module = a->mech.gear_module;

    char err[256] = { 0 };
    if (!print_parse_options(&params, opts, err, sizeof err)) {
        app_message(a, STATUS_ERROR, "%s", err);
        return;
    }

    char dir[APP_PATH_MAX];
    resolve_path(line, dir, sizeof dir);

    char report[512] = { 0 };
    if (print3d_export(&a->mech, params, dir, report, sizeof report)) {
        app_message(a, STATUS_INFO, "%s", report);
    } else {
        app_message(a, STATUS_ERROR, "%s",
                     report[0] ? report : "Nothing could be written -- is there anything to print?");
    }
}

/* What a prompt does when Return is pressed. */
static void prompt_commit(App *a) {
    PromptKind kind = a->prompt.kind;
    char typed[APP_PATH_MAX];
    snprintf(typed, sizeof typed, "%s", a->prompt.text);
    char pending[APP_PATH_MAX];
    snprintf(pending, sizeof pending, "%s", a->prompt.pending);

    if (prompt_takes_typing(kind) && typed[0] == '\0') {
        app_message(a, STATUS_WARN, "No name given, so nothing was written.");
        prompt_close(a);
        return;
    }
    prompt_close(a);

    switch (kind) {
    case PROMPT_SAVE:
        app_save_to(a, typed);
        break;
    case PROMPT_OPEN:
        app_open_from(a, typed);
        break;
    case PROMPT_EXPORT: {
        char path[APP_PATH_MAX];
        resolve_path(typed, path, sizeof path);
        ensure_extension(path, sizeof path, ".py");
        /* EXPORT used to write over whatever was there without a word. */
        if (scene_file_exists(path)) {
            char title[160];
            snprintf(title, sizeof title, "%s ALREADY EXISTS", path_basename(path));
            prompt_open(a, PROMPT_OVERWRITE, title,
                         "PRESS Y TO WRITE OVER IT, ESC TO KEEP IT", NULL);
            snprintf(a->prompt.pending, sizeof a->prompt.pending, "%s", path);
        } else {
            app_export_to(a, path);
        }
        break;
    }
    case PROMPT_PRINT:
        app_print3d_to(a, typed);
        break;
    case PROMPT_OVERWRITE:
        app_export_to(a, pending);
        break;
    case PROMPT_QUIT:
        /* Handled by the event loop, which owns whether the app is running. */
    case PROMPT_NONE:
        break;
    }
}

/* --------------------------------------------------------------------------
 * Actions -- one per command, shared by the hotkeys and the toolbar so there
 * is exactly one implementation of each behaviour.
 * ----------------------------------------------------------------------- */

static void app_add_joint(App *a) {
    /* The middle of the canvas -- which is not the middle of the window, and
     * used to be neither: JOINT dropped its pin a hundred pixels below where
     * every other insert put one. */
    Vec2 p = screen_to_world(rect_centre(a->layout.canvas), a->view_pan, a->view_zoom);
    push_undo(a);
    clear_selection(&a->mech);
    int id = mechanism_add_connector(&a->mech, p, false);
    a->mech.connectors[id].selected = true;
}

static void app_link_selected(App *a) {
    int ids[256];
    int n = gather_selected_connectors(&a->mech, ids, 256);
    if (n < 2) {
        app_message(a, STATUS_WARN, "Select at least 2 connectors before linking.");
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
    if (!has_selection(&a->mech)) {
        app_message(a, STATUS_WARN, "Select one or more pins first, then ANCHOR grounds them.");
        return;
    }
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
        app_message(a, STATUS_WARN, "Select exactly one link before toggling a motor.");
        return;
    }
    if (!a->mech.links[lid].is_driven && link_anchor_count(&a->mech, lid) != 1) {
        app_message(a, STATUS_WARN, "A driven link needs exactly one anchor connector.");
        return;
    }
    push_undo(a);
    /* Naming a wheel as the driver turns its whole train round to suit, so it
     * works on any wheel rather than only the one the train was meshed from. */
    bool wheel = mechanism_is_gear_body(&a->mech, lid);
    if (wheel && !a->mech.links[lid].is_driven) mechanism_orient_train_from(&a->mech, lid);
    if (!mechanism_toggle_driven(&a->mech, lid, DEFAULT_MOTOR_SPEED_DEG_S)) {
        discard_last_undo(a);
        app_message(a, STATUS_WARN, "A driven link needs exactly one anchor connector.");
        return;
    }
    if (wheel && a->mech.links[lid].is_driven) {
        app_message(a, STATUS_INFO, "This wheel now drives; the rest of its train follows from it.");
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
        app_message(a, STATUS_WARN, "Could not create a cam there.");
        return;
    }
    Cam *cam = &a->mech.cams[cam_id];

    if (drawn) {
        Vec2 *local = malloc((size_t)n * sizeof(Vec2));
        for (int i = 0; i < n; i++) local[i] = vec2_sub(outline[i], centre);
        if (!cam_set_from_drawn_outline(cam, local, n)) {
            app_message(a, STATUS_WARN, "Couldn't read a profile from that outline -- using a default cam. "
                    "Try drawing a single loop right around the centre.");
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
    app_message(a, STATUS_INFO, "Cam added: base radius %.1f, lift %.1f. Press R to run. "
            "Select the cam to reshape it: +/- lift, [ and ] timing.",
            cam->base_radius, cam->lift);
    if (cam_is_undercut(cam)) {
        app_message(a, STATUS_WARN, "Warning: this cam undercuts -- it has a concave notch tighter than "
                "the roller, so it could not be cut to give this motion.");
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
    return screen_to_world(rect_centre(a->layout.canvas), a->view_pan, a->view_zoom);
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
            app_message(a, STATUS_WARN, "Those three don't make a slider.");
            return;
        }
        app_message(a, STATUS_INFO, "Slider added: that pin now runs along the line through the other two.");
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
    app_message(a, STATUS_INFO, "Crank-slider added. Drag the crank pin to change the stroke, or either "
            "rail anchor to aim the slide.");
}

/* Pans and zooms so the whole mechanism is on screen.
 *
 * `allow_zoom_in` is the difference between the two callers. Asked for by
 * hand (the FIT button, F), it should fill the canvas with whatever is there.
 * Called for you because a wheel was just placed past the edge, it must only
 * ever zoom OUT -- the canvas changing scale under a mechanism that already
 * fitted is not what you asked for by pressing GEAR. */
static void app_fit_view(App *a, bool allow_zoom_in) {
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
    if (!any) {
        app_message(a, STATUS_WARN, "There is nothing on the canvas to fit the view to yet.");
        return;
    }

    const double margin = 40.0;
    double w = (x1 - x0) + 2 * margin, h = (y1 - y0) + 2 * margin;
    if (w < 1.0) w = 1.0;
    if (h < 1.0) h = 1.0;
    double zoom = fmin((double)a->layout.canvas.w / w, (double)a->layout.canvas.h / h);
    if (zoom > ZOOM_MAX) zoom = ZOOM_MAX;
    if (!allow_zoom_in && zoom > a->view_zoom) zoom = a->view_zoom;
    if (zoom < ZOOM_MIN) zoom = ZOOM_MIN;
    a->view_zoom = zoom;
    Vec2 mid = { (x0 + x1) / 2.0, (y0 + y1) / 2.0 };
    Vec2 centre = rect_centre(a->layout.canvas);
    a->view_pan.x = centre.x - mid.x * zoom;
    a->view_pan.y = centre.y - mid.y * zoom;
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
            app_message(a, STATUS_WARN, "Those wheels are meshed already, or the second one is turned by "
                    "something else -- a wheel takes its motion from one place.");
            return;
        }
        app_fit_view(a, false);
        app_message(a, STATUS_INFO, "Meshed %d pair%s; each driven wheel slid into contact with its driver.%s "
                "Press M on a wheel to make it the one that drives.",
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
            app_message(a, STATUS_WARN, "Those two don't make a rack and pinion.");
            return;
        }
        app_message(a, STATUS_INFO, "Rack and pinion made. The pinion's pitch radius is how far it stands "
                "off the bar, so sliding the bar retimes it.");
        return;
    }

    /* Otherwise: one wheel, on its own, somewhere clear. */
    push_undo(a);
    double r = DEFAULT_WHEEL_RADIUS;
    int link = mechanism_add_wheel(&a->mech, free_wheel_spot(a, r), r);
    if (link < 0) { discard_last_undo(a); app_message(a, STATUS_WARN, "Couldn't place a wheel."); return; }
    clear_selection(&a->mech);
    a->mech.links[link].selected = true;
    int mark = mechanism_wheel_mark(&a->mech, link);
    if (mark >= 0) mechanism_set_traced(&a->mech, mark, true);
    app_fit_view(a, false);   /* wheels step outwards to find space; keep them in view */
    app_message(a, STATUS_INFO, "Wheel added, radius %.0f. +/- resizes it (Alt+/- sets its speed "
            "once it drives), M makes it the driver, and selecting two or more wheels and "
            "pressing GEAR meshes them.", r);
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
            if (gid < 0) { discard_last_undo(a); app_message(a, STATUS_WARN, "Those two don't make a Geneva."); return; }
            a->mech.genevas[gid].selected = true;
            app_message(a, STATUS_INFO, "Geneva added, %d slots. Select the wheel and use +/- to change the "
                    "slot count.", a->mech.genevas[gid].slot_count);
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
    app_message(a, STATUS_INFO, "Geneva added, %d slots -- one turn of the motor indexes it %.0f degrees. "
            "Use +/- to change the slot count, or drag a centre to resize it.",
            slots, 360.0 / slots);
}

static void app_toggle_cam_draw(App *a) {
    a->cam_draw_armed = !a->cam_draw_armed;
    a->cam_stroke_count = 0;
    if (a->cam_draw_armed) {
        app_message(a, STATUS_INFO, "Cam tool armed: drag on the canvas to draw the cam's outline "
                "(or click once for a default cam). Escape cancels.");
    }
}

static void app_adjust_cam_lift(App *a, double factor) {
    int cid = find_single_selected_cam(&a->mech);
    if (cid < 0) return;
    push_undo(a);
    Cam *c = &a->mech.cams[cid];
    cam_scale_lift(c, factor);
    app_message(a, STATUS_INFO, "Cam lift: %.1f", c->lift);
    if (cam_is_undercut(c)) app_message(a, STATUS_WARN, "Warning: this cam now undercuts.");
}

/* Rotating the profile against the shaft is cam timing: same motion, earlier
 * or later in the turn. */
static void app_adjust_cam_timing(App *a, double delta_rad) {
    int cid = find_single_selected_cam(&a->mech);
    if (cid < 0) return;
    push_undo(a);
    cam_rotate_profile(&a->mech.cams[cid], delta_rad);
    app_message(a, STATUS_INFO, "Cam timing shifted by %.0f deg.", delta_rad * 180.0 / M_PI);
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
    /* On a wheel that drives, +/- is ambiguous -- size or speed? Size wins,
     * and this is where to mention the key that means the other one. */
    if (a->mech.links[wheels[0]].is_driven) {
        app_message(a, STATUS_INFO, "Wheel radius %.0f. Alt+/- changes its speed instead.",
                     a->mech.links[wheels[0]].wheel_radius);
    } else {
        app_message(a, STATUS_INFO, "Wheel radius %.0f.", a->mech.links[wheels[0]].wheel_radius);
    }
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
    if (n == gv->slot_count) {
        app_message(a, STATUS_WARN, "A Geneva holds between %d and %d slots.",
                    GENEVA_MIN_SLOTS, GENEVA_MAX_SLOTS);
        return;
    }
    push_undo(a);
    a->mech.genevas[vi].slot_count = n;
    mechanism_refresh_joint_sizes(&a->mech);
    app_message(a, STATUS_INFO, "Geneva: %d slots, indexing %.0f degrees a turn.", n, 360.0 / n);
}

/* The gallery is a grid of tiles laid over the canvas.
 *
 * It pages rather than growing without limit: the panel is clipped to the
 * canvas when it is drawn, so a gallery taller than the window used to put its
 * last row somewhere it could be neither seen nor clicked. */
static int gallery_rows_per_page(const App *a) {
    int avail = a->layout.canvas.h - GALLERY_TITLE_H - GALLERY_FOOTER_H - 2 * GALLERY_PAD;
    int rows = avail / (GALLERY_TILE_H + GALLERY_PAD);
    if (rows < 1) rows = 1;
    int total = (templates_count() + GALLERY_COLS - 1) / GALLERY_COLS;
    if (rows > total) rows = total;
    return rows;
}

static int gallery_per_page(const App *a) { return gallery_rows_per_page(a) * GALLERY_COLS; }

static int gallery_page_count(const App *a) {
    int per = gallery_per_page(a);
    return (templates_count() + per - 1) / per;
}

static int gallery_page_first(const App *a) { return a->gallery_page * gallery_per_page(a); }

/* How many tiles this page actually shows -- the last one is usually short. */
static int gallery_page_size(const App *a) {
    int left = templates_count() - gallery_page_first(a);
    int per = gallery_per_page(a);
    return left < per ? left : per;
}

static UiRect gallery_panel(const App *a) {
    int shown = gallery_page_size(a);
    int rows = (shown + GALLERY_COLS - 1) / GALLERY_COLS;
    if (rows < 1) rows = 1;
    int w = GALLERY_COLS * GALLERY_TILE_W + (GALLERY_COLS + 1) * GALLERY_PAD;
    int h = rows * GALLERY_TILE_H + (rows + 1) * GALLERY_PAD + GALLERY_TITLE_H;
    if (gallery_page_count(a) > 1) h += GALLERY_FOOTER_H;
    UiRect canvas = a->layout.canvas;
    return (UiRect){ canvas.x + (canvas.w - w) / 2, canvas.y + (canvas.h - h) / 2, w, h };
}

/* `index` is a template id, not a slot: tiles on other pages have no rect. */
static UiRect gallery_tile(const App *a, int index) {
    UiRect panel = gallery_panel(a);
    int slot = index - gallery_page_first(a);
    int col = slot % GALLERY_COLS, row = slot / GALLERY_COLS;
    return (UiRect){ panel.x + GALLERY_PAD + col * (GALLERY_TILE_W + GALLERY_PAD),
                      panel.y + GALLERY_TITLE_H + GALLERY_PAD + row * (GALLERY_TILE_H + GALLERY_PAD),
                      GALLERY_TILE_W, GALLERY_TILE_H };
}

static int gallery_hit(const App *a, int x, int y) {
    int first = gallery_page_first(a);
    for (int i = first; i < first + gallery_page_size(a); i++) {
        UiRect r = gallery_tile(a, i);
        if (x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h) return i;
    }
    return -1;
}

static void gallery_turn_page(App *a, int delta) {
    int pages = gallery_page_count(a);
    if (pages <= 1) return;
    a->gallery_page = (a->gallery_page + delta % pages + pages) % pages;
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

/* The keys that have no button, so the help can list them too. Everything
 * else in the overlay is read straight off the toolbar, which is what keeps
 * the two from drifting apart. */
typedef struct { const char *keys, *what; } KeyNote;

static const KeyNote EXTRA_KEYS[] = {
    { "CLICK EMPTY",   "PLACE A PIN THERE" },
    { "DRAG EMPTY",    "BOX-SELECT PINS" },
    { "SHIFT-CLICK",   "ADD TO OR REMOVE FROM THE SELECTION" },
    { "DRAG A PIN",    "MOVE THE WHOLE SELECTION" },
    { "MIDDLE-DRAG",   "PAN THE VIEW" },
    { "WHEEL",         "ZOOM ABOUT THE POINTER" },
    { "ARROWS",        "PAN (TURN GALLERY PAGES)" },
    { "0",             "RESET THE VIEW TO 1:1" },
    { "+ -",           "RESIZE OR RETIME WHATEVER IS SELECTED" },
    { "ALT + -",       "ALWAYS THE MOTOR'S SPEED, EVEN ON A WHEEL" },
    { "[ ]",           "SHIFT A SELECTED CAM'S TIMING" },
    { ".",             "STEP ONE FRAME WHILE PAUSED" },
    { "< >",           "RUN SLOWER OR FASTER" },
    { "SHIFT+E",       "PRINTABLE STL PARTS, NOT A BLENDER SCRIPT" },
    { "SHIFT+CMD+S",   "SAVE UNDER A NEW NAME" },
    { "ESC",           "CANCEL, OR CLEAR THE SELECTION" },
};
#define EXTRA_KEY_COUNT ((int)(sizeof EXTRA_KEYS / sizeof EXTRA_KEYS[0]))

/* Every command in one place. The toolbar half is generated from the buttons
 * themselves, so a command can never be added without appearing here. */
static void draw_help(SDL_Renderer *ren, const App *a) {
    UiRect canvas = a->layout.canvas;
    UiRect panel = { canvas.x + 20, canvas.y + 16, canvas.w - 40, canvas.h - 32 };
    if (panel.w < 200 || panel.h < 160) return;
    render_rect_filled(ren, panel, 22, 23, 28, 246);
    render_rect_outline(ren, panel, 110, 118, 140, 255);

    const char *title = "EVERY COMMAND. PRESS H OR ESC TO CLOSE.";
    double tw = render_text_width(10.0, title);
    render_text(ren, (Vec2){ panel.x + (panel.w - tw) / 2.0, panel.y + 12.0 }, 10.0, title,
                 220, 225, 235, 255);

    const double row = 15.0, text_h = 8.5;
    double top = panel.y + 38.0;
    double col_w = (panel.w - 48.0) / 2.0;
    int rows_per_col = (int)((panel.h - 56.0) / row);
    if (rows_per_col < 1) rows_per_col = 1;

    int total = a->toolbar.count + 1 + EXTRA_KEY_COUNT;   /* +1 for the divider */
    for (int i = 0; i < total; i++) {
        int col = i / rows_per_col, r = i % rows_per_col;
        if (col > 1) break;                       /* two columns is all there is room for */
        double x = panel.x + 20.0 + col * (col_w + 8.0);
        double y = top + r * row;

        const char *keys, *what;
        if (i < a->toolbar.count) {
            keys = a->toolbar.buttons[i].hint;
            what = a->toolbar.buttons[i].label;
        } else if (i == a->toolbar.count) {
            render_text(ren, (Vec2){ x, y }, text_h, "MOUSE AND THE REST", 150, 156, 172, 255);
            continue;
        } else {
            const KeyNote *n = &EXTRA_KEYS[i - a->toolbar.count - 1];
            keys = n->keys;
            what = n->what;
        }
        render_text(ren, (Vec2){ x, y }, text_h, keys, 255, 225, 110, 255);
        render_text(ren, (Vec2){ x + 96.0, y }, text_h, what, 200, 206, 218, 255);
    }
}

static void app_toggle_gallery(App *a) {
    a->gallery_open = !a->gallery_open;
    a->gallery_page = 0;
    if (a->gallery_open) {
        app_message(a, STATUS_INFO, "Template gallery: click a mechanism to drop it on the canvas. Escape closes.");
    }
}

static void app_insert_template(App *a, int index) {
    const Template *t = templates_get(index);
    if (!t) return;
    push_undo(a);
    clear_selection(&a->mech);

    t->build(&a->mech, app_view_centre(a), 1.0);
    a->gallery_open = false;
    app_message(a, STATUS_INFO, "Inserted %s -- %s. Press R to run it.", t->name, t->blurb);
}

static void app_arm_path_tool(App *a, PathTool tool) {
    a->path_tool = (a->path_tool == tool) ? PATH_TOOL_NONE : tool;
    a->path_stroke_count = 0;
    if (a->path_tool == PATH_TOOL_LINKAGE) {
        app_message(a, STATUS_INFO, "Linkage tool armed: draw a curve and a four-bar will be fitted to it -- "
                "five parts and one motor, but only the curves a four-bar can trace. "
                "Escape cancels.");
    } else if (a->path_tool == PATH_TOOL_ARMS) {
        app_message(a, STATUS_INFO, "Arms tool armed: draw any curve at all and a chain of rotating arms "
                "will be built to redraw it exactly. Escape cancels.");
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
        app_message(a, STATUS_WARN, "That path decomposed to nothing usable.");
        return;
    }

    mechanism_set_traced(&a->mech, previous, true);
    a->mech.connectors[previous].selected = true;

    app_message(a, STATUS_INFO, "Built a %d-arm drawing machine for that %s path. Average miss %.2f units "
            "(%.2f%% of its size). Press R to watch it draw.",
            built, closed ? "closed" : "open", rms, size > 0.0 ? 100.0 * rms / size : 0.0);
}

/* Turns a fitted four-bar into real, editable mechanism parts: two grounded
 * pivots, the crank driven by a motor, a ternary coupler carrying the traced
 * point, and the rocker closing the loop. */
static void app_build_four_bar(App *a, const FourBar *fb, double error, double size) {
    Vec2 crank_end, coupler_end, traced;
    if (!fourbar_pose(fb, 0.0, &crank_end, &coupler_end, &traced)) {
        app_message(a, STATUS_WARN, "The fitted linkage could not be assembled; nothing was added.");
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
    app_message(a, STATUS_INFO, "Fitted a four-bar: crank %.1f, coupler %.1f, rocker %.1f, ground %.1f. "
            "Average miss %.2f units (%.2f%% of the path). Press R to watch it trace.",
            fb->crank, fb->coupler, fb->rocker, vec2_dist(fb->ground_a, fb->ground_b),
            error, percent);
    if (error > size * PATH_POOR_FIT_FRACTION) {
        app_message(a, STATUS_WARN, "That path is outside what a four-bar can trace. Undo and use ARMS "
                "for a machine that will follow it exactly.");
    }
}

static void app_synthesize_linkage(App *a) {
    int n = a->path_stroke_count;
    if (n < 4) {
        app_message(a, STATUS_WARN, "That stroke is too short to fit a linkage to.");
        return;
    }
    bool closed = synth_stroke_is_closed(a->path_stroke, n);

    a->path_ghost_count = PATH_GHOST_POINTS;
    a->path_ghost_closed = closed;
    synth_resample(a->path_stroke, n, closed, a->path_ghost, PATH_GHOST_POINTS);

    Vec2 target[PATH_TARGET_POINTS];
    synth_resample(a->path_stroke, n, closed, target, PATH_TARGET_POINTS);
    double size = synth_path_size(a->path_stroke, n);

    app_message(a, STATUS_INFO, "Searching for a four-bar that traces that %s path...", closed ? "closed" : "open");
    FourBar fb;
    double error = 0.0;
    if (!synth_fit_four_bar(target, PATH_TARGET_POINTS, closed, synth_default_params(), &fb, &error)) {
        app_message(a, STATUS_WARN, "No four-bar linkage could be fitted to that path. Try ARMS instead.");
        return;
    }
    app_build_four_bar(a, &fb, error, size);
}

static void app_synthesize_arms(App *a) {
    int n = a->path_stroke_count;
    if (n < 4) {
        app_message(a, STATUS_WARN, "That stroke is too short to build a mechanism from.");
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
        app_message(a, STATUS_WARN, "Couldn't read a usable path from that stroke.");
        return;
    }
    app_build_fourier_chain(a, anchor, arms, count, rms, size, closed);
}

static void app_toggle_vary(App *a) {
    int lid = find_single_selected_link(&a->mech);
    if (lid < 0) {
        app_message(a, STATUS_WARN, "Select exactly one link before toggling its length.");
        return;
    }
    if (a->mech.links[lid].is_driven) {
        app_message(a, STATUS_WARN, "A driven link's shape is always rigid; turn its motor off first.");
        return;
    }
    push_undo(a);
    bool now_rigid = !a->mech.links[lid].rigid;
    mechanism_set_rigid(&a->mech, lid, now_rigid);
    app_message(a, STATUS_INFO, "Link length is now %s.", now_rigid ? "fixed" : "variable");
}

static void app_toggle_trace(App *a) {
    if (!has_selection(&a->mech)) {
        app_message(a, STATUS_WARN, "Select one or more pins first, then TRACE records where they go.");
        return;
    }
    push_undo(a);
    for (int i = 0; i < a->mech.connector_count; i++) {
        if (a->mech.connectors[i].alive && a->mech.connectors[i].selected) {
            mechanism_set_traced(&a->mech, i, !a->mech.connectors[i].traced);
        }
    }
}

static void app_adjust_motor_speed(App *a, double step) {
    int lid = find_single_selected_link(&a->mech);
    if (lid < 0 || !a->mech.links[lid].is_driven) {
        /* The end of the +/- chain, so this is what "+ did nothing" means:
         * nothing that has a size or a speed is selected. */
        app_message(a, STATUS_WARN, "+ and - resize whatever is selected: a wheel, a cam's lift, "
                     "a Geneva's slots, or a motor's speed. Select one of those first.");
        return;
    }
    /* Retiming a motor mid-run is the one edit worth making while watching,
     * and the solver reads the speed fresh every frame. No snapshot for it
     * though: an undo taken mid-run would restore running positions. */
    if (a->state == APP_EDIT) push_undo(a);
    a->mech.links[lid].motor_speed_deg_s += step;
    app_message(a, STATUS_INFO, "Motor speed: %.1f deg/s", a->mech.links[lid].motor_speed_deg_s);
}

static void app_delete_selection(App *a) {
    if (!has_selection(&a->mech)) {
        app_message(a, STATUS_WARN, "Nothing is selected, so there is nothing to delete.");
        return;
    }
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
        app_message(a, STATUS_INFO, "Gravity ON.");
    } else {
        a->params.gravity = (Vec2){ 0.0, 0.0 };
        app_message(a, STATUS_INFO, "Gravity OFF.");
    }
}

/* Asks for a name first. It used to write "linkage_export.py" in whatever the
 * working directory happened to be, over whatever was already called that. */
static void app_export(App *a) {
    char suggestion[APP_PATH_MAX];
    if (a->current_path[0]) {
        snprintf(suggestion, sizeof suggestion, "%s", path_basename(a->current_path));
        char *dot = strrchr(suggestion, '.');
        if (dot) *dot = '\0';
        size_t len = strlen(suggestion);
        snprintf(suggestion + len, sizeof suggestion - len, ".py");
    } else {
        snprintf(suggestion, sizeof suggestion, "linkage_export.py");
    }
    prompt_open(a, PROMPT_EXPORT, "EXPORT A BLENDER SCRIPT AS",
                 "TYPE A FILE NAME, RETURN TO WRITE IT, ESC TO CANCEL", suggestion);
}

/* A folder, not a file: one STL per part, plus the manifest that says how they
 * go together. */
static void app_print3d(App *a) {
    char suggestion[APP_PATH_MAX];
    if (a->current_path[0]) {
        snprintf(suggestion, sizeof suggestion, "%s", path_basename(a->current_path));
        char *dot = strrchr(suggestion, '.');
        if (dot) *dot = '\0';
        size_t len = strlen(suggestion);
        snprintf(suggestion + len, sizeof suggestion - len, "_parts");
    } else {
        snprintf(suggestion, sizeof suggestion, "linkage_parts");
    }
    prompt_open(a, PROMPT_PRINT, "PRINT STL PARTS INTO FOLDER",
                 "FOLDER, THEN OPTIONS: M= PIN= T= CLR= GAP= WALL= BL= PA= FIT= BASE= ASM=", suggestion);
}

static void app_save(App *a) {
    /* Once it has a name, SAVE means save. Without one, ask for it. */
    if (a->current_path[0]) {
        app_save_to(a, a->current_path);
        return;
    }
    prompt_open(a, PROMPT_SAVE, "SAVE THE MECHANISM AS",
                 "TYPE A FILE NAME, RETURN TO SAVE, ESC TO CANCEL", "mechanism.linkage");
}

static void app_save_as(App *a) {
    prompt_open(a, PROMPT_SAVE, "SAVE THE MECHANISM AS",
                 "TYPE A FILE NAME, RETURN TO SAVE, ESC TO CANCEL",
                 a->current_path[0] ? path_basename(a->current_path) : "mechanism.linkage");
}

static void app_open(App *a) {
    prompt_open(a, PROMPT_OPEN, "OPEN A MECHANISM",
                 "TYPE A FILE NAME, RETURN TO OPEN, ESC TO CANCEL",
                 a->current_path[0] ? path_basename(a->current_path) : "mechanism.linkage");
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
        app_message(a, STATUS_WARN, "No motor in this mechanism -- running it under gravity. "
                "Use the GRAVITY button (or G) to control gravity yourself.");
    }

    a->sim_time = 0.0;
    a->paused = false;
    a->step_once = false;
    status_clear_sticky(&a->status);
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
    status_clear_sticky(&a->status);
    a->state = APP_EDIT;
}

static void app_toggle_pause(App *a) {
    if (a->state != APP_RUNNING) {
        app_message(a, STATUS_WARN, "Nothing is running to pause. Press R to start it.");
        return;
    }
    a->paused = !a->paused;
    a->step_once = false;
    app_message(a, STATUS_INFO, a->paused
                 ? "Paused. Space runs on again; . steps one frame at a time."
                 : "Running.");
}

static void app_step_frame(App *a) {
    if (a->state != APP_RUNNING) {
        app_message(a, STATUS_WARN, "Nothing is running to step. Press R to start it.");
        return;
    }
    a->paused = true;      /* stepping is what you do while paused */
    a->step_once = true;
}

static void app_adjust_sim_rate(App *a, double factor) {
    a->sim_rate = clampd(a->sim_rate * factor, SIM_RATE_MIN, SIM_RATE_MAX);
    app_message(a, STATUS_INFO, "Simulation speed %.2fx real time.", a->sim_rate);
}

static void app_toggle_run(App *a) {
    if (a->state == APP_EDIT) app_start_run(a);
    else app_stop_run(a);
}

/* The view is not the mechanism: this puts the canvas back where the app
 * started, rather than wherever the parts happen to be. */
static void app_reset_view(App *a) {
    a->view_pan = (Vec2){ UI_TOOLBAR_W, 0 };
    a->view_zoom = 1.0;
    app_message(a, STATUS_INFO, "View reset to 1:1.");
}

/* Pans by a screen-space amount. Panning is the one view operation that needs
 * no world coordinates at all -- the mechanism does not move, the window does. */
static void app_pan_by(App *a, double dx, double dy) {
    a->view_pan.x += dx;
    a->view_pan.y += dy;
}

static void app_dispatch(App *a, UiAction action) {
    switch (action) {
    case UI_OPEN:     app_open(a); break;
    case UI_SAVE:     app_save(a); break;
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
    case UI_FIT:     app_fit_view(a, true); break;
    case UI_CLEAR:
        /* Traces only. The drawn target path is not a trace -- it is what you
         * asked the machine for, and wiping it takes away the one thing the
         * result can be judged against. It clears when a new path is drawn. */
        mechanism_clear_traces(&a->mech);
        app_message(a, STATUS_INFO, "Traces cleared.");
        break;
    case UI_EXPORT:  app_export(a); break;
    case UI_PRINT:   app_print3d(a); break;
    case UI_RUN:     app_toggle_run(a); break;
    case UI_PAUSE:   app_toggle_pause(a); break;
    case UI_HELP:    a->help_open = !a->help_open; break;
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
    s.jammed = a->jammed;
    s.paused = a->paused;
    s.help_open = a->help_open;
    /* Always offered: with nothing selected each one draws a whole working
     * assembly, so there is no selection they could be waiting for. */
    s.can_make_slider = true;
    s.can_make_gear = true;
    s.can_make_geneva = true;
    return s;
}

/* Every command goes through here, from the toolbar and from the keyboard
 * alike, so the two can never disagree about what is allowed. A refused
 * command explains itself in the same words the button's tooltip uses -- the
 * keyboard used to fail silently while the button beside it greyed out. */
static void app_try_action(App *a, UiAction action) {
    /* The buttons' flags are refreshed once a frame for drawing; an event
     * earlier in this same frame may have changed the selection since. */
    ui_apply_state(&a->toolbar, app_ui_state(a));

    if (ui_action_enabled(&a->toolbar, action)) {
        app_dispatch(a, action);
        return;
    }
    const char *label = ui_action_label(&a->toolbar, action);
    if (a->state == APP_RUNNING) {
        app_message(a, STATUS_WARN,
                     "%s is an edit and the simulation is running. Press R (or STOP) first.", label);
    } else {
        app_message(a, STATUS_WARN, "%s: %s", label, ui_action_tip(&a->toolbar, action));
    }
}

/* The keyboard half of the toolbar. Every command that has a button is named
 * here and nowhere else, so a key and its button stay one command with one
 * set of rules. Returns UI_NONE for keys that are not toolbar commands. */
static UiAction action_for_key(SDL_Keycode k, bool cmd, bool shift) {
    if (cmd) {
        /* Held with a modifier, a letter is a different key. C used to wipe
         * every trace on Cmd+C, which is the shortcut for the opposite. */
        switch (k) {
        case SDLK_z: return shift ? UI_REDO : UI_UNDO;
        case SDLK_y: return UI_REDO;
        case SDLK_s: return UI_SAVE;    /* Shift+Cmd+S is handled as "save as" */
        case SDLK_o: return UI_OPEN;
        default:     return UI_NONE;
        }
    }
    switch (k) {
    case SDLK_n:         return UI_TEMPLATE;
    case SDLK_j:         return UI_JOINT;
    case SDLK_a:         return UI_ANCHOR;
    case SDLK_l:         return UI_LINK;
    case SDLK_m:         return UI_MOTOR;
    case SDLK_s:         return UI_SLIDER;
    case SDLK_o:         return UI_GEAR;
    case SDLK_w:         return UI_GENEVA;
    case SDLK_k:         return UI_CAM;
    case SDLK_p:         return UI_LINKAGE;
    case SDLK_b:         return UI_ARMS;
    case SDLK_v:         return UI_VARY;
    case SDLK_t:         return UI_TRACE;
    case SDLK_e:         return shift ? UI_PRINT : UI_EXPORT;
    case SDLK_f:         return UI_FIT;
    case SDLK_c:         return UI_CLEAR;
    case SDLK_h:         return UI_HELP;
    case SDLK_SLASH:     return UI_HELP;   /* '?' is shift-slash */
    case SDLK_g:         return UI_GRAVITY;
    case SDLK_r:         return UI_RUN;
    case SDLK_DELETE:
    case SDLK_BACKSPACE: return UI_DELETE;
    default:             return UI_NONE;
    }
}

static void draw_mechanism(SDL_Renderer *ren, const Mechanism *m, DragMode drag_mode, Vec2 drag_start, Vec2 drag_current,
                            Vec2 view_pan, double view_zoom) {
    /* Traces first, behind the parts. Projected into a scratch buffer and
     * drawn as one run: these are the longest thing on the canvas by far. */
    static Vec2 trace_screen[MECHANISM_TRACE_MAX];
    for (int i = 0; i < m->connector_count; i++) {
        const Connector *c = &m->connectors[i];
        if (!c->alive || !c->traced || c->path_count < 2) continue;
        Uint8 tr, tg, tb;
        render_trace_color(i, &tr, &tg, &tb);
        int n = c->path_count < MECHANISM_TRACE_MAX ? c->path_count : MECHANISM_TRACE_MAX;
        for (int k = 0; k < n; k++) {
            trace_screen[k] = world_to_screen(c->path[k], view_pan, view_zoom);
        }
        render_polyline(ren, trace_screen, n, tr, tg, tb, 255);
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
            /* Touching is solid, floating is hollow. Green against red is
             * exactly the pair that reads as one colour to some people. */
            double rr = cam->roller_radius * view_zoom;
            if (cam->in_contact) render_circle(ren, f, rr, 120, 215, 140, 255);
            else {
                render_circle_outline(ren, f, rr, 235, 110, 90, 255);
                render_circle_outline(ren, f, rr - 2.0, 235, 110, 90, 255);
            }
        }

        /* Base radius and lift, in the same seven-segment numerals the link
         * lengths use. */
        double dh = label_height(view_zoom);
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
            render_circle_outline(ren, pin, pin_ring_radius(view_zoom), 130, 190, 235, 255);
        }
    }

    /* Wheels are drawn from the bodies themselves, not from the meshes, so a
     * wheel put down on its own shows up straight away and is there to be
     * sized and moved before anything is connected to it.
     *
     * Room for the biggest wheel's outline, rebuilt each frame like the
     * traces are. Coarser than the exported profile: on screen a flank is a
     * few pixels, and the whole point is the tooth COUNT being visible. */
    static Vec2 tooth_scratch[GEARING_MAX_TEETH * (2 * CANVAS_FLANK_SAMPLES + 9) + 8];
    static Vec2 tooth_screen[GEARING_MAX_TEETH * (2 * CANVAS_FLANK_SAMPLES + 9) + 8];
    /* The orientation each wheel has to be drawn at for its teeth to fall into
     * its neighbour's spaces rather than land on their tips. Recomputed each
     * frame, which is cheap and means it stays right as the train turns and as
     * wheels are dragged about. */
    static double *tooth_phase = NULL;
    static int tooth_phase_cap = 0;
    if (m->link_count > tooth_phase_cap) {
        double *grown = realloc(tooth_phase, (size_t)m->link_count * sizeof(double));
        if (grown) { tooth_phase = grown; tooth_phase_cap = m->link_count; }
    }
    if (tooth_phase && tooth_phase_cap >= m->link_count) mechanism_gear_phases(m, tooth_phase);
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
        /* The teeth themselves, from the same generator the STL export
         * extrudes -- so the mesh you judge by eye is the mesh that prints.
         * They are drawn at the wheel's live rotation, so a running train
         * shows its teeth going through each other's spaces. */
        int teeth = mechanism_wheel_teeth(m, li);
        double spin = (tooth_phase && li < tooth_phase_cap) ? tooth_phase[li] : 0.0;
        GearSpec spec = { m->gear_module, teeth, GEARING_PRESSURE_ANGLE, 0.0,
                          CANVAS_FLANK_SAMPLES };
        int cap = gearing_outline_capacity(&spec);
        bool drew_teeth = false;
        if (cap > 0 && cap <= (int)(sizeof tooth_scratch / sizeof tooth_scratch[0])) {
            int n = gearing_tooth_outline(&spec, tooth_scratch, cap);
            if (n > 2) {
                for (int i = 0; i < n; i++) {
                    Vec2 p = vec2_add(m->connectors[centre].pos, vec2_rotate(tooth_scratch[i], spin));
                    tooth_screen[i] = world_to_screen(p, view_pan, view_zoom);
                }
                tooth_screen[n] = tooth_screen[0];
                render_polyline(ren, tooth_screen, n + 1, cr, cg, cb, 255);
                drew_teeth = true;
            }
        }

        /* The pitch circle stays as a faint guide -- it is where two wheels
         * touch, which the tooth outline itself does not show. Dashed means
         * "something else turns this one", the same fact the olive colour
         * carries, said a second way. */
        if (l->driven_externally) render_dashed_circle(ren, c0, r * view_zoom, cr, cg, cb, drew_teeth ? 90 : 255);
        else render_circle_outline(ren, c0, r * view_zoom, cr, cg, cb, drew_teeth ? 90 : 255);

        /* The tooth count, not the radius: it is what decides whether two
         * wheels can mesh and what ratio they give. */
        double dh = label_height(view_zoom);
        render_number(ren, (Vec2){ c0.x + 5.0, c0.y - r * view_zoom - dh - 4.0 },
                      0.0, dh, teeth > 0 ? (double)teeth : r, cr, cg, cb, 255);
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
        double dh = label_height(view_zoom);
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
        /* A body with n pins is drawn as all n(n-1)/2 edges between them, but
         * labelling every one buries a five-pin plate under ten numerals. Only
         * the longest edge -- the body's overall size -- gets a dimension. */
        int label_i = -1, label_j = -1;
        double longest = -1.0;
        for (int i = 0; i < l->connector_count; i++) {
            for (int j = i + 1; j < l->connector_count; j++) {
                double d = vec2_dist(m->connectors[l->connector_ids[i]].pos,
                                      m->connectors[l->connector_ids[j]].pos);
                if (d > longest) { longest = d; label_i = i; label_j = j; }
            }
        }

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
                double digit_height = label_height(view_zoom);
                bool label_this = (i == label_i && j == label_j);
                if (label_this && dir_len > 1e-6 &&
                    dir_len > render_number_width(digit_height, length) + 8.0) {
                    Vec2 dir_unit = vec2_scale(dir, 1.0 / dir_len);
                    double angle = atan2(dir_unit.y, dir_unit.x);
                    /* Keep text reading left-to-right rather than upside down
                     * when the edge points leftward. */
                    if (dir_unit.x < 0.0) {
                        angle += M_PI;
                        dir_unit = vec2_scale(dir_unit, -1.0);
                    }
                    Vec2 perp_unit = vec2_perp(dir_unit); /* consistent side once dir_unit is normalized above */
                    double gap = clampd(6.0 * view_zoom, 4.0, 9.0);
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
        double hr = handle_radius(view_zoom);
        render_circle(ren, sp, hr, r, g, b, 255);
        /* Selected pins get a ring as well as the yellow. */
        if (c->selected) render_circle_outline(ren, sp, hr + 3.0, 255, 225, 70, 255);
        if (c->is_anchor) render_ground_hatch(ren, sp, hatch_size(view_zoom), r, g, b, 255);
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

/* One place where a key press becomes something happening. Everything that is
 * a toolbar command goes through app_try_action; what is left is the handful
 * of keys that have no button -- the view, the modifiers on a selection, and
 * cancelling out of whatever is armed. */
/* While a prompt is up it has the keyboard to itself: nothing else should be
 * happening to the mechanism behind a question you are part-way through
 * answering. Returns whether the key was the prompt's. */
static bool prompt_handle_key(App *a, SDL_Keycode k) {
    if (a->prompt.kind == PROMPT_NONE) return false;

    if (k == SDLK_ESCAPE) {
        prompt_close(a);
        app_message(a, STATUS_INFO, "Cancelled.");
        return true;
    }
    if (prompt_takes_typing(a->prompt.kind)) {
        if (k == SDLK_RETURN || k == SDLK_KP_ENTER) { prompt_commit(a); return true; }
        if (k == SDLK_BACKSPACE) {
            size_t len = strlen(a->prompt.text);
            if (len > 0) a->prompt.text[len - 1] = '\0';
            return true;
        }
        return true;   /* everything else is typing, or nothing at all */
    }
    /* A confirmation: one key means yes, anything else waits. */
    if (k == SDLK_y || k == SDLK_RETURN || k == SDLK_KP_ENTER) { prompt_commit(a); return true; }
    return true;
}

static void app_handle_key(App *a, const SDL_KeyboardEvent *key) {
    SDL_Keycode k = key->keysym.sym;
    SDL_Keymod mod = SDL_GetModState();
    bool cmd = (mod & (KMOD_CTRL | KMOD_GUI)) != 0;
    bool shift = (mod & KMOD_SHIFT) != 0;
    /* Alt, not shift, is the modifier on +/-: on most keyboards "+" IS
     * shift-equals, so shift is not free to mean anything there. */
    bool alt = (mod & KMOD_ALT) != 0;

    if (prompt_handle_key(a, k)) return;

    /* Save-as is the one command with no button of its own: SAVE means save
     * once the mechanism has a name, and this is how you give it another. */
    if (cmd && shift && k == SDLK_s) {
        if (a->state == APP_EDIT) app_save_as(a);
        return;
    }

    /* Auto-repeat used to fire the command again on every repeat: holding J
     * sprayed joints (and undo snapshots), holding N strobed the gallery, and
     * holding K armed and disarmed the cam tool a dozen times. Only the keys
     * that mean "more of this" want repeating. */
    bool repeatable = (k == SDLK_EQUALS || k == SDLK_KP_PLUS ||
                        k == SDLK_MINUS  || k == SDLK_KP_MINUS ||
                        k == SDLK_LEFTBRACKET || k == SDLK_RIGHTBRACKET ||
                        k == SDLK_LEFT || k == SDLK_RIGHT || k == SDLK_UP || k == SDLK_DOWN);
    if (key->repeat && !repeatable) return;

    UiAction action = action_for_key(k, cmd, shift);
    if (action != UI_NONE) {
        app_try_action(a, action);
        return;
    }

    /* While the gallery is up, the arrows page it rather than moving a view
     * nobody can see. */
    if (a->gallery_open && (k == SDLK_LEFT || k == SDLK_RIGHT)) {
        gallery_turn_page(a, k == SDLK_RIGHT ? 1 : -1);
        return;
    }

    /* View controls, which work whatever the app is doing. */
    switch (k) {
    case SDLK_0:     app_reset_view(a); return;
    case SDLK_SPACE: app_toggle_pause(a); return;
    case SDLK_PERIOD:
        /* Shifted, the same key means "faster": > and . on one keycap. */
        if (shift) app_adjust_sim_rate(a, SIM_RATE_STEP);
        else app_step_frame(a);
        return;
    case SDLK_COMMA:
        if (shift) app_adjust_sim_rate(a, 1.0 / SIM_RATE_STEP);
        return;
    case SDLK_LEFT:  app_pan_by(a,  VIEW_PAN_STEP, 0.0); return;
    case SDLK_RIGHT: app_pan_by(a, -VIEW_PAN_STEP, 0.0); return;
    case SDLK_UP:    app_pan_by(a, 0.0,  VIEW_PAN_STEP); return;
    case SDLK_DOWN:  app_pan_by(a, 0.0, -VIEW_PAN_STEP); return;
    default: break;
    }

    if (a->state != APP_EDIT) {
        /* Changing a running motor's speed is allowed -- watching a mechanism
         * and asking "what if it turned faster" is the same action. */
        if (k == SDLK_EQUALS || k == SDLK_KP_PLUS) {
            app_adjust_motor_speed(a, MOTOR_SPEED_STEP_DEG_S);
            return;
        }
        if (k == SDLK_MINUS || k == SDLK_KP_MINUS) {
            app_adjust_motor_speed(a, -MOTOR_SPEED_STEP_DEG_S);
            return;
        }
        /* The rest are edits. They used to do nothing at all here, which reads
         * exactly like a key that does not exist. */
        if (k == SDLK_ESCAPE || k == SDLK_LEFTBRACKET || k == SDLK_RIGHTBRACKET) {
            app_message(a, STATUS_WARN,
                         "That changes the mechanism, and the simulation is running. "
                         "Press R (or STOP) first.");
        }
        return;
    }

    switch (k) {
    case SDLK_LEFTBRACKET:  app_adjust_cam_timing(a, -CAM_TIMING_STEP); return;
    case SDLK_RIGHTBRACKET: app_adjust_cam_timing(a, CAM_TIMING_STEP); return;

    /* +/- means "more of whatever is selected": a cam's lift, a wheel's
     * radius, a Geneva's slot count -- and, when the selected thing has no
     * size of its own, its motor's speed.
     *
     * A DRIVEN WHEEL is both of those things at once, and size won the
     * cascade, which left the speed of a motorised gear with no key at all.
     * Alt+/- always means the speed, so both are reachable on the one part. */
    case SDLK_EQUALS:
    case SDLK_KP_PLUS:
        if (alt) app_adjust_motor_speed(a, MOTOR_SPEED_STEP_DEG_S);
        else if (find_single_selected_cam(&a->mech) >= 0) app_adjust_cam_lift(a, CAM_LIFT_SCALE);
        else if (gather_selected_wheels(&a->mech, NULL, 0) == 1) app_adjust_wheel_radius(a, WHEEL_RESIZE_STEP);
        else if (find_single_selected_geneva(&a->mech) >= 0) app_adjust_geneva_slots(a, 1);
        else app_adjust_motor_speed(a, MOTOR_SPEED_STEP_DEG_S);
        return;
    case SDLK_MINUS:
    case SDLK_KP_MINUS:
        if (alt) app_adjust_motor_speed(a, -MOTOR_SPEED_STEP_DEG_S);
        else if (find_single_selected_cam(&a->mech) >= 0) app_adjust_cam_lift(a, 1.0 / CAM_LIFT_SCALE);
        else if (gather_selected_wheels(&a->mech, NULL, 0) == 1) app_adjust_wheel_radius(a, 1.0 / WHEEL_RESIZE_STEP);
        else if (find_single_selected_geneva(&a->mech) >= 0) app_adjust_geneva_slots(a, -1);
        else app_adjust_motor_speed(a, -MOTOR_SPEED_STEP_DEG_S);
        return;

    case SDLK_ESCAPE:
        /* Backs out of one thing at a time, innermost first. */
        if (a->help_open) {
            a->help_open = false;
        } else if (a->gallery_open) {
            a->gallery_open = false;
        } else if (a->cam_draw_armed) {
            a->cam_draw_armed = false;
            a->cam_stroke_count = 0;
            app_message(a, STATUS_INFO, "Cam tool cancelled.");
        } else if (a->path_tool != PATH_TOOL_NONE) {
            a->path_tool = PATH_TOOL_NONE;
            a->path_stroke_count = 0;
            app_message(a, STATUS_INFO, "Path tool cancelled.");
        } else {
            clear_selection(&a->mech);
        }
        return;

    default:
        return;
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

    /* Open no bigger than the screen it has to fit on. The old fixed size was
     * taller than a 1280x800 laptop display, which put the plot panel out of
     * reach on a window that could not then be resized. */
    int start_w = WIN_DEFAULT_W, start_h = WIN_DEFAULT_H;
    SDL_Rect usable;
    if (SDL_GetDisplayUsableBounds(0, &usable) == 0) {
        if (start_w > usable.w) start_w = usable.w;
        if (start_h > usable.h) start_h = usable.h;
    }
    if (start_w < WIN_MIN_W) start_w = WIN_MIN_W;
    if (start_h < WIN_MIN_H) start_h = WIN_MIN_H;

    SDL_Window *win = SDL_CreateWindow("Linkage Design",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, start_w, start_h,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!win) {
        fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_SetWindowMinimumSize(win, WIN_MIN_W, WIN_MIN_H);

    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
    if (!ren) {
        fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(win);
        SDL_Quit();
        return 1;
    }

    /* The terminal gets a pointer, not a manual. Every one of these commands
     * has a button with its key on it and a tooltip saying what it wants, and
     * H lists the lot inside the window -- which is where someone who started
     * the app by double-clicking it can actually read them. */
    printf("Linkage Design - a 2D mechanism editor.\n");
    printf("  Press H in the window for every key and what it does.\n");
    printf("  Every command is also a button down the left edge, with its key on it.\n");
    printf("  Cmd/Ctrl+S saves the mechanism, Cmd/Ctrl+O opens one, E exports it to Blender.\n");
    printf("  Shift+E writes a folder of STL parts to print and bolt together.\n");
    printf("  Anything the program has to say now appears on the canvas as well as here.\n");

    App app;
    mechanism_init(&app.mech);
    app.params = solver_default_params();
    app.gravity_set_by_user = false;
    app.undo_count = 0;
    app.redo_count = 0;
    app.state = APP_EDIT;
    app.drag_mode = DRAG_NONE;
    app.drag_start = app.drag_last = app.drag_current = (Vec2){ 0, 0 };
    app.pan_last_screen = (Vec2){ 0, 0 };
    app.drag_pushed_undo = false;
    app.pre_run_positions = NULL;
    app.pre_run_count = 0;
    app.frame_positions = NULL;
    app.frame_angles = NULL;
    app.frame_link_count = 0;
    app.jammed = false;
    app.view_pan = (Vec2){ UI_TOOLBAR_W, 0 };
    app.view_zoom = 1.0;
    app.sim_time = 0.0;
    app.paused = false;
    app.step_once = false;
    app.sim_rate = 1.0;
    app.cam_draw_armed = false;
    app.cam_stroke = NULL;
    app.cam_stroke_count = 0;
    app.cam_stroke_capacity = 0;
    app.path_tool = PATH_TOOL_NONE;
    app.gallery_open = false;
    app.gallery_page = 0;
    app.help_open = false;
    app.path_stroke = NULL;
    app.path_stroke_count = 0;
    app.path_stroke_capacity = 0;
    app.path_ghost_count = 0;
    app.path_ghost_closed = false;
    app.current_path[0] = '\0';
    app.dirty = false;
    app.prompt.kind = PROMPT_NONE;
    app.prompt.text[0] = '\0';
    app.prompt.pending[0] = '\0';
    app.window = win;
    status_init(&app.status);
    /* Everything -- app maths, SDL mouse coordinates, drawing -- stays in
     * window POINTS, and the renderer scales that to whatever pixels the
     * display actually has. That is what makes the window sharp on a Retina
     * screen without a single coordinate conversion anywhere else. */
    SDL_RenderSetLogicalSize(ren, start_w, start_h);
    app.layout = layout_for(start_w, start_h);
    ui_init(&app.toolbar, app.layout.win_h);
    app_update_title(&app);

    Uint32 hover_since = 0;   /* when the cursor settled on the hovered button */
    int mouse_x = 0, mouse_y = 0;
    Uint32 last_ticks = SDL_GetTicks();
    bool running = true;

    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_TEXTINPUT) {
                if (prompt_takes_typing(app.prompt.kind)) {
                    size_t len = strlen(app.prompt.text);
                    snprintf(app.prompt.text + len, sizeof app.prompt.text - len, "%s", ev.text.text);
                }
            } else if (app.prompt.kind != PROMPT_NONE &&
                        (ev.type == SDL_MOUSEBUTTONDOWN || ev.type == SDL_MOUSEBUTTONUP ||
                         ev.type == SDL_MOUSEWHEEL)) {
                /* The canvas is behind a question; clicking it would edit a
                 * mechanism the user cannot currently see the whole of. */
            } else if (ev.type == SDL_QUIT) {
                /* Unsaved work is not thrown away without being asked. */
                if (app.dirty && app.prompt.kind != PROMPT_QUIT) {
                    prompt_open(&app, PROMPT_QUIT, "THIS MECHANISM HAS UNSAVED CHANGES",
                                 "PRESS Y TO QUIT ANYWAY, ESC TO GO BACK AND SAVE IT", NULL);
                } else {
                    running = false;
                }
            } else if (ev.type == SDL_WINDOWEVENT && ev.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                int w = ev.window.data1, h = ev.window.data2;
                SDL_RenderSetLogicalSize(ren, w, h);
                app.layout = layout_for(w, h);
                ui_init(&app.toolbar, app.layout.win_h);
            } else if (ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_MIDDLE) {
                /* Panning works in every mode, including mid-run: it is the
                 * window moving, not the mechanism. */
                app.drag_mode = DRAG_PAN;
                app.pan_last_screen = (Vec2){ (double)ev.button.x, (double)ev.button.y };
            } else if (ev.type == SDL_MOUSEBUTTONUP && ev.button.button == SDL_BUTTON_MIDDLE) {
                if (app.drag_mode == DRAG_PAN) app.drag_mode = DRAG_NONE;
            } else if (ev.type == SDL_MOUSEMOTION) {
                mouse_x = ev.motion.x;
                mouse_y = ev.motion.y;
                if (app.drag_mode == DRAG_PAN) {
                    app_pan_by(&app, ev.motion.x - app.pan_last_screen.x,
                                      ev.motion.y - app.pan_last_screen.y);
                    app.pan_last_screen = (Vec2){ (double)ev.motion.x, (double)ev.motion.y };
                }
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
                        /* Snapshot on the first real movement, not on the press:
                         * a click that moves nothing leaves no history behind. */
                        if (!app.drag_pushed_undo && vec2_len(delta) > 0.0) {
                            push_undo(&app);
                            app.drag_pushed_undo = true;
                        }
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
                /* Through app_try_action, so pressing a greyed-out button says
                 * what it wants rather than doing nothing at all. */
                if (released_on == app.toolbar.pressed) app_try_action(&app, b->action);
                app.toolbar.pressed = -1;
            } else if (ev.type == SDL_KEYDOWN) {
                bool quitting = (app.prompt.kind == PROMPT_QUIT) &&
                                 (ev.key.keysym.sym == SDLK_y || ev.key.keysym.sym == SDLK_RETURN);
                app_handle_key(&app, &ev.key);
                if (quitting) running = false;
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
            } else if (app.help_open && ev.type == SDL_MOUSEBUTTONDOWN &&
                        ev.button.button == SDL_BUTTON_LEFT) {
                /* A click anywhere puts the key list away, rather than landing
                 * on the mechanism hidden behind it. */
                app.help_open = false;
            } else if (app.state == APP_EDIT && ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_LEFT &&
                        app.gallery_open) {
                int tile = gallery_hit(&app, ev.button.x, ev.button.y);
                if (tile >= 0) app_insert_template(&app, tile);
                else app.gallery_open = false;   /* a click outside dismisses it */
            } else if (app.state == APP_EDIT && ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_LEFT &&
                        app.path_tool != PATH_TOOL_NONE) {
                Vec2 p = screen_to_world((Vec2){ (double)ev.button.x, (double)ev.button.y }, app.view_pan, app.view_zoom);
                app.drag_mode = DRAG_DRAW_PATH;
                app.drag_start = p;
                app.path_stroke_count = 0;
                app.path_ghost_count = 0;   /* the previous target, superseded */
                stroke_push(&app.path_stroke, &app.path_stroke_count, &app.path_stroke_capacity, p);
            } else if (app.state == APP_EDIT && ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_LEFT &&
                        app.cam_draw_armed) {
                Vec2 p = screen_to_world((Vec2){ (double)ev.button.x, (double)ev.button.y }, app.view_pan, app.view_zoom);
                app.drag_mode = DRAG_DRAW_CAM;
                app.drag_start = p;
                app.cam_stroke_count = 0;
                stroke_push(&app.cam_stroke, &app.cam_stroke_count, &app.cam_stroke_capacity, p);
            } else if (ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_LEFT) {
                /* Selecting works while running too: you have to be able to
                 * point at the motor you want to speed up. Nothing that MOVES
                 * a part is allowed -- see `editing` below. */
                bool editing = (app.state == APP_EDIT);
                Vec2 p = screen_to_world((Vec2){ (double)ev.button.x, (double)ev.button.y }, app.view_pan, app.view_zoom);
                bool shift = (SDL_GetModState() & KMOD_SHIFT) != 0;
                /* Aim at what is drawn: at high zoom the dot is bigger than a
                 * flat ten-pixel target, and clicking it used to miss. */
                double hit_px = fmax(CONNECTOR_HIT_RADIUS, handle_radius(app.view_zoom) + 2.0);
                int hit_conn = mechanism_pick_connector(&app.mech, p, hit_px / app.view_zoom);
                if (hit_conn >= 0) {
                    if (shift) {
                        app.mech.connectors[hit_conn].selected = !app.mech.connectors[hit_conn].selected;
                        app.drag_mode = DRAG_NONE;
                    } else {
                        if (!app.mech.connectors[hit_conn].selected) {
                            clear_selection(&app.mech);
                            app.mech.connectors[hit_conn].selected = true;
                        }
                        if (editing) {
                            app.drag_mode = DRAG_MOVE_CONNECTORS;
                            app.drag_pushed_undo = false;
                            app.drag_last = p;
                        }
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
                    /* Shift extends the selection whatever was clicked. It
                     * used to do so only for pins and wheels, so the rule you
                     * learn meshing two wheels quietly failed on a bar. */
                    bool *flag = NULL;
                    if (hit_link >= 0)       flag = &app.mech.links[hit_link].selected;
                    else if (hit_cam >= 0)   flag = &app.mech.cams[hit_cam].selected;
                    else if (hit_wheel >= 0) flag = &app.mech.links[hit_wheel].selected;
                    else if (hit_gear >= 0)  flag = &app.mech.gears[hit_gear].selected;
                    else if (hit_gen >= 0)   flag = &app.mech.genevas[hit_gen].selected;
                    else if (hit_slid >= 0)  flag = &app.mech.sliders[hit_slid].selected;

                    if (flag) {
                        if (shift) {
                            *flag = !*flag;
                        } else {
                            clear_selection(&app.mech);
                            *flag = true;
                        }
                        app.drag_mode = DRAG_NONE;
                    } else if (editing) {
                        app.drag_mode = DRAG_PENDING_EMPTY;
                        app.drag_start = p;
                        app.drag_current = p;
                    } else {
                        clear_selection(&app.mech);
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
                    /* A body entirely inside the box is inside the box: drawing
                     * round a mechanism and pressing DELETE should take the
                     * bars with the pins. */
                    for (int li = 0; li < app.mech.link_count; li++) {
                        Link *l = &app.mech.links[li];
                        if (!l->alive) continue;
                        bool all_in = true;
                        for (int k = 0; k < l->connector_count && all_in; k++) {
                            if (!app.mech.connectors[l->connector_ids[k]].selected) all_in = false;
                        }
                        if (all_in) l->selected = true;
                    }
                }
                app.drag_mode = DRAG_NONE;
            }
        }

        Uint32 now = SDL_GetTicks();
        double dt = (double)(now - last_ticks) / 1000.0;
        last_ticks = now;

        if (app.state == APP_RUNNING && !app.jammed && (!app.paused || app.step_once)) {
            if (dt > 0.05) dt = 0.05; /* clamp huge stalls (e.g. window drag) */
            /* Paused-and-stepping advances one fixed frame; otherwise real
             * time, scaled by whatever speed is set. */
            dt = app.step_once ? SIM_STEP_DT : dt * app.sim_rate;
            app.step_once = false;

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
                /* A condition that is still true, not a moment that has
                 * passed: it stays up until the run is stopped or restarted. */
                status_set_sticky(&app.status, STATUS_ERROR,
                                   "JAMMED: A FIXED-LENGTH LINK WOULD HAVE TO CHANGE LENGTH HERE");
                app_message(&app, STATUS_ERROR,
                             "Mechanism jammed: a fixed-length link would have to change length here. "
                             "Press STOP (or R), then adjust the geometry (or press VARY to let a "
                             "link's length vary).");
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
        SDL_Rect canvas_clip = { app.layout.canvas.x, app.layout.canvas.y,
                                  app.layout.canvas.w, app.layout.canvas.h };
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

        /* What the simulation is doing right now. Paused used to look exactly
         * like jammed, which looked exactly like a motor set to zero. */
        if (app.state == APP_RUNNING && !app.jammed) {
            char banner[96];
            if (app.paused) snprintf(banner, sizeof banner, "PAUSED - SPACE RUNS ON, . STEPS");
            else snprintf(banner, sizeof banner, "RUNNING AT %.2fX - < AND > CHANGE IT", app.sim_rate);
            double bw = render_text_width(8.5, banner);
            render_text(ren, (Vec2){ app.layout.canvas.x + app.layout.canvas.w - bw - 12.0, 12.0 },
                        8.5, banner, app.paused ? 245 : 130, app.paused ? 195 : 200,
                        app.paused ? 90 : 150, 255);
        }

        if (app.path_tool != PATH_TOOL_NONE) {
            const char *hint = (app.path_tool == PATH_TOOL_LINKAGE)
                ? "DRAW A PATH FOR A FOUR-BAR LINKAGE TO TRACE   ESC TO CANCEL"
                : "DRAW ANY PATH FOR A CHAIN OF ARMS TO REDRAW   ESC TO CANCEL";
            double w = render_text_width(9.0, hint);
            render_text(ren, (Vec2){ app.layout.canvas.x + (app.layout.canvas.w - w) / 2.0, 16.0 },
                        9.0, hint, 150, 200, 255, 255);
        }
        if (app.cam_draw_armed) {
            const char *hint = "DRAG TO DRAW THE CAM OUTLINE   CLICK FOR A DEFAULT CAM   ESC TO CANCEL";
            double w = render_text_width(9.0, hint);
            render_text(ren, (Vec2){ app.layout.canvas.x + (app.layout.canvas.w - w) / 2.0, 16.0 },
                        9.0, hint, 200, 175, 235, 255);
        }
        /* An empty canvas used to say nothing at all, with every instruction
         * in a terminal the user may never have seen. */
        if (!app.gallery_open && !app.help_open && app.prompt.kind == PROMPT_NONE &&
            !mechanism_has_any_part(&app.mech)) {
            const char *l1 = "CLICK TO PLACE A PIN, OR PRESS N FOR A GALLERY OF MECHANISMS";
            const char *l2 = "PRESS H FOR EVERY KEY";
            double w1 = render_text_width(10.0, l1), w2 = render_text_width(9.0, l2);
            Vec2 mid = { app.layout.canvas.x + app.layout.canvas.w / 2.0,
                          app.layout.canvas.y + app.layout.canvas.h / 2.0 };
            render_text(ren, (Vec2){ mid.x - w1 / 2.0, mid.y - 14.0 }, 10.0, l1, 120, 126, 140, 255);
            render_text(ren, (Vec2){ mid.x - w2 / 2.0, mid.y + 10.0 }, 9.0, l2, 100, 106, 120, 255);
        }

        if (app.gallery_open) {
            UiRect panel = gallery_panel(&app);
            render_rect_filled(ren, panel, 24, 25, 30, 255);
            render_rect_outline(ren, panel, 90, 95, 110, 255);
            const char *title = "PICK A MECHANISM";
            double tw = render_text_width(10.0, title);
            render_text(ren, (Vec2){ panel.x + (panel.w - tw) / 2.0, panel.y + 8.0 }, 10.0, title,
                        205, 210, 220, 255);
            int first = gallery_page_first(&app);
            for (int i = first; i < first + gallery_page_size(&app); i++) {
                UiRect tile = gallery_tile(&app, i);
                bool hot = (gallery_hit(&app, mouse_x, mouse_y) == i);
                render_rect_filled(ren, tile, hot ? 44 : 34, hot ? 46 : 36, hot ? 54 : 42, 255);
                render_rect_outline(ren, tile, hot ? 255 : 66, hot ? 225 : 70, hot ? 70 : 80, 255);
                draw_template_preview(ren, i, tile, hot);
            }
            int pages = gallery_page_count(&app);
            if (pages > 1) {
                char footer[64];
                snprintf(footer, sizeof footer, "PAGE %d OF %d - ARROW KEYS TO TURN",
                          app.gallery_page + 1, pages);
                double fw = render_text_width(8.0, footer);
                render_text(ren, (Vec2){ panel.x + (panel.w - fw) / 2.0,
                                          panel.y + panel.h - GALLERY_FOOTER_H + 4.0 },
                            8.0, footer, 150, 155, 168, 255);
            }
        }

        if (app.help_open) draw_help(ren, &app);

        /* The prompt sits over the canvas: it is a question about the whole
         * mechanism, not about any part of it. */
        if (app.prompt.kind != PROMPT_NONE) {
            UiRect canvas = app.layout.canvas;
            int w = canvas.w - 120; if (w > 640) w = 640; if (w < 240) w = 240;
            UiRect panel = { canvas.x + (canvas.w - w) / 2, canvas.y + canvas.h / 3, w, 104 };
            render_rect_filled(ren, panel, 26, 27, 33, 245);
            render_rect_outline(ren, panel, 120, 130, 155, 255);

            double tw = render_text_width(10.0, app.prompt.title);
            render_text(ren, (Vec2){ panel.x + (panel.w - tw) / 2.0, panel.y + 14.0 },
                        10.0, app.prompt.title, 215, 220, 232, 255);

            if (prompt_takes_typing(app.prompt.kind)) {
                /* The typed name, with a caret blinking on the end of it. */
                UiRect field = { panel.x + 16, panel.y + 40, panel.w - 32, 26 };
                render_rect_filled(ren, field, 16, 17, 21, 255);
                render_rect_outline(ren, field, 80, 86, 100, 255);
                double text_w = render_text_width(10.0, app.prompt.text);
                double x = field.x + 8.0;
                if (text_w > field.w - 16.0) x -= (text_w - (field.w - 16.0));  /* scroll left */
                render_text(ren, (Vec2){ x, field.y + 8.0 }, 10.0, app.prompt.text,
                            235, 238, 245, 255);
                if ((SDL_GetTicks() / 500) % 2 == 0) {
                    double cx = x + text_w + 2.0;
                    render_line(ren, (Vec2){ cx, field.y + 6.0 }, (Vec2){ cx, field.y + 20.0 },
                                235, 238, 245, 255);
                }
            }

            double hw = render_text_width(8.0, app.prompt.hint);
            render_text(ren, (Vec2){ panel.x + (panel.w - hw) / 2.0, panel.y + panel.h - 22.0 },
                        8.0, app.prompt.hint, 150, 156, 170, 255);
        }

        SDL_RenderSetClipRect(ren, NULL);

        render_status(ren, &app.status, app.layout.canvas, SDL_GetTicks());
        render_plot(ren, &app.mech, app.layout.plot);
        render_toolbar(ren, &app.toolbar, app.layout.win_h);
        /* Last of all, so it sits over the canvas and the panels. */
        if (app.toolbar.hover >= 0 && app.toolbar.hover < app.toolbar.count &&
            SDL_GetTicks() - hover_since > TOOLTIP_DELAY_MS) {
            const UiButton *hb = &app.toolbar.buttons[app.toolbar.hover];
            render_tooltip(ren, hb->rect, hb->tip, app.layout.win_w, app.layout.win_h);
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
