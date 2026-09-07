#ifndef UI_H
#define UI_H

#include <stdbool.h>

/* Toolbar model: layout, hit-testing and per-button enable/active rules.
 * Deliberately free of any SDL dependency so the rules can be unit-tested
 * headlessly; drawing lives in render.c (render_toolbar). */

#define UI_TOOLBAR_W 140
#define UI_BUTTON_W 120
#define UI_BUTTON_H 30
#define UI_BUTTON_MIN_H 18      /* how far buttons squeeze to fit a short window */
#define UI_BUTTON_GAP 4
#define UI_GROUP_GAP 10
#define UI_MARGIN 10
#define UI_TOP_MARGIN 12

typedef enum {
    UI_NONE = 0,
    UI_OPEN,
    UI_SAVE,
    UI_TEMPLATE,
    UI_JOINT,
    UI_ANCHOR,
    UI_LINK,
    UI_MOTOR,
    UI_SLIDER,
    UI_GEAR,
    UI_GENEVA,
    UI_CAM,
    UI_LINKAGE,
    UI_ARMS,
    UI_VARY,
    UI_TRACE,
    UI_DELETE,
    UI_UNDO,
    UI_REDO,
    UI_FIT,
    UI_GRAVITY,
    UI_CLEAR,
    UI_EXPORT,
    UI_RUN,
    UI_PAUSE,
    UI_HELP,
    UI_ACTION_COUNT
} UiAction;

typedef struct { int x, y, w, h; } UiRect;

typedef struct {
    UiAction action;
    const char *label;      /* e.g. "ANCHOR" */
    const char *hint;       /* hotkey reminder, e.g. "A", "^Z" ('^' draws a caret) */
    const char *tip;        /* what it does and what it needs, shown on hover */
    UiRect rect;
    bool enabled;           /* preconditions met right now */
    bool active;            /* toggled on: gravity, motor, variable, traced, running */
    bool separator_above;   /* draw a group rule above this button */
} UiButton;

typedef struct {
    UiButton buttons[UI_ACTION_COUNT];
    int count;
    int hover;   /* button index under the cursor, or -1 */
    int pressed; /* button index the mouse went down on, or -1 */
} Toolbar;

/* Everything ui_apply_state needs to know about the app, so ui.c never has
 * to reach into the Mechanism itself. */
typedef struct {
    bool editing;                 /* app is in edit mode (not running) */
    int  selected_connector_count;
    int  selected_link;           /* the single selected link's id, else -1 */
    bool selected_link_driven;
    bool selected_link_rigid;
    bool selected_link_can_drive; /* exactly one of its connectors is an anchor */
    bool all_selected_traced;     /* every selected connector is already traced */
    bool drawing_cam;             /* the cam drawing tool is armed */
    bool drawing_linkage;         /* the four-bar path tool is armed */
    bool drawing_arms;            /* the arm-chain path tool is armed */
    bool gallery_open;            /* the template gallery is showing */
    bool can_make_slider;         /* three connectors selected */
    bool can_make_gear;           /* two centres, or a pinion and a rack bar */
    bool can_make_geneva;         /* a motor's centre and a free wheel's centre */
    bool has_selection;           /* any connector or link selected */
    bool can_undo, can_redo;
    bool gravity_on;              /* gravity is in effect (incl. the motorless default) */
    bool running;
    bool jammed;                  /* the run stopped dead on a rigid link */
    bool paused;                  /* running, but holding still to be looked at */
    bool help_open;               /* the key list is showing */
} UiState;

/* Builds the button layout to fit a strip `strip_height` tall (the window's
 * height). Call at startup and again whenever the window is resized: with a
 * fixed layout the buttons below the fold were simply unreachable. */
void ui_init(Toolbar *t, int strip_height);

/* Index of the button containing (x, y), or -1. Ignores `enabled` -- the
 * caller decides what to do about a disabled button. */
int ui_hit_test(const Toolbar *t, int x, int y);

/* Whether (x, y) is anywhere in the toolbar strip (including its gaps), i.e.
 * whether the canvas should ignore this event. */
bool ui_contains(const Toolbar *t, int x, int y);

/* Whether `action`'s button is enabled right now, and its label and hover
 * text. The hotkeys go through these so a key can never disagree with the
 * button beside it -- and so a refused key can explain itself in the same
 * words the button's tooltip uses. Unknown actions read as disabled. */
bool ui_action_enabled(const Toolbar *t, UiAction action);
const char *ui_action_label(const Toolbar *t, UiAction action);
const char *ui_action_tip(const Toolbar *t, UiAction action);

/* Refreshes every button's enabled/active flags (and the Run button's label)
 * from the current app state. Call once per frame before drawing. */
void ui_apply_state(Toolbar *t, UiState s);

#endif
