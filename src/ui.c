#include "ui.h"

#include <stddef.h>

/* The toolbar's contents, in order. A `true` group flag starts a new group,
 * drawn with a separator rule above it and a little extra breathing room. */
typedef struct {
    UiAction action;
    const char *label;
    const char *hint;
    const char *tip;        /* hover text: what it does, and what it wants selected */
    bool group_start;
} ButtonSpec;

static const ButtonSpec BUTTON_SPECS[] = {
    { UI_OPEN,    "OPEN",    "^O",
      "Open a mechanism you saved earlier. You will be asked for the file name.", false },
    { UI_SAVE,    "SAVE",    "^S",
      "Save this mechanism so you can come back to it. Unlike EXPORT, which writes an "
      "animation, this writes the machine itself.", false },
    { UI_TEMPLATE,"TEMPLATE","N",
      "Open the gallery of standard mechanisms. Click one to drop it in, running.", false },
    { UI_JOINT,   "JOINT",   "J",
      "Put a pin joint in the middle of the view. Clicking bare canvas does the same.", false },
    { UI_ANCHOR,  "ANCHOR",  "A",
      "Ground the selected pins, or free them again. An anchor cannot move.", false },
    { UI_LINK,    "LINK",    "L",
      "Join the selected pins into one rigid bar. Select 2 or more pins first.", false },
    { UI_MOTOR,   "MOTOR",   "M",
      "Turn the selected body at a steady speed about its anchor. On a gear wheel this "
      "makes it the one that drives. +/- change the speed.", false },
    { UI_SLIDER,  "SLIDER",  "S",
      "With nothing selected, draw a whole crank-slider. With 3 pins selected, the two "
      "farthest apart become the rail and the third slides along it.", true  },
    { UI_GEAR,    "GEAR",    "O",
      "With nothing selected, add ONE wheel on its own. With 2 or more wheels selected, "
      "mesh them together. Click a wheel to select it, shift-click to add another.", false },
    { UI_GENEVA,  "GENEVA",  "W",
      "With nothing selected, draw a whole Geneva: steady rotation in, one step at a "
      "time out. +/- change the slot count.", false },
    { UI_CAM,     "CAM",     "K",
      "Arm the cam tool, then drag on the canvas to draw the cam outline by hand. The "
      "shaft, motor and roller follower are made for you.", true  },
    { UI_LINKAGE, "LINKAGE", "P",
      "Draw a path and get a four-bar linkage that traces it. Five parts and one motor, "
      "but only the curves a four-bar can reach.", false },
    { UI_ARMS,    "ARMS",    "B",
      "Draw a path and get a chain of turning arms that traces it exactly. Any shape at "
      "all, at the cost of many parts.", false },
    { UI_VARY,    "VARY",    "V",
      "Let the selected bar change length instead of holding it fixed. Drawn green.", false },
    { UI_TRACE,   "TRACE",   "T",
      "Record the path of the selected points while it runs, and plot them against time "
      "in the panel below. On a gear, the traced mark rides the rim.", false },
    { UI_DELETE,  "DELETE",  "DEL",
      "Remove whatever is selected. Anything that depended on it goes too.", false },
    { UI_UNDO,    "UNDO",    "^Z",
      "Step back through your edits.", true  },
    { UI_REDO,    "REDO",    "^Y",
      "Step forward again through undone edits.", false },
    { UI_FIT,     "FIT",     "F",
      "Pan and zoom until the whole mechanism is on screen. Use it when something "
      "has wandered off the edge. Works while running.", false },
    { UI_GRAVITY, "GRAVITY", "G",
      "Turn gravity on or off. A mechanism with no motor runs under gravity by default.", true  },
    { UI_CLEAR,   "CLEAR",   "C",
      "Wipe the traced paths and the plot. Works while running.", false },
    { UI_EXPORT,  "EXPORT",  "E",
      "Write the motion out as a Blender script, ready to run and play.", false },
    { UI_RUN,     "RUN",     "R",
      "Start or stop the simulation. Stopping puts every part back where it started.", false },
    { UI_HELP,    "HELP",    "H",
      "List every key and what it does. Everything here has a button as well, and every "
      "button shows its key on the right.", true  },
    { UI_PAUSE,   "PAUSE",   "SPC",
      "Hold the mechanism still where it is, without putting it back. Then . steps one "
      "frame at a time, and < and > slow it down or speed it up.", false },
};
#define BUTTON_SPEC_COUNT ((int)(sizeof BUTTON_SPECS / sizeof BUTTON_SPECS[0]))

void ui_init(Toolbar *t, int strip_height) {
    t->count = 0;
    t->hover = -1;
    t->pressed = -1;

    int n = BUTTON_SPEC_COUNT < UI_ACTION_COUNT ? BUTTON_SPEC_COUNT : UI_ACTION_COUNT;
    int groups = 0;
    for (int i = 1; i < n; i++) if (BUTTON_SPECS[i].group_start) groups++;

    /* Squeeze to fit rather than running off the bottom of the window, where
     * a button can neither be read nor pressed. Spacing goes first, since it
     * costs the least, and only then the buttons themselves -- down to a floor
     * where the label would stop being legible. */
    int button_h = UI_BUTTON_H, gap = UI_BUTTON_GAP, group_gap = UI_GROUP_GAP;
    int needed = UI_TOP_MARGIN + n * (button_h + gap) + groups * group_gap;
    if (strip_height > 0 && needed > strip_height) {
        gap = 2;
        group_gap = 4;
        needed = UI_TOP_MARGIN + n * (button_h + gap) + groups * group_gap;
        if (needed > strip_height) {
            int room = strip_height - UI_TOP_MARGIN - groups * group_gap - n * gap;
            button_h = room / n;
            if (button_h < UI_BUTTON_MIN_H) button_h = UI_BUTTON_MIN_H;
        }
    }

    int y = UI_TOP_MARGIN;
    for (int i = 0; i < n; i++) {
        const ButtonSpec *spec = &BUTTON_SPECS[i];
        if (spec->group_start && i > 0) y += group_gap;

        UiButton *b = &t->buttons[t->count++];
        b->action = spec->action;
        b->label = spec->label;
        b->hint = spec->hint;
        b->tip = spec->tip;
        b->rect = (UiRect){ UI_MARGIN, y, UI_BUTTON_W, button_h };
        b->enabled = true;
        b->active = false;
        b->separator_above = spec->group_start && i > 0;

        y += button_h + gap;
    }
}

int ui_hit_test(const Toolbar *t, int x, int y) {
    for (int i = 0; i < t->count; i++) {
        const UiRect *r = &t->buttons[i].rect;
        if (x >= r->x && x < r->x + r->w && y >= r->y && y < r->y + r->h) return i;
    }
    return -1;
}

bool ui_contains(const Toolbar *t, int x, int y) {
    (void)t;
    (void)y;
    return x >= 0 && x < UI_TOOLBAR_W;
}

static const UiButton *button_for_action(const Toolbar *t, UiAction action) {
    for (int i = 0; i < t->count; i++) {
        if (t->buttons[i].action == action) return &t->buttons[i];
    }
    return NULL;
}

bool ui_action_enabled(const Toolbar *t, UiAction action) {
    const UiButton *b = button_for_action(t, action);
    return b && b->enabled;
}

const char *ui_action_label(const Toolbar *t, UiAction action) {
    const UiButton *b = button_for_action(t, action);
    return b ? b->label : "";
}

const char *ui_action_tip(const Toolbar *t, UiAction action) {
    const UiButton *b = button_for_action(t, action);
    return b ? b->tip : "";
}

void ui_apply_state(Toolbar *t, UiState s) {
    bool have_link = (s.selected_link >= 0);
    for (int i = 0; i < t->count; i++) {
        UiButton *b = &t->buttons[i];
        switch (b->action) {
        case UI_OPEN:
        case UI_SAVE:
            /* The document, not the mechanism: not while it is moving. */
            b->enabled = s.editing;
            b->active = false;
            break;
        case UI_TEMPLATE:
            b->enabled = s.editing;
            b->active = s.gallery_open;
            break;
        case UI_JOINT:
            b->enabled = s.editing;
            b->active = false;
            break;
        case UI_ANCHOR:
            b->enabled = s.editing && s.selected_connector_count > 0;
            b->active = false;
            break;
        case UI_LINK:
            b->enabled = s.editing && s.selected_connector_count >= 2;
            b->active = false;
            break;
        case UI_MOTOR:
            /* Un-driving always works; driving needs exactly one anchor. */
            b->enabled = s.editing && have_link &&
                            (s.selected_link_driven || s.selected_link_can_drive);
            b->active = have_link && s.selected_link_driven;
            break;
        case UI_SLIDER:
            b->enabled = s.editing && s.can_make_slider;
            b->active = false;
            break;
        case UI_GEAR:
            b->enabled = s.editing && s.can_make_gear;
            b->active = false;
            break;
        case UI_GENEVA:
            b->enabled = s.editing && s.can_make_geneva;
            b->active = false;
            break;
        case UI_CAM:
            /* A drawing tool, not an operation on a selection: always offered
             * while editing, and lit while it is armed. */
            b->enabled = s.editing;
            b->active = s.drawing_cam;
            break;
        case UI_LINKAGE:
            /* Both path tools are drawing tools, not operations on a
             * selection, so they need nothing selected. Lit while armed. */
            b->enabled = s.editing;
            b->active = s.drawing_linkage;
            break;
        case UI_ARMS:
            b->enabled = s.editing;
            b->active = s.drawing_arms;
            break;
        case UI_VARY:
            /* A driven link is posed directly, so its shape is always rigid. */
            b->enabled = s.editing && have_link && !s.selected_link_driven;
            b->active = have_link && !s.selected_link_driven && !s.selected_link_rigid;
            break;
        case UI_TRACE:
            b->enabled = s.editing && s.selected_connector_count > 0;
            b->active = s.selected_connector_count > 0 && s.all_selected_traced;
            break;
        case UI_DELETE:
            b->enabled = s.editing && s.has_selection;
            b->active = false;
            break;
        case UI_UNDO:
            b->enabled = s.editing && s.can_undo;
            b->active = false;
            break;
        case UI_REDO:
            b->enabled = s.editing && s.can_redo;
            b->active = false;
            break;
        case UI_EXPORT:
            b->enabled = s.editing;
            b->active = false;
            break;
        case UI_FIT:
            /* A view control, not an edit: always available, running or not. */
            b->enabled = true;
            b->active = false;
            break;
        case UI_GRAVITY:
            /* Works mid-run, like the G key. */
            b->enabled = true;
            b->active = s.gravity_on;
            break;
        case UI_CLEAR:
            b->enabled = true;
            b->active = false;
            break;
        case UI_HELP:
            b->enabled = true;
            b->active = s.help_open;
            break;
        case UI_PAUSE:
            b->enabled = s.running;
            b->active = s.paused;
            b->label = s.paused ? "RESUME" : "PAUSE";
            break;
        case UI_RUN:
            b->enabled = true;
            b->active = s.running;
            /* A jam is not a pause: say so on the one control that is still
             * worth pressing. */
            b->label = s.running ? (s.jammed ? "JAMMED" : "STOP") : "RUN";
            break;
        case UI_NONE:
        case UI_ACTION_COUNT:
            break;
        }
    }
}
