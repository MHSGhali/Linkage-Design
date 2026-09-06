#include "ui.h"

#include <stddef.h>

/* The toolbar's contents, in order. A `true` group flag starts a new group,
 * drawn with a separator rule above it and a little extra breathing room. */
typedef struct {
    UiAction action;
    const char *label;
    const char *hint;
    bool group_start;
} ButtonSpec;

static const ButtonSpec BUTTON_SPECS[] = {
    { UI_JOINT,   "JOINT",   "J",   false },
    { UI_ANCHOR,  "ANCHOR",  "A",   false },
    { UI_LINK,    "LINK",    "L",   false },
    { UI_MOTOR,   "MOTOR",   "M",   false },
    { UI_CAM,     "CAM",     "K",   false },
    { UI_LINKAGE, "LINKAGE", "P",   false },
    { UI_ARMS,    "ARMS",    "B",   false },
    { UI_VARY,    "VARY",    "V",   false },
    { UI_TRACE,   "TRACE",   "T",   false },
    { UI_DELETE,  "DELETE",  "DEL", false },
    { UI_UNDO,    "UNDO",    "^Z",  true  },
    { UI_REDO,    "REDO",    "^Y",  false },
    { UI_GRAVITY, "GRAVITY", "G",   true  },
    { UI_CLEAR,   "CLEAR",   "C",   false },
    { UI_EXPORT,  "EXPORT",  "E",   false },
    { UI_RUN,     "RUN",     "R",   false },
};
#define BUTTON_SPEC_COUNT ((int)(sizeof BUTTON_SPECS / sizeof BUTTON_SPECS[0]))

void ui_init(Toolbar *t) {
    t->count = 0;
    t->hover = -1;
    t->pressed = -1;

    int y = UI_TOP_MARGIN;
    for (int i = 0; i < BUTTON_SPEC_COUNT && i < UI_ACTION_COUNT; i++) {
        const ButtonSpec *spec = &BUTTON_SPECS[i];
        if (spec->group_start && i > 0) y += UI_GROUP_GAP;

        UiButton *b = &t->buttons[t->count++];
        b->action = spec->action;
        b->label = spec->label;
        b->hint = spec->hint;
        b->rect = (UiRect){ UI_MARGIN, y, UI_BUTTON_W, UI_BUTTON_H };
        b->enabled = true;
        b->active = false;
        b->separator_above = spec->group_start && i > 0;

        y += UI_BUTTON_H + UI_BUTTON_GAP;
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

void ui_apply_state(Toolbar *t, UiState s) {
    bool have_link = (s.selected_link >= 0);
    for (int i = 0; i < t->count; i++) {
        UiButton *b = &t->buttons[i];
        switch (b->action) {
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
        case UI_GRAVITY:
            /* Works mid-run, like the G key. */
            b->enabled = true;
            b->active = s.gravity_on;
            break;
        case UI_CLEAR:
            b->enabled = true;
            b->active = false;
            break;
        case UI_RUN:
            b->enabled = true;
            b->active = s.running;
            b->label = s.running ? "STOP" : "RUN";
            break;
        case UI_NONE:
        case UI_ACTION_COUNT:
            break;
        }
    }
}
