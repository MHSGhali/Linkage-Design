#include "render.h"

#include <math.h>
#include <stdio.h>

void render_line(SDL_Renderer *ren, Vec2 a, Vec2 b, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha) {
    SDL_SetRenderDrawColor(ren, red, green, blue, alpha);
    SDL_RenderDrawLine(ren, (int)lround(a.x), (int)lround(a.y), (int)lround(b.x), (int)lround(b.y));
}

void render_circle(SDL_Renderer *ren, Vec2 center, double radius, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha) {
    SDL_SetRenderDrawColor(ren, red, green, blue, alpha);
    int ir = (int)ceil(radius);
    double r2 = radius * radius;
    for (int dy = -ir; dy <= ir; dy++) {
        for (int dx = -ir; dx <= ir; dx++) {
            if ((double)(dx * dx + dy * dy) <= r2) {
                SDL_RenderDrawPoint(ren, (int)lround(center.x) + dx, (int)lround(center.y) + dy);
            }
        }
    }
}

void render_rect_filled(SDL_Renderer *ren, UiRect r, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha) {
    SDL_SetRenderDrawColor(ren, red, green, blue, alpha);
    SDL_Rect sr = { r.x, r.y, r.w, r.h };
    SDL_RenderFillRect(ren, &sr);
}

void render_rect_outline(SDL_Renderer *ren, UiRect r, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha) {
    SDL_SetRenderDrawColor(ren, red, green, blue, alpha);
    SDL_Rect sr = { r.x, r.y, r.w, r.h };
    SDL_RenderDrawRect(ren, &sr);
}

void render_ground_hatch(SDL_Renderer *ren, Vec2 pos, double size, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha) {
    SDL_SetRenderDrawColor(ren, red, green, blue, alpha);
    double y0 = pos.y + size;
    SDL_RenderDrawLine(ren, (int)lround(pos.x - size), (int)lround(y0), (int)lround(pos.x + size), (int)lround(y0));
    const int n = 4;
    for (int i = 0; i < n; i++) {
        double x0 = pos.x - size + (2.0 * size) * i / (double)(n - 1);
        SDL_RenderDrawLine(ren, (int)lround(x0), (int)lround(y0),
                            (int)lround(x0 - size * 0.4), (int)lround(y0 + size * 0.6));
    }
}

/* Seven-segment layout, normalized to a 1-wide, 2-tall cell:
 *   (0,0)--a--(1,0)
 *     |         |
 *     f         b
 *     |         |
 *   (0,1)--g--(1,1)
 *     |         |
 *     e         c
 *     |         |
 *   (0,2)--d--(1,2)
 */
typedef struct { double x0, y0, x1, y1; } Segment;
static const Segment SEVEN_SEGMENTS[7] = {
    { 0, 0, 1, 0 }, /* a */
    { 1, 0, 1, 1 }, /* b */
    { 1, 1, 1, 2 }, /* c */
    { 0, 2, 1, 2 }, /* d */
    { 0, 1, 0, 2 }, /* e */
    { 0, 0, 0, 1 }, /* f */
    { 0, 1, 1, 1 }, /* g */
};
#define SEG_A (1 << 0)
#define SEG_B (1 << 1)
#define SEG_C (1 << 2)
#define SEG_D (1 << 3)
#define SEG_E (1 << 4)
#define SEG_F (1 << 5)
#define SEG_G (1 << 6)
static const int DIGIT_MASK[10] = {
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,         /* 0 */
    SEG_B | SEG_C,                                         /* 1 */
    SEG_A | SEG_B | SEG_G | SEG_E | SEG_D,                 /* 2 */
    SEG_A | SEG_B | SEG_G | SEG_C | SEG_D,                 /* 3 */
    SEG_F | SEG_G | SEG_B | SEG_C,                         /* 4 */
    SEG_A | SEG_F | SEG_G | SEG_C | SEG_D,                 /* 5 */
    SEG_A | SEG_F | SEG_G | SEG_E | SEG_C | SEG_D,         /* 6 */
    SEG_A | SEG_B | SEG_C,                                 /* 7 */
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G, /* 8 */
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,         /* 9 */
};

/* Maps a point in the digit's own local space (x: along the reading
 * direction, y: downward) to world space, given the text's rotated origin. */
static Vec2 local_to_world(Vec2 origin, double angle, double local_x, double local_y) {
    return vec2_add(origin, vec2_rotate((Vec2){ local_x, local_y }, angle));
}

static void render_digit(SDL_Renderer *ren, Vec2 origin, double angle, double local_x, double height, int digit,
                          Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha) {
    double w = height * 0.6;
    double half_h = height * 0.5;
    int mask = DIGIT_MASK[digit];
    for (int i = 0; i < 7; i++) {
        if (!(mask & (1 << i))) continue;
        Vec2 p0 = local_to_world(origin, angle, local_x + SEVEN_SEGMENTS[i].x0 * w, SEVEN_SEGMENTS[i].y0 * half_h);
        Vec2 p1 = local_to_world(origin, angle, local_x + SEVEN_SEGMENTS[i].x1 * w, SEVEN_SEGMENTS[i].y1 * half_h);
        render_line(ren, p0, p1, red, green, blue, alpha);
    }
}

double render_number_width(double digit_height, double value) {
    char buf[32];
    snprintf(buf, sizeof buf, "%.1f", value);
    double w = digit_height * 0.6;
    double spacing = w * 1.3;
    double total = 0.0;
    for (const char *p = buf; *p; p++) {
        total += (*p == '.') ? w * 0.5 : spacing;
    }
    return total;
}

void render_number(SDL_Renderer *ren, Vec2 top_left, double angle, double digit_height, double value,
                    Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha) {
    char buf[32];
    snprintf(buf, sizeof buf, "%.1f", value);

    double w = digit_height * 0.6;
    double spacing = w * 1.3;
    double cursor_x = 0.0;

    for (const char *p = buf; *p; p++) {
        if (*p == '.') {
            Vec2 dot = local_to_world(top_left, angle, cursor_x + w * 0.15, digit_height);
            render_circle(ren, dot, digit_height * 0.06, red, green, blue, alpha);
            cursor_x += w * 0.5;
        } else if (*p == '-') {
            Vec2 p0 = local_to_world(top_left, angle, cursor_x, digit_height * 0.5);
            Vec2 p1 = local_to_world(top_left, angle, cursor_x + w * 0.6, digit_height * 0.5);
            render_line(ren, p0, p1, red, green, blue, alpha);
            cursor_x += spacing;
        } else if (*p >= '0' && *p <= '9') {
            render_digit(ren, top_left, angle, cursor_x, digit_height, *p - '0', red, green, blue, alpha);
            cursor_x += spacing;
        }
    }
}

/* ---------------------------------------------------------------------------
 * Stroke alphabet for UI labels.
 *
 * Each glyph is a run of polylines over the same normalized 1-wide by 2-tall
 * cell the seven-segment digits use (x right, y down). PEN_UP lifts the pen
 * between strokes. Uppercase only, which is all the toolbar needs -- and it
 * keeps the app free of any font dependency.
 * ------------------------------------------------------------------------ */
#define PEN_UP (-1.0f)
/* Lifts the pen between strokes. Occupies a whole x,y pair so the table stays
 * readable two floats at a time. */
#define BREAK_STROKE PEN_UP, PEN_UP

static const float GLYPH_A[] = { 0,2, 0.5f,0, 1,2, BREAK_STROKE, 0.18f,1.3f, 0.82f,1.3f };
static const float GLYPH_B[] = { 0,0, 0,2, BREAK_STROKE, 0,0, 0.72f,0, 1,0.28f, 0.72f,0.95f, 0,0.95f,
                                 BREAK_STROKE, 0,0.95f, 0.78f,0.95f, 1,1.3f, 0.75f,2, 0,2 };
static const float GLYPH_C[] = { 1,0.35f, 0.72f,0, 0.28f,0, 0,0.4f, 0,1.6f, 0.28f,2, 0.72f,2, 1,1.65f };
static const float GLYPH_D[] = { 0,0, 0,2, BREAK_STROKE, 0,0, 0.6f,0, 1,0.5f, 1,1.5f, 0.6f,2, 0,2 };
static const float GLYPH_E[] = { 1,0, 0,0, 0,2, 1,2, BREAK_STROKE, 0,1, 0.72f,1 };
static const float GLYPH_F[] = { 1,0, 0,0, 0,2, BREAK_STROKE, 0,1, 0.72f,1 };
static const float GLYPH_G[] = { 1,0.35f, 0.72f,0, 0.28f,0, 0,0.4f, 0,1.6f, 0.28f,2, 0.72f,2,
                                 1,1.62f, 1,1.1f, 0.52f,1.1f };
static const float GLYPH_H[] = { 0,0, 0,2, BREAK_STROKE, 1,0, 1,2, BREAK_STROKE, 0,1, 1,1 };
static const float GLYPH_I[] = { 0.18f,0, 0.82f,0, BREAK_STROKE, 0.5f,0, 0.5f,2, BREAK_STROKE, 0.18f,2, 0.82f,2 };
static const float GLYPH_J[] = { 0.85f,0, 0.85f,1.55f, 0.6f,2, 0.25f,2, 0,1.6f };
static const float GLYPH_K[] = { 0,0, 0,2, BREAK_STROKE, 1,0, 0.05f,1.05f, BREAK_STROKE, 0.35f,0.72f, 1,2 };
static const float GLYPH_L[] = { 0,0, 0,2, 1,2 };
static const float GLYPH_M[] = { 0,2, 0,0, 0.5f,0.95f, 1,0, 1,2 };
static const float GLYPH_N[] = { 0,2, 0,0, 1,2, 1,0 };
static const float GLYPH_O[] = { 0.3f,0, 0.7f,0, 1,0.4f, 1,1.6f, 0.7f,2, 0.3f,2, 0,1.6f, 0,0.4f, 0.3f,0 };
static const float GLYPH_P[] = { 0,2, 0,0, 0.72f,0, 1,0.3f, 0.72f,1.05f, 0,1.05f };
static const float GLYPH_Q[] = { 0.3f,0, 0.7f,0, 1,0.4f, 1,1.6f, 0.7f,2, 0.3f,2, 0,1.6f, 0,0.4f, 0.3f,0,
                                 BREAK_STROKE, 0.62f,1.5f, 1.02f,2.05f };
static const float GLYPH_R[] = { 0,2, 0,0, 0.72f,0, 1,0.3f, 0.72f,1.05f, 0,1.05f,
                                 BREAK_STROKE, 0.5f,1.05f, 1,2 };
static const float GLYPH_S[] = { 1,0.32f, 0.7f,0, 0.3f,0, 0,0.32f, 0,0.68f, 0.3f,1, 0.7f,1,
                                 1,1.32f, 1,1.68f, 0.7f,2, 0.3f,2, 0,1.68f };
static const float GLYPH_T[] = { 0,0, 1,0, BREAK_STROKE, 0.5f,0, 0.5f,2 };
static const float GLYPH_U[] = { 0,0, 0,1.6f, 0.3f,2, 0.7f,2, 1,1.6f, 1,0 };
static const float GLYPH_V[] = { 0,0, 0.5f,2, 1,0 };
static const float GLYPH_W[] = { 0,0, 0.25f,2, 0.5f,0.85f, 0.75f,2, 1,0 };
static const float GLYPH_X[] = { 0,0, 1,2, BREAK_STROKE, 1,0, 0,2 };
static const float GLYPH_Y[] = { 0,0, 0.5f,1, 1,0, BREAK_STROKE, 0.5f,1, 0.5f,2 };
static const float GLYPH_Z[] = { 0,0, 1,0, 0,2, 1,2 };
/* Modifier mark for hotkey hints ("^Z"): a caret, not a letter. */
static const float GLYPH_CARET[] = { 0.15f,0.55f, 0.5f,0.05f, 0.85f,0.55f };

typedef struct { const float *pts; int n; } Glyph;
#define GLYPH_ENTRY(g) { g, (int)(sizeof(g) / sizeof((g)[0])) }

static const Glyph LETTER_GLYPHS[26] = {
    GLYPH_ENTRY(GLYPH_A), GLYPH_ENTRY(GLYPH_B), GLYPH_ENTRY(GLYPH_C), GLYPH_ENTRY(GLYPH_D),
    GLYPH_ENTRY(GLYPH_E), GLYPH_ENTRY(GLYPH_F), GLYPH_ENTRY(GLYPH_G), GLYPH_ENTRY(GLYPH_H),
    GLYPH_ENTRY(GLYPH_I), GLYPH_ENTRY(GLYPH_J), GLYPH_ENTRY(GLYPH_K), GLYPH_ENTRY(GLYPH_L),
    GLYPH_ENTRY(GLYPH_M), GLYPH_ENTRY(GLYPH_N), GLYPH_ENTRY(GLYPH_O), GLYPH_ENTRY(GLYPH_P),
    GLYPH_ENTRY(GLYPH_Q), GLYPH_ENTRY(GLYPH_R), GLYPH_ENTRY(GLYPH_S), GLYPH_ENTRY(GLYPH_T),
    GLYPH_ENTRY(GLYPH_U), GLYPH_ENTRY(GLYPH_V), GLYPH_ENTRY(GLYPH_W), GLYPH_ENTRY(GLYPH_X),
    GLYPH_ENTRY(GLYPH_Y), GLYPH_ENTRY(GLYPH_Z),
};
static const Glyph CARET_GLYPH = GLYPH_ENTRY(GLYPH_CARET);

static const Glyph *glyph_for(char c) {
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    if (c >= 'A' && c <= 'Z') return &LETTER_GLYPHS[c - 'A'];
    if (c == '^') return &CARET_GLYPH;
    return NULL; /* space and anything unsupported: advance without drawing */
}

double render_text_width(double height, const char *text) {
    double w = height * 0.55;
    double spacing = w * 1.35;
    double total = 0.0;
    for (const char *p = text; *p; p++) total += (*p == ' ') ? spacing * 0.85 : spacing;
    if (total > 0.0) total -= (spacing - w); /* trim the trailing gap */
    return total;
}

void render_text(SDL_Renderer *ren, Vec2 top_left, double height, const char *text,
                  Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha) {
    double w = height * 0.55;
    double spacing = w * 1.35;
    double half_h = height * 0.5;
    double cursor_x = 0.0;

    for (const char *p = text; *p; p++) {
        const Glyph *g = glyph_for(*p);
        if (g) {
            bool pen_down = false;
            Vec2 prev = { 0, 0 };
            for (int i = 0; i + 1 < g->n; i += 2) {
                if (g->pts[i] == PEN_UP) { pen_down = false; continue; }
                Vec2 pnt = {
                    top_left.x + cursor_x + (double)g->pts[i] * w,
                    top_left.y + (double)g->pts[i + 1] * half_h,
                };
                if (pen_down) render_line(ren, prev, pnt, red, green, blue, alpha);
                prev = pnt;
                pen_down = true;
            }
        }
        cursor_x += (*p == ' ') ? spacing * 0.85 : spacing;
    }
}

/* ---------------------------------------------------------------------------
 * Toolbar
 * ------------------------------------------------------------------------ */
#define LABEL_HEIGHT 9.0
#define HINT_HEIGHT 7.0
#define TEXT_PAD 9.0

void render_toolbar(SDL_Renderer *ren, const Toolbar *t, int strip_height) {
    render_rect_filled(ren, (UiRect){ 0, 0, UI_TOOLBAR_W, strip_height }, 30, 30, 34, 255);
    render_line(ren, (Vec2){ UI_TOOLBAR_W - 0.5, 0 }, (Vec2){ UI_TOOLBAR_W - 0.5, strip_height },
                55, 57, 64, 255);

    for (int i = 0; i < t->count; i++) {
        const UiButton *b = &t->buttons[i];
        const UiRect *r = &b->rect;

        if (b->separator_above) {
            double sy = r->y - UI_GROUP_GAP * 0.5 - UI_BUTTON_GAP * 0.5;
            render_line(ren, (Vec2){ r->x + 8, sy }, (Vec2){ r->x + r->w - 8, sy }, 60, 62, 70, 255);
        }

        Uint8 bg_r, bg_g, bg_b;
        if (!b->enabled) {
            bg_r = 34; bg_g = 35; bg_b = 39;
        } else if (t->pressed == i) {
            bg_r = 78; bg_g = 82; bg_b = 92;
        } else if (t->hover == i) {
            bg_r = 62; bg_g = 65; bg_b = 72;
        } else {
            bg_r = 45; bg_g = 47; bg_b = 52;
        }
        render_rect_filled(ren, *r, bg_r, bg_g, bg_b, 255);

        /* Toggled-on buttons borrow the canvas's selection yellow. */
        if (b->active && b->enabled) render_rect_outline(ren, *r, 255, 225, 70, 255);
        else if (b->enabled) render_rect_outline(ren, *r, 70, 73, 82, 255);
        else render_rect_outline(ren, *r, 48, 49, 54, 255);

        Uint8 lr, lg, lb, hr, hg, hb;
        if (!b->enabled) {
            lr = 92; lg = 95; lb = 102;
            hr = 70; hg = 72; hb = 78;
        } else if (b->active) {
            lr = 255; lg = 225; lb = 70;
            hr = 200; hg = 180; hb = 70;
        } else {
            lr = 205; lg = 210; lb = 220;
            hr = 130; hg = 136; hb = 148;
        }

        Vec2 label_at = { r->x + TEXT_PAD, r->y + (r->h - LABEL_HEIGHT) / 2.0 };
        render_text(ren, label_at, LABEL_HEIGHT, b->label, lr, lg, lb, 255);

        if (b->hint && b->hint[0]) {
            double hw = render_text_width(HINT_HEIGHT, b->hint);
            Vec2 hint_at = { r->x + r->w - TEXT_PAD - hw, r->y + (r->h - HINT_HEIGHT) / 2.0 };
            render_text(ren, hint_at, HINT_HEIGHT, b->hint, hr, hg, hb, 255);
        }
    }
}

/* ---------------------------------------------------------------------------
 * Cams
 * ------------------------------------------------------------------------ */
#define CAM_OUTLINE_SAMPLES 240

void render_cam(SDL_Renderer *ren, const Cam *cam, Vec2 center_screen, double angle, double zoom,
                 Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha) {
    static Vec2 pts[CAM_OUTLINE_SAMPLES];
    cam_sample_surface(cam, pts, CAM_OUTLINE_SAMPLES);

    Vec2 prev = { 0, 0 };
    for (int i = 0; i <= CAM_OUTLINE_SAMPLES; i++) {
        Vec2 local = pts[i % CAM_OUTLINE_SAMPLES];
        Vec2 rotated = vec2_rotate(local, angle);
        Vec2 p = { center_screen.x + rotated.x * zoom, center_screen.y + rotated.y * zoom };
        if (i > 0) render_line(ren, prev, p, red, green, blue, alpha);
        prev = p;
    }
}

/* ---------------------------------------------------------------------------
 * Time-series plot
 * ------------------------------------------------------------------------ */
#define PLOT_PAD_LEFT 52
#define PLOT_PAD_RIGHT 26
#define PLOT_PAD_TOP 14
#define PLOT_PAD_BOTTOM 20
#define PLOT_LABEL_HEIGHT 7.0

static const Uint8 TRACE_PALETTE[][3] = {
    {  80, 200, 220 },
    { 120, 215, 140 },
    { 240, 170,  80 },
    { 225, 130, 205 },
    { 200, 210,  90 },
    { 120, 165, 245 },
};
#define TRACE_PALETTE_COUNT ((int)(sizeof TRACE_PALETTE / sizeof TRACE_PALETTE[0]))

void render_trace_color(int connector_id, Uint8 *r, Uint8 *g, Uint8 *b) {
    int i = connector_id % TRACE_PALETTE_COUNT;
    if (i < 0) i += TRACE_PALETTE_COUNT;
    *r = TRACE_PALETTE[i][0];
    *g = TRACE_PALETTE[i][1];
    *b = TRACE_PALETTE[i][2];
}

static bool connector_is_plottable(const Connector *c) {
    return c->alive && c->traced && c->path_count >= 2;
}

void render_plot(SDL_Renderer *ren, const Mechanism *m, UiRect panel) {
    render_rect_filled(ren, panel, 26, 26, 30, 255);
    render_line(ren, (Vec2){ panel.x, panel.y + 0.5 }, (Vec2){ panel.x + panel.w, panel.y + 0.5 },
                55, 57, 64, 255);

    /* Shared ranges across every plotted series -- one set of axes, so x and
     * y curves are directly comparable. */
    double t_min = 0.0, t_max = 0.0, v_min = 0.0, v_max = 0.0;
    bool any = false;
    for (int i = 0; i < m->connector_count; i++) {
        const Connector *c = &m->connectors[i];
        if (!connector_is_plottable(c)) continue;
        for (int k = 0; k < c->path_count; k++) {
            double t = c->path_time[k];
            double x = c->path[k].x, y = c->path[k].y;
            if (!any) {
                t_min = t_max = t;
                v_min = fmin(x, y);
                v_max = fmax(x, y);
                any = true;
            } else {
                t_min = fmin(t_min, t); t_max = fmax(t_max, t);
                v_min = fmin(v_min, fmin(x, y));
                v_max = fmax(v_max, fmax(x, y));
            }
        }
    }

    if (!any) {
        const char *hint = "TRACE A CONNECTOR TO PLOT ITS POSITION AGAINST TIME";
        double w = render_text_width(8.0, hint);
        Vec2 at = { panel.x + (panel.w - w) / 2.0, panel.y + panel.h / 2.0 - 4.0 };
        render_text(ren, at, 8.0, hint, 96, 100, 110, 255);
        return;
    }

    double plot_x = panel.x + PLOT_PAD_LEFT;
    double plot_y = panel.y + PLOT_PAD_TOP;
    double plot_w = panel.w - PLOT_PAD_LEFT - PLOT_PAD_RIGHT;
    double plot_h = panel.h - PLOT_PAD_TOP - PLOT_PAD_BOTTOM;
    if (plot_w < 10.0 || plot_h < 10.0) return;

    double t_span = t_max - t_min;
    if (t_span < 1e-9) t_span = 1.0;
    double v_span = v_max - v_min;
    if (v_span < 1e-9) v_span = 1.0;
    /* A little headroom so curves never sit exactly on the frame. */
    v_min -= v_span * 0.05;
    v_max += v_span * 0.05;
    v_span = v_max - v_min;

    /* Frame and gridlines. */
    render_rect_outline(ren, (UiRect){ (int)plot_x, (int)plot_y, (int)plot_w, (int)plot_h },
                        58, 60, 68, 255);
    for (int g = 1; g < 4; g++) {
        double gy = plot_y + plot_h * (double)g / 4.0;
        render_line(ren, (Vec2){ plot_x, gy }, (Vec2){ plot_x + plot_w, gy }, 40, 42, 48, 255);
    }

    /* Value axis labels at top, middle and bottom. */
    for (int g = 0; g <= 2; g++) {
        double frac = (double)g / 2.0;
        double value = v_max - frac * v_span;
        double gy = plot_y + plot_h * frac;
        double w = render_number_width(PLOT_LABEL_HEIGHT, value);
        render_number(ren, (Vec2){ plot_x - 8.0 - w, gy - PLOT_LABEL_HEIGHT / 2.0 }, 0.0,
                      PLOT_LABEL_HEIGHT, value, 130, 135, 145, 255);
    }

    /* Time axis: 0 at the left, the newest sample at the right. */
    render_number(ren, (Vec2){ plot_x, plot_y + plot_h + 5.0 }, 0.0, PLOT_LABEL_HEIGHT,
                  t_min, 130, 135, 145, 255);
    double tw = render_number_width(PLOT_LABEL_HEIGHT, t_max);
    render_number(ren, (Vec2){ plot_x + plot_w - tw, plot_y + plot_h + 5.0 }, 0.0, PLOT_LABEL_HEIGHT,
                  t_max, 130, 135, 145, 255);
    render_text(ren, (Vec2){ plot_x + plot_w / 2.0 - 12.0, plot_y + plot_h + 5.0 },
                PLOT_LABEL_HEIGHT, "TIME S", 110, 115, 125, 255);

    for (int i = 0; i < m->connector_count; i++) {
        const Connector *c = &m->connectors[i];
        if (!connector_is_plottable(c)) continue;

        Uint8 cr, cg, cb;
        render_trace_color(i, &cr, &cg, &cb);
        /* x at full strength, y in a dimmer shade of the same hue: one glance
         * ties both curves to the same dot in the canvas. */
        Uint8 dr = (Uint8)(cr * 0.55), dg = (Uint8)(cg * 0.55), db = (Uint8)(cb * 0.55);

        Vec2 prev_x = { 0, 0 }, prev_y = { 0, 0 };
        for (int k = 0; k < c->path_count; k++) {
            double px = plot_x + plot_w * (c->path_time[k] - t_min) / t_span;
            double py_x = plot_y + plot_h * (v_max - c->path[k].x) / v_span;
            double py_y = plot_y + plot_h * (v_max - c->path[k].y) / v_span;
            Vec2 cur_x = { px, py_x }, cur_y = { px, py_y };
            if (k > 0) {
                render_line(ren, prev_x, cur_x, cr, cg, cb, 255);
                render_line(ren, prev_y, cur_y, dr, dg, db, 255);
            }
            prev_x = cur_x;
            prev_y = cur_y;
        }

        /* Label each curve where it ends, instead of a separate legend box. */
        render_text(ren, (Vec2){ prev_x.x + 4.0, prev_x.y - 3.0 }, PLOT_LABEL_HEIGHT, "X", cr, cg, cb, 255);
        render_text(ren, (Vec2){ prev_y.x + 4.0, prev_y.y - 3.0 }, PLOT_LABEL_HEIGHT, "Y", dr, dg, db, 255);
    }
}
