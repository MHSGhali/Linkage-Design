#ifndef RENDER_H
#define RENDER_H

#include <SDL2/SDL.h>
#include "vec2.h"
#include "ui.h"
#include "mechanism.h"
#include "status.h"

void render_line(SDL_Renderer *ren, Vec2 a, Vec2 b, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);
void render_circle(SDL_Renderer *ren, Vec2 center, double radius, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);
void render_circle_outline(SDL_Renderer *ren, Vec2 center, double radius, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);
/* A line drawn as dashes -- used for guides that aren't parts, like a
 * slider's rail or a gear's pitch circle. */
void render_dashed_line(SDL_Renderer *ren, Vec2 a, Vec2 b, double dash, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);

void render_rect_filled(SDL_Renderer *ren, UiRect r, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);
void render_rect_outline(SDL_Renderer *ren, UiRect r, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);

/* Draws a ground/anchor symbol (a short horizontal bar plus a few diagonal
 * hatch ticks) below `pos`, sized relative to `size`. */
void render_ground_hatch(SDL_Renderer *ren, Vec2 pos, double size, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);

/* Draws `value` formatted as "%.1f" as a small seven-segment-style numeral
 * (digits + '.' + '-'), reading left to right starting with its top-left
 * corner at `top_left`, rotated by `angle` (radians) around that corner, so
 * it can be laid out parallel to an arbitrary line. Each digit is
 * `digit_height` tall. No font/SDL_ttf dependency needed. */
void render_number(SDL_Renderer *ren, Vec2 top_left, double angle, double digit_height, double value,
                    Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);

/* Width (in the same local units as digit_height) that render_number will
 * occupy for `value` -- use to center/position text before drawing it. */
double render_number_width(double digit_height, double value);

/* Draws `text` as small stroke-drawn glyphs with its top-left corner at
 * `top_left`, each glyph `height` tall, reading left to right. Understands
 * A-Z (lowercase is upcased), space, and '^' (drawn as a caret, used as the
 * modifier mark in hotkey hints). Unknown characters advance without drawing.
 * Like render_number, this needs no font/SDL_ttf dependency. */
void render_text(SDL_Renderer *ren, Vec2 top_left, double height, const char *text,
                  Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);

/* Width render_text will occupy for `text` -- use to right-align or center. */
double render_text_width(double height, const char *text);

/* Draws the whole toolbar strip (background, buttons, labels, hotkey hints
 * and group separators) down the left edge, `strip_height` tall. */
void render_toolbar(SDL_Renderer *ren, const Toolbar *t, int strip_height);

/* A circle drawn as dashes -- a guide, or a state ("this wheel is meshed")
 * that should not rest on colour alone. */
void render_dashed_circle(SDL_Renderer *ren, Vec2 center, double radius,
                           Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);

/* Draws `count` points as one connected run, in a handful of calls rather
 * than one per segment. */
void render_polyline(SDL_Renderer *ren, const Vec2 *pts, int count,
                      Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);

/* Draws the status messages over `canvas`: the sticky condition banner across
 * its top, and the recent messages stacked up from its bottom-left corner,
 * fading with age. */
void render_status(SDL_Renderer *ren, const StatusLog *log, UiRect canvas, unsigned int now_ms);

/* Word-wrapping. RENDER_WRAP_LINE_CHARS is the widest line render_wrap_text
 * will produce; longer words are simply cut at that length. */
#define RENDER_WRAP_LINE_CHARS 96
#define RENDER_WRAP_MAX_LINES 12

/* Breaks `text` into at most `max_lines` lines no wider than `max_width` when
 * drawn at `height`, on word boundaries. Returns the line count and, through
 * `widest_out` (may be NULL), the width of the longest line -- enough to size
 * a panel around it. */
int render_wrap_text(double height, double max_width, const char *text,
                      char lines[][RENDER_WRAP_LINE_CHARS], int max_lines, double *widest_out);

/* Wraps `text` to `max_width` and draws it from `top_left` down, `line_height`
 * apart. Returns the height drawn. */
double render_text_wrapped(SDL_Renderer *ren, Vec2 top_left, double height, double line_height,
                            double max_width, const char *text,
                            Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);

/* A hover tooltip for `anchor` (the button being pointed at), wrapped to fit
 * and kept inside the window. Drawn last, over everything else. */
void render_tooltip(SDL_Renderer *ren, UiRect anchor, const char *text, int win_w, int win_h);

/* The colour a traced connector is drawn in, both as a dot in the canvas and
 * as its curves in the plot, so the two can be matched by eye. Stable per
 * connector id. */
void render_trace_color(int connector_id, Uint8 *r, Uint8 *g, Uint8 *b);

/* Draws a cam's physical surface (the pitch curve inset by the roller
 * radius), rotated by `angle` about `center_screen`. `zoom` converts the
 * cam's world-space dimensions to pixels. */
void render_cam(SDL_Renderer *ren, const Cam *cam, Vec2 center_screen, double angle, double zoom,
                 Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);

/* Draws a Geneva wheel: its rim, the radial slots cut into it, and the locking
 * arcs between them, turned to `angle`. */
void render_geneva_wheel(SDL_Renderer *ren, Vec2 center_screen, double radius_px,
                          int slots, double angle, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);

/* Draws the x(t) and y(t) time-series panel for every traced connector, on a
 * shared auto-scaled axis, filling `panel`. */
void render_plot(SDL_Renderer *ren, const Mechanism *m, UiRect panel);

#endif
