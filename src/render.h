#ifndef RENDER_H
#define RENDER_H

#include <SDL2/SDL.h>
#include "vec2.h"
#include "ui.h"

void render_line(SDL_Renderer *ren, Vec2 a, Vec2 b, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);
void render_circle(SDL_Renderer *ren, Vec2 center, double radius, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);

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

#endif
