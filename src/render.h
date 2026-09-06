#ifndef RENDER_H
#define RENDER_H

#include <SDL2/SDL.h>
#include "vec2.h"

void render_line(SDL_Renderer *ren, Vec2 a, Vec2 b, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);
void render_circle(SDL_Renderer *ren, Vec2 center, double radius, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha);

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

#endif
