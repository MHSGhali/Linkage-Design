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
