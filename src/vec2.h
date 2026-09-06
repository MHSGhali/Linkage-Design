#ifndef VEC2_H
#define VEC2_H

#include <math.h>

typedef struct {
    double x, y;
} Vec2;

static inline Vec2 vec2_add(Vec2 a, Vec2 b) { return (Vec2){ a.x + b.x, a.y + b.y }; }
static inline Vec2 vec2_sub(Vec2 a, Vec2 b) { return (Vec2){ a.x - b.x, a.y - b.y }; }
static inline Vec2 vec2_scale(Vec2 a, double s) { return (Vec2){ a.x * s, a.y * s }; }
static inline double vec2_dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
static inline double vec2_len(Vec2 a) { return sqrt(vec2_dot(a, a)); }
static inline double vec2_dist2(Vec2 a, Vec2 b) { Vec2 d = vec2_sub(a, b); return vec2_dot(d, d); }
static inline double vec2_dist(Vec2 a, Vec2 b) { return sqrt(vec2_dist2(a, b)); }
static inline Vec2 vec2_perp(Vec2 a) { return (Vec2){ -a.y, a.x }; }
static inline Vec2 vec2_from_angle(double angle) { return (Vec2){ cos(angle), sin(angle) }; }
static inline Vec2 vec2_rotate(Vec2 v, double angle) {
    double c = cos(angle), s = sin(angle);
    return (Vec2){ v.x * c - v.y * s, v.x * s + v.y * c };
}

#endif
