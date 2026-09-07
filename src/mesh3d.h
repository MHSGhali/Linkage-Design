#ifndef MESH3D_H
#define MESH3D_H

#include <stdbool.h>
#include <stddef.h>
#include "vec2.h"

/* Turning flat outlines into printable solids.
 *
 * Everything the export writes is a PRISM: a closed 2D region -- one outer
 * boundary and any number of holes -- swept to a thickness. A gear, a link
 * plate, a cam, a washer and a baseplate differ only in the outline they are
 * swept from, so there is one pipeline here and no special cases downstream.
 *
 * The output is a triangle soup, because that is what STL is. It is a soup
 * with a rule, though: every solid this builds must be CLOSED (every edge
 * shared by exactly two triangles wound oppositely). An unclosed mesh is not
 * a solid, and a slicer will either refuse it or silently print something
 * else -- so mesh3d_is_closed() is checked before anything is written.
 *
 * Pure geometry: no SDL, no Blender, and every function is testable headlessly.
 * Units are millimetres throughout, matching 1 world unit = 1 mm.
 */

typedef struct { double x, y, z; } Vec3;

/* A closed loop of points; the closing edge from the last back to the first
 * is implied, so do not repeat the first point. */
typedef struct {
    Vec2 *p;
    int count;
} Loop2;

/* One outer boundary with holes punched in it. Winding is NOT the caller's
 * problem: the outer loop is forced counter-clockwise and every hole
 * clockwise before anything is triangulated. */
typedef struct {
    Loop2 outer;
    const Loop2 *holes;
    int hole_count;
} Region2;

/* A triangle soup: `verts` holds 3 * tri_count points, three per triangle,
 * wound counter-clockwise seen from outside. */
typedef struct {
    Vec3 *verts;
    int tri_count;
    int capacity;
} Mesh3;

void mesh3d_init(Mesh3 *m);
void mesh3d_free(Mesh3 *m);
void mesh3d_add_tri(Mesh3 *m, Vec3 a, Vec3 b, Vec3 c);

/* Sweeps `r` from z0 to z1 and appends the resulting closed solid: a floor,
 * a ceiling and a wall around the outer boundary and around every hole.
 * False if the region could not be triangulated (a self-crossing outline, a
 * hole that pokes outside its boundary, fewer than three points). */
bool mesh3d_extrude(Mesh3 *m, const Region2 *r, double z0, double z1);

/* Triangulates the flat region on its own, appending triangles at height `z`
 * wound counter-clockwise when `up` (i.e. seen from +z). Exposed mainly
 * because mesh3d_extrude is built on it and the tests want it directly. */
bool mesh3d_triangulate(Mesh3 *m, const Region2 *r, double z, bool up);

/* Whether the soup is ONE closed surface: every directed edge is matched by
 * exactly one edge running the other way. This is the printability check --
 * a mesh that fails it is not a solid. An empty mesh is not closed. */
bool mesh3d_is_closed(const Mesh3 *m);

/* Whether the soup is a closed surface, allowing SEVERAL of them: every
 * directed edge is matched by as many running the other way. Use this for an
 * assembly, where separate solids touch face to face -- coincident faces make
 * an edge appear four times rather than two, which is not a hole and must not
 * be read as one. Every mesh that passes mesh3d_is_closed passes this too. */
bool mesh3d_shells_are_closed(const Mesh3 *m);

/* Writes a binary STL. `name` goes in the 80-byte header, for anyone who
 * opens the file in a text editor. False if the file could not be written. */
bool mesh3d_write_stl(const Mesh3 *m, const char *name, const char *path);

/* --- Outline builders ----------------------------------------------------
 *
 * Each writes a closed counter-clockwise loop into the caller's buffer and
 * returns the number of points, or 0 if the buffer is too small or the shape
 * is degenerate. The matching _capacity call says how much room to leave. */

int mesh3d_circle_capacity(int segments);
int mesh3d_circle(Vec2 centre, double radius, int segments, Vec2 *out, int cap);

/* A capsule: the set of points within `radius` of the segment a..b. Also the
 * shape of a two-pin link and of a slot a pin runs in. */
int mesh3d_stadium_capacity(int cap_segments);
int mesh3d_stadium(Vec2 a, Vec2 b, double radius, int cap_segments, Vec2 *out, int cap);

/* The convex hull of `pts` grown outwards by `radius`, with an arc at every
 * corner -- the outline of a link plate with a round boss at each pin. Copes
 * with one point (a disc) and with collinear points (a capsule). */
int mesh3d_hull_offset_capacity(int point_count, int corner_segments);
int mesh3d_hull_offset(const Vec2 *pts, int point_count, double radius,
                        int corner_segments, Vec2 *out, int cap);

/* Signed area; positive means counter-clockwise. Useful to callers deciding
 * whether an outline they built by hand needs reversing. */
double mesh3d_signed_area(const Vec2 *p, int count);

#endif
