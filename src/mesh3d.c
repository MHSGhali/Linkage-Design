#include "mesh3d.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "xalloc.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Points closer together than this are the same point. Outlines are in
 * millimetres and the finest feature we draw is a fifth of a millimetre, so
 * this is far below anything real and far above double-precision noise. */
#define WELD_EPS 1e-9

/* --- Triangle soup ------------------------------------------------------- */

void mesh3d_init(Mesh3 *m) {
    m->verts = NULL;
    m->tri_count = 0;
    m->capacity = 0;
}

void mesh3d_free(Mesh3 *m) {
    free(m->verts);
    mesh3d_init(m);
}

void mesh3d_add_tri(Mesh3 *m, Vec3 a, Vec3 b, Vec3 c) {
    if (m->tri_count * 3 + 3 > m->capacity) {
        int want = (m->capacity > 0) ? m->capacity * 2 : 256;
        Vec3 *grown = xrealloc(m->verts, (size_t)want * sizeof(Vec3));
        if (!grown) return;   /* out of memory: the closedness check will catch it */
        m->verts = grown;
        m->capacity = want;
    }
    m->verts[m->tri_count * 3 + 0] = a;
    m->verts[m->tri_count * 3 + 1] = b;
    m->verts[m->tri_count * 3 + 2] = c;
    m->tri_count++;
}

double mesh3d_signed_area(const Vec2 *p, int count) {
    double a = 0.0;
    for (int i = 0, j = count - 1; i < count; j = i++) {
        a += p[j].x * p[i].y - p[i].x * p[j].y;
    }
    return a * 0.5;
}

/* --- Ear clipping -------------------------------------------------------- */

static bool same_point(Vec2 a, Vec2 b) {
    return fabs(a.x - b.x) < WELD_EPS && fabs(a.y - b.y) < WELD_EPS;
}

/* Strictly inside; a point sitting exactly on an edge does not block an ear.
 * That matters because bridging a hole leaves two vertices lying ON the
 * bridge, and treating those as blockers would stall the clip. Works whichever
 * way round a, b, c happen to run. */
static bool strictly_inside(Vec2 a, Vec2 b, Vec2 c, Vec2 q) {
    double d1 = vec2_cross(vec2_sub(b, a), vec2_sub(q, a));
    double d2 = vec2_cross(vec2_sub(c, b), vec2_sub(q, b));
    double d3 = vec2_cross(vec2_sub(a, c), vec2_sub(q, c));
    return (d1 > WELD_EPS && d2 > WELD_EPS && d3 > WELD_EPS) ||
            (d1 < -WELD_EPS && d2 < -WELD_EPS && d3 < -WELD_EPS);
}

/* Do the two segments cross each other properly -- not merely touch at a
 * shared endpoint, and not merely graze? */
static bool segments_cross(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    double d1 = vec2_cross(vec2_sub(b, a), vec2_sub(c, a));
    double d2 = vec2_cross(vec2_sub(b, a), vec2_sub(d, a));
    double d3 = vec2_cross(vec2_sub(d, c), vec2_sub(a, c));
    double d4 = vec2_cross(vec2_sub(d, c), vec2_sub(b, c));
    return ((d1 > WELD_EPS && d2 < -WELD_EPS) || (d1 < -WELD_EPS && d2 > WELD_EPS)) &&
            ((d3 > WELD_EPS && d4 < -WELD_EPS) || (d3 < -WELD_EPS && d4 > WELD_EPS));
}

/* Does `q` lie strictly between `a` and `b` on the segment joining them? */
static bool point_on_segment(Vec2 a, Vec2 b, Vec2 q) {
    Vec2 ab = vec2_sub(b, a);
    double len = vec2_len(ab);
    if (len < WELD_EPS) return false;
    if (fabs(vec2_cross(ab, vec2_sub(q, a))) / len > 1e-7) return false;
    double t = vec2_dot(vec2_sub(q, a), ab) / (len * len);
    return t > 1e-9 && t < 1.0 - 1e-9;
}

/* Clips `v` (a simple counter-clockwise polygon, possibly with bridge seams)
 * into triangles, calling back with each. Returns false if it gets stuck,
 * which means the outline was not simple to begin with. */
static bool ear_clip(const Vec2 *v, int n, Mesh3 *out, double z, bool up) {
    if (n < 3) return false;

    int *prev = xmalloc((size_t)n * sizeof(int));
    int *next = xmalloc((size_t)n * sizeof(int));
    bool *gone = xcalloc((size_t)n, sizeof(bool));
    if (!prev || !next || !gone) { free(prev); free(next); free(gone); return false; }

    for (int i = 0; i < n; i++) { prev[i] = (i + n - 1) % n; next[i] = (i + 1) % n; }

    int remaining = n;
    int cur = 0;
    /* Every clip removes one vertex, and finding the next ear can cost a full
     * sweep of what is left, so a quadratic allowance is the honest
     * bound; the slack on top absorbs the sweeps that find nothing. This turns
     * "no ear anywhere" into a clean failure rather than a hang. */
    long budget = 4L * (long)n * (long)n + 64L * n;
    bool ok = true;

    while (remaining > 3) {
        if (budget-- <= 0) { ok = false; break; }
        int i = cur, ip = prev[cur], in = next[cur];
        Vec2 a = v[ip], b = v[i], c = v[in];

        double turn = vec2_cross(vec2_sub(b, a), vec2_sub(c, b));
        /* A vertex sitting exactly on the straight line between its two
         * neighbours is neither convex nor reflex, so it is never an ear --
         * and a polygon can end up as nothing but such vertices. A rack does:
         * once its tooth tips are clipped away, what is left along the root
         * line is a long collinear run, and the clip stalls with a hundred
         * vertices to go. Clipping it as a zero-area ear costs a sliver
         * triangle, which a slicer ignores, and keeps the vertex in the mesh
         * so the cap still matches the wall built from the same loop.
         *
         * A vertex that doubles BACK along the line is a different thing --
         * that is the turn at the end of a bridging seam -- so require the
         * neighbours to lie on opposite sides before treating it this way. */
        bool flat = fabs(turn) <= WELD_EPS &&
                     vec2_dot(vec2_sub(a, b), vec2_sub(c, b)) < 0.0;
        bool ear = flat || turn > WELD_EPS;
        if (ear && !flat) {
            for (int k = next[in]; k != ip; k = next[k]) {
                Vec2 q = v[k];
                if (same_point(q, a) || same_point(q, b) || same_point(q, c)) continue;
                if (strictly_inside(a, b, c, q)) { ear = false; break; }
            }
        }
        if (ear && !flat) {
            /* Asking only about vertices is not enough once bridging seams are
             * in play. A seam is a zero-width channel, so a pair of its edges
             * can run clean across a candidate ear with every endpoint ON the
             * triangle's boundary -- inside nothing, caught by nothing, and
             * blocking everything. Clipping that ear takes the same material
             * away twice, and what is left is wound the wrong way with no ear
             * anywhere in it. So ask about the edges themselves. */
            for (int k = in; ; ) {
                int k2 = next[k];
                if (segments_cross(a, c, v[k], v[k2])) { ear = false; break; }
                Vec2 mid = vec2_scale(vec2_add(v[k], v[k2]), 0.5);
                if (strictly_inside(a, b, c, mid)) { ear = false; break; }
                k = k2;
                if (k == ip) break;
            }
        }

        if (ear) {
            Vec3 A = { a.x, a.y, z }, B = { b.x, b.y, z }, C = { c.x, c.y, z };
            if (up) mesh3d_add_tri(out, A, B, C); else mesh3d_add_tri(out, C, B, A);
            next[ip] = in;
            prev[in] = ip;
            gone[i] = true;
            remaining--;
            cur = ip;
        } else {
            cur = in;
        }
    }

    if (ok) {
        int i = cur;
        while (gone[i]) i = (i + 1) % n;
        Vec2 a = v[prev[i]], b = v[i], c = v[next[i]];
        Vec3 A = { a.x, a.y, z }, B = { b.x, b.y, z }, C = { c.x, c.y, z };
        if (up) mesh3d_add_tri(out, A, B, C); else mesh3d_add_tri(out, C, B, A);
    }

    free(prev);
    free(next);
    free(gone);
    return ok;
}

/* --- Hole bridging -------------------------------------------------------
 *
 * Ear clipping only understands ONE loop, so each hole is cut into the outer
 * boundary along a seam: a pair of coincident edges out to the hole and back.
 * The seam has zero width, so the solid is unchanged, and what is left is a
 * single simple polygon.
 *
 * The seam runs from the hole to a vertex of the ORIGINAL outer boundary --
 * never to a point on another hole's seam or on another hole. That restriction
 * is the whole trick. Landing one seam on another leaves two zero-width
 * channels sharing a point, which is a polygon that still passes an
 * edge-crossing test but no longer has an ear anywhere -- so ear clipping
 * stalls on geometry that looks perfectly fine. Rows of anchors at the same
 * height, which is how baseplates usually come out, hit that case routinely.
 *
 * Seams land on existing vertices rather than splitting edges, so the outer
 * loop the walls are built from is exactly the one the caps are triangulated
 * from. Splitting an edge here instead would leave the cap with a vertex the
 * wall has never heard of, and the solid would not close along that seam.
 */

/* Can a seam run from `m` to the vertex poly[v] without touching the boundary
 * anywhere but at its two ends? `m` is on the hole and poly[v] on the outer
 * boundary, so a seam that touches nothing else stays inside the material, and
 * splicing along it leaves a simple polygon.
 *
 * Running THROUGH a vertex counts as touching: that is the case an
 * edge-crossing test alone misses, and the one that stalls the clip later. */
/* Does the segment a..b cut across this loop? Ends that touch the loop at `m`
 * are the seam's own attachment and do not count. */
static bool seam_hits_loop(Vec2 a, Vec2 b, const Vec2 *loop, int count, Vec2 m) {
    for (int i = 0; i < count; i++) {
        int j = (i + 1) % count;
        if (same_point(loop[i], m) || same_point(loop[j], m)) continue;
        if (segments_cross(a, b, loop[i], loop[j])) return true;
        if (!same_point(loop[i], a) && !same_point(loop[i], b) &&
            point_on_segment(a, b, loop[i])) return true;
    }
    return false;
}

static bool seam_is_clear(const Vec2 *poly, int n, Vec2 m, int v,
                           const Vec2 *hole, int hole_n,
                           const Region2 *r, const int *order, int first_pending,
                           int pending_count) {
    Vec2 cand = poly[v];
    if (same_point(m, cand)) return false;
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        if (i != v && j != v && segments_cross(m, cand, poly[i], poly[j])) return false;
        if (i == v || same_point(poly[i], m) || same_point(poly[i], cand)) continue;
        if (point_on_segment(m, cand, poly[i])) return false;
    }
    /* This hole is not spliced in yet, so check it separately: a seam that cut
     * back across its own hole would join the wrong sides of it. */
    if (seam_hits_loop(m, cand, hole, hole_n, m)) return false;

    /* And so are the holes still queued behind it. A seam laid across empty
     * material today is a seam laid across a HOLE once that hole arrives, and
     * by then it is already part of the boundary and cannot be moved. Two
     * anchors a few millimetres apart on the same edge -- a rail's mounting
     * pin beside the pivot it braces, say -- land exactly on that. */
    for (int k = first_pending; k < pending_count; k++) {
        const Loop2 *later = &r->holes[order[k]];
        if (later->count < 3) continue;
        if (seam_hits_loop(m, cand, later->p, later->count, m)) return false;
    }
    return true;
}

/* Forces `src` into `dst` with the winding `ccw` asks for. */
static void copy_wound(Vec2 *dst, const Vec2 *src, int n, bool ccw) {
    bool is_ccw = mesh3d_signed_area(src, n) > 0.0;
    for (int i = 0; i < n; i++) dst[i] = is_ccw == ccw ? src[i] : src[n - 1 - i];
}

/* Merges the outer loop and every hole into one simple polygon. Returns the
 * allocated array and its length, or NULL. */
static Vec2 *bridge_holes(const Region2 *r, int *count_out) {
    int total = r->outer.count;
    for (int h = 0; h < r->hole_count; h++) total += r->holes[h].count + 4;

    Vec2 *poly = xmalloc((size_t)total * sizeof(Vec2));
    Vec2 *scratch = xmalloc((size_t)total * sizeof(Vec2));
    /* outer_edge[i] marks the edge from poly[i] to poly[i+1] as part of the
     * ORIGINAL boundary, i.e. somewhere a seam may still land. */
    bool *outer_edge = xmalloc((size_t)total * sizeof(bool));
    if (!poly || !scratch || !outer_edge) {
        free(poly); free(scratch); free(outer_edge);
        return NULL;
    }
    bool *scratch_edge = xmalloc((size_t)total * sizeof(bool));
    if (!scratch_edge) { free(poly); free(scratch); free(outer_edge); return NULL; }

    int n = r->outer.count;
    copy_wound(poly, r->outer.p, n, true);
    for (int i = 0; i < n; i++) outer_edge[i] = true;

    /* Rightmost hole first, so the seams fan out in a consistent order. */
    int *order = xmalloc((size_t)(r->hole_count > 0 ? r->hole_count : 1) * sizeof(int));
    if (!order) { free(poly); free(scratch); free(outer_edge); free(scratch_edge); return NULL; }
    for (int h = 0; h < r->hole_count; h++) order[h] = h;
    for (int i = 1; i < r->hole_count; i++) {
        int key = order[i];
        double kx = -HUGE_VAL;
        for (int k = 0; k < r->holes[key].count; k++) {
            if (r->holes[key].p[k].x > kx) kx = r->holes[key].p[k].x;
        }
        int j = i - 1;
        while (j >= 0) {
            double jx = -HUGE_VAL;
            for (int k = 0; k < r->holes[order[j]].count; k++) {
                if (r->holes[order[j]].p[k].x > jx) jx = r->holes[order[j]].p[k].x;
            }
            if (jx >= kx) break;
            order[j + 1] = order[j];
            j--;
        }
        order[j + 1] = key;
    }

    bool ok = true;
    for (int hi = 0; hi < r->hole_count && ok; hi++) {
        const Loop2 *hole = &r->holes[order[hi]];
        if (hole->count < 3) continue;

        /* A hole runs the opposite way round to the boundary, so that walking
         * in through the seam and out again keeps the merged loop simple. */
        Vec2 *hw = xmalloc((size_t)hole->count * sizeof(Vec2));
        if (!hw) { ok = false; break; }
        copy_wound(hw, hole->p, hole->count, false);

        /* Try every point of the hole against every vertex still on the
         * original boundary, and take the shortest seam that is genuinely
         * clear. Leaving from the hole's rightmost vertex alone is the
         * textbook choice, but it has no answer at all when the only clear
         * seam leaves from somewhere else on the hole. */
        int best_vertex = -1, best_start = -1;
        double best_len = 0.0;
        for (int e = 0; e < n; e++) {
            /* A vertex is landable if it still starts an original outer edge. */
            if (!outer_edge[e]) continue;
            for (int s = 0; s < hole->count; s++) {
                double len = vec2_dist(hw[s], poly[e]);
                if (best_vertex >= 0 && len >= best_len) continue;
                if (!seam_is_clear(poly, n, hw[s], e, hw, hole->count,
                                    r, order, hi + 1, r->hole_count)) continue;
                best_vertex = e; best_start = s; best_len = len;
            }
        }
        if (best_vertex < 0) { free(hw); ok = false; break; }

        int p = best_vertex;
        int w = 0;
        for (int k = 0; k <= p; k++) {
            scratch[w] = poly[k];
            scratch_edge[w] = outer_edge[k];
            w++;
        }
        scratch_edge[p] = false;   /* poly[p] now leads into the hole: a seam */
        for (int k = 0; k < hole->count; k++) {
            scratch[w] = hw[(best_start + k) % hole->count];
            scratch_edge[w] = false;
            w++;
        }
        scratch[w] = hw[best_start]; scratch_edge[w] = false; w++;  /* close the hole */
        /* Back out along the seam. This copy of poly[p] carries on round the
         * original boundary, but it is no longer landable: a second seam
         * arriving at the same vertex would make two zero-width channels meet
         * at a point, which is the same earless polygon as landing on a seam. */
        scratch[w] = poly[p];        scratch_edge[w] = false; w++;
        for (int k = p + 1; k < n; k++) {
            scratch[w] = poly[k];
            scratch_edge[w] = outer_edge[k];
            w++;
        }

        memcpy(poly, scratch, (size_t)w * sizeof(Vec2));
        memcpy(outer_edge, scratch_edge, (size_t)w * sizeof(bool));
        n = w;
        free(hw);
    }

    free(order);
    free(scratch);
    free(scratch_edge);
    free(outer_edge);
    if (!ok) { free(poly); return NULL; }
    *count_out = n;
    return poly;
}

bool mesh3d_triangulate(Mesh3 *m, const Region2 *r, double z, bool up) {
    if (!r || r->outer.count < 3) return false;
    int n = 0;
    Vec2 *poly = bridge_holes(r, &n);
    if (!poly) return false;
    bool ok = ear_clip(poly, n, m, z, up);
    free(poly);
    return ok;
}

/* --- Extrusion ----------------------------------------------------------- */

/* One loop's worth of wall. The loop must already be wound the way the region
 * wants it -- anticlockwise for the outer boundary, clockwise for a hole --
 * and that alone decides which way the wall faces: a hole is just a boundary
 * walked the other way round, so the same two triangles serve both. Flipping
 * the pattern as well would turn the hole's wall back inside out, leaving the
 * bore's edges running the same way as the caps' instead of against them. */
static void wall_from_loop(Mesh3 *m, const Vec2 *p, int n, double z0, double z1) {
    for (int i = 0; i < n; i++) {
        Vec2 a = p[i], b = p[(i + 1) % n];
        Vec3 a0 = { a.x, a.y, z0 }, a1 = { a.x, a.y, z1 };
        Vec3 b0 = { b.x, b.y, z0 }, b1 = { b.x, b.y, z1 };
        mesh3d_add_tri(m, a0, b0, b1);
        mesh3d_add_tri(m, a0, b1, a1);
    }
}

bool mesh3d_extrude(Mesh3 *m, const Region2 *r, double z0, double z1) {
    if (!r || r->outer.count < 3 || fabs(z1 - z0) < WELD_EPS) return false;
    if (z1 < z0) { double t = z0; z0 = z1; z1 = t; }

    int before = m->tri_count;
    /* The floor faces down and the ceiling up, so one of them is wound the
     * other way about. */
    if (!mesh3d_triangulate(m, r, z0, false)) { m->tri_count = before; return false; }
    if (!mesh3d_triangulate(m, r, z1, true)) { m->tri_count = before; return false; }

    Vec2 *buf = xmalloc((size_t)r->outer.count * sizeof(Vec2));
    if (!buf) { m->tri_count = before; return false; }
    copy_wound(buf, r->outer.p, r->outer.count, true);
    wall_from_loop(m, buf, r->outer.count, z0, z1);
    free(buf);

    for (int h = 0; h < r->hole_count; h++) {
        const Loop2 *hole = &r->holes[h];
        if (hole->count < 3) continue;
        Vec2 *hb = xmalloc((size_t)hole->count * sizeof(Vec2));
        if (!hb) { m->tri_count = before; return false; }
        copy_wound(hb, hole->p, hole->count, false);
        wall_from_loop(m, hb, hole->count, z0, z1);
        free(hb);
    }
    return true;
}

/* --- Closedness ---------------------------------------------------------- */

typedef struct { int a, b; } Edge;

static int cmp_vec3(const void *pa, const void *pb) {
    const Vec3 *a = *(const Vec3 *const *)pa, *b = *(const Vec3 *const *)pb;
    if (a->x < b->x - WELD_EPS) return -1;
    if (a->x > b->x + WELD_EPS) return 1;
    if (a->y < b->y - WELD_EPS) return -1;
    if (a->y > b->y + WELD_EPS) return 1;
    if (a->z < b->z - WELD_EPS) return -1;
    if (a->z > b->z + WELD_EPS) return 1;
    return 0;
}

static int cmp_edge(const void *pa, const void *pb) {
    const Edge *a = pa, *b = pb;
    if (a->a != b->a) return (a->a < b->a) ? -1 : 1;
    if (a->b != b->b) return (a->b < b->b) ? -1 : 1;
    return 0;
}

static int edge_count(const Edge *edges, int n, int a, int b) {
    int lo = 0, hi = n;
    Edge key = { a, b };
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (cmp_edge(&edges[mid], &key) < 0) lo = mid + 1; else hi = mid;
    }
    int count = 0;
    while (lo + count < n && cmp_edge(&edges[lo + count], &key) == 0) count++;
    return count;
}

static bool edges_all_matched(const Mesh3 *m, bool strict) {
    if (!m || m->tri_count <= 0) return false;
    int vn = m->tri_count * 3;

    /* Weld coincident corners so that two triangles meeting along an edge
     * agree about which edge it is. */
    const Vec3 **sorted = xmalloc((size_t)vn * sizeof(Vec3 *));
    int *id = xmalloc((size_t)vn * sizeof(int));
    if (!sorted || !id) { free(sorted); free(id); return false; }
    for (int i = 0; i < vn; i++) sorted[i] = &m->verts[i];
    qsort(sorted, (size_t)vn, sizeof(Vec3 *), cmp_vec3);

    int groups = 0;
    for (int i = 0; i < vn; i++) {
        if (i > 0 && cmp_vec3(&sorted[i - 1], &sorted[i]) != 0) groups++;
        id[sorted[i] - m->verts] = groups;
    }

    Edge *edges = xmalloc((size_t)(vn) * sizeof(Edge));
    if (!edges) { free(sorted); free(id); return false; }
    int en = 0;
    for (int t = 0; t < m->tri_count; t++) {
        int v0 = id[t * 3], v1 = id[t * 3 + 1], v2 = id[t * 3 + 2];
        if (v0 == v1 || v1 == v2 || v2 == v0) {   /* a degenerate sliver has no edges to match */
            free(sorted); free(id); free(edges);
            return false;
        }
        edges[en++] = (Edge){ v0, v1 };
        edges[en++] = (Edge){ v1, v2 };
        edges[en++] = (Edge){ v2, v0 };
    }
    qsort(edges, (size_t)en, sizeof(Edge), cmp_edge);

    bool closed = true;
    for (int i = 0; i < en && closed; i++) {
        int forward = edge_count(edges, en, edges[i].a, edges[i].b);
        int back = edge_count(edges, en, edges[i].b, edges[i].a);
        if (forward != back) closed = false;
        else if (strict && forward != 1) closed = false;
    }

    free(sorted);
    free(id);
    free(edges);
    return closed;
}

bool mesh3d_is_closed(const Mesh3 *m) {
    return edges_all_matched(m, true);
}

bool mesh3d_shells_are_closed(const Mesh3 *m) {
    return edges_all_matched(m, false);
}

/* --- Binary STL ---------------------------------------------------------- */

static void put_float(FILE *f, double v) {
    float x = (float)v;
    fwrite(&x, sizeof x, 1, f);
}

bool mesh3d_write_stl(const Mesh3 *m, const char *name, const char *path) {
    if (!m || m->tri_count <= 0) return false;
    FILE *f = fopen(path, "wb");
    if (!f) return false;

    char header[80];
    memset(header, 0, sizeof header);
    snprintf(header, sizeof header, "Linkage Design %s", name ? name : "part");
    fwrite(header, 1, sizeof header, f);

    uint32_t count = (uint32_t)m->tri_count;
    fwrite(&count, sizeof count, 1, f);

    for (int t = 0; t < m->tri_count; t++) {
        Vec3 a = m->verts[t * 3], b = m->verts[t * 3 + 1], c = m->verts[t * 3 + 2];
        double ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
        double vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
        double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        double len = sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 1e-18) { nx /= len; ny /= len; nz /= len; } else { nx = ny = nz = 0.0; }
        put_float(f, nx); put_float(f, ny); put_float(f, nz);
        put_float(f, a.x); put_float(f, a.y); put_float(f, a.z);
        put_float(f, b.x); put_float(f, b.y); put_float(f, b.z);
        put_float(f, c.x); put_float(f, c.y); put_float(f, c.z);
        uint16_t attr = 0;
        fwrite(&attr, sizeof attr, 1, f);
    }

    bool ok = (ferror(f) == 0);
    if (fclose(f) != 0) ok = false;
    return ok;
}

/* --- Outline builders ---------------------------------------------------- */

int mesh3d_circle_capacity(int segments) {
    return (segments < 8) ? 8 : segments;
}

int mesh3d_circle(Vec2 centre, double radius, int segments, Vec2 *out, int cap) {
    int n = mesh3d_circle_capacity(segments);
    if (!out || cap < n || !(radius > WELD_EPS)) return 0;
    for (int i = 0; i < n; i++) {
        double a = 2.0 * M_PI * (double)i / (double)n;
        out[i] = (Vec2){ centre.x + radius * cos(a), centre.y + radius * sin(a) };
    }
    return n;
}

int mesh3d_stadium_capacity(int cap_segments) {
    if (cap_segments < 3) cap_segments = 3;
    return 2 * (cap_segments + 1);
}

int mesh3d_stadium(Vec2 a, Vec2 b, double radius, int cap_segments, Vec2 *out, int cap) {
    if (cap_segments < 3) cap_segments = 3;
    int n = mesh3d_stadium_capacity(cap_segments);
    if (!out || cap < n || !(radius > WELD_EPS)) return 0;

    Vec2 d = vec2_sub(b, a);
    double len = vec2_len(d);
    if (len < WELD_EPS) return mesh3d_circle(a, radius, n, out, cap);
    double theta = atan2(d.y, d.x);

    int w = 0;
    /* Round the far end, then the near one; sweeping anticlockwise from the
     * right-hand side of the axis keeps the whole loop anticlockwise. */
    for (int i = 0; i <= cap_segments; i++) {
        double t = theta - M_PI / 2.0 + M_PI * (double)i / (double)cap_segments;
        out[w++] = (Vec2){ b.x + radius * cos(t), b.y + radius * sin(t) };
    }
    for (int i = 0; i <= cap_segments; i++) {
        double t = theta + M_PI / 2.0 + M_PI * (double)i / (double)cap_segments;
        out[w++] = (Vec2){ a.x + radius * cos(t), a.y + radius * sin(t) };
    }
    return w;
}

int mesh3d_hull_offset_capacity(int point_count, int corner_segments) {
    if (corner_segments < 2) corner_segments = 2;
    int k = (point_count < 1) ? 1 : point_count;
    int as_polygon = k * (corner_segments + 2);
    /* One or two pins collapse the hull to a disc or a capsule, and a capsule
     * spends more points than a two-cornered polygon would -- so the buffer
     * has to be big enough for the widest of the three shapes, not just the
     * general one. Sizing it for the polygon alone silently refused every
     * two-pin link and every baseplate with two anchors. */
    int as_capsule = 4 * corner_segments + 2;
    int as_disc = mesh3d_circle_capacity(corner_segments * 4);
    int most = as_polygon;
    if (as_capsule > most) most = as_capsule;
    if (as_disc > most) most = as_disc;
    return most + 8;
}

/* Andrew's monotone chain. Writes the hull anticlockwise into `hull`, which
 * must have room for 2n + 1 points. Collinear points are dropped, so a bar's
 * three pins give a hull of two, and coincident pins give a hull of one. */
static int convex_hull(const Vec2 *pts, int n, Vec2 *hull) {
    if (n < 1) return 0;
    if (n == 1) { hull[0] = pts[0]; return 1; }

    int *idx = xmalloc((size_t)n * sizeof(int));
    if (!idx) return 0;
    for (int i = 0; i < n; i++) idx[i] = i;
    for (int i = 1; i < n; i++) {
        int key = idx[i];
        int j = i - 1;
        while (j >= 0 && (pts[idx[j]].x > pts[key].x ||
                          (pts[idx[j]].x == pts[key].x && pts[idx[j]].y > pts[key].y))) {
            idx[j + 1] = idx[j]; j--;
        }
        idx[j + 1] = key;
    }

    int k = 0;
    for (int i = 0; i < n; i++) {
        Vec2 p = pts[idx[i]];
        while (k >= 2 && vec2_cross(vec2_sub(hull[k - 1], hull[k - 2]),
                                     vec2_sub(p, hull[k - 2])) <= WELD_EPS) k--;
        hull[k++] = p;
    }
    for (int i = n - 2, floor_k = k + 1; i >= 0; i--) {
        Vec2 p = pts[idx[i]];
        while (k >= floor_k && vec2_cross(vec2_sub(hull[k - 1], hull[k - 2]),
                                           vec2_sub(p, hull[k - 2])) <= WELD_EPS) k--;
        hull[k++] = p;
    }
    free(idx);

    int h = (k > 1) ? k - 1 : k;   /* the closing point repeats the first */
    /* One or two distinct points leave a degenerate "hull"; say how many are
     * really there so the caller can round it into a disc or a capsule. */
    if (h >= 2 && same_point(hull[0], hull[1])) h = 1;
    return h;
}

int mesh3d_hull_offset(const Vec2 *pts, int point_count, double radius,
                        int corner_segments, Vec2 *out, int cap) {
    if (!pts || point_count < 1 || !out || !(radius > WELD_EPS)) return 0;
    if (corner_segments < 2) corner_segments = 2;
    if (cap < mesh3d_hull_offset_capacity(point_count, corner_segments)) return 0;

    Vec2 *hull = xmalloc((size_t)(2 * point_count + 2) * sizeof(Vec2));
    if (!hull) return 0;
    int h = convex_hull(pts, point_count, hull);

    int n = 0;
    if (h == 1) {
        n = mesh3d_circle(hull[0], radius, corner_segments * 4, out, cap);
    } else if (h == 2) {
        n = mesh3d_stadium(hull[0], hull[1], radius, corner_segments * 2, out, cap);
    } else {
        /* Push each edge out along its outward normal and join consecutive
         * offsets with an arc: the boss at every pin, with flats between. */
        for (int i = 0; i < h; i++) {
            Vec2 a = hull[i], b = hull[(i + 1) % h];
            Vec2 d = vec2_sub(b, a);
            double len = vec2_len(d);
            if (len < WELD_EPS) continue;
            Vec2 nrm = { d.y / len, -d.x / len };   /* outward, for an anticlockwise hull */
            out[n++] = vec2_add(a, vec2_scale(nrm, radius));
            out[n++] = vec2_add(b, vec2_scale(nrm, radius));

            Vec2 e = vec2_sub(hull[(i + 2) % h], b);
            double elen = vec2_len(e);
            if (elen < WELD_EPS) continue;
            Vec2 nrm2 = { e.y / elen, -e.x / elen };
            double a0 = atan2(nrm.y, nrm.x), a1 = atan2(nrm2.y, nrm2.x);
            double sweep = a1 - a0;
            while (sweep <= 0.0) sweep += 2.0 * M_PI;
            while (sweep > 2.0 * M_PI) sweep -= 2.0 * M_PI;
            for (int k = 1; k < corner_segments; k++) {
                double t = a0 + sweep * (double)k / (double)corner_segments;
                out[n++] = (Vec2){ b.x + radius * cos(t), b.y + radius * sin(t) };
            }
        }
    }
    free(hull);
    return n;
}
