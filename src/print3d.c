#include "print3d.h"

#include "gearing.h"
#include "mesh3d.h"

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include "xalloc.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* How round a hole or a boss is drawn. A pin hole is only a few millimetres
 * across, so this is already finer than a printer can resolve. */
#define HOLE_SEG 32
#define BOSS_SEG 12
/* Involute points per flank for a printed gear. The canvas uses fewer. */
#define EXPORT_FLANK_SAMPLES 8
/* How finely a cam's surface is written. Matches the Blender export. */
#define CAM_SEG 360

/* A press fit is this much UNDER the nominal pin, so the two grip. */
#define PRESS_INTERFERENCE 0.05
/* The baseplate is stiffer than the parts standing on it. */
#define BASE_THICKNESS_FACTOR 1.5
/* How far the baseplate reaches past the outermost anchor. */
#define BASE_MARGIN 8.0
/* How far a pin stands proud of the top layer, for its cap to grip. */
#define PIN_CAP_ENGAGEMENT 1.6
/* How tall a pin's head is. */
#define PIN_HEAD_HEIGHT 1.6
/* In the exploded view, how far apart consecutive levels are pulled, as a
 * multiple of the plate thickness. Far enough to see between them. */
#define EXPLODE_SPREAD 7.0

PrintParams print_default_params(void) {
    PrintParams p;
    p.module = MECHANISM_DEFAULT_GEAR_MODULE;
    p.pin_diameter = PRINT_DEFAULT_PIN_DIAMETER;
    p.thickness = PRINT_DEFAULT_THICKNESS;
    p.clearance = PRINT_DEFAULT_CLEARANCE;
    p.layer_gap = PRINT_DEFAULT_LAYER_GAP;
    p.wall = PRINT_DEFAULT_WALL;
    p.backlash = PRINT_DEFAULT_BACKLASH;
    p.pressure_angle = GEARING_PRESSURE_ANGLE;
    p.m3_hardware = false;
    p.baseplate = true;
    p.assembly = true;
    return p;
}

/* --- Options ------------------------------------------------------------- */

static double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

bool print_parse_options(PrintParams *p, const char *opts, char *err, size_t err_size) {
    if (err && err_size) err[0] = '\0';
    if (!opts) return true;
    PrintParams w = *p;

    const char *s = opts;
    while (*s) {
        while (*s == ' ' || *s == '\t' || *s == ',') s++;
        if (!*s) break;

        char key[32];
        int k = 0;
        while (*s && *s != '=' && *s != ' ' && *s != '\t' && k < (int)sizeof key - 1) key[k++] = *s++;
        key[k] = '\0';
        if (*s != '=') {
            if (err) snprintf(err, err_size, "'%s' needs to look like key=value.", key);
            return false;
        }
        s++;

        char val[32];
        int v = 0;
        while (*s && *s != ' ' && *s != '\t' && *s != ',' && v < (int)sizeof val - 1) val[v++] = *s++;
        val[v] = '\0';
        if (val[0] == '\0') {
            if (err) snprintf(err, err_size, "'%s' was given no value.", key);
            return false;
        }

        if (strcmp(key, "fit") == 0) {
            if (strcmp(val, "m3") == 0) w.m3_hardware = true;
            else if (strcmp(val, "pin") == 0) w.m3_hardware = false;
            else { if (err) snprintf(err, err_size, "fit= takes 'pin' or 'm3', not '%s'.", val); return false; }
            continue;
        }
        if (strcmp(key, "asm") == 0) {
            if (strcmp(val, "on") == 0) w.assembly = true;
            else if (strcmp(val, "off") == 0) w.assembly = false;
            else { if (err) snprintf(err, err_size, "asm= takes 'on' or 'off', not '%s'.", val); return false; }
            continue;
        }
        if (strcmp(key, "base") == 0) {
            if (strcmp(val, "on") == 0) w.baseplate = true;
            else if (strcmp(val, "off") == 0) w.baseplate = false;
            else { if (err) snprintf(err, err_size, "base= takes 'on' or 'off', not '%s'.", val); return false; }
            continue;
        }

        char *end = NULL;
        double num = strtod(val, &end);
        if (end == val || (end && *end != '\0')) {
            if (err) snprintf(err, err_size, "'%s' is not a number (in %s=%s).", val, key, val);
            return false;
        }

        if      (strcmp(key, "m") == 0)    w.module = clamp(num, 0.4, 10.0);
        else if (strcmp(key, "pin") == 0)  w.pin_diameter = clamp(num, 1.0, 12.0);
        else if (strcmp(key, "t") == 0)    w.thickness = clamp(num, 1.0, 20.0);
        else if (strcmp(key, "clr") == 0)  w.clearance = clamp(num, 0.0, 1.5);
        else if (strcmp(key, "gap") == 0)  w.layer_gap = clamp(num, 0.0, 2.0);
        else if (strcmp(key, "wall") == 0) w.wall = clamp(num, 0.8, 10.0);
        else if (strcmp(key, "bl") == 0)   w.backlash = clamp(num, 0.0, 1.0);
        else if (strcmp(key, "pa") == 0)   w.pressure_angle = clamp(num, 14.0, 30.0) * M_PI / 180.0;
        else {
            if (err) snprintf(err, err_size,
                              "No option called '%s'. Try m, pin, t, clr, gap, wall, bl, pa, fit, base, asm.",
                              key);
            return false;
        }
    }
    *p = w;
    return true;
}

/* --- Layers --------------------------------------------------------------
 *
 * Two bodies that share a pin cannot both sit at z = 0, and two gears that
 * mesh MUST. So: contract every meshed group to a single node, join two nodes
 * whenever they share a joint, and colour the result. The colour is the layer.
 */

typedef struct {
    int node_count;      /* links first, then cams */
    int link_count;
    const Mechanism *m;
    int *parent;         /* union-find: meshed bodies share a layer */
    int *layer;
    int max_layer;
} Layout;

static int uf_find(int *parent, int a) {
    while (parent[a] != a) { parent[a] = parent[parent[a]]; a = parent[a]; }
    return a;
}

static void uf_union(int *parent, int a, int b) {
    a = uf_find(parent, a); b = uf_find(parent, b);
    if (a != b) parent[b] = a;
}

/* The joints a node occupies. Links own theirs; a cam occupies its centre and
 * the roller's pin, because the disc and the roller it pushes must be in one
 * plane to touch at all. */
static int node_connectors(const Layout *L, int node, const int **out) {
    if (node < L->link_count) {
        *out = L->m->links[node].connector_ids;
        return L->m->links[node].connector_count;
    }
    static int pair[2];
    const Cam *c = &L->m->cams[node - L->link_count];
    int n = 0;
    if (c->center_connector_id >= 0) pair[n++] = c->center_connector_id;
    if (c->follower_connector_id >= 0) pair[n++] = c->follower_connector_id;
    *out = pair;
    return n;
}

static bool node_alive(const Layout *L, int node) {
    if (node < L->link_count) return L->m->links[node].alive;
    return L->m->cams[node - L->link_count].alive;
}

static bool nodes_share_a_joint(const Layout *L, int a, int b) {
    const int *ca, *cb;
    int na = node_connectors(L, a, &ca);
    int store[2];
    /* node_connectors hands back a shared static buffer for cams, so take a
     * copy of the first answer before asking the second question. */
    if (a >= L->link_count) { for (int i = 0; i < na; i++) store[i] = ca[i]; ca = store; }
    int nb = node_connectors(L, b, &cb);
    for (int i = 0; i < na; i++) {
        for (int j = 0; j < nb; j++) if (ca[i] == cb[j]) return true;
    }
    return false;
}

static void layout_build(Layout *L, const Mechanism *m) {
    L->m = m;
    L->link_count = m->link_count;
    L->node_count = m->link_count + m->cam_count;
    L->parent = xmalloc((size_t)(L->node_count > 0 ? L->node_count : 1) * sizeof(int));
    L->layer = xmalloc((size_t)(L->node_count > 0 ? L->node_count : 1) * sizeof(int));
    L->max_layer = 0;
    if (!L->parent || !L->layer) { free(L->parent); free(L->layer); L->parent = NULL; L->layer = NULL; return; }
    for (int i = 0; i < L->node_count; i++) { L->parent[i] = i; L->layer[i] = -1; }

    /* Anything that meshes, indexes or rolls against another body has to be in
     * the same plane as it, whatever else they touch. */
    for (int i = 0; i < m->gear_count; i++) {
        const Gear *g = &m->gears[i];
        if (!g->alive) continue;
        if (g->driver_link_id >= 0 && g->driven_link_id >= 0) {
            uf_union(L->parent, g->driver_link_id, g->driven_link_id);
        }
    }
    for (int i = 0; i < m->geneva_count; i++) {
        const Geneva *gv = &m->genevas[i];
        if (!gv->alive) continue;
        if (gv->driver_link_id >= 0 && gv->wheel_link_id >= 0) {
            uf_union(L->parent, gv->driver_link_id, gv->wheel_link_id);
        }
    }

    /* Greedy colouring, busiest group first: the node that conflicts with the
     * most others is hardest to place, so place it while every layer is free. */
    int groups = 0;
    int *reps = xmalloc((size_t)(L->node_count > 0 ? L->node_count : 1) * sizeof(int));
    int *degree = xmalloc((size_t)(L->node_count > 0 ? L->node_count : 1) * sizeof(int));
    if (!reps || !degree) { free(reps); free(degree); return; }
    for (int i = 0; i < L->node_count; i++) {
        if (!node_alive(L, i) || uf_find(L->parent, i) != i) continue;
        reps[groups++] = i;
    }
    for (int i = 0; i < groups; i++) degree[i] = 0;

    for (int i = 0; i < groups; i++) {
        for (int j = i + 1; j < groups; j++) {
            bool clash = false;
            for (int a = 0; a < L->node_count && !clash; a++) {
                if (!node_alive(L, a) || uf_find(L->parent, a) != reps[i]) continue;
                for (int b = 0; b < L->node_count && !clash; b++) {
                    if (!node_alive(L, b) || uf_find(L->parent, b) != reps[j]) continue;
                    if (nodes_share_a_joint(L, a, b)) clash = true;
                }
            }
            if (clash) { degree[i]++; degree[j]++; }
        }
    }

    for (int i = 1; i < groups; i++) {
        int kr = reps[i], kd = degree[i], j = i - 1;
        while (j >= 0 && degree[j] < kd) { reps[j + 1] = reps[j]; degree[j + 1] = degree[j]; j--; }
        reps[j + 1] = kr; degree[j + 1] = kd;
    }

    int *colour = xmalloc((size_t)(groups > 0 ? groups : 1) * sizeof(int));
    if (!colour) { free(reps); free(degree); return; }
    for (int i = 0; i < groups; i++) colour[i] = -1;

    for (int i = 0; i < groups; i++) {
        bool taken[64] = { false };
        for (int j = 0; j < i; j++) {
            if (colour[j] < 0 || colour[j] >= 64) continue;
            bool clash = false;
            for (int a = 0; a < L->node_count && !clash; a++) {
                if (!node_alive(L, a) || uf_find(L->parent, a) != reps[i]) continue;
                for (int b = 0; b < L->node_count && !clash; b++) {
                    if (!node_alive(L, b) || uf_find(L->parent, b) != reps[j]) continue;
                    if (nodes_share_a_joint(L, a, b)) clash = true;
                }
            }
            if (clash) taken[colour[j]] = true;
        }
        int c = 0;
        while (c < 63 && taken[c]) c++;
        colour[i] = c;
        if (c > L->max_layer) L->max_layer = c;
    }

    for (int i = 0; i < L->node_count; i++) {
        if (!node_alive(L, i)) continue;
        int root = uf_find(L->parent, i);
        for (int g = 0; g < groups; g++) if (reps[g] == root) { L->layer[i] = colour[g]; break; }
    }

    free(colour);
    free(reps);
    free(degree);
}

static void layout_free(Layout *L) {
    free(L->parent);
    free(L->layer);
    L->parent = NULL;
    L->layer = NULL;
}

/* --- Writing parts ------------------------------------------------------- */

/* Where a part goes in the assembled machine: its own origin rotated by
 * `angle`, moved to `offset`, with its underside at height `z`. Parts already
 * built in world coordinates (link plates, rails, the baseplate) just use a
 * zero angle and offset. `shelf` is which level it lifts to in the exploded
 * view, where the point is to see what goes where rather than what touches. */
typedef struct {
    double angle;
    Vec2 offset;
    double z;
    int shelf;
} Place;

typedef struct {
    const char *dir;
    FILE *man;
    PrintParams p;
    const Mechanism *m;
    int written;
    int failed;
    int warnings;
    /* Every part, stamped where it belongs -- once touching, once pulled
     * apart. These are what say how the pile goes together; the individual
     * STLs only say what to print. */
    Mesh3 assembled;
    Mesh3 exploded;
    double shelf_gap;
    int top_shelf;
} Ctx;

static Place place_at(double angle, Vec2 offset, double z, int shelf) {
    Place at = { angle, offset, z, shelf };
    return at;
}

/* Appends `src` to `dst`, turned and moved into place. */
static void stamp(Mesh3 *dst, const Mesh3 *src, Place at, double lift) {
    double ca = cos(at.angle), sa = sin(at.angle);
    for (int t = 0; t < src->tri_count; t++) {
        Vec3 v[3];
        for (int k = 0; k < 3; k++) {
            Vec3 q = src->verts[t * 3 + k];
            v[k].x = q.x * ca - q.y * sa + at.offset.x;
            v[k].y = q.x * sa + q.y * ca + at.offset.y;
            v[k].z = q.z + at.z + lift;
        }
        mesh3d_add_tri(dst, v[0], v[1], v[2]);
    }
}

static void record_placements(Ctx *c, const Mesh3 *mesh, const Place *places, int count) {
    if (!c->p.assembly) return;
    for (int i = 0; i < count; i++) {
        stamp(&c->assembled, mesh, places[i], 0.0);
        stamp(&c->exploded, mesh, places[i], (double)places[i].shelf * c->shelf_gap);
        if (places[i].shelf > c->top_shelf) c->top_shelf = places[i].shelf;
    }
}

static void warn(Ctx *c, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    fprintf(c->man, "  ! ");
    vfprintf(c->man, fmt, ap);
    fprintf(c->man, "\n");
    va_end(ap);
    c->warnings++;
}

/* A hole a part turns freely on, and one it grips. */
static double running_hole(const Ctx *c) {
    return c->p.m3_hardware ? 3.4 : c->p.pin_diameter + c->p.clearance;
}
static double press_hole(const Ctx *c) {
    return c->p.m3_hardware ? 2.9 : c->p.pin_diameter - PRESS_INTERFERENCE;
}
static double boss_radius(const Ctx *c) {
    return running_hole(c) * 0.5 + c->p.wall;
}

/* A Geneva's wheel and driver are bodies in their own right -- a slotted disc
 * and a crank arm -- so emit_geneva writes them and nothing else should. */
static bool is_geneva_body(const Mechanism *m, int link_id) {
    for (int i = 0; i < m->geneva_count; i++) {
        const Geneva *gv = &m->genevas[i];
        if (!gv->alive) continue;
        if (gv->wheel_link_id == link_id || gv->driver_link_id == link_id) return true;
    }
    return false;
}

/* The mark on a wheel's rim is a handle for reading rotation, not a joint:
 * nothing is pinned there, so it must not be given a pin of its own. */
static bool is_wheel_rim_mark(const Mechanism *m, int cid) {
    int link = mechanism_wheel_at_connector(m, cid);
    return link >= 0 && mechanism_wheel_mark(m, link) == cid;
}

/* True when a shaft has to drive this joint rather than a pin standing still
 * in it: something pivots there under power. Its baseplate hole is left open
 * for a motor shaft, and the body on it grips rather than turns. */
static bool is_driven_pivot(const Mechanism *m, int cid) {
    for (int li = 0; li < m->link_count; li++) {
        const Link *l = &m->links[li];
        if (l->alive && l->is_driven && l->pivot_connector_id == cid) return true;
    }
    return false;
}

/* Writes a finished mesh out and stamps it into the assembly wherever it
 * belongs -- which for a pin or a spacer is several places at once. */
static bool publish(Ctx *c, const char *name, const char *what, const char *where,
                     Mesh3 *mesh, bool ok, const Place *places, int place_count) {
    if (ok && !mesh3d_is_closed(mesh)) {
        ok = false;
        warn(c, "%s: the sweep did not close up, so it was not written.", name);
    }

    char path[1024];
    snprintf(path, sizeof path, "%s/%s.stl", c->dir, name);
    if (ok && !(ok = mesh3d_write_stl(mesh, name, path))) {
        warn(c, "%s: could not be written to %s.", name, path);
    }

    if (ok) {
        fprintf(c->man, "  %-26s %-38s %s, %d triangles\n", name, what, where, mesh->tri_count);
        record_placements(c, mesh, places, place_count);
        c->written++;
    } else {
        c->failed++;
    }
    return ok;
}

/* Extrudes one region, checks it is a solid, writes it and logs it. */
static bool emit_placed(Ctx *c, const char *name, const char *what, int layer,
                         const Vec2 *outer, int outer_n, const Loop2 *holes, int hole_n,
                         double thickness, const Place *places, int place_count) {
    if (outer_n < 3) { c->failed++; warn(c, "%s: no outline to sweep.", name); return false; }

    Mesh3 mesh;
    mesh3d_init(&mesh);
    Loop2 outer_loop = { (Vec2 *)outer, outer_n };
    Region2 region = { outer_loop, holes, hole_n };

    bool ok = mesh3d_extrude(&mesh, &region, 0.0, thickness);
    if (!ok) {
        warn(c, "%s: could not be swept into a solid -- its outline crosses itself, "
                "or a hole falls outside it.", name);
    }

    char where[64];
    snprintf(where, sizeof where, "layer %d, %.1f mm thick", layer, thickness);
    ok = publish(c, name, what, where, &mesh, ok, places, place_count);
    mesh3d_free(&mesh);
    return ok;
}

static bool emit(Ctx *c, const char *name, const char *what, int layer,
                  const Vec2 *outer, int outer_n, const Loop2 *holes, int hole_n,
                  double thickness, Place at) {
    return emit_placed(c, name, what, layer, outer, outer_n, holes, hole_n, thickness, &at, 1);
}

/* --- Outline scratch ----------------------------------------------------- */

typedef struct {
    Vec2 *outer;
    int outer_cap;
    Vec2 *holepts;     /* hole_cap loops of HOLE_SEG points each */
    Loop2 *holes;
    int hole_cap;
    int hole_n;
} Scratch;

static bool scratch_init(Scratch *s, int outer_cap, int hole_cap) {
    s->outer_cap = outer_cap;
    s->hole_cap = hole_cap;
    s->hole_n = 0;
    s->outer = xmalloc((size_t)outer_cap * sizeof(Vec2));
    s->holepts = xmalloc((size_t)hole_cap * HOLE_SEG * sizeof(Vec2));
    s->holes = xmalloc((size_t)hole_cap * sizeof(Loop2));
    if (!s->outer || !s->holepts || !s->holes) {
        free(s->outer); free(s->holepts); free(s->holes);
        s->outer = NULL; s->holepts = NULL; s->holes = NULL;
        return false;
    }
    return true;
}

static void scratch_free(Scratch *s) {
    free(s->outer); free(s->holepts); free(s->holes);
    s->outer = NULL; s->holepts = NULL; s->holes = NULL;
}

static void add_hole(Scratch *s, Vec2 at, double diameter) {
    if (s->hole_n >= s->hole_cap || !(diameter > 0.05)) return;
    /* Two joints can sit on the same spot -- a scissor's two feet share a
     * pivot, for one -- and punching the same hole twice is a hole overlapping
     * itself, which is not a region anything can be swept from. One pin goes
     * through both parts anyway, so one hole is the right answer. */
    for (int i = 0; i < s->hole_n; i++) {
        Vec2 c = s->holes[i].p[0];
        Vec2 centre = { 0.0, 0.0 };
        for (int k = 0; k < s->holes[i].count; k++) {
            centre.x += s->holes[i].p[k].x;
            centre.y += s->holes[i].p[k].y;
        }
        centre.x /= (double)s->holes[i].count;
        centre.y /= (double)s->holes[i].count;
        (void)c;
        if (vec2_dist(centre, at) < 1e-6) return;
    }
    Vec2 *dst = s->holepts + (size_t)s->hole_n * HOLE_SEG;
    int n = mesh3d_circle(at, diameter * 0.5, HOLE_SEG, dst, HOLE_SEG);
    if (n <= 0) return;
    s->holes[s->hole_n] = (Loop2){ dst, n };
    s->hole_n++;
}

/* --- Heights -------------------------------------------------------------
 *
 * Everything is measured from the TOP FACE of the baseplate, which is z = 0;
 * the plate itself hangs below. Layer 0 does not sit straight on the plate: a
 * pin through a joint that is NOT anchored has to get its head in somewhere,
 * and that somewhere is the gap underneath. */

static double part_standoff(const Ctx *c) {
    return PIN_HEAD_HEIGHT + c->p.layer_gap;
}

/* z of the underside, and of the top face, of layer `l`. */
static double layer_base(const Ctx *c, int l) {
    return part_standoff(c) + (double)l * (c->p.thickness + c->p.layer_gap);
}

static double layer_top(const Ctx *c, int l) {
    return layer_base(c, l) + c->p.thickness;
}

/* Where a part on layer `l` goes, given how its own origin is turned and where
 * that origin sits in the world. */
static Place on_layer(const Ctx *c, int l, double angle, Vec2 offset) {
    if (l < 0) l = 0;
    return place_at(angle, offset, layer_base(c, l), l + 1);
}

/* --- Parts --------------------------------------------------------------- */

/* Every link that is not something more specific: a flat plate covering its
 * pins, with a running hole at each. The outline is the convex hull of the
 * pins grown by a wall's worth, so a two-pin bar comes out as a dogbone and a
 * ternary link as a rounded triangle. */
static void emit_link_plate(Ctx *c, const Layout *L, int li) {
    const Link *l = &c->m->links[li];
    Vec2 *pins = xmalloc((size_t)l->connector_count * sizeof(Vec2));
    if (!pins) return;
    int np = 0;
    for (int i = 0; i < l->connector_count; i++) {
        int cid = l->connector_ids[i];
        if (cid < 0 || !c->m->connectors[cid].alive) continue;
        pins[np++] = c->m->connectors[cid].pos;
    }
    if (np < 1) { free(pins); return; }

    double r = boss_radius(c);
    Scratch s;
    int cap = mesh3d_hull_offset_capacity(np, BOSS_SEG);
    if (!scratch_init(&s, cap, np)) { free(pins); return; }

    int n = mesh3d_hull_offset(pins, np, r, BOSS_SEG, s.outer, cap);
    for (int i = 0; i < l->connector_count; i++) {
        int cid = l->connector_ids[i];
        if (cid < 0 || !c->m->connectors[cid].alive) continue;
        /* A body driven about this pin has to grip the shaft that turns it;
         * everything else has to spin on it. */
        bool grip = (l->is_driven && l->pivot_connector_id == cid);
        add_hole(&s, c->m->connectors[cid].pos, grip ? press_hole(c) : running_hole(c));
    }

    char name[64], what[160];
    snprintf(name, sizeof name, "link_%d", li);
    snprintf(what, sizeof what, "link plate, %d pin%s%s", np, np == 1 ? "" : "s",
              l->is_driven ? ", motor-driven" : "");
    /* A plate is drawn round the pins where they actually are, so it is
     * already in world coordinates and only has to be lifted to its layer. */
    emit(c, name, what, L->layer[li], s.outer, n, s.holes, s.hole_n, c->p.thickness,
         on_layer(c, L->layer[li], 0.0, (Vec2){ 0.0, 0.0 }));

    scratch_free(&s);
    free(pins);
}

static void emit_gear(Ctx *c, const Layout *L, int li, const double *phase) {
    int centre = -1;
    (void)mechanism_gear_wheel_radius(c->m, li, &centre);
    int teeth = mechanism_wheel_teeth(c->m, li);
    if (teeth < GEARING_MIN_TEETH || centre < 0) return;

    GearSpec spec = { c->p.module, teeth, c->p.pressure_angle,
                      c->p.backlash, EXPORT_FLANK_SAMPLES };
    int cap = gearing_outline_capacity(&spec);
    if (cap <= 0) return;

    Scratch s;
    if (!scratch_init(&s, cap, 2)) return;
    int n = gearing_tooth_outline(&spec, s.outer, cap);
    /* The gear is written about its own centre, ready to lie on the bed. */
    bool grip = c->m->links[li].is_driven || is_driven_pivot(c->m, centre);
    add_hole(&s, (Vec2){ 0.0, 0.0 }, grip ? press_hole(c) : running_hole(c));

    char name[64], what[160];
    snprintf(name, sizeof name, "gear_%d", li);
    snprintf(what, sizeof what, "spur gear, %d teeth, module %.2f, pitch r %.2f",
              teeth, c->p.module, gearing_pitch_radius(c->p.module, teeth));
    /* Turned to the angle that drops its teeth into its neighbours' spaces --
     * placed at each wheel's own rim mark instead, a train would assemble tip
     * to tip and not turn at all. */
    emit(c, name, what, L->layer[li], s.outer, n, s.holes, s.hole_n, c->p.thickness,
         on_layer(c, L->layer[li], phase ? phase[li] : 0.0, c->m->connectors[centre].pos));

    if (teeth < GEARING_WEAK_TEETH) {
        warn(c, "gear_%d has only %d teeth; below %d the flanks are short and the "
                "pair runs roughly. A smaller module would give it more.",
             li, teeth, GEARING_WEAK_TEETH);
    }
    scratch_free(&s);
}

/* A rack is written along +x with its teeth up, so it prints flat and its
 * pitch line is a straight edge you can measure from. Two slots let it slide
 * on pins standing in the baseplate. */
static void emit_rack(Ctx *c, const Layout *L, int gi) {
    const Gear *g = &c->m->gears[gi];
    if (g->driven_link_id < 0 || !c->m->links[g->driven_link_id].alive) return;
    const Link *bar = &c->m->links[g->driven_link_id];
    if (bar->connector_count < 2) return;

    Vec2 b0 = c->m->connectors[bar->connector_ids[0]].pos;
    Vec2 b1 = c->m->connectors[bar->connector_ids[1]].pos;
    double length = vec2_dist(b0, b1);
    if (length < c->p.module * M_PI) return;

    GearSpec spec = { c->p.module, GEARING_MIN_TEETH, c->p.pressure_angle,
                      c->p.backlash, EXPORT_FLANK_SAMPLES };
    double back = boss_radius(c) * 2.0;
    int cap = gearing_rack_capacity(&spec, length);
    Scratch s;
    if (!scratch_init(&s, cap, 2)) return;
    int n = gearing_rack_outline(&spec, length, back, s.outer, cap);

    /* Two guide slots down the middle of the bar's body, long enough for the
     * rack to travel most of its own length. */
    double slot_y = -c->p.module * 1.25 - back * 0.5;
    double travel = length * 0.3;
    for (int k = -1; k <= 1; k += 2) {
        double cx = (double)k * length * 0.25;
        Vec2 *dst = s.holepts + (size_t)s.hole_n * HOLE_SEG;
        int sn = mesh3d_stadium((Vec2){ cx - travel * 0.5, slot_y },
                                 (Vec2){ cx + travel * 0.5, slot_y },
                                 running_hole(c) * 0.5, 8, dst, HOLE_SEG);
        if (sn > 0 && s.hole_n < s.hole_cap) { s.holes[s.hole_n] = (Loop2){ dst, sn }; s.hole_n++; }
    }

    char name[64], what[160];
    snprintf(name, sizeof name, "rack_%d", gi);
    snprintf(what, sizeof what, "rack, %d teeth, module %.2f, %.1f mm long",
              gearing_rack_teeth(c->p.module, length), c->p.module, length);
    int layer = (g->driven_link_id < L->link_count) ? L->layer[g->driven_link_id] : 0;

    /* The rack is cut with its pitch line along its own x axis, so placing it
     * means putting that line exactly a pitch radius from the pinion's centre,
     * with the teeth facing the pinion -- not lining it up with the bar that
     * was drawn, which sits wherever it was dragged to. */
    Vec2 axis = vec2_scale(vec2_sub(b1, b0), 1.0 / length);
    Vec2 left = vec2_perp(axis);
    Vec2 pinion = c->m->connectors[g->driver_center_id].pos;
    double side = vec2_dot(vec2_sub(pinion, b0), left);
    double angle = atan2(axis.y, axis.x);
    Vec2 towards = left;
    if (side < 0.0) { towards = vec2_scale(left, -1.0); angle += M_PI; }
    double rp = gearing_pitch_radius(c->p.module,
                                     mechanism_wheel_teeth(c->m, g->driver_link_id));
    Vec2 mid = vec2_scale(vec2_add(b0, b1), 0.5);
    Vec2 on_pitch_line = vec2_sub(pinion, vec2_scale(towards, rp));
    Vec2 origin = vec2_add(on_pitch_line,
                            vec2_scale(axis, vec2_dot(vec2_sub(mid, pinion), axis)));
    emit(c, name, what, layer, s.outer, n, s.holes, s.hole_n, c->p.thickness,
         on_layer(c, layer, angle, origin));
    scratch_free(&s);
}

static void emit_cam(Ctx *c, const Layout *L, int ci) {
    const Cam *cam = &c->m->cams[ci];
    if (cam->center_connector_id < 0) return;

    Scratch s;
    if (!scratch_init(&s, CAM_SEG, 3)) return;
    /* cam_sample_surface gives the real cut surface -- the pitch curve already
     * inset by the roller -- in cam-local coordinates, which is exactly what
     * wants extruding. */
    cam_sample_surface(cam, s.outer, CAM_SEG);
    if (mesh3d_signed_area(s.outer, CAM_SEG) < 0.0) {
        for (int i = 0; i < CAM_SEG / 2; i++) {
            Vec2 t = s.outer[i]; s.outer[i] = s.outer[CAM_SEG - 1 - i]; s.outer[CAM_SEG - 1 - i] = t;
        }
    }
    add_hole(&s, (Vec2){ 0.0, 0.0 }, press_hole(c));

    /* A cam that only had a bore would spin on its shaft instead of with it.
     * A second pin, keyed to the body link it belongs to, stops that -- when
     * the cam is big enough to take one. */
    int ref = -1;
    if (cam->body_link_id >= 0 && cam->body_link_id < c->m->link_count) {
        const Link *body = &c->m->links[cam->body_link_id];
        for (int i = 0; i < body->connector_count; i++) {
            if (body->connector_ids[i] != cam->center_connector_id) { ref = body->connector_ids[i]; break; }
        }
    }
    double key_r = 0.0;
    if (ref >= 0 && c->m->connectors[ref].alive) {
        Vec2 d = vec2_sub(c->m->connectors[ref].pos, c->m->connectors[cam->center_connector_id].pos);
        double len = vec2_len(d);
        double room = cam->base_radius - running_hole(c) * 0.5 - c->p.wall;
        key_r = fmin(len * 0.45, room);
        if (key_r > running_hole(c) * 0.5 + c->p.wall && len > 1e-6) {
            add_hole(&s, vec2_scale(vec2_scale(d, 1.0 / len), key_r), running_hole(c));
        } else {
            key_r = 0.0;
            warn(c, "cam_%d is too small for a key pin; glue or screw it to link_%d "
                    "so it turns with the shaft.", ci, cam->body_link_id);
        }
    }

    char name[64], what[160];
    snprintf(name, sizeof name, "cam_%d", ci);
    snprintf(what, sizeof what, "disc cam, base r %.1f, lift %.1f%s",
              cam->base_radius, cam->lift, key_r > 0.0 ? ", keyed" : "");
    int layer = L->layer[L->link_count + ci];
    /* A cam's profile is already in the frame its body is in at rest, so it
     * needs no turning -- only moving onto its shaft. */
    emit(c, name, what, layer < 0 ? 0 : layer, s.outer, CAM_SEG, s.holes, s.hole_n,
         c->p.thickness,
         on_layer(c, layer, 0.0, c->m->connectors[cam->center_connector_id].pos));

    if (cam_is_undercut(cam)) {
        warn(c, "cam_%d is undercut: the roller cannot reach into its tightest "
                "concave stretch, so the follower will not track the profile there.", ci);
    }

    /* The roller that rides it. */
    if (cam->follower_connector_id >= 0 && cam->roller_radius > 0.3) {
        Scratch rs;
        if (scratch_init(&rs, HOLE_SEG * 2, 1)) {
            int rn = mesh3d_circle((Vec2){ 0.0, 0.0 }, cam->roller_radius, HOLE_SEG * 2,
                                    rs.outer, HOLE_SEG * 2);
            add_hole(&rs, (Vec2){ 0.0, 0.0 }, running_hole(c));
            snprintf(name, sizeof name, "roller_%d", ci);
            snprintf(what, sizeof what, "cam roller, r %.2f", cam->roller_radius);
            emit(c, name, what, layer < 0 ? 0 : layer, rs.outer, rn, rs.holes, rs.hole_n,
                 c->p.thickness,
                 on_layer(c, layer, 0.0, c->m->connectors[cam->follower_connector_id].pos));
            scratch_free(&rs);
        }
    }

    /* The key pin is a joint of its own, so it needs its hole in the body too;
     * that is handled by emit_link_plate seeing the same connector, except for
     * the keyed offset, which belongs to no connector. Say so plainly. */
    if (key_r > 0.0) {
        fprintf(c->man, "     cam_%d is keyed to link_%d by a second pin %.2f mm from the "
                        "centre, on the line towards the body's other pin; drill or model "
                        "the matching hole in link_%d.\n",
                ci, cam->body_link_id, key_r, cam->body_link_id);
    }
    scratch_free(&s);
}

/* A Geneva wheel: a disc with one radial slot per index position, each cut
 * right through the rim so the driver's pin can enter and leave. The slot
 * bottoms out where the pin passes closest to the wheel's centre, which is
 * what makes the index land in the right place. */
static void emit_geneva(Ctx *c, const Layout *L, int vi) {
    const Geneva *gv = &c->m->genevas[vi];
    int slots = gv->slot_count;
    if (slots < 3) return;
    double rim = mechanism_geneva_wheel_radius(gv);
    double inner = gv->center_distance - gv->crank_radius;
    double half_w = (running_hole(c) * 0.5);
    if (!(rim > inner + half_w * 2.0)) {
        warn(c, "geneva_%d is proportioned too tightly to cut slots in; skipped.", vi);
        return;
    }

    int per_slot = 2 + HOLE_SEG / 2 + 2 + 24;
    int cap = slots * per_slot + 16;
    Scratch s;
    if (!scratch_init(&s, cap, 2)) return;

    double t_rim = sqrt(rim * rim - half_w * half_w);   /* where a slot side meets the rim */
    int n = 0;
    for (int k = 0; k < slots; k++) {
        double phi = 2.0 * M_PI * (double)k / (double)slots;
        Vec2 u = { cos(phi), sin(phi) };
        Vec2 side = vec2_perp(u);

        /* In along the near side, round the bottom, out along the far side.
         * The bottom arc sweeps AWAY from the rim -- from the near wall
         * through the inward direction to the far wall -- so it is a pocket
         * the pin can sit in. Sweeping the other way round the same circle
         * also joins the two walls, but bulges back towards the rim and puts
         * a lump where the pin has to reach, leaving the slot too shallow to
         * index. */
        s.outer[n++] = vec2_sub(vec2_scale(u, t_rim), vec2_scale(side, half_w));
        s.outer[n++] = vec2_sub(vec2_scale(u, inner), vec2_scale(side, half_w));
        for (int i = 1; i < HOLE_SEG / 2; i++) {
            double a = phi - M_PI / 2.0 - M_PI * (double)i / (double)(HOLE_SEG / 2);
            s.outer[n++] = vec2_add(vec2_scale(u, inner),
                                     (Vec2){ half_w * cos(a), half_w * sin(a) });
        }
        s.outer[n++] = vec2_add(vec2_scale(u, inner), vec2_scale(side, half_w));
        s.outer[n++] = vec2_add(vec2_scale(u, t_rim), vec2_scale(side, half_w));

        /* Round the rim to the next slot's mouth. Both ends are known in
         * closed form -- a slot's side leaves the rim asin(w/r) either side of
         * its axis -- so they are computed rather than read back off the last
         * point with atan2, which wraps at pi and would send the arc the long
         * way round the wheel for every slot past halfway. */
        double mouth = asin(half_w / rim);
        double a0 = phi + mouth;
        double a1 = phi + 2.0 * M_PI / (double)slots - mouth;
        for (int i = 1; i < 24; i++) {
            double a = a0 + (a1 - a0) * (double)i / 24.0;
            s.outer[n++] = (Vec2){ rim * cos(a), rim * sin(a) };
        }
    }
    add_hole(&s, (Vec2){ 0.0, 0.0 }, running_hole(c));

    char name[64], what[160];
    snprintf(name, sizeof name, "geneva_wheel_%d", vi);
    snprintf(what, sizeof what, "Geneva wheel, %d slots, rim r %.1f", slots, rim);
    int layer = (gv->wheel_link_id >= 0) ? L->layer[gv->wheel_link_id] : 0;
    /* Show it at the instant that explains it: one slot facing the driver with
     * the crank pin sitting at the bottom of it. Anywhere else in the cycle and
     * the two parts look unrelated. */
    Vec2 wheel_c = c->m->connectors[gv->wheel_center_id].pos;
    Vec2 driver_c = c->m->connectors[gv->driver_center_id].pos;
    Vec2 apart = vec2_sub(driver_c, wheel_c);
    double towards_driver = atan2(apart.y, apart.x);
    emit(c, name, what, layer < 0 ? 0 : layer, s.outer, n, s.holes, s.hole_n, c->p.thickness,
         on_layer(c, layer, towards_driver, wheel_c));
    scratch_free(&s);

    /* The driver: an arm from the shaft out to the crank pin. */
    Scratch ds;
    Vec2 arm[2] = { { 0.0, 0.0 }, { gv->crank_radius, 0.0 } };
    int dcap = mesh3d_hull_offset_capacity(2, BOSS_SEG);
    if (scratch_init(&ds, dcap, 2)) {
        int dn = mesh3d_hull_offset(arm, 2, boss_radius(c), BOSS_SEG, ds.outer, dcap);
        add_hole(&ds, arm[0], press_hole(c));
        add_hole(&ds, arm[1], press_hole(c));
        snprintf(name, sizeof name, "geneva_driver_%d", vi);
        snprintf(what, sizeof what, "Geneva driver arm, crank r %.2f", gv->crank_radius);
        int dl = (gv->driver_link_id >= 0) ? L->layer[gv->driver_link_id] : 0;
        emit(c, name, what, dl < 0 ? 0 : dl, ds.outer, dn, ds.holes, ds.hole_n, c->p.thickness,
             on_layer(c, dl, towards_driver + M_PI, driver_c));
        scratch_free(&ds);
    }

    warn(c, "geneva_%d indexes correctly but is not LOCKED between steps: the "
            "locking disc and its matching rim scallops sit in a second plane "
            "this exporter does not generate. Add a detent, or hold the wheel "
            "by friction.", vi);
}

/* A slider rail: a bar with a slot the pin runs in, and a mounting hole
 * beyond each end so it can be pinned to the baseplate. */
static bool emit_rail(Ctx *c, int si, Vec2 *mount_a, Vec2 *mount_b) {
    const Slider *sl = &c->m->sliders[si];
    if (sl->rail_a_id < 0 || sl->rail_b_id < 0) return false;
    Vec2 a = c->m->connectors[sl->rail_a_id].pos;
    Vec2 b = c->m->connectors[sl->rail_b_id].pos;
    double len = vec2_dist(a, b);
    if (len < 1e-6) return false;

    Vec2 u = vec2_scale(vec2_sub(b, a), 1.0 / len);
    double pad = boss_radius(c) * 2.2;
    Vec2 ea = vec2_sub(a, vec2_scale(u, pad));
    Vec2 eb = vec2_add(b, vec2_scale(u, pad));

    double slot_r = running_hole(c) * 0.5;
    double bar_r = slot_r + c->p.wall;
    int cap = mesh3d_stadium_capacity(BOSS_SEG) + 8;
    Scratch s;
    if (!scratch_init(&s, cap, 4)) return false;
    int n = mesh3d_stadium(ea, eb, bar_r, BOSS_SEG, s.outer, cap);

    Vec2 *dst = s.holepts;
    int sn = mesh3d_stadium(a, b, slot_r, 8, dst, HOLE_SEG);
    if (sn > 0) { s.holes[s.hole_n] = (Loop2){ dst, sn }; s.hole_n++; }
    add_hole(&s, ea, press_hole(c));
    add_hole(&s, eb, press_hole(c));

    char name[64], what[160];
    snprintf(name, sizeof name, "rail_%d", si);
    snprintf(what, sizeof what, "slider rail, %.1f mm of travel", len);
    bool ok = emit(c, name, what, 0, s.outer, n, s.holes, s.hole_n, c->p.thickness,
                    on_layer(c, 0, 0.0, (Vec2){ 0.0, 0.0 }));
    scratch_free(&s);

    /* Only a rail that was actually written needs bolting down. Reporting the
     * mounts regardless would put two holes at the origin of the baseplate,
     * for a part that is not there. */
    if (!ok) return false;
    *mount_a = ea;
    *mount_b = eb;
    return true;
}

/* --- Assembly hardware --------------------------------------------------- */

/* A headed pin, plus the cap that goes on the far end of it. `places` says
 * every joint this length of pin belongs in, so the assembly gets one at each
 * while only one STL is written. */
static void emit_pin(Ctx *c, double length, const Place *places, const Place *cap_places,
                      int quantity) {
    double shaft_r = c->p.pin_diameter * 0.5;
    double head_r = shaft_r + c->p.wall;

    Mesh3 mesh;
    mesh3d_init(&mesh);
    Vec2 ring[HOLE_SEG];
    int hn = mesh3d_circle((Vec2){ 0, 0 }, head_r, HOLE_SEG, ring, HOLE_SEG);
    Region2 head = { { ring, hn }, NULL, 0 };
    bool ok = mesh3d_extrude(&mesh, &head, 0.0, PIN_HEAD_HEIGHT);

    Vec2 shaft_ring[HOLE_SEG];
    int sn = mesh3d_circle((Vec2){ 0, 0 }, shaft_r, HOLE_SEG, shaft_ring, HOLE_SEG);
    Region2 shaft = { { shaft_ring, sn }, NULL, 0 };
    /* Head and shaft are two separate closed solids sitting face to face, not
     * one shell. A slicer unions overlapping bodies, which is exactly what a
     * printer does with them, and the closedness check is happy either way
     * because every edge still has its partner within its own solid. */
    ok = mesh3d_extrude(&mesh, &shaft, PIN_HEAD_HEIGHT, PIN_HEAD_HEIGHT + length) && ok;

    char name[64], what[64], where[64];
    snprintf(name, sizeof name, "pin_%.1fmm", length);
    snprintf(what, sizeof what, "%s", c->p.m3_hardware ? "pin (unused in M3 mode)" : "headed pin");
    snprintf(where, sizeof where, "x%d", quantity);
    publish(c, name, what, where, &mesh, ok, places, quantity);
    mesh3d_free(&mesh);

    /* The cap that stops the stack sliding back off the pin. */
    Scratch cs;
    if (scratch_init(&cs, HOLE_SEG, 1)) {
        int cn = mesh3d_circle((Vec2){ 0, 0 }, head_r, HOLE_SEG, cs.outer, HOLE_SEG);
        add_hole(&cs, (Vec2){ 0, 0 }, press_hole(c));
        char cname[64], cwhat[160];
        snprintf(cname, sizeof cname, "cap_%.1fmm", length);
        snprintf(cwhat, sizeof cwhat, "push-on cap for pin_%.1fmm, x%d", length, quantity);
        emit_placed(c, cname, cwhat, 0, cs.outer, cn, cs.holes, cs.hole_n,
                     PIN_CAP_ENGAGEMENT, cap_places, quantity);
        scratch_free(&cs);
    }
}

static void emit_spacer(Ctx *c, double height, const Place *places, int quantity) {
    Scratch s;
    if (!scratch_init(&s, HOLE_SEG, 1)) return;
    int n = mesh3d_circle((Vec2){ 0, 0 }, running_hole(c) * 0.5 + c->p.wall, HOLE_SEG,
                           s.outer, HOLE_SEG);
    add_hole(&s, (Vec2){ 0, 0 }, running_hole(c) + 0.1);
    char name[64], what[160];
    snprintf(name, sizeof name, "spacer_%.2fmm", height);
    snprintf(what, sizeof what, "spacer washer, %.2f mm tall, x%d", height, quantity);
    emit_placed(c, name, what, 0, s.outer, n, s.holes, s.hole_n, height, places, quantity);
    scratch_free(&s);
}

/* --- The whole job ------------------------------------------------------- */

/* Creates `path` and any parent it needs, so typing a folder two levels down
 * works rather than failing on the level that is not there yet. */
static bool make_dirs(const char *path) {
    char work[1024];
    snprintf(work, sizeof work, "%s", path);
    for (char *s = work + 1; *s; s++) {
        if (*s != '/') continue;
        *s = '\0';
        if (mkdir(work, 0777) != 0 && errno != EEXIST) return false;
        *s = '/';
    }
    return mkdir(work, 0777) == 0 || errno == EEXIST;
}

bool print3d_export(const Mechanism *m, PrintParams p, const char *dir,
                     char *report, size_t report_size) {
    if (!m || !dir || !dir[0]) return false;
    if (!make_dirs(dir)) {
        if (report) snprintf(report, report_size, "Could not create %s.", dir);
        return false;
    }

    char manpath[1024];
    snprintf(manpath, sizeof manpath, "%s/MANIFEST.txt", dir);
    FILE *man = fopen(manpath, "w");
    if (!man) {
        if (report) snprintf(report, report_size, "Could not write %s.", manpath);
        return false;
    }

    Ctx c = { dir, man, p, m, 0, 0, 0, { NULL, 0, 0 }, { NULL, 0, 0 },
              p.thickness * EXPLODE_SPREAD, 0 };
    mesh3d_init(&c.assembled);
    mesh3d_init(&c.exploded);

    Layout layout;
    layout_build(&layout, m);
    if (!layout.layer) { fclose(man); return false; }

    /* Wheels have to be turned so their teeth interleave; each wheel's own rim
     * mark points wherever it happens to point. */
    double *phase = (m->link_count > 0)
                     ? xmalloc((size_t)m->link_count * sizeof(double)) : NULL;
    if (phase) mechanism_gear_phases(m, phase);

    fprintf(man, "Linkage Design -- printable parts\n");
    fprintf(man, "=================================\n\n");
    fprintf(man, "Units are millimetres. Every solid here has been checked watertight.\n\n");
    fprintf(man, "Settings\n");
    fprintf(man, "  gear module      %.3f mm/tooth (m=)\n", p.module);
    fprintf(man, "  pressure angle   %.1f degrees (pa=)\n", p.pressure_angle * 180.0 / M_PI);
    fprintf(man, "  backlash         %.2f mm per gear (bl=)\n", p.backlash);
    fprintf(man, "  pin diameter     %.2f mm (pin=)\n", p.pin_diameter);
    fprintf(man, "  plate thickness  %.2f mm (t=)\n", p.thickness);
    fprintf(man, "  running fit      %.2f mm hole (clr=%.2f)\n", running_hole(&c), p.clearance);
    fprintf(man, "  press fit        %.2f mm hole\n", press_hole(&c));
    fprintf(man, "  layer gap        %.2f mm (gap=)\n", p.layer_gap);
    fprintf(man, "  wall             %.2f mm (wall=)\n", p.wall);
    fprintf(man, "  fasteners        %s (fit=)\n", p.m3_hardware ? "M3 hardware" : "printed pins");
    fprintf(man, "\n");

    if (fabs(p.module - m->gear_module) > 1e-9) {
        fprintf(man, "NOTE: printing at module %.3f, but the mechanism was drawn at %.3f.\n"
                     "      Tooth counts are unchanged, so every ratio still holds, but the\n"
                     "      whole train comes out %.1f%% of the size on screen.\n\n",
                p.module, m->gear_module, 100.0 * p.module / m->gear_module);
    }

    fprintf(man, "Parts\n");

    /* Links, gears and racks. */
    int rack_link[64];
    int rack_links = 0;
    for (int gi = 0; gi < m->gear_count; gi++) {
        if (m->gears[gi].alive && m->gears[gi].kind == GEAR_RACK &&
            m->gears[gi].driven_link_id >= 0 && rack_links < 64) {
            rack_link[rack_links++] = m->gears[gi].driven_link_id;
        }
    }

    for (int li = 0; li < m->link_count; li++) {
        const Link *l = &m->links[li];
        if (!l->alive) continue;
        bool is_rack = false;
        for (int k = 0; k < rack_links; k++) if (rack_link[k] == li) is_rack = true;
        if (is_rack) continue;
        if (is_geneva_body(m, li)) continue;
        if (mechanism_is_gear_body(m, li)) emit_gear(&c, &layout, li, phase);
        else emit_link_plate(&c, &layout, li);
    }
    for (int gi = 0; gi < m->gear_count; gi++) {
        if (m->gears[gi].alive && m->gears[gi].kind == GEAR_RACK) emit_rack(&c, &layout, gi);
    }
    for (int ci = 0; ci < m->cam_count; ci++) {
        if (m->cams[ci].alive) emit_cam(&c, &layout, ci);
    }
    for (int vi = 0; vi < m->geneva_count; vi++) {
        if (m->genevas[vi].alive) emit_geneva(&c, &layout, vi);
    }

    /* Rails, remembering where they need pinning down. */
    Vec2 extra_mounts[128];
    int extra_mount_count = 0;
    for (int si = 0; si < m->slider_count; si++) {
        if (!m->sliders[si].alive) continue;
        Vec2 ma = { 0, 0 }, mb = { 0, 0 };
        if (!emit_rail(&c, si, &ma, &mb)) continue;
        if (extra_mount_count + 2 <= 128) {
            extra_mounts[extra_mount_count++] = ma;
            extra_mounts[extra_mount_count++] = mb;
        }
    }

    /* Which layers each joint actually carries, so pins can be cut to length
     * and the holes in the stack filled. */
    int *lowest = xmalloc((size_t)(m->connector_count > 0 ? m->connector_count : 1) * sizeof(int));
    int *highest = xmalloc((size_t)(m->connector_count > 0 ? m->connector_count : 1) * sizeof(int));
    unsigned long long *used = xcalloc((size_t)(m->connector_count > 0 ? m->connector_count : 1),
                                       sizeof(unsigned long long));
    if (!lowest || !highest || !used) {
        free(lowest); free(highest); free(used);
        layout_free(&layout); fclose(man);
        return false;
    }
    for (int i = 0; i < m->connector_count; i++) { lowest[i] = 99; highest[i] = -1; }
    for (int node = 0; node < layout.node_count; node++) {
        if (!node_alive(&layout, node) || layout.layer[node] < 0) continue;
        const int *conns;
        int store[2];
        int nc = node_connectors(&layout, node, &conns);
        if (node >= layout.link_count) { for (int i = 0; i < nc; i++) store[i] = conns[i]; conns = store; }
        for (int i = 0; i < nc; i++) {
            int cid = conns[i];
            if (cid < 0 || cid >= m->connector_count) continue;
            int lay = layout.layer[node];
            if (lay < lowest[cid]) lowest[cid] = lay;
            if (lay > highest[cid]) highest[cid] = lay;
            if (lay < 64) used[cid] |= 1ULL << lay;
        }
    }

    fprintf(man, "\nAssembly\n");
    fprintf(man, "  The baseplate's top face is height 0. Layer 0 stands %.2f mm above it --\n",
             part_standoff(&c));
    fprintf(man, "  room for a pin head at a joint that is not anchored -- and each layer\n");
    fprintf(man, "  above that is a further %.2f mm up.\n", p.thickness + p.layer_gap);

    /* Pins and spacers: one STL per distinct size, but one PLACE per joint, so
     * the assembly shows a pin standing in every hole. */
    double pin_lengths[64];
    int pin_counts[64];
    int pin_kinds = 0;
    double spacer_heights[64];
    int spacer_counts[64];
    int spacer_kinds = 0;
    double base_thickness = p.thickness * BASE_THICKNESS_FACTOR;

    int max_places = m->connector_count > 0 ? m->connector_count : 1;
    Place *pin_places = xcalloc((size_t)(64 * max_places), sizeof(Place));
    Place *cap_places = xcalloc((size_t)(64 * max_places), sizeof(Place));
    Place *spacer_places = xcalloc((size_t)(64 * max_places), sizeof(Place));
    if (!pin_places || !cap_places || !spacer_places) {
        free(pin_places); free(cap_places); free(spacer_places);
        free(lowest); free(highest); free(used);
        layout_free(&layout); fclose(man);
        return false;
    }
    /* Pins and caps sit above everything in the exploded view, which is where
     * you would be holding them if you were putting the thing together. */
    int pin_shelf = layout.max_layer + 2;

    for (int cid = 0; cid < m->connector_count; cid++) {
        if (!m->connectors[cid].alive || highest[cid] < 0) continue;
        if (is_wheel_rim_mark(m, cid)) continue;
        Vec2 at = m->connectors[cid].pos;
        bool anchored = p.baseplate && m->connectors[cid].is_anchor;

        /* An anchored pin is pressed through the baseplate and headed
         * underneath it. A moving one cannot be headed under the plate -- the
         * plate is solid there -- so its head goes in the standoff gap ABOVE
         * the plate, resting on it, which is what that gap is for. */
        double head_bottom = anchored ? -base_thickness - PIN_HEAD_HEIGHT : 0.0;
        double shaft_top = layer_top(&c, highest[cid]) + PIN_CAP_ENGAGEMENT;
        double length = shaft_top - (head_bottom + PIN_HEAD_HEIGHT);
        length = ceil(length * 2.0) / 2.0;   /* round to the half millimetre */

        int slot = -1;
        for (int i = 0; i < pin_kinds; i++) if (fabs(pin_lengths[i] - length) < 1e-6) slot = i;
        if (slot < 0 && pin_kinds < 64) { pin_lengths[pin_kinds] = length; pin_counts[pin_kinds] = 0; slot = pin_kinds++; }
        if (slot >= 0 && pin_counts[slot] < max_places) {
            pin_places[slot * max_places + pin_counts[slot]] =
                place_at(0.0, at, head_bottom, pin_shelf);
            cap_places[slot * max_places + pin_counts[slot]] =
                place_at(0.0, at, layer_top(&c, highest[cid]), pin_shelf);
            pin_counts[slot]++;
        }

        /* Anything on the pin below the lowest occupied layer, or in a gap
         * between two occupied ones, has to be packed out. */
        for (int lay = 0; lay <= highest[cid]; lay++) {
            if (lay < 64 && (used[cid] & (1ULL << lay))) continue;
            double h = p.thickness + p.layer_gap;
            int si = -1;
            for (int i = 0; i < spacer_kinds; i++) if (fabs(spacer_heights[i] - h) < 1e-6) si = i;
            if (si < 0 && spacer_kinds < 64) { spacer_heights[spacer_kinds] = h; spacer_counts[spacer_kinds] = 0; si = spacer_kinds++; }
            if (si >= 0 && spacer_counts[si] < max_places) {
                spacer_places[si * max_places + spacer_counts[si]] =
                    place_at(0.0, at, layer_base(&c, lay), lay + 1);
                spacer_counts[si]++;
            }
        }
    }

    if (!p.m3_hardware) {
        for (int i = 0; i < pin_kinds; i++) {
            emit_pin(&c, pin_lengths[i], pin_places + (size_t)i * max_places,
                      cap_places + (size_t)i * max_places, pin_counts[i]);
        }
    } else {
        fprintf(man, "  Hardware to buy:\n");
        for (int i = 0; i < pin_kinds; i++) {
            fprintf(man, "    %d x M3 screw, at least %.0f mm long, with a nut\n",
                     pin_counts[i], ceil(pin_lengths[i]));
        }
    }
    for (int i = 0; i < spacer_kinds; i++) {
        emit_spacer(&c, spacer_heights[i], spacer_places + (size_t)i * max_places,
                     spacer_counts[i]);
    }
    free(pin_places);
    free(cap_places);
    free(spacer_places);

    /* The baseplate, which is what actually holds the gear centres apart. */
    if (p.baseplate) {
        int anchor_count = 0;
        for (int i = 0; i < m->connector_count; i++) {
            if (m->connectors[i].alive && m->connectors[i].is_anchor) anchor_count++;
        }
        int total = anchor_count + extra_mount_count;
        if (total >= 1) {
            Vec2 *pts = xmalloc((size_t)total * sizeof(Vec2));
            if (pts) {
                int n = 0;
                for (int i = 0; i < m->connector_count; i++) {
                    if (!m->connectors[i].alive || !m->connectors[i].is_anchor) continue;
                    if (is_wheel_rim_mark(m, i)) continue;
                    pts[n++] = m->connectors[i].pos;
                }
                for (int i = 0; i < extra_mount_count; i++) pts[n++] = extra_mounts[i];

                int cap = mesh3d_hull_offset_capacity(n, BOSS_SEG);
                Scratch s;
                if (scratch_init(&s, cap, total)) {
                    int on = mesh3d_hull_offset(pts, n, BASE_MARGIN, BOSS_SEG, s.outer, cap);
                    for (int i = 0; i < m->connector_count; i++) {
                        if (!m->connectors[i].alive || !m->connectors[i].is_anchor) continue;
                        if (is_wheel_rim_mark(m, i)) continue;
                        /* Where a motor has to reach through, leave the hole
                         * open; everywhere else grip the pin. */
                        add_hole(&s, m->connectors[i].pos,
                                  is_driven_pivot(m, i) ? running_hole(&c) : press_hole(&c));
                    }
                    for (int i = 0; i < extra_mount_count; i++) add_hole(&s, extra_mounts[i], press_hole(&c));
                    emit(&c, "baseplate", "ground plate; holds every anchor", 0,
                         s.outer, on, s.holes, s.hole_n, base_thickness,
                         place_at(0.0, (Vec2){ 0.0, 0.0 }, -base_thickness, 0));
                    scratch_free(&s);
                }
                free(pts);
            }
        }
    }

    /* What meshes with what, and at what distance -- the numbers to check a
     * print against before assembling it. */
    bool any_mesh = false;
    for (int gi = 0; gi < m->gear_count; gi++) {
        const Gear *g = &m->gears[gi];
        if (!g->alive || g->kind == GEAR_RACK) continue;
        int ta = mechanism_wheel_teeth(m, g->driver_link_id);
        int tb = mechanism_wheel_teeth(m, g->driven_link_id);
        if (ta <= 0 || tb <= 0) continue;
        if (!any_mesh) { fprintf(man, "\nMeshes\n"); any_mesh = true; }
        fprintf(man, "  gear_%d (%d teeth) drives gear_%d (%d teeth): ratio %d:%d, "
                     "centre distance %.4f mm\n",
                 g->driver_link_id, ta, g->driven_link_id, tb, tb, ta,
                 gearing_center_distance(p.module, ta, tb));
    }
    for (int gi = 0; gi < m->gear_count; gi++) {
        const Gear *g = &m->gears[gi];
        if (!g->alive || g->kind != GEAR_RACK) continue;
        int ta = mechanism_wheel_teeth(m, g->driver_link_id);
        if (ta <= 0) continue;
        if (!any_mesh) { fprintf(man, "\nMeshes\n"); any_mesh = true; }
        fprintf(man, "  gear_%d (%d teeth) drives rack_%d: the pinion's centre must sit "
                     "%.4f mm from the rack's pitch line\n",
                 g->driver_link_id, ta, gi, gearing_pitch_radius(p.module, ta));
    }

    /* The two views of the whole thing. Neither is a part to print: they are
     * what tells you which part goes where, which a folder of separate STLs
     * cannot say on its own. */
    if (p.assembly && c.assembled.tri_count > 0) {
        char path[1024];
        fprintf(man, "\nHow it goes together\n");

        if (!mesh3d_shells_are_closed(&c.assembled)) {
            warn(&c, "the assembled view has an open edge in it, which means a part "
                     "was placed wrong; trust the individual parts over it.");
        }

        snprintf(path, sizeof path, "%s/assembly.stl", dir);
        if (mesh3d_write_stl(&c.assembled, "assembly", path)) {
            fprintf(man, "  %-26s %-38s %d triangles\n", "assembly",
                     "every part where it belongs", c.assembled.tri_count);
        } else {
            warn(&c, "the assembled view could not be written.");
        }

        snprintf(path, sizeof path, "%s/assembly_exploded.stl", dir);
        if (mesh3d_write_stl(&c.exploded, "assembly_exploded", path)) {
            fprintf(man, "  %-26s %-38s %d triangles\n", "assembly_exploded",
                     "the same, lifted apart by layer", c.exploded.tri_count);
        } else {
            warn(&c, "the exploded view could not be written.");
        }
        fprintf(man, "  Layers are pulled %.1f mm apart in the exploded view; pins and caps\n",
                 c.shelf_gap);
        fprintf(man, "  sit above everything, over the holes they drop into.\n");
        fprintf(man, "  Neither file is a part to print. They are several solids touching,\n");
        fprintf(man, "  not one watertight shell -- open them to see what goes where.\n");
    }

    fprintf(man, "\n%d part%s written", c.written, c.written == 1 ? "" : "s");
    if (c.failed) fprintf(man, ", %d could not be", c.failed);
    if (c.warnings) fprintf(man, ", %d warning%s above", c.warnings, c.warnings == 1 ? "" : "s");
    fprintf(man, ".\n");

    free(lowest);
    free(highest);
    free(used);
    free(phase);
    mesh3d_free(&c.assembled);
    mesh3d_free(&c.exploded);
    layout_free(&layout);
    fclose(man);

    if (report) {
        snprintf(report, report_size,
                  "Wrote %d part%s to %s (%d layer%s, %d warning%s) -- see MANIFEST.txt.",
                  c.written, c.written == 1 ? "" : "s", dir,
                  layout.max_layer + 1, layout.max_layer == 0 ? "" : "s",
                  c.warnings, c.warnings == 1 ? "" : "s");
    }
    return c.written > 0;
}
