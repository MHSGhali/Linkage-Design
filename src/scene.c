#include "scene.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "xalloc.h"

/* Doubles round-trip exactly at 17 significant digits, so reopening a file
 * gives back the mechanism that was saved rather than one very close to it. */
#define REAL "%.17g"

static void fail(char *err, size_t err_size, const char *fmt, ...) {
    if (!err || err_size == 0) return;
    va_list args;
    va_start(args, fmt);
    vsnprintf(err, err_size, fmt, args);
    va_end(args);
}

/* ---------------------------------------------------------------------------
 * Saving
 * ------------------------------------------------------------------------ */

/* Maps an entity's in-memory id (an array index, with holes where things were
 * deleted) to its position among the live ones, which is what gets written. */
typedef struct {
    int *slot;   /* slot[id] = written id, or -1 for a dead entity */
    int count;   /* how many live ones there are */
} IdMap;

static bool idmap_build(IdMap *map, int n) {
    map->slot = (n > 0) ? xmalloc((size_t)n * sizeof(int)) : NULL;
    if (n > 0 && !map->slot) return false;
    for (int i = 0; i < n; i++) map->slot[i] = -1;
    map->count = 0;
    return true;
}

static void idmap_free(IdMap *map) { free(map->slot); map->slot = NULL; }

static int idmap_of(const IdMap *map, int id) {
    if (id < 0) return -1;
    return map->slot[id];
}

bool scene_file_exists(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    fclose(f);
    return true;
}

bool scene_save(const Mechanism *m, const SolverParams *params, const char *path,
                 char *err, size_t err_size) {
    IdMap cmap, lmap;
    if (!idmap_build(&cmap, m->connector_count) || !idmap_build(&lmap, m->link_count)) {
        idmap_free(&cmap);
        idmap_free(&lmap);
        fail(err, err_size, "Out of memory while saving.");
        return false;
    }
    for (int i = 0; i < m->connector_count; i++) {
        if (m->connectors[i].alive) cmap.slot[i] = cmap.count++;
    }
    for (int i = 0; i < m->link_count; i++) {
        if (m->links[i].alive) lmap.slot[i] = lmap.count++;
    }

    FILE *f = fopen(path, "w");
    if (!f) {
        idmap_free(&cmap);
        idmap_free(&lmap);
        fail(err, err_size, "Could not open %s for writing.", path);
        return false;
    }

    fprintf(f, "LINKAGE %d\n", SCENE_FORMAT_VERSION);
    fprintf(f, "# A Linkage Design mechanism. C pins, L bodies, S sliders,\n");
    fprintf(f, "# G gears, V Geneva wheels, M cams, P world settings,\n");
    fprintf(f, "# T the gear module every wheel is cut to.\n");
    fprintf(f, "P " REAL " " REAL "\n", params->gravity.x, params->gravity.y);
    fprintf(f, "T " REAL "\n", m->gear_module);

    for (int i = 0; i < m->connector_count; i++) {
        const Connector *c = &m->connectors[i];
        if (!c->alive) continue;
        fprintf(f, "C " REAL " " REAL " %d %d\n", c->pos.x, c->pos.y,
                 c->is_anchor ? 1 : 0, c->traced ? 1 : 0);
    }

    for (int i = 0; i < m->link_count; i++) {
        const Link *l = &m->links[i];
        if (!l->alive) continue;
        fprintf(f, "L %d", l->connector_count);
        for (int k = 0; k < l->connector_count; k++) {
            fprintf(f, " %d", idmap_of(&cmap, l->connector_ids[k]));
        }
        fprintf(f, " %d %d %d " REAL " " REAL " %d\n",
                 l->rigid ? 1 : 0, l->is_driven ? 1 : 0,
                 idmap_of(&cmap, l->pivot_connector_id),
                 l->motor_speed_deg_s, l->wheel_radius, l->wheel_teeth);
    }

    for (int i = 0; i < m->slider_count; i++) {
        const Slider *s = &m->sliders[i];
        if (!s->alive) continue;
        fprintf(f, "S %d %d %d\n", idmap_of(&cmap, s->pin_connector_id),
                 idmap_of(&cmap, s->rail_a_id), idmap_of(&cmap, s->rail_b_id));
    }

    for (int i = 0; i < m->gear_count; i++) {
        const Gear *g = &m->gears[i];
        if (!g->alive) continue;
        fprintf(f, "G %d %d %d %d %d " REAL " " REAL " " REAL "\n",
                 (int)g->kind,
                 idmap_of(&lmap, g->driver_link_id), idmap_of(&cmap, g->driver_center_id),
                 idmap_of(&lmap, g->driven_link_id), idmap_of(&cmap, g->driven_center_id),
                 g->ratio, g->rack_axis.x, g->rack_axis.y);
    }

    for (int i = 0; i < m->geneva_count; i++) {
        const Geneva *gv = &m->genevas[i];
        if (!gv->alive) continue;
        fprintf(f, "V %d %d %d %d %d\n",
                 idmap_of(&lmap, gv->driver_link_id), idmap_of(&cmap, gv->driver_center_id),
                 idmap_of(&lmap, gv->wheel_link_id), idmap_of(&cmap, gv->wheel_center_id),
                 gv->slot_count);
    }

    for (int i = 0; i < m->cam_count; i++) {
        const Cam *c = &m->cams[i];
        if (!c->alive) continue;
        /* A drawn profile is the user's own line, not something that could be
         * recomputed from anything else, so the whole table is written out. */
        fprintf(f, "M %d %d %d " REAL " " REAL " " REAL "\n",
                 idmap_of(&lmap, c->body_link_id), idmap_of(&cmap, c->center_connector_id),
                 idmap_of(&cmap, c->follower_connector_id),
                 c->roller_radius, c->spring_k, c->spring_preload);
        for (int k = 0; k < CAM_PROFILE_SAMPLES; k++) {
            fprintf(f, k % 6 == 5 || k == CAM_PROFILE_SAMPLES - 1 ? REAL "\n" : REAL " ",
                     c->pitch_r[k]);
        }
    }

    bool ok = (fflush(f) == 0) && (ferror(f) == 0);
    if (fclose(f) != 0) ok = false;
    idmap_free(&cmap);
    idmap_free(&lmap);
    if (!ok) fail(err, err_size, "Could not finish writing %s.", path);
    return ok;
}

/* ---------------------------------------------------------------------------
 * Loading
 * ------------------------------------------------------------------------ */

#define SCENE_LINE_MAX 4096

static bool connector_in_range(const Mechanism *m, int id) {
    return id >= 0 && id < m->connector_count;
}

static bool link_in_range(const Mechanism *m, int id) {
    return id >= 0 && id < m->link_count;
}

bool scene_load(Mechanism *m, SolverParams *params, const char *path,
                 char *err, size_t err_size) {
    FILE *f = fopen(path, "r");
    if (!f) {
        fail(err, err_size, "Could not open %s.", path);
        return false;
    }

    char line[SCENE_LINE_MAX];
    if (!fgets(line, sizeof line, f)) {
        fclose(f);
        fail(err, err_size, "%s is empty.", path);
        return false;
    }
    int version = 0;
    if (sscanf(line, "LINKAGE %d", &version) != 1) {
        fclose(f);
        fail(err, err_size, "%s is not a Linkage Design mechanism.", path);
        return false;
    }
    if (version > SCENE_FORMAT_VERSION) {
        fclose(f);
        fail(err, err_size, "%s was written by a newer version (format %d).", path, version);
        return false;
    }

    /* Built to one side, so a file that turns out to be broken half-way
     * through costs nothing that is already on screen. */
    Mechanism loaded;
    mechanism_init(&loaded);
    SolverParams loaded_params = *params;
    int line_no = 1;
    const char *problem = NULL;

    while (!problem && fgets(line, sizeof line, f)) {
        line_no++;
        char kind = 0;
        if (sscanf(line, " %c", &kind) != 1) continue;   /* blank */
        if (kind == '#') continue;

        if (kind == 'P') {
            double gx = 0.0, gy = 0.0;
            if (sscanf(line, " P %lf %lf", &gx, &gy) != 2) { problem = "world settings"; break; }
            loaded_params.gravity = (Vec2){ gx, gy };

        } else if (kind == 'T') {
            double module = 0.0;
            if (sscanf(line, " T %lf", &module) != 1) { problem = "the gear module"; break; }
            loaded.gear_module = module;

        } else if (kind == 'C') {
            double x = 0.0, y = 0.0;
            int anchor = 0, traced = 0;
            if (sscanf(line, " C %lf %lf %d %d", &x, &y, &anchor, &traced) != 4) {
                problem = "a pin"; break;
            }
            int id = mechanism_add_connector(&loaded, (Vec2){ x, y }, anchor != 0);
            if (id < 0) { problem = "a pin"; break; }
            if (traced) mechanism_set_traced(&loaded, id, true);

        } else if (kind == 'L') {
            int n = 0, consumed = 0;
            if (sscanf(line, " L %d%n", &n, &consumed) != 1 || n < 2 || n > 4096) {
                problem = "a body"; break;
            }
            int *ids = xmalloc((size_t)n * sizeof(int));
            if (!ids) { problem = "a body (out of memory)"; break; }
            const char *p = line + consumed;
            bool bad = false;
            for (int k = 0; k < n; k++) {
                int used = 0;
                if (sscanf(p, " %d%n", &ids[k], &used) != 1 || !connector_in_range(&loaded, ids[k])) {
                    bad = true; break;
                }
                p += used;
            }
            int rigid = 1, driven = 0, pivot = -1, teeth = 0;
            double speed = 0.0, wheel_radius = 0.0;
            if (!bad) {
                /* Format 1 stopped at the wheel radius. Read the tooth count
                 * when it is there and work it out from the radius when it is
                 * not, so a file written before gears had teeth still opens. */
                int fields = sscanf(p, " %d %d %d %lf %lf %d",
                                     &rigid, &driven, &pivot, &speed, &wheel_radius, &teeth);
                if (fields < 5) bad = true;
                else if (fields == 5) teeth = 0;
            }
            int lid = bad ? -1 : mechanism_add_link(&loaded, ids, n);
            free(ids);
            if (bad || lid < 0) { problem = "a body"; break; }

            loaded.links[lid].wheel_radius = wheel_radius;
            loaded.links[lid].wheel_teeth = teeth;
            if (!rigid) mechanism_set_rigid(&loaded, lid, false);
            /* set_driven_about rather than toggle_driven: a motor may sit on a
             * pivot that something else moves, which is what an arm chain is. */
            if (driven && (!connector_in_range(&loaded, pivot) ||
                            !mechanism_set_driven_about(&loaded, lid, pivot, speed))) {
                problem = "a motor"; break;
            }

        } else if (kind == 'S') {
            int pin = -1, ra = -1, rb = -1;
            if (sscanf(line, " S %d %d %d", &pin, &ra, &rb) != 3 ||
                mechanism_add_slider(&loaded, pin, ra, rb) < 0) {
                problem = "a slider"; break;
            }

        } else if (kind == 'G') {
            int gkind = 0, dl = -1, dc = -1, nl = -1, nc = -1;
            double ratio = 1.0, ax = 0.0, ay = 0.0;
            if (sscanf(line, " G %d %d %d %d %d %lf %lf %lf",
                        &gkind, &dl, &dc, &nl, &nc, &ratio, &ax, &ay) != 8 ||
                !link_in_range(&loaded, dl) || !link_in_range(&loaded, nl)) {
                problem = "a gear"; break;
            }
            int made = (gkind == GEAR_RACK)
                ? mechanism_add_rack(&loaded, dl, dc, nl, nc, (Vec2){ ax, ay })
                : mechanism_add_gear(&loaded, dl, dc, nl, nc, ratio, gkind == GEAR_INTERNAL);
            if (made < 0) { problem = "a gear"; break; }

        } else if (kind == 'V') {
            int dl = -1, dc = -1, wl = -1, wc = -1, slots = 0;
            if (sscanf(line, " V %d %d %d %d %d", &dl, &dc, &wl, &wc, &slots) != 5 ||
                mechanism_add_geneva(&loaded, dl, dc, wl, wc, slots) < 0) {
                problem = "a Geneva wheel"; break;
            }

        } else if (kind == 'M') {
            int body = -1, centre = -1, follower = -1;
            double roller = 0.0, spring_k = 0.0, preload = 0.0;
            if (sscanf(line, " M %d %d %d %lf %lf %lf",
                        &body, &centre, &follower, &roller, &spring_k, &preload) != 6) {
                problem = "a cam"; break;
            }
            int cid = mechanism_add_cam(&loaded, body, centre, follower);
            if (cid < 0) { problem = "a cam"; break; }
            Cam *cam = &loaded.cams[cid];
            bool bad = false;
            for (int k = 0; k < CAM_PROFILE_SAMPLES; k++) {
                if (fscanf(f, " %lf", &cam->pitch_r[k]) != 1 || !(cam->pitch_r[k] > 0.0)) {
                    bad = true; break;
                }
            }
            if (bad) { problem = "a cam profile"; break; }
            cam->roller_radius = roller;
            cam->spring_k = spring_k;
            cam->spring_preload = preload;
            /* base radius and lift are a summary of the table, so they are
             * read back off it rather than trusted from the file. */
            double lo = cam->pitch_r[0], hi = cam->pitch_r[0];
            for (int k = 1; k < CAM_PROFILE_SAMPLES; k++) {
                if (cam->pitch_r[k] < lo) lo = cam->pitch_r[k];
                if (cam->pitch_r[k] > hi) hi = cam->pitch_r[k];
            }
            cam->base_radius = lo;
            cam->lift = hi - lo;

        } else {
            problem = "an unrecognised record"; break;
        }
    }

    fclose(f);

    if (problem) {
        mechanism_free(&loaded);
        fail(err, err_size, "%s is damaged: could not read %s on line %d.", path, problem, line_no);
        return false;
    }

    /* A format-1 file recorded a wheel's radius but not its tooth count, and a
     * radius on its own is not a gear. Put each such wheel on the nearest whole
     * tooth -- which also walks its rim mark out to the snapped radius, so what
     * reopens is a mechanism that could be printed. */
    for (int li = 0; li < loaded.link_count; li++) {
        const Link *l = &loaded.links[li];
        if (!l->alive || !(l->wheel_radius > 0.0) || l->wheel_teeth > 0) continue;
        mechanism_set_wheel_radius(&loaded, li, l->wheel_radius);
    }

    /* Pitch radii, crank radii and the like follow from where the parts are. */
    mechanism_refresh_joint_sizes(&loaded);

    mechanism_free(m);
    *m = loaded;
    *params = loaded_params;
    return true;
}
