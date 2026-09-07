#include "mechanism.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* How finely a cam's surface is sampled for hit-testing. */
#define CAM_PICK_SAMPLES 180

int mechanism_pair_index(int i, int j, int k) {
    return i * (2 * k - i - 1) / 2 + (j - i - 1);
}

static void *grow(void *arr, int *capacity, int count, size_t elem_size) {
    if (count < *capacity) return arr;
    int new_cap = (*capacity == 0) ? 4 : (*capacity) * 2;
    while (new_cap <= count) new_cap *= 2;
    *capacity = new_cap;
    return realloc(arr, (size_t)new_cap * elem_size);
}

void mechanism_init(Mechanism *m) {
    m->connectors = NULL;
    m->connector_count = 0;
    m->connector_capacity = 0;
    m->links = NULL;
    m->link_count = 0;
    m->link_capacity = 0;
    m->cams = NULL;
    m->cam_count = 0;
    m->cam_capacity = 0;
    m->sliders = NULL;
    m->slider_count = 0;
    m->slider_capacity = 0;
    m->gears = NULL;
    m->gear_count = 0;
    m->gear_capacity = 0;
    m->genevas = NULL;
    m->geneva_count = 0;
    m->geneva_capacity = 0;
}

void mechanism_free(Mechanism *m) {
    for (int i = 0; i < m->connector_count; i++) {
        free(m->connectors[i].path);
        free(m->connectors[i].path_time);
    }
    for (int i = 0; i < m->link_count; i++) {
        free(m->links[i].connector_ids);
        free(m->links[i].rest_dist);
        free(m->links[i].frozen_local_offset);
    }
    free(m->genevas);
    free(m->gears);
    free(m->sliders);
    free(m->cams);
    free(m->links);
    free(m->connectors);
    mechanism_init(m);
}

void mechanism_clone(const Mechanism *src, Mechanism *dst) {
    dst->connector_count = src->connector_count;
    dst->connector_capacity = src->connector_count;
    dst->connectors = (src->connector_count > 0) ? malloc((size_t)src->connector_count * sizeof(Connector)) : NULL;
    for (int i = 0; i < src->connector_count; i++) {
        Connector *d = &dst->connectors[i];
        *d = src->connectors[i];
        if (src->connectors[i].path_count > 0) {
            size_t n = (size_t)src->connectors[i].path_count;
            d->path = malloc(n * sizeof(Vec2));
            memcpy(d->path, src->connectors[i].path, n * sizeof(Vec2));
            d->path_time = malloc(n * sizeof(double));
            memcpy(d->path_time, src->connectors[i].path_time, n * sizeof(double));
            d->path_capacity = src->connectors[i].path_count;
        } else {
            d->path = NULL;
            d->path_time = NULL;
            d->path_capacity = 0;
        }
    }

    dst->link_count = src->link_count;
    dst->link_capacity = src->link_count;
    dst->links = (src->link_count > 0) ? malloc((size_t)src->link_count * sizeof(Link)) : NULL;
    for (int i = 0; i < src->link_count; i++) {
        Link *d = &dst->links[i];
        *d = src->links[i];
        int k = src->links[i].connector_count;
        if (k > 0) {
            d->connector_ids = malloc((size_t)k * sizeof(int));
            memcpy(d->connector_ids, src->links[i].connector_ids, (size_t)k * sizeof(int));
            int npairs = k * (k - 1) / 2;
            d->rest_dist = malloc((size_t)npairs * sizeof(double));
            memcpy(d->rest_dist, src->links[i].rest_dist, (size_t)npairs * sizeof(double));
            if (src->links[i].frozen_local_offset) {
                d->frozen_local_offset = malloc((size_t)k * sizeof(Vec2));
                memcpy(d->frozen_local_offset, src->links[i].frozen_local_offset, (size_t)k * sizeof(Vec2));
            } else {
                d->frozen_local_offset = NULL;
            }
        } else {
            d->connector_ids = NULL;
            d->rest_dist = NULL;
            d->frozen_local_offset = NULL;
        }
    }

    /* Cams, sliders, gears and Geneva wheels own no heap memory, so a flat
     * copy of each array is a full deep copy. */
    dst->cam_count = dst->cam_capacity = src->cam_count;
    dst->cams = (src->cam_count > 0) ? malloc((size_t)src->cam_count * sizeof(Cam)) : NULL;
    if (src->cam_count > 0) memcpy(dst->cams, src->cams, (size_t)src->cam_count * sizeof(Cam));

    dst->slider_count = dst->slider_capacity = src->slider_count;
    dst->sliders = (src->slider_count > 0) ? malloc((size_t)src->slider_count * sizeof(Slider)) : NULL;
    if (src->slider_count > 0) memcpy(dst->sliders, src->sliders, (size_t)src->slider_count * sizeof(Slider));

    dst->gear_count = dst->gear_capacity = src->gear_count;
    dst->gears = (src->gear_count > 0) ? malloc((size_t)src->gear_count * sizeof(Gear)) : NULL;
    if (src->gear_count > 0) memcpy(dst->gears, src->gears, (size_t)src->gear_count * sizeof(Gear));

    dst->geneva_count = dst->geneva_capacity = src->geneva_count;
    dst->genevas = (src->geneva_count > 0) ? malloc((size_t)src->geneva_count * sizeof(Geneva)) : NULL;
    if (src->geneva_count > 0) memcpy(dst->genevas, src->genevas, (size_t)src->geneva_count * sizeof(Geneva));
}

int mechanism_add_connector(Mechanism *m, Vec2 pos, bool is_anchor) {
    m->connectors = grow(m->connectors, &m->connector_capacity, m->connector_count, sizeof(Connector));
    Connector *c = &m->connectors[m->connector_count];
    c->pos = pos;
    c->prev_pos = pos;
    c->is_anchor = is_anchor;
    c->selected = false;
    c->traced = false;
    c->path = NULL;
    c->path_time = NULL;
    c->path_count = 0;
    c->path_capacity = 0;
    c->alive = true;
    return m->connector_count++;
}

void mechanism_delete_cam(Mechanism *m, int cam_id) {
    if (cam_id < 0 || cam_id >= m->cam_count) return;
    Cam *c = &m->cams[cam_id];
    if (!c->alive) return;
    c->alive = false;
    c->selected = false;
    c->body_link_id = -1;
    c->center_connector_id = -1;
    c->follower_connector_id = -1;
}

/* A cam is defined by a link and two connectors; if any of them goes away the
 * cam is meaningless, so it goes too. */
static void delete_cams_using_link(Mechanism *m, int link_id) {
    for (int i = 0; i < m->cam_count; i++) {
        if (m->cams[i].alive && m->cams[i].body_link_id == link_id) mechanism_delete_cam(m, i);
    }
}

void mechanism_delete_slider(Mechanism *m, int slider_id) {
    if (slider_id < 0 || slider_id >= m->slider_count) return;
    Slider *s = &m->sliders[slider_id];
    s->alive = false;
    s->selected = false;
    s->pin_connector_id = s->rail_a_id = s->rail_b_id = -1;
}

void mechanism_delete_gear(Mechanism *m, int gear_id) {
    if (gear_id < 0 || gear_id >= m->gear_count) return;
    Gear *g = &m->gears[gear_id];
    if (g->alive && g->driven_link_id >= 0 && g->driven_link_id < m->link_count) {
        m->links[g->driven_link_id].driven_externally = false;
    }
    g->alive = false;
    g->selected = false;
    g->driver_link_id = g->driven_link_id = -1;
}

void mechanism_delete_geneva(Mechanism *m, int geneva_id) {
    if (geneva_id < 0 || geneva_id >= m->geneva_count) return;
    Geneva *gv = &m->genevas[geneva_id];
    if (gv->alive && gv->wheel_link_id >= 0 && gv->wheel_link_id < m->link_count) {
        m->links[gv->wheel_link_id].driven_externally = false;
    }
    gv->alive = false;
    gv->selected = false;
    gv->driver_link_id = gv->wheel_link_id = -1;
}

/* Each of these elements is defined by links and connectors; if any of them
 * goes away the element is meaningless, so it goes too. */
static void delete_joints_using_link(Mechanism *m, int link_id) {
    for (int i = 0; i < m->gear_count; i++) {
        const Gear *g = &m->gears[i];
        if (g->alive && (g->driver_link_id == link_id || g->driven_link_id == link_id)) {
            mechanism_delete_gear(m, i);
        }
    }
    for (int i = 0; i < m->geneva_count; i++) {
        const Geneva *gv = &m->genevas[i];
        if (gv->alive && (gv->driver_link_id == link_id || gv->wheel_link_id == link_id)) {
            mechanism_delete_geneva(m, i);
        }
    }
}

static void delete_joints_using_connector(Mechanism *m, int connector_id) {
    for (int i = 0; i < m->slider_count; i++) {
        const Slider *s = &m->sliders[i];
        if (s->alive && (s->pin_connector_id == connector_id ||
                          s->rail_a_id == connector_id || s->rail_b_id == connector_id)) {
            mechanism_delete_slider(m, i);
        }
    }
    for (int i = 0; i < m->gear_count; i++) {
        const Gear *g = &m->gears[i];
        if (g->alive && (g->driver_center_id == connector_id || g->driven_center_id == connector_id)) {
            mechanism_delete_gear(m, i);
        }
    }
    for (int i = 0; i < m->geneva_count; i++) {
        const Geneva *gv = &m->genevas[i];
        if (gv->alive && (gv->driver_center_id == connector_id || gv->wheel_center_id == connector_id)) {
            mechanism_delete_geneva(m, i);
        }
    }
}

static void delete_cams_using_connector(Mechanism *m, int connector_id) {
    for (int i = 0; i < m->cam_count; i++) {
        const Cam *c = &m->cams[i];
        if (c->alive && (c->center_connector_id == connector_id ||
                          c->follower_connector_id == connector_id)) {
            mechanism_delete_cam(m, i);
        }
    }
}

void mechanism_delete_link(Mechanism *m, int link_id) {
    if (link_id < 0 || link_id >= m->link_count) return;
    Link *l = &m->links[link_id];
    if (!l->alive) return;
    delete_cams_using_link(m, link_id);
    delete_joints_using_link(m, link_id);
    free(l->connector_ids);
    free(l->rest_dist);
    free(l->frozen_local_offset);
    l->connector_ids = NULL;
    l->rest_dist = NULL;
    l->frozen_local_offset = NULL;
    l->connector_count = 0;
    l->is_driven = false;
    l->pivot_connector_id = -1;
    l->selected = false;
    l->alive = false;
}

void mechanism_delete_connector(Mechanism *m, int connector_id) {
    if (connector_id < 0 || connector_id >= m->connector_count) return;
    if (!m->connectors[connector_id].alive) return;

    for (int li = 0; li < m->link_count; li++) {
        Link *l = &m->links[li];
        if (!l->alive) continue;
        for (int i = 0; i < l->connector_count; i++) {
            if (l->connector_ids[i] == connector_id) {
                mechanism_delete_link(m, li);
                break;
            }
        }
    }
    delete_cams_using_connector(m, connector_id);
    delete_joints_using_connector(m, connector_id);
    free(m->connectors[connector_id].path);
    free(m->connectors[connector_id].path_time);
    m->connectors[connector_id].path = NULL;
    m->connectors[connector_id].path_time = NULL;
    m->connectors[connector_id].path_count = 0;
    m->connectors[connector_id].path_capacity = 0;
    m->connectors[connector_id].traced = false;
    m->connectors[connector_id].alive = false;
    m->connectors[connector_id].selected = false;
}

int mechanism_add_link(Mechanism *m, const int *connector_ids, int count) {
    if (count < 2) return -1;
    for (int i = 0; i < count; i++) {
        int cid = connector_ids[i];
        if (cid < 0 || cid >= m->connector_count || !m->connectors[cid].alive) return -1;
    }

    m->links = grow(m->links, &m->link_capacity, m->link_count, sizeof(Link));
    Link *l = &m->links[m->link_count];

    l->connector_count = count;
    l->connector_ids = malloc((size_t)count * sizeof(int));
    memcpy(l->connector_ids, connector_ids, (size_t)count * sizeof(int));

    int npairs = count * (count - 1) / 2;
    l->rest_dist = malloc((size_t)npairs * sizeof(double));
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            l->rest_dist[mechanism_pair_index(i, j, count)] =
                vec2_dist(m->connectors[connector_ids[i]].pos, m->connectors[connector_ids[j]].pos);
        }
    }

    l->rigid = true;
    l->is_driven = false;
    l->pivot_connector_id = -1;
    l->motor_speed_deg_s = 0.0;
    l->accumulated_angle_rad = 0.0;
    l->frozen_local_offset = NULL;
    l->driven_externally = false;
    l->frozen_pivot_pos = (Vec2){ 0.0, 0.0 };
    l->wheel_radius = 0.0;
    l->selected = false;
    l->alive = true;

    return m->link_count++;
}

bool mechanism_set_driven_about(Mechanism *m, int link_id, int pivot_connector_id, double speed_deg_s) {
    if (link_id < 0 || link_id >= m->link_count) return false;
    Link *l = &m->links[link_id];
    if (!l->alive) return false;
    if (pivot_connector_id < 0 || pivot_connector_id >= m->connector_count) return false;
    if (!m->connectors[pivot_connector_id].alive) return false;

    bool on_link = false;
    for (int i = 0; i < l->connector_count; i++) {
        if (l->connector_ids[i] == pivot_connector_id) on_link = true;
    }
    if (!on_link) return false;

    free(l->frozen_local_offset);
    l->frozen_local_offset = NULL;
    l->is_driven = true;
    l->pivot_connector_id = pivot_connector_id;
    l->motor_speed_deg_s = speed_deg_s;
    l->accumulated_angle_rad = 0.0;
    return true;
}

void mechanism_set_rigid(Mechanism *m, int link_id, bool rigid) {
    if (link_id < 0 || link_id >= m->link_count) return;
    Link *l = &m->links[link_id];
    if (!l->alive) return;
    l->rigid = rigid;
}

bool mechanism_has_any_part(const Mechanism *m) {
    for (int i = 0; i < m->connector_count; i++) {
        if (m->connectors[i].alive) return true;
    }
    return false;
}

bool mechanism_has_driven_link(const Mechanism *m) {
    for (int li = 0; li < m->link_count; li++) {
        if (m->links[li].alive && m->links[li].is_driven) return true;
    }
    return false;
}

void mechanism_set_anchor(Mechanism *m, int connector_id, bool is_anchor) {
    if (connector_id < 0 || connector_id >= m->connector_count) return;
    if (!m->connectors[connector_id].alive) return;

    m->connectors[connector_id].is_anchor = is_anchor;
    if (is_anchor) return;

    for (int li = 0; li < m->link_count; li++) {
        Link *l = &m->links[li];
        if (l->alive && l->is_driven && l->pivot_connector_id == connector_id) {
            l->is_driven = false;
            l->pivot_connector_id = -1;
            free(l->frozen_local_offset);
            l->frozen_local_offset = NULL;
        }
    }
}

bool mechanism_toggle_driven(Mechanism *m, int link_id, double default_speed_deg_s) {
    if (link_id < 0 || link_id >= m->link_count) return false;
    Link *l = &m->links[link_id];
    if (!l->alive) return false;

    if (l->is_driven) {
        l->is_driven = false;
        l->pivot_connector_id = -1;
        free(l->frozen_local_offset);
        l->frozen_local_offset = NULL;
        return true;
    }

    int anchor_count = 0, pivot = -1;
    for (int i = 0; i < l->connector_count; i++) {
        int cid = l->connector_ids[i];
        if (m->connectors[cid].is_anchor) {
            anchor_count++;
            pivot = cid;
        }
    }
    if (anchor_count != 1) return false;

    free(l->frozen_local_offset);
    l->frozen_local_offset = NULL;
    l->is_driven = true;
    l->pivot_connector_id = pivot;
    l->motor_speed_deg_s = default_speed_deg_s;
    l->accumulated_angle_rad = 0.0;
    return true;
}

void mechanism_set_traced(Mechanism *m, int connector_id, bool traced) {
    if (connector_id < 0 || connector_id >= m->connector_count) return;
    Connector *c = &m->connectors[connector_id];
    if (!c->alive) return;

    c->traced = traced;
    if (!traced) {
        free(c->path);
        free(c->path_time);
        c->path = NULL;
        c->path_time = NULL;
        c->path_count = 0;
        c->path_capacity = 0;
    }
}

void mechanism_clear_traces(Mechanism *m) {
    for (int i = 0; i < m->connector_count; i++) {
        Connector *c = &m->connectors[i];
        if (c->alive && c->traced) c->path_count = 0;
    }
}

void mechanism_trace_step(Mechanism *m, double sim_time) {
    for (int i = 0; i < m->connector_count; i++) {
        Connector *c = &m->connectors[i];
        if (!c->alive || !c->traced) continue;
        /* At the cap, drop every other sample rather than the oldest ones: a
         * shorter, coarser record of the whole run is more use than a precise
         * record of the last few seconds of it. */
        if (c->path_count >= MECHANISM_TRACE_MAX) {
            int kept = 0;
            for (int k = 0; k < c->path_count; k += 2) {
                c->path[kept] = c->path[k];
                c->path_time[kept] = c->path_time[k];
                kept++;
            }
            c->path_count = kept;
        }
        /* path and path_time are parallel, so they share one capacity and
         * have to grow together. */
        if (c->path_count >= c->path_capacity) {
            int new_cap = (c->path_capacity == 0) ? 4 : c->path_capacity * 2;
            while (new_cap <= c->path_count) new_cap *= 2;
            c->path = realloc(c->path, (size_t)new_cap * sizeof(Vec2));
            c->path_time = realloc(c->path_time, (size_t)new_cap * sizeof(double));
            c->path_capacity = new_cap;
        }
        c->path_time[c->path_count] = sim_time;
        c->path[c->path_count++] = c->pos;
    }
}

static bool connector_ok(const Mechanism *m, int id) {
    return id >= 0 && id < m->connector_count && m->connectors[id].alive;
}

static bool link_ok(const Mechanism *m, int id) {
    return id >= 0 && id < m->link_count && m->links[id].alive;
}

static bool connector_on_link(const Mechanism *m, int link_id, int connector_id) {
    const Link *l = &m->links[link_id];
    for (int i = 0; i < l->connector_count; i++) {
        if (l->connector_ids[i] == connector_id) return true;
    }
    return false;
}

int mechanism_add_slider(Mechanism *m, int pin_connector_id, int rail_a_id, int rail_b_id) {
    if (!connector_ok(m, pin_connector_id) || !connector_ok(m, rail_a_id) || !connector_ok(m, rail_b_id)) return -1;
    if (pin_connector_id == rail_a_id || pin_connector_id == rail_b_id || rail_a_id == rail_b_id) return -1;
    /* A rail of zero length defines no direction to slide along. */
    if (vec2_dist(m->connectors[rail_a_id].pos, m->connectors[rail_b_id].pos) < 1e-6) return -1;

    m->sliders = grow(m->sliders, &m->slider_capacity, m->slider_count, sizeof(Slider));
    Slider *s = &m->sliders[m->slider_count];
    s->pin_connector_id = pin_connector_id;
    s->rail_a_id = rail_a_id;
    s->rail_b_id = rail_b_id;
    s->selected = false;
    s->alive = true;
    return m->slider_count++;
}

/* Shared by the gear and rack constructors: the driven body is posed rather
 * than solved, so it needs a pivot to be posed about and the flag that tells
 * the solver to leave its own distances alone. */
static Gear *begin_gear(Mechanism *m, int driven_link_id, int pivot_id) {
    m->gears = grow(m->gears, &m->gear_capacity, m->gear_count, sizeof(Gear));
    Gear *g = &m->gears[m->gear_count];
    m->links[driven_link_id].driven_externally = true;
    m->links[driven_link_id].pivot_connector_id = pivot_id;
    g->driver_ref_id = -1;
    g->frozen_driver_angle = 0.0;
    g->rack_axis = (Vec2){ 1.0, 0.0 };
    g->selected = false;
    g->alive = true;
    return g;
}

int mechanism_add_gear(Mechanism *m, int driver_link_id, int driver_center_id,
                        int driven_link_id, int driven_center_id, double ratio, bool internal) {
    if (!link_ok(m, driver_link_id) || !link_ok(m, driven_link_id)) return -1;
    if (driver_link_id == driven_link_id) return -1;
    if (!connector_ok(m, driver_center_id) || !connector_ok(m, driven_center_id)) return -1;
    if (!connector_on_link(m, driver_link_id, driver_center_id)) return -1;
    if (!connector_on_link(m, driven_link_id, driven_center_id)) return -1;
    if (!(ratio > 1e-4)) return -1;
    if (vec2_dist(m->connectors[driver_center_id].pos, m->connectors[driven_center_id].pos) < 1e-6) return -1;

    Gear *g = begin_gear(m, driven_link_id, driven_center_id);
    g->kind = internal ? GEAR_INTERNAL : GEAR_EXTERNAL;
    g->driver_link_id = driver_link_id;
    g->driver_center_id = driver_center_id;
    g->driven_link_id = driven_link_id;
    g->driven_center_id = driven_center_id;
    g->ratio = ratio;
    g->driver_radius = 1.0;
    g->driven_radius = ratio;
    int id = m->gear_count++;
    mechanism_refresh_joint_sizes(m);
    return id;
}

int mechanism_add_rack(Mechanism *m, int driver_link_id, int driver_center_id,
                        int rack_link_id, int rack_reference_id, Vec2 axis) {
    if (!link_ok(m, driver_link_id) || !link_ok(m, rack_link_id)) return -1;
    if (driver_link_id == rack_link_id) return -1;
    if (!connector_ok(m, driver_center_id) || !connector_ok(m, rack_reference_id)) return -1;
    if (!connector_on_link(m, driver_link_id, driver_center_id)) return -1;
    if (!connector_on_link(m, rack_link_id, rack_reference_id)) return -1;
    double len = vec2_len(axis);
    if (len < 1e-9) return -1;

    Gear *g = begin_gear(m, rack_link_id, rack_reference_id);
    g->kind = GEAR_RACK;
    g->driver_link_id = driver_link_id;
    g->driver_center_id = driver_center_id;
    g->driven_link_id = rack_link_id;
    g->driven_center_id = rack_reference_id;
    g->ratio = 1.0;
    g->driver_radius = 1.0;
    g->driven_radius = 0.0;
    g->rack_axis = vec2_scale(axis, 1.0 / len);
    int id = m->gear_count++;
    mechanism_refresh_joint_sizes(m);
    return id;
}

int mechanism_add_geneva(Mechanism *m, int driver_link_id, int driver_center_id,
                          int wheel_link_id, int wheel_center_id, int slot_count) {
    if (!link_ok(m, driver_link_id) || !link_ok(m, wheel_link_id)) return -1;
    if (driver_link_id == wheel_link_id) return -1;
    if (!connector_ok(m, driver_center_id) || !connector_ok(m, wheel_center_id)) return -1;
    if (!connector_on_link(m, driver_link_id, driver_center_id)) return -1;
    if (!connector_on_link(m, wheel_link_id, wheel_center_id)) return -1;
    if (slot_count < 3) return -1;

    double centres = vec2_dist(m->connectors[driver_center_id].pos, m->connectors[wheel_center_id].pos);
    if (centres < 1e-6) return -1;

    m->genevas = grow(m->genevas, &m->geneva_capacity, m->geneva_count, sizeof(Geneva));
    Geneva *gv = &m->genevas[m->geneva_count];
    m->links[wheel_link_id].driven_externally = true;
    m->links[wheel_link_id].pivot_connector_id = wheel_center_id;

    gv->driver_link_id = driver_link_id;
    gv->driver_center_id = driver_center_id;
    gv->wheel_link_id = wheel_link_id;
    gv->wheel_center_id = wheel_center_id;
    gv->slot_count = slot_count;
    gv->center_distance = centres;
    /* The shock-free proportion: the pin must enter and leave along the slot. */
    gv->crank_radius = geneva_crank_radius(slot_count, centres);
    gv->driver_ref_id = -1;
    gv->frozen_driver_angle = 0.0;
    gv->engaged = false;
    gv->selected = false;
    gv->alive = true;
    return m->geneva_count++;
}

void mechanism_refresh_joint_sizes(Mechanism *m) {
    for (int gi = 0; gi < m->gear_count; gi++) {
        Gear *g = &m->gears[gi];
        if (!g->alive) continue;
        if (!connector_ok(m, g->driver_center_id)) continue;

        if (g->kind == GEAR_RACK) {
            /* A pinion's pitch radius is how far its centre stands off the
             * rack's line -- so sliding the rack nearer makes a smaller
             * pinion, and one turn moves the rack less. */
            if (!link_ok(m, g->driven_link_id)) continue;
            const Link *bar = &m->links[g->driven_link_id];
            if (bar->connector_count < 2) continue;
            Vec2 b0 = m->connectors[bar->connector_ids[0]].pos;
            Vec2 b1 = m->connectors[bar->connector_ids[1]].pos;
            Vec2 d = vec2_sub(b1, b0);
            double len = vec2_len(d);
            if (len < 1e-9) continue;
            double r = fabs(vec2_cross(d, vec2_sub(m->connectors[g->driver_center_id].pos, b0)) / len);
            if (r > 1e-6) {
                g->driver_radius = r;
                if (link_ok(m, g->driver_link_id)) m->links[g->driver_link_id].wheel_radius = r;
            }
            continue;
        }

        if (!connector_ok(m, g->driven_center_id)) continue;

        /* Each wheel is its own size, so the mesh takes its radii from the two
         * bodies rather than dividing up the gap between them. */
        double ra = mechanism_gear_wheel_radius(m, g->driver_link_id, NULL);
        double rb = mechanism_gear_wheel_radius(m, g->driven_link_id, NULL);
        if (!(ra > 1e-6) || !(rb > 1e-6)) continue;
        g->driver_radius = ra;
        g->driven_radius = rb;
        g->ratio = rb / ra;

        /* Two wheels of fixed size mesh at exactly one distance, so the driven
         * one slides along the line of centres until they touch. Dragging it
         * therefore swings it round its driver instead of pulling the teeth
         * apart, and resizing either wheel closes the gap again by itself. */
        Vec2 hub = m->connectors[g->driver_center_id].pos;
        Vec2 at = m->connectors[g->driven_center_id].pos;
        Vec2 away = vec2_sub(at, hub);
        double d = vec2_len(away);
        Vec2 u = (d > 1e-9) ? vec2_scale(away, 1.0 / d) : (Vec2){ 1.0, 0.0 };
        double want = ra + rb;
        if (fabs(d - want) > 1e-9) {
            Vec2 shift = vec2_sub(vec2_add(hub, vec2_scale(u, want)), at);
            const Link *wheel = &m->links[g->driven_link_id];
            for (int i = 0; i < wheel->connector_count; i++) {
                Connector *cn = &m->connectors[wheel->connector_ids[i]];
                cn->pos = vec2_add(cn->pos, shift);
                cn->prev_pos = vec2_add(cn->prev_pos, shift);
            }
        }
    }

    for (int vi = 0; vi < m->geneva_count; vi++) {
        Geneva *gv = &m->genevas[vi];
        if (!gv->alive) continue;
        if (!connector_ok(m, gv->driver_center_id) || !connector_ok(m, gv->wheel_center_id)) continue;
        double d = vec2_dist(m->connectors[gv->driver_center_id].pos,
                              m->connectors[gv->wheel_center_id].pos);
        if (d < 1e-6) continue;
        gv->center_distance = d;
        gv->crank_radius = geneva_crank_radius(gv->slot_count, d);
    }
}

double mechanism_body_rotation(const Mechanism *m, int center_id, int ref_id, double frozen_angle) {
    if (!connector_ok(m, center_id) || !connector_ok(m, ref_id)) return 0.0;
    Vec2 d = vec2_sub(m->connectors[ref_id].pos, m->connectors[center_id].pos);
    if (vec2_len(d) < 1e-12) return 0.0;
    return atan2(d.y, d.x) - frozen_angle;
}

int mechanism_pick_connector(const Mechanism *m, Vec2 p, double radius) {
    int best = -1;
    double best_d2 = radius * radius;
    for (int i = 0; i < m->connector_count; i++) {
        if (!m->connectors[i].alive) continue;
        double d2 = vec2_dist2(m->connectors[i].pos, p);
        if (d2 <= best_d2) {
            best_d2 = d2;
            best = i;
        }
    }
    return best;
}

static double point_segment_dist(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 ab = vec2_sub(b, a);
    double len2 = vec2_dot(ab, ab);
    double t = (len2 > 1e-12) ? vec2_dot(vec2_sub(p, a), ab) / len2 : 0.0;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    Vec2 proj = vec2_add(a, vec2_scale(ab, t));
    return vec2_dist(p, proj);
}

int mechanism_add_cam(Mechanism *m, int body_link_id, int center_connector_id, int follower_connector_id) {
    if (body_link_id < 0 || body_link_id >= m->link_count || !m->links[body_link_id].alive) return -1;
    if (center_connector_id < 0 || center_connector_id >= m->connector_count) return -1;
    if (follower_connector_id < 0 || follower_connector_id >= m->connector_count) return -1;
    if (center_connector_id == follower_connector_id) return -1;
    if (!m->connectors[center_connector_id].alive || !m->connectors[follower_connector_id].alive) return -1;

    /* The centre has to belong to the body link, or "turning with it" is
     * meaningless. */
    const Link *l = &m->links[body_link_id];
    bool center_on_link = false;
    for (int i = 0; i < l->connector_count; i++) {
        if (l->connector_ids[i] == center_connector_id) center_on_link = true;
    }
    if (!center_on_link) return -1;

    Vec2 centre = m->connectors[center_connector_id].pos;
    Vec2 follower = m->connectors[follower_connector_id].pos;
    double reach = vec2_dist(centre, follower);
    if (reach < 1e-6) return -1; /* no axis direction to be had */

    m->cams = grow(m->cams, &m->cam_capacity, m->cam_count, sizeof(Cam));
    Cam *c = &m->cams[m->cam_count];

    /* Size the cam so the follower starts sitting on the profile: the
     * follower's current distance is the pitch radius at the low dwell. */
    cam_set_defaults(c, reach);
    c->body_link_id = body_link_id;
    c->center_connector_id = center_connector_id;
    c->follower_connector_id = follower_connector_id;
    c->axis_origin = centre;
    c->axis_dir = vec2_scale(vec2_sub(follower, centre), 1.0 / reach);

    return m->cam_count++;
}

double mechanism_cam_angle(const Mechanism *m, int cam_id) {
    if (cam_id < 0 || cam_id >= m->cam_count) return 0.0;
    const Cam *c = &m->cams[cam_id];
    if (!c->alive || c->ref_connector_id < 0 || c->center_connector_id < 0) return 0.0;
    if (!m->connectors[c->ref_connector_id].alive) return 0.0;

    Vec2 d = vec2_sub(m->connectors[c->ref_connector_id].pos, m->connectors[c->center_connector_id].pos);
    if (vec2_len(d) < 1e-12) return 0.0;
    return atan2(d.y, d.x) - c->frozen_ref_angle;
}


/* If `link_id` is a gear wheel, its pitch radius and the connector at its
 * centre; 0 and -1 if it is not one. A wheel can appear as the driver of one
 * mesh and the driven of another, so both sides are checked. A rack's bar is
 * not a wheel. */
/* ---------------------------------------------------------------------------
 * Gear wheels
 *
 * A wheel is a body with a size of its own. It is placed on its own, resized
 * on its own, and only meshes with another wheel when you say so -- which is
 * what lets one wheel drive several, and what stops a button press from ever
 * dropping a wheel on top of one already there.
 * ------------------------------------------------------------------------ */

/* Re-reads a link's rest distances from where its connectors are now. Needed
 * whenever geometry is changed by something other than the user dragging --
 * resizing a wheel moves its rim mark, and the body must still be rigid. */
void mechanism_refresh_link_rest_lengths(Mechanism *m, int link_id) {
    if (link_id < 0 || link_id >= m->link_count) return;
    Link *l = &m->links[link_id];
    if (!l->alive) return;
    int count = l->connector_count;
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            l->rest_dist[mechanism_pair_index(i, j, count)] =
                vec2_dist(m->connectors[l->connector_ids[i]].pos,
                           m->connectors[l->connector_ids[j]].pos);
        }
    }
}

double mechanism_gear_wheel_radius(const Mechanism *m, int link_id, int *center_out) {
    if (center_out) *center_out = -1;
    if (link_id < 0 || link_id >= m->link_count) return 0.0;
    const Link *l = &m->links[link_id];
    if (!l->alive || !(l->wheel_radius > 0.0)) return 0.0;
    if (center_out) {
        /* The centre is the anchor it turns about; failing that, its pivot. */
        int centre = (l->pivot_connector_id >= 0) ? l->pivot_connector_id : -1;
        if (centre < 0) {
            for (int i = 0; i < l->connector_count; i++) {
                if (m->connectors[l->connector_ids[i]].is_anchor) { centre = l->connector_ids[i]; break; }
            }
        }
        if (centre < 0 && l->connector_count > 0) centre = l->connector_ids[0];
        *center_out = centre;
    }
    return l->wheel_radius;
}

bool mechanism_is_gear_body(const Mechanism *m, int link_id) {
    return link_id >= 0 && link_id < m->link_count &&
            m->links[link_id].alive && m->links[link_id].wheel_radius > 0.0;
}

int mechanism_wheel_at_connector(const Mechanism *m, int connector_id) {
    for (int li = 0; li < m->link_count; li++) {
        const Link *l = &m->links[li];
        if (!l->alive || !(l->wheel_radius > 0.0)) continue;
        for (int i = 0; i < l->connector_count; i++) {
            if (l->connector_ids[i] == connector_id) return li;
        }
    }
    return -1;
}

/* The mark on the wheel's rim: the point whose motion you can trace. It sits
 * ON the pitch circle, not somewhere inside it, so tracing a wheel draws the
 * circle the wheel actually is rather than a smaller one of no significance. */
int mechanism_wheel_mark(const Mechanism *m, int link_id) {
    int centre = -1;
    if (!(mechanism_gear_wheel_radius(m, link_id, &centre) > 0.0)) return -1;
    const Link *l = &m->links[link_id];
    for (int i = 0; i < l->connector_count; i++) {
        if (l->connector_ids[i] != centre) return l->connector_ids[i];
    }
    return -1;
}

int mechanism_add_wheel(Mechanism *m, Vec2 centre, double radius) {
    if (!(radius > 1e-6)) return -1;
    int c = mechanism_add_connector(m, centre, true);
    int mark = mechanism_add_connector(m, (Vec2){ centre.x, centre.y - radius }, false);
    int ids[2] = { c, mark };
    int link = mechanism_add_link(m, ids, 2);
    if (link < 0) return -1;
    m->links[link].wheel_radius = radius;
    return link;
}

void mechanism_set_wheel_radius(Mechanism *m, int link_id, double radius) {
    int centre = -1;
    if (!(mechanism_gear_wheel_radius(m, link_id, &centre) > 0.0) || centre < 0) return;
    if (radius < MECHANISM_WHEEL_MIN_RADIUS) radius = MECHANISM_WHEEL_MIN_RADIUS;
    if (radius > MECHANISM_WHEEL_MAX_RADIUS) radius = MECHANISM_WHEEL_MAX_RADIUS;
    m->links[link_id].wheel_radius = radius;

    /* The mark rides the rim, so it moves out (or in) with the new radius. */
    int mark = mechanism_wheel_mark(m, link_id);
    if (mark >= 0) {
        Vec2 d = vec2_sub(m->connectors[mark].pos, m->connectors[centre].pos);
        double len = vec2_len(d);
        Vec2 u = (len > 1e-9) ? vec2_scale(d, 1.0 / len) : (Vec2){ 0.0, -1.0 };
        m->connectors[mark].pos = vec2_add(m->connectors[centre].pos, vec2_scale(u, radius));
        m->connectors[mark].prev_pos = m->connectors[mark].pos;
        mechanism_refresh_link_rest_lengths(m, link_id);
    }
}

/* Turns a mesh round, so what was the driven wheel now drives. */
static void gear_swap_sides(Gear *g) {
    int l = g->driver_link_id; g->driver_link_id = g->driven_link_id; g->driven_link_id = l;
    int c = g->driver_center_id; g->driver_center_id = g->driven_center_id; g->driven_center_id = c;
    double r = g->driver_radius; g->driver_radius = g->driven_radius; g->driven_radius = r;
    g->ratio = (g->driver_radius > 1e-9) ? g->driven_radius / g->driver_radius : 1.0;
}

bool mechanism_orient_train_from(Mechanism *m, int root_link) {
    if (!mechanism_is_gear_body(m, root_link)) return false;
    if (m->link_count <= 0) return false;

    /* Drive flows outwards from the wheel you chose: walk the meshes away from
     * it and turn round any that were pointing the other way. Without this,
     * naming a wheel as the driver would only work if you happened to have
     * meshed the train in the right order to begin with. */
    bool *seen = calloc((size_t)m->link_count, sizeof(bool));
    int *queue = malloc((size_t)m->link_count * sizeof(int));
    if (!seen || !queue) { free(seen); free(queue); return false; }

    int head = 0, tail = 0;
    seen[root_link] = true;
    queue[tail++] = root_link;
    m->links[root_link].driven_externally = false;

    while (head < tail) {
        int cur = queue[head++];
        for (int i = 0; i < m->gear_count; i++) {
            Gear *g = &m->gears[i];
            if (!g->alive || g->kind == GEAR_RACK) continue;
            int other;
            if (g->driver_link_id == cur) other = g->driven_link_id;
            else if (g->driven_link_id == cur) other = g->driver_link_id;
            else continue;
            if (other < 0 || other >= m->link_count || seen[other]) continue;

            if (g->driver_link_id != cur) gear_swap_sides(g);
            m->links[other].driven_externally = true;
            m->links[other].is_driven = false;      /* only one wheel drives */
            seen[other] = true;
            queue[tail++] = other;
        }
    }
    free(seen);
    free(queue);
    return true;
}

bool mechanism_wheels_are_meshed(const Mechanism *m, int link_a, int link_b) {
    for (int i = 0; i < m->gear_count; i++) {
        const Gear *g = &m->gears[i];
        if (!g->alive) continue;
        if ((g->driver_link_id == link_a && g->driven_link_id == link_b) ||
            (g->driver_link_id == link_b && g->driven_link_id == link_a)) return true;
    }
    return false;
}

int mechanism_mesh_wheels(Mechanism *m, int driver_link, int driven_link) {
    if (driver_link == driven_link) return -1;
    int ca = -1, cb = -1;
    double ra = mechanism_gear_wheel_radius(m, driver_link, &ca);
    double rb = mechanism_gear_wheel_radius(m, driven_link, &cb);
    if (!(ra > 0.0) || !(rb > 0.0) || ca < 0 || cb < 0) return -1;
    if (mechanism_wheels_are_meshed(m, driver_link, driven_link)) return -1;
    /* A wheel takes its motion from one place: meshing a second driver onto it
     * would be two answers to the same question. */
    if (m->links[driven_link].driven_externally || m->links[driven_link].is_driven) return -1;

    int gid = mechanism_add_gear(m, driver_link, ca, driven_link, cb, rb / ra, false);
    if (gid >= 0) mechanism_refresh_joint_sizes(m);   /* slides the driven wheel into contact */
    return gid;
}

double mechanism_geneva_wheel_radius(const Geneva *gv) {
    /* At entry the crank stands perpendicular to the slot, so the pin, the
     * driver's centre and the wheel's centre make a right triangle: the slot
     * mouth is sqrt(d^2 - a^2) from the wheel's centre, which is the rim. (Not
     * d - a: that is the gap along the line of centres, and it collapses to
     * nothing for a four-slot wheel.) */
    double d = gv->center_distance, a = gv->crank_radius;
    double inner = d * d - a * a;
    return (inner > 0.0) ? sqrt(inner) : 0.0;
}

/* How far p is from a circle's edge, not its middle. */
static double dist_to_circle(Vec2 p, Vec2 centre, double radius) {
    return fabs(vec2_dist(p, centre) - radius);
}

int mechanism_pick_wheel(const Mechanism *m, Vec2 p, double dist_thresh) {
    int best = -1;
    double best_d = dist_thresh;
    for (int li = 0; li < m->link_count; li++) {
        int centre = -1;
        double r = mechanism_gear_wheel_radius(m, li, &centre);
        if (!(r > 0.0) || centre < 0 || !m->connectors[centre].alive) continue;
        /* A wheel is a disc, so anywhere on its face counts -- having to land
         * on the rim is exactly the fiddliness that made gears hard to place.
         * Distance is to the edge, so where wheels overlap the nearer wins. */
        double from_centre = vec2_dist(p, m->connectors[centre].pos);
        double d = fabs(from_centre - r);
        if (from_centre < r && d > dist_thresh) d = dist_thresh;
        if (d <= best_d) { best_d = d; best = li; }
    }
    return best;
}

int mechanism_pick_gear(const Mechanism *m, Vec2 p, double dist_thresh) {
    /* Only a rack's pitch line is left to pick here: a wheel is picked as the
     * body it is, by mechanism_pick_wheel. */
    int best = -1;
    double best_d = dist_thresh;
    for (int i = 0; i < m->gear_count; i++) {
        const Gear *g = &m->gears[i];
        if (!g->alive || g->kind != GEAR_RACK) continue;
        const Link *bar = &m->links[g->driven_link_id];
        if (!bar->alive || bar->connector_count < 2) continue;
        double d = point_segment_dist(p, m->connectors[bar->connector_ids[0]].pos,
                                          m->connectors[bar->connector_ids[1]].pos);
        if (d <= best_d) { best_d = d; best = i; }
    }
    return best;
}

int mechanism_pick_geneva(const Mechanism *m, Vec2 p, double dist_thresh) {
    int best = -1;
    double best_d = dist_thresh;
    for (int i = 0; i < m->geneva_count; i++) {
        const Geneva *gv = &m->genevas[i];
        if (!gv->alive) continue;
        double d = 1e30;
        if (m->connectors[gv->wheel_center_id].alive) {
            d = dist_to_circle(p, m->connectors[gv->wheel_center_id].pos,
                                mechanism_geneva_wheel_radius(gv));
        }
        if (m->connectors[gv->driver_center_id].alive) {
            /* The locking disc, drawn just touching the wheel's rim. */
            double dd = dist_to_circle(p, m->connectors[gv->driver_center_id].pos,
                                        gv->center_distance - mechanism_geneva_wheel_radius(gv));
            if (dd < d) d = dd;
        }
        if (d <= best_d) { best_d = d; best = i; }
    }
    return best;
}

int mechanism_pick_slider(const Mechanism *m, Vec2 p, double dist_thresh) {
    int best = -1;
    double best_d = dist_thresh;
    for (int i = 0; i < m->slider_count; i++) {
        const Slider *sl = &m->sliders[i];
        if (!sl->alive) continue;
        if (!m->connectors[sl->rail_a_id].alive || !m->connectors[sl->rail_b_id].alive) continue;
        double d = point_segment_dist(p, m->connectors[sl->rail_a_id].pos,
                                          m->connectors[sl->rail_b_id].pos);
        if (d <= best_d) { best_d = d; best = i; }
    }
    return best;
}

int mechanism_pick_cam(const Mechanism *m, Vec2 p, double dist_thresh) {
    int best = -1;
    double best_d = dist_thresh;
    Vec2 pts[CAM_PICK_SAMPLES];

    for (int i = 0; i < m->cam_count; i++) {
        const Cam *c = &m->cams[i];
        if (!c->alive || c->center_connector_id < 0) continue;
        if (!m->connectors[c->center_connector_id].alive) continue;

        Vec2 centre = m->connectors[c->center_connector_id].pos;
        double angle = mechanism_cam_angle(m, i);
        cam_sample_surface(c, pts, CAM_PICK_SAMPLES);
        for (int k = 0; k < CAM_PICK_SAMPLES; k++) {
            Vec2 a = vec2_add(centre, vec2_rotate(pts[k], angle));
            Vec2 b = vec2_add(centre, vec2_rotate(pts[(k + 1) % CAM_PICK_SAMPLES], angle));
            double d = point_segment_dist(p, a, b);
            if (d <= best_d) {
                best_d = d;
                best = i;
            }
        }
    }
    return best;
}

int mechanism_pick_link_edge(const Mechanism *m, Vec2 p, double dist_thresh) {
    int best = -1;
    double best_d = dist_thresh;
    for (int li = 0; li < m->link_count; li++) {
        const Link *l = &m->links[li];
        if (!l->alive) continue;
        /* A gear wheel has no bar drawn across it, so there is none to click:
         * an invisible edge that still answers to the mouse is worse than no
         * edge at all. Reach a wheel by its centre or its pitch circle. */
        if (mechanism_is_gear_body(m, li)) continue;
        for (int i = 0; i < l->connector_count; i++) {
            for (int j = i + 1; j < l->connector_count; j++) {
                Vec2 a = m->connectors[l->connector_ids[i]].pos;
                Vec2 b = m->connectors[l->connector_ids[j]].pos;
                double d = point_segment_dist(p, a, b);
                if (d <= best_d) {
                    best_d = d;
                    best = li;
                }
            }
        }
    }
    return best;
}
