#include "mechanism.h"

#include <stdlib.h>
#include <string.h>

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
}

void mechanism_free(Mechanism *m) {
    for (int i = 0; i < m->connector_count; i++) {
        free(m->connectors[i].path);
    }
    for (int i = 0; i < m->link_count; i++) {
        free(m->links[i].connector_ids);
        free(m->links[i].rest_dist);
        free(m->links[i].frozen_local_offset);
    }
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
            d->path = malloc((size_t)src->connectors[i].path_count * sizeof(Vec2));
            memcpy(d->path, src->connectors[i].path, (size_t)src->connectors[i].path_count * sizeof(Vec2));
            d->path_capacity = src->connectors[i].path_count;
        } else {
            d->path = NULL;
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
    c->path_count = 0;
    c->path_capacity = 0;
    c->alive = true;
    return m->connector_count++;
}

void mechanism_delete_link(Mechanism *m, int link_id) {
    if (link_id < 0 || link_id >= m->link_count) return;
    Link *l = &m->links[link_id];
    if (!l->alive) return;
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
    free(m->connectors[connector_id].path);
    m->connectors[connector_id].path = NULL;
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
    l->selected = false;
    l->alive = true;

    return m->link_count++;
}

void mechanism_set_rigid(Mechanism *m, int link_id, bool rigid) {
    if (link_id < 0 || link_id >= m->link_count) return;
    Link *l = &m->links[link_id];
    if (!l->alive) return;
    l->rigid = rigid;
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
        c->path = NULL;
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

void mechanism_trace_step(Mechanism *m) {
    for (int i = 0; i < m->connector_count; i++) {
        Connector *c = &m->connectors[i];
        if (!c->alive || !c->traced) continue;
        c->path = grow(c->path, &c->path_capacity, c->path_count, sizeof(Vec2));
        c->path[c->path_count++] = c->pos;
    }
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

int mechanism_pick_link_edge(const Mechanism *m, Vec2 p, double dist_thresh) {
    int best = -1;
    double best_d = dist_thresh;
    for (int li = 0; li < m->link_count; li++) {
        const Link *l = &m->links[li];
        if (!l->alive) continue;
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
