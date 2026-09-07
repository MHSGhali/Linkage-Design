#include "solver.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "linalg.h"
#include "xalloc.h"

/* Two kinds of constraint share one least-squares problem.
 *
 * RES_PAIR is the original: a squared-distance residual between two
 * connectors, in units of length^2. RES_AXIS keeps a cam's airborne follower
 * on its guide axis, and is naturally linear -- a perpendicular offset, in
 * units of length. Mixing the two unscaled would be a mistake: the single
 * Levenberg damping term is scaled by the largest diagonal of JtJ across the
 * whole problem, so a residual an order of magnitude smaller in its natural
 * units would simply be ignored. RES_AXIS is therefore multiplied by a
 * characteristic length of the mechanism, putting both kinds in length^2.
 *
 * RES_RAIL is the same idea for a slider or pin-in-slot: a pin held on the
 * line through two other connectors. Unlike RES_AXIS the line can itself be
 * moving -- that is what makes it a pin running in a slot rather than a
 * prismatic joint on ground -- so the rail's own endpoints get Jacobian
 * entries too. */
typedef enum { RES_PAIR, RES_AXIS, RES_RAIL } ResidualKind;

typedef struct {
    ResidualKind kind;
    int ci, cj;        /* RES_PAIR: both connectors. RES_AXIS: ci only. */
    double rest;       /* RES_PAIR */
    Vec2 axis_point;   /* RES_AXIS: a point on the axis (the cam centre) */
    Vec2 axis_normal;  /* RES_AXIS: unit normal to the axis */
    int rail_a, rail_b; /* RES_RAIL: the two connectors defining the line */
    double scale;      /* RES_AXIS / RES_RAIL: characteristic length */
} Residual;

/* Largest rest distance anywhere in the mechanism -- the natural length scale
 * for making the two residual kinds dimensionally comparable. */
static double characteristic_length(const Mechanism *m) {
    double best = 0.0;
    for (int li = 0; li < m->link_count; li++) {
        const Link *l = &m->links[li];
        if (!l->alive) continue;
        int k = l->connector_count;
        for (int i = 0; i < k; i++) {
            for (int j = i + 1; j < k; j++) {
                double d = l->rest_dist[mechanism_pair_index(i, j, k)];
                if (d > best) best = d;
            }
        }
    }
    for (int ci = 0; ci < m->cam_count; ci++) {
        const Cam *c = &m->cams[ci];
        if (c->alive && c->base_radius + c->lift > best) best = c->base_radius + c->lift;
    }
    return (best > 1e-9) ? best : 1.0;
}

/* The cam's follower axis: a line through the cam's CURRENT centre along the
 * direction frozen at run start. */
static Vec2 cam_center_pos(const Mechanism *m, const Cam *c) {
    return m->connectors[c->center_connector_id].pos;
}

static bool cam_is_usable(const Mechanism *m, const Cam *c) {
    if (!c->alive) return false;
    if (c->center_connector_id < 0 || c->center_connector_id >= m->connector_count) return false;
    if (c->follower_connector_id < 0 || c->follower_connector_id >= m->connector_count) return false;
    return m->connectors[c->center_connector_id].alive && m->connectors[c->follower_connector_id].alive;
}

/* How far along its axis the follower currently sits, measured from the cam
 * centre. This is exactly the pitch radius whenever the two are touching. */
static double follower_axis_position(const Mechanism *m, const Cam *c) {
    Vec2 d = vec2_sub(m->connectors[c->follower_connector_id].pos, cam_center_pos(m, c));
    return vec2_dot(d, c->axis_dir);
}

/* The cam-local angle the follower axis currently points down. */
static double follower_local_angle(const Mechanism *m, const Cam *c, int cam_id) {
    return atan2(c->axis_dir.y, c->axis_dir.x) - mechanism_cam_angle(m, cam_id);
}

/* Picks the connector of `link` farthest from `centre` to read the body's
 * rotation from -- the longest lever gives the least noisy angle. */
static int farthest_reference(const Mechanism *m, int link_id, int center_id) {
    const Link *l = &m->links[link_id];
    Vec2 centre = m->connectors[center_id].pos;
    int ref = -1;
    double best = 0.0;
    for (int i = 0; i < l->connector_count; i++) {
        int cid = l->connector_ids[i];
        if (cid == center_id) continue;
        double d = vec2_dist(m->connectors[cid].pos, centre);
        if (d > best) { best = d; ref = cid; }
    }
    return ref;
}

SolverParams solver_default_params(void) {
    SolverParams p;
    p.max_iters = 30;
    p.tol = 1e-6;
    p.lambda_init = 1e-3;
    p.damping_floor = 1e-9;
    p.gravity = (Vec2){ 0.0, 0.0 };
    /* Deliberately tight: a solve that actually converges lands within
     * ~1e-9 of the rest length, so anything approaching a visible fraction
     * of a unit means the solver is straining against geometry it cannot
     * satisfy. */
    p.length_tol_abs = 0.02;
    p.length_tol_rel = 0.0001;
    return p;
}

void solver_freeze(Mechanism *m) {
    mechanism_refresh_joint_sizes(m);
    for (int i = 0; i < m->connector_count; i++) {
        m->connectors[i].prev_pos = m->connectors[i].pos;
    }

    for (int li = 0; li < m->link_count; li++) {
        Link *l = &m->links[li];
        if (!l->alive) continue;

        int k = l->connector_count;
        for (int i = 0; i < k; i++) {
            for (int j = i + 1; j < k; j++) {
                Vec2 pi = m->connectors[l->connector_ids[i]].pos;
                Vec2 pj = m->connectors[l->connector_ids[j]].pos;
                l->rest_dist[mechanism_pair_index(i, j, k)] = vec2_dist(pi, pj);
            }
        }

        if ((l->is_driven || l->driven_externally) &&
            l->pivot_connector_id >= 0 && l->pivot_connector_id < m->connector_count) {
            free(l->frozen_local_offset);
            l->frozen_local_offset = xmalloc((size_t)k * sizeof(Vec2));
            Vec2 pivot_pos = m->connectors[l->pivot_connector_id].pos;
            for (int i = 0; i < k; i++) {
                l->frozen_local_offset[i] = vec2_sub(m->connectors[l->connector_ids[i]].pos, pivot_pos);
            }
            /* A rack translates from where it started rather than turning
             * about a pivot, so remember where that was. */
            l->frozen_pivot_pos = pivot_pos;
            l->accumulated_angle_rad = 0.0;
        }
    }

    for (int ci = 0; ci < m->cam_count; ci++) {
        Cam *c = &m->cams[ci];
        if (!cam_is_usable(m, c)) continue;
        if (c->body_link_id < 0 || c->body_link_id >= m->link_count) continue;
        Vec2 centre = m->connectors[c->center_connector_id].pos;

        int ref = farthest_reference(m, c->body_link_id, c->center_connector_id);
        c->ref_connector_id = ref;
        if (ref >= 0) {
            Vec2 d = vec2_sub(m->connectors[ref].pos, centre);
            c->frozen_ref_angle = atan2(d.y, d.x);
        } else {
            c->frozen_ref_angle = 0.0;
        }

        /* The follower axis is radial: the line through the centre and the
         * follower's authored position. */
        Vec2 f = vec2_sub(m->connectors[c->follower_connector_id].pos, centre);
        double flen = vec2_len(f);
        c->axis_origin = centre;
        if (flen > 1e-9) c->axis_dir = vec2_scale(f, 1.0 / flen);
        c->last_angle = 0.0;
        c->in_contact = false;
    }

    for (int gi = 0; gi < m->gear_count; gi++) {
        Gear *g = &m->gears[gi];
        if (!g->alive || g->driver_link_id < 0 || g->driver_center_id < 0) continue;
        g->driver_ref_id = farthest_reference(m, g->driver_link_id, g->driver_center_id);
        g->frozen_driver_angle = 0.0;
        if (g->driver_ref_id >= 0) {
            Vec2 d = vec2_sub(m->connectors[g->driver_ref_id].pos, m->connectors[g->driver_center_id].pos);
            g->frozen_driver_angle = atan2(d.y, d.x);
        }
    }

    for (int vi = 0; vi < m->geneva_count; vi++) {
        Geneva *gv = &m->genevas[vi];
        if (!gv->alive || gv->driver_link_id < 0 || gv->driver_center_id < 0) continue;
        gv->driver_ref_id = farthest_reference(m, gv->driver_link_id, gv->driver_center_id);
        gv->frozen_driver_angle = 0.0;
        if (gv->driver_ref_id >= 0) {
            Vec2 d = vec2_sub(m->connectors[gv->driver_ref_id].pos, m->connectors[gv->driver_center_id].pos);
            gv->frozen_driver_angle = atan2(d.y, d.x);
        }
        gv->center_distance = vec2_dist(m->connectors[gv->driver_center_id].pos,
                                         m->connectors[gv->wheel_center_id].pos);
        gv->crank_radius = geneva_crank_radius(gv->slot_count, gv->center_distance);
        gv->engaged = false;
    }
}

/* How far a body has turned since the run started -- the UNWRAPPED total, not
 * an angle.
 *
 * That distinction matters: a Geneva counts whole turns of its driver, and a
 * gear ratio multiplies them. Reading the angle back off the geometry with
 * atan2 wraps at half a turn, which would make the driven body jump every time
 * its driver passed the wrap point.
 *
 * A motor keeps exactly the number wanted in its accumulated angle. A body
 * that is itself posed by a gear or a Geneva does not, but its driver's total
 * is knowable the same way, so the chain is followed back to whatever motor is
 * at the end of it. That is what lets a gear drive a gear that drives another
 * without the last one stuttering. Anything else -- a body moved by a linkage
 * rather than by a motor -- falls back to the measured angle, which is right so
 * long as it does not itself go right round. */
static double body_total_rotation(const Mechanism *m, int link_id, int center_id,
                                   int ref_id, double frozen_angle, int depth) {
    if (link_id >= 0 && link_id < m->link_count && depth < 32) {
        const Link *l = &m->links[link_id];
        if (l->is_driven && l->pivot_connector_id == center_id) return l->accumulated_angle_rad;

        if (l->driven_externally) {
            for (int gi = 0; gi < m->gear_count; gi++) {
                const Gear *g = &m->gears[gi];
                if (!g->alive || g->kind == GEAR_RACK || g->driven_link_id != link_id) continue;
                double in = body_total_rotation(m, g->driver_link_id, g->driver_center_id,
                                                 g->driver_ref_id, g->frozen_driver_angle, depth + 1);
                return gear_driven_rotation(g, in);
            }
            for (int vi = 0; vi < m->geneva_count; vi++) {
                const Geneva *gv = &m->genevas[vi];
                if (!gv->alive || gv->wheel_link_id != link_id) continue;
                double in = body_total_rotation(m, gv->driver_link_id, gv->driver_center_id,
                                                 gv->driver_ref_id, gv->frozen_driver_angle, depth + 1);
                return geneva_wheel_angle(gv->slot_count, in, NULL);
            }
        }
    }
    return mechanism_body_rotation(m, center_id, ref_id, frozen_angle);
}

static double driver_rotation(const Mechanism *m, int link_id, int center_id,
                               int ref_id, double frozen_angle) {
    return body_total_rotation(m, link_id, center_id, ref_id, frozen_angle, 0);
}

/* Places a link's connectors by rotating its frozen shape about its pivot. */
static void pose_link_rotated(Mechanism *m, Link *l, double angle, bool *settled) {
    int pivot = l->pivot_connector_id;
    Vec2 pivot_pos = m->connectors[pivot].pos;
    for (int i = 0; i < l->connector_count; i++) {
        int cid = l->connector_ids[i];
        if (cid == pivot) continue;
        m->connectors[cid].pos = vec2_add(pivot_pos, vec2_rotate(l->frozen_local_offset[i], angle));
        if (settled) settled[cid] = true;
    }
    if (settled) settled[pivot] = true;
}

/* Places a rack: its whole frozen shape slid along the rack axis. */
static void pose_link_translated(Mechanism *m, Link *l, Vec2 offset, bool *settled) {
    for (int i = 0; i < l->connector_count; i++) {
        int cid = l->connector_ids[i];
        m->connectors[cid].pos = vec2_add(vec2_add(l->frozen_pivot_pos, l->frozen_local_offset[i]), offset);
        if (settled) settled[cid] = true;
    }
}

static bool link_fully_settled(const Mechanism *m, int link_id, const bool *settled) {
    const Link *l = &m->links[link_id];
    for (int i = 0; i < l->connector_count; i++) {
        if (!settled[l->connector_ids[i]]) return false;
    }
    return true;
}

/* Poses everything whose position is dictated rather than solved: motors,
 * gears and Geneva wheels.
 *
 * A motor's pivot is usually an anchor, but it need not be: a motor can be
 * mounted on a part that another motor moves, which is exactly what a chain of
 * rotating arms is. Gears and Geneva wheels go further still -- each reads its
 * driver's rotation, so it can only be placed once that driver has been. So
 * all three are posed in dependency order, anchors first and then whatever
 * their motion has settled, rather than in array order, which would place a
 * body from its driver's stale position.
 *
 * The frozen offsets are captured relative to each pivot, and each driven
 * link's accumulated angle is absolute, so an arm turning at k times the base
 * rate sweeps k turns per cycle in world terms. That is what makes a chain of
 * arms sum a Fourier series rather than nest relative rotations. */
static void pose_driven_links(Mechanism *m) {
    if (m->link_count <= 0) return;

    bool *settled = xmalloc((size_t)(m->connector_count > 0 ? m->connector_count : 1) * sizeof(bool));
    bool *posed = xcalloc((size_t)m->link_count, sizeof(bool));
    for (int i = 0; i < m->connector_count; i++) settled[i] = m->connectors[i].is_anchor;

    bool progress = true;
    while (progress) {
        progress = false;

        for (int li = 0; li < m->link_count; li++) {
            Link *l = &m->links[li];
            if (!l->alive || !l->is_driven || posed[li]) continue;
            int pivot = l->pivot_connector_id;
            if (pivot < 0 || pivot >= m->connector_count || !settled[pivot]) continue;
            if (!l->frozen_local_offset) continue;
            pose_link_rotated(m, l, l->accumulated_angle_rad, settled);
            posed[li] = true;
            progress = true;
        }

        for (int gi = 0; gi < m->gear_count; gi++) {
            Gear *g = &m->gears[gi];
            if (!g->alive || g->driven_link_id < 0 || posed[g->driven_link_id]) continue;
            if (g->driver_link_id < 0 || !link_fully_settled(m, g->driver_link_id, settled)) continue;
            Link *driven = &m->links[g->driven_link_id];
            if (!driven->alive || !driven->frozen_local_offset) continue;

            double turn = driver_rotation(m, g->driver_link_id, g->driver_center_id,
                                           g->driver_ref_id, g->frozen_driver_angle);
            if (g->kind == GEAR_RACK) {
                pose_link_translated(m, driven, vec2_scale(g->rack_axis, gear_rack_travel(g, turn)), settled);
            } else {
                pose_link_rotated(m, driven, gear_driven_rotation(g, turn), settled);
            }
            posed[g->driven_link_id] = true;
            progress = true;
        }

        for (int vi = 0; vi < m->geneva_count; vi++) {
            Geneva *gv = &m->genevas[vi];
            if (!gv->alive || gv->wheel_link_id < 0 || posed[gv->wheel_link_id]) continue;
            if (gv->driver_link_id < 0 || !link_fully_settled(m, gv->driver_link_id, settled)) continue;
            Link *wheel = &m->links[gv->wheel_link_id];
            if (!wheel->alive || !wheel->frozen_local_offset) continue;

            double turn = driver_rotation(m, gv->driver_link_id, gv->driver_center_id,
                                           gv->driver_ref_id, gv->frozen_driver_angle);
            bool engaged = false;
            double angle = geneva_wheel_angle(gv->slot_count, turn, &engaged);
            gv->engaged = engaged;
            pose_link_rotated(m, wheel, angle, settled);
            posed[gv->wheel_link_id] = true;
            progress = true;
        }
    }

    free(posed);
    free(settled);
}

static bool connector_is_anchored_or_driven(const Mechanism *m, int cid) {
    if (m->connectors[cid].is_anchor) return true;
    for (int li = 0; li < m->link_count; li++) {
        const Link *l = &m->links[li];
        if (!l->alive || !(l->is_driven || l->driven_externally)) continue;
        for (int i = 0; i < l->connector_count; i++) {
            if (l->connector_ids[i] == cid) return true;
        }
    }
    return false;
}

static bool connector_is_fixed(const Mechanism *m, int cid) {
    if (connector_is_anchored_or_driven(m, cid)) return true;
    /* A follower resting on its cam has its position dictated by the profile,
     * exactly like a driven link's connector, so the rest of the mechanism
     * must solve around it. One that has lifted off is free again. */
    for (int ci = 0; ci < m->cam_count; ci++) {
        const Cam *c = &m->cams[ci];
        if (c->alive && c->in_contact && c->follower_connector_id == cid) return true;
    }
    return false;
}

/* Decides, for this frame, whether each follower is touching its cam, and if
 * so places it on the profile.
 *
 * Contact is one-sided: the cam can push the follower out but never pull it
 * back, so a profile falling away faster than the return spring can push the
 * follower down leaves it airborne. That is real cam float, and it is why
 * this is resolved by a test rather than by a bilateral constraint.
 *
 * Call after the motor angles have advanced and the driven links have been
 * posed, but before the Gauss-Newton solve. */
static void resolve_cam_contact(Mechanism *m) {
    for (int ci = 0; ci < m->cam_count; ci++) {
        Cam *c = &m->cams[ci];
        if (!cam_is_usable(m, c)) { if (c->alive) c->in_contact = false; continue; }

        double theta = mechanism_cam_angle(m, ci);
        double phi = follower_local_angle(m, c, ci);
        double s_min = cam_pitch_radius(c, phi);
        double s = follower_axis_position(m, c);

        if (s < s_min) {
            Vec2 centre = cam_center_pos(m, c);
            Vec2 new_pos = vec2_add(centre, vec2_scale(c->axis_dir, s_min));

            /* Hand the follower the surface's own velocity rather than
             * letting Verlet infer one from a snapped position -- otherwise
             * every re-contact injects a spurious impulse and the follower
             * chatters. phi runs backwards as the cam turns forwards, hence
             * the sign.
             *
             * The cam angle comes from an atan2 and so wraps by a full turn
             * once per revolution. Taken literally that reads as the cam
             * having spun 360 degrees in a single frame, which would fling
             * the follower off; take the shorter way round instead. */
            double dtheta = theta - c->last_angle;
            while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
            while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
            double ds = -cam_pitch_radius_deriv(c, phi) * dtheta;

            Connector *f = &m->connectors[c->follower_connector_id];
            f->prev_pos = vec2_sub(new_pos, vec2_scale(c->axis_dir, ds));
            f->pos = new_pos;
            c->in_contact = true;
        } else {
            c->in_contact = false;
        }
        c->last_angle = theta;
    }
}

static Vec2 get_pos(const Mechanism *m, const int *free_index, const double *xvec, int cid) {
    int fi = free_index[cid];
    if (fi >= 0) return (Vec2){ xvec[2 * fi], xvec[2 * fi + 1] };
    return m->connectors[cid].pos;
}

static double eval_residuals(const Mechanism *m, const int *free_index, const double *xvec,
                              const Residual *res_list, int nres, double *r_out) {
    double cost = 0.0;
    for (int k = 0; k < nres; k++) {
        double rr;
        if (res_list[k].kind == RES_AXIS) {
            Vec2 p = get_pos(m, free_index, xvec, res_list[k].ci);
            rr = vec2_dot(vec2_sub(p, res_list[k].axis_point), res_list[k].axis_normal) * res_list[k].scale;
        } else if (res_list[k].kind == RES_RAIL) {
            Vec2 p = get_pos(m, free_index, xvec, res_list[k].ci);
            Vec2 a = get_pos(m, free_index, xvec, res_list[k].rail_a);
            Vec2 b = get_pos(m, free_index, xvec, res_list[k].rail_b);
            Vec2 d = vec2_sub(b, a);
            double len = vec2_len(d);
            /* Signed distance from the pin to the rail line. */
            rr = (len > 1e-12) ? (vec2_cross(d, vec2_sub(p, a)) / len) * res_list[k].scale : 0.0;
        } else {
            Vec2 pi = get_pos(m, free_index, xvec, res_list[k].ci);
            Vec2 pj = get_pos(m, free_index, xvec, res_list[k].cj);
            double dx = pi.x - pj.x, dy = pi.y - pj.y;
            rr = dx * dx + dy * dy - res_list[k].rest * res_list[k].rest;
        }
        r_out[k] = rr;
        cost += rr * rr;
    }
    return cost;
}

static void build_jacobian(const Mechanism *m, const int *free_index, const double *xvec,
                            const Residual *res_list, int nres, int n, double *J) {
    memset(J, 0, (size_t)nres * (size_t)n * sizeof(double));
    for (int k = 0; k < nres; k++) {
        if (res_list[k].kind == RES_AXIS) {
            int ci = res_list[k].ci;
            if (free_index[ci] >= 0) {
                J[k * n + 2 * free_index[ci]] = res_list[k].axis_normal.x * res_list[k].scale;
                J[k * n + 2 * free_index[ci] + 1] = res_list[k].axis_normal.y * res_list[k].scale;
            }
            continue;
        }
        if (res_list[k].kind == RES_RAIL) {
            int pin = res_list[k].ci, ra = res_list[k].rail_a, rb = res_list[k].rail_b;
            Vec2 p = get_pos(m, free_index, xvec, pin);
            Vec2 a = get_pos(m, free_index, xvec, ra);
            Vec2 b = get_pos(m, free_index, xvec, rb);
            Vec2 d = vec2_sub(b, a), q = vec2_sub(p, a);
            double len = vec2_len(d);
            if (len < 1e-12) continue;
            double f = vec2_cross(d, q);          /* len * signed distance */
            double sc = res_list[k].scale;

            /* r = cross(d, q) / |d|, differentiated through both the pin and
             * the rail's own endpoints (the rail may be part of a moving
             * link). */
            if (free_index[pin] >= 0) {
                J[k * n + 2 * free_index[pin]] += (-d.y / len) * sc;
                J[k * n + 2 * free_index[pin] + 1] += (d.x / len) * sc;
            }
            if (free_index[ra] >= 0) {
                double dfx = -q.y + d.y, dfy = -d.x + q.x;
                J[k * n + 2 * free_index[ra]] += (dfx / len + f * d.x / (len * len * len)) * sc;
                J[k * n + 2 * free_index[ra] + 1] += (dfy / len + f * d.y / (len * len * len)) * sc;
            }
            if (free_index[rb] >= 0) {
                double dfx = q.y, dfy = -q.x;
                J[k * n + 2 * free_index[rb]] += (dfx / len - f * d.x / (len * len * len)) * sc;
                J[k * n + 2 * free_index[rb] + 1] += (dfy / len - f * d.y / (len * len * len)) * sc;
            }
            continue;
        }
        int ci = res_list[k].ci, cj = res_list[k].cj;
        Vec2 pi = get_pos(m, free_index, xvec, ci);
        Vec2 pj = get_pos(m, free_index, xvec, cj);
        double dx = pi.x - pj.x, dy = pi.y - pj.y;
        if (free_index[ci] >= 0) {
            J[k * n + 2 * free_index[ci]] += 2.0 * dx;
            J[k * n + 2 * free_index[ci] + 1] += 2.0 * dy;
        }
        if (free_index[cj] >= 0) {
            J[k * n + 2 * free_index[cj]] -= 2.0 * dx;
            J[k * n + 2 * free_index[cj] + 1] -= 2.0 * dy;
        }
    }
}

/* One Gauss-Newton solve. `enforce_variable_links` decides whether links
 * whose length has been toggled variable are held at their rest length
 * (treated exactly like rigid ones) or left entirely unconstrained. */
static bool solve_pass(Mechanism *m, SolverParams params, bool enforce_variable_links) {
    int nconn = m->connector_count;
    int *free_index = xmalloc((size_t)nconn * sizeof(int));
    int num_free = 0;
    for (int c = 0; c < nconn; c++) {
        if (m->connectors[c].alive && !connector_is_fixed(m, c)) {
            free_index[c] = num_free++;
        } else {
            free_index[c] = -1;
        }
    }

    bool result = true;

    if (num_free > 0) {
        int cap = 16, nres = 0;
        Residual *res_list = xmalloc((size_t)cap * sizeof(Residual));
        for (int li = 0; li < m->link_count; li++) {
            Link *l = &m->links[li];
            /* Driven links are exactly satisfied by construction. Variable
             * links are enforced or not depending on which pass this is. */
            if (!l->alive || l->is_driven || l->driven_externally) continue;
            if (!l->rigid && !enforce_variable_links) continue;
            int k = l->connector_count;
            for (int i = 0; i < k; i++) {
                for (int j = i + 1; j < k; j++) {
                    int ci = l->connector_ids[i], cj = l->connector_ids[j];
                    if (free_index[ci] < 0 && free_index[cj] < 0) continue; /* both fixed: nothing to solve */
                    if (nres >= cap) {
                        cap *= 2;
                        res_list = xrealloc(res_list, (size_t)cap * sizeof(Residual));
                    }
                    res_list[nres].kind = RES_PAIR;
                    res_list[nres].ci = ci;
                    res_list[nres].cj = cj;
                    res_list[nres].rest = l->rest_dist[mechanism_pair_index(i, j, k)];
                    nres++;
                }
            }
        }

        double scale = characteristic_length(m);

        /* Sliders and pins-in-slots: the pin is held on the rail's line. */
        for (int si = 0; si < m->slider_count; si++) {
            const Slider *sl = &m->sliders[si];
            if (!sl->alive) continue;
            if (sl->pin_connector_id < 0 || sl->rail_a_id < 0 || sl->rail_b_id < 0) continue;
            if (!m->connectors[sl->pin_connector_id].alive) continue;
            /* Nothing to solve if every point involved is already placed. */
            if (free_index[sl->pin_connector_id] < 0 &&
                free_index[sl->rail_a_id] < 0 && free_index[sl->rail_b_id] < 0) continue;
            if (nres >= cap) {
                cap *= 2;
                res_list = xrealloc(res_list, (size_t)cap * sizeof(Residual));
            }
            res_list[nres].kind = RES_RAIL;
            res_list[nres].ci = sl->pin_connector_id;
            res_list[nres].cj = sl->pin_connector_id;
            res_list[nres].rest = 0.0;
            res_list[nres].rail_a = sl->rail_a_id;
            res_list[nres].rail_b = sl->rail_b_id;
            res_list[nres].scale = scale;
            nres++;
        }

        /* An airborne follower still slides on its guide axis. (In contact it
         * is a fixed connector, so there is nothing to constrain.) */
        for (int ci = 0; ci < m->cam_count; ci++) {
            const Cam *c = &m->cams[ci];
            if (!cam_is_usable(m, c)) continue;
            int fid = c->follower_connector_id;
            if (free_index[fid] < 0) continue;
            if (nres >= cap) {
                cap *= 2;
                res_list = xrealloc(res_list, (size_t)cap * sizeof(Residual));
            }
            res_list[nres].kind = RES_AXIS;
            res_list[nres].ci = fid;
            res_list[nres].cj = fid;
            res_list[nres].rest = 0.0;
            res_list[nres].axis_point = cam_center_pos(m, c);
            res_list[nres].axis_normal = vec2_perp(c->axis_dir);
            res_list[nres].scale = scale;
            nres++;
        }

        if (nres > 0) {
            int n = 2 * num_free;
            double *x = xmalloc((size_t)n * sizeof(double));
            for (int c = 0; c < nconn; c++) {
                if (free_index[c] >= 0) {
                    x[2 * free_index[c]] = m->connectors[c].pos.x;
                    x[2 * free_index[c] + 1] = m->connectors[c].pos.y;
                }
            }

            double *r = xmalloc((size_t)nres * sizeof(double));
            double *rnew = xmalloc((size_t)nres * sizeof(double));
            double *J = xmalloc((size_t)nres * (size_t)n * sizeof(double));
            double *Mm = xmalloc((size_t)n * (size_t)n * sizeof(double));
            double *rhs = xmalloc((size_t)n * sizeof(double));
            double *Mtmp = xmalloc((size_t)n * (size_t)n * sizeof(double));
            double *rhs_tmp = xmalloc((size_t)n * sizeof(double));
            double *delta = xmalloc((size_t)n * sizeof(double));
            double *xnew = xmalloc((size_t)n * sizeof(double));

            double cost = eval_residuals(m, free_index, x, res_list, nres, r);
            double lambda = params.lambda_init;
            bool converged = (cost <= params.tol * params.tol);

            for (int iter = 0; iter < params.max_iters && !converged; iter++) {
                build_jacobian(m, free_index, x, res_list, nres, n, J);

                for (int a = 0; a < n; a++) {
                    for (int b = 0; b < n; b++) {
                        double s = 0.0;
                        for (int k = 0; k < nres; k++) s += J[k * n + a] * J[k * n + b];
                        Mm[a * n + b] = s;
                    }
                    double s = 0.0;
                    for (int k = 0; k < nres; k++) s += J[k * n + a] * r[k];
                    rhs[a] = -s;
                }

                /* Damping is scaled to the whole problem (the largest diagonal
                 * of JtJ), not to each diagonal entry individually. Per-entry
                 * (Marquardt) scaling fails badly in a near-null direction:
                 * a pendulum hanging straight down has dx~0, so the x
                 * diagonal ~4dx^2 is nearly zero and lambda*4dx^2 damps it
                 * essentially not at all, letting Gauss-Newton propose an
                 * enormous sideways step (the direction that changes length
                 * only to second order). Every such step is rejected, the
                 * solve gives up, and the link silently stretches. Damping by
                 * the problem's overall magnitude restrains that direction. */
                double max_diag = 0.0;
                for (int d = 0; d < n; d++) max_diag = fmax(max_diag, Mm[d * n + d]);
                if (max_diag <= 0.0) max_diag = 1.0;

                bool improved = false;
                for (int sub = 0; sub < 12 && !improved; sub++) {
                    memcpy(Mtmp, Mm, (size_t)n * (size_t)n * sizeof(double));
                    for (int d = 0; d < n; d++) Mtmp[d * n + d] += lambda * max_diag + params.damping_floor;
                    memcpy(rhs_tmp, rhs, (size_t)n * sizeof(double));

                    if (!linalg_solve(Mtmp, rhs_tmp, n, delta)) {
                        lambda *= 4.0;
                        continue;
                    }

                    for (int d = 0; d < n; d++) xnew[d] = x[d] + delta[d];
                    double cost_new = eval_residuals(m, free_index, xnew, res_list, nres, rnew);

                    if (cost_new < cost) {
                        memcpy(x, xnew, (size_t)n * sizeof(double));
                        memcpy(r, rnew, (size_t)nres * sizeof(double));
                        cost = cost_new;
                        lambda = fmax(lambda / 4.0, 1e-12);
                        improved = true;
                        if (cost <= params.tol * params.tol) converged = true;
                    } else {
                        lambda *= 4.0;
                        if (lambda > 1e12) {
                            improved = true; /* give up on this frame, but stop cleanly */
                        }
                    }
                }
                if (!improved) break;
            }

            for (int c = 0; c < nconn; c++) {
                if (free_index[c] >= 0) {
                    m->connectors[c].pos.x = x[2 * free_index[c]];
                    m->connectors[c].pos.y = x[2 * free_index[c] + 1];
                }
            }

            result = converged;

            free(x);
            free(r);
            free(rnew);
            free(J);
            free(Mm);
            free(rhs);
            free(Mtmp);
            free(rhs_tmp);
            free(delta);
            free(xnew);
        }
        free(res_list);
    }

    free(free_index);
    return result;
}

static bool has_length_violation(const Mechanism *m, double abs_tol, double rel_tol, bool include_variable) {
    for (int li = 0; li < m->link_count; li++) {
        const Link *l = &m->links[li];
        /* Driven and externally posed links are placed rigidly by
         * construction, so they can't bind. */
        if (!l->alive || l->is_driven || l->driven_externally) continue;
        if (!l->rigid && !include_variable) continue;

        int k = l->connector_count;
        for (int i = 0; i < k; i++) {
            for (int j = i + 1; j < k; j++) {
                double rest = l->rest_dist[mechanism_pair_index(i, j, k)];
                double actual = vec2_dist(m->connectors[l->connector_ids[i]].pos,
                                           m->connectors[l->connector_ids[j]].pos);
                double allowed = fmax(abs_tol, rel_tol * rest);
                if (fabs(actual - rest) > allowed) return true;
            }
        }
    }

    for (int si = 0; si < m->slider_count; si++) {
        const Slider *sl = &m->sliders[si];
        if (!sl->alive || sl->pin_connector_id < 0) continue;
        if (!m->connectors[sl->pin_connector_id].alive) continue;
        Vec2 a = m->connectors[sl->rail_a_id].pos, b = m->connectors[sl->rail_b_id].pos;
        Vec2 d = vec2_sub(b, a);
        double len = vec2_len(d);
        if (len < 1e-9) continue;
        double off = fabs(vec2_cross(d, vec2_sub(m->connectors[sl->pin_connector_id].pos, a)) / len);
        if (off > fmax(abs_tol, rel_tol * len)) return true;
    }

    /* Cam contact is one-sided and so can never bind, but the follower's
     * guide axis is a hard constraint like any other: if the linkage drags
     * the follower off its axis, that is a genuine lock-up. */
    for (int ci = 0; ci < m->cam_count; ci++) {
        const Cam *c = &m->cams[ci];
        if (!cam_is_usable(m, c)) continue;
        Vec2 d = vec2_sub(m->connectors[c->follower_connector_id].pos, cam_center_pos(m, c));
        double off_axis = fabs(vec2_dot(d, vec2_perp(c->axis_dir)));
        double allowed = fmax(abs_tol, rel_tol * (c->base_radius + c->lift));
        if (off_axis > allowed) return true;
    }
    return false;
}

bool solver_has_length_violation(const Mechanism *m, double abs_tol, double rel_tol) {
    return has_length_violation(m, abs_tol, rel_tol, false);
}

static bool has_variable_link(const Mechanism *m) {
    for (int li = 0; li < m->link_count; li++) {
        const Link *l = &m->links[li];
        if (l->alive && !l->is_driven && !l->rigid) return true;
    }
    return false;
}

bool solver_solve_at_current_angle(Mechanism *m, SolverParams params) {
    /* Sizes follow the geometry, so a part that has been dragged changes the
     * mechanism's proportions rather than leaving a gear pair separated. */
    mechanism_refresh_joint_sizes(m);
    pose_driven_links(m);

    /* With no variable-length links there is nothing to decide. */
    if (!has_variable_link(m)) return solve_pass(m, params, false);

    /* Otherwise, first try to hold EVERY link at its rest length, variable
     * ones included: a variable link should only give way when the geometry
     * genuinely leaves it no choice, not merely because it is allowed to.
     * If that succeeds, nothing needed to stretch and we keep it. */
    Vec2 *saved = xmalloc((size_t)m->connector_count * sizeof(Vec2));
    for (int i = 0; i < m->connector_count; i++) saved[i] = m->connectors[i].pos;

    /* Judge success by the lengths themselves, not solve_pass's convergence
     * flag: that flag compares an absolute least-squares cost against a
     * fixed tolerance, and the residuals are in units of length SQUARED, so
     * at real coordinate scales it reads "not converged" even for a
     * perfectly good fit. */
    solve_pass(m, params, true);
    if (!has_length_violation(m, params.length_tol_abs, params.length_tol_rel, true)) {
        free(saved);
        return true;
    }

    /* It didn't fit. Restore the frame's warm start and re-solve enforcing
     * only the genuinely rigid links, letting the variable ones absorb
     * whatever the rigid geometry demands of them. */
    for (int i = 0; i < m->connector_count; i++) m->connectors[i].pos = saved[i];
    free(saved);
    return solve_pass(m, params, false);
}

#define GRAVITY_VELOCITY_DAMPING 0.98

static bool has_live_cam(const Mechanism *m) {
    for (int ci = 0; ci < m->cam_count; ci++) {
        if (m->cams[ci].alive) return true;
    }
    return false;
}

bool solver_advance(Mechanism *m, double dt, SolverParams params) {
    for (int li = 0; li < m->link_count; li++) {
        Link *l = &m->links[li];
        if (!l->alive || !l->is_driven) continue;
        l->accumulated_angle_rad += l->motor_speed_deg_s * (M_PI / 180.0) * dt;
    }

    /* Pose the driven links now, before contact is resolved, so each cam's
     * rotation already reflects this frame. solve_pass poses them again;
     * doing so twice is harmless because the pose depends only on the
     * accumulated angle. */
    pose_driven_links(m);

    bool gravity_on = (params.gravity.x != 0.0 || params.gravity.y != 0.0);
    if (gravity_on || has_live_cam(m)) {
        /* External accelerations: gravity on everything, plus each cam's
         * return spring on its own follower. The spring is what makes
         * one-sided contact meaningful -- it is the only thing pressing the
         * follower back onto the profile once the cam stops pushing. */
        Vec2 *accel = xmalloc((size_t)m->connector_count * sizeof(Vec2));
        for (int i = 0; i < m->connector_count; i++) accel[i] = params.gravity;

        for (int ci = 0; ci < m->cam_count; ci++) {
            const Cam *c = &m->cams[ci];
            if (!cam_is_usable(m, c)) continue;
            double s = follower_axis_position(m, c);
            /* Preloaded: the rest position sits inside the base circle, so
             * the spring always presses inward, never lifts. */
            double s0 = c->base_radius - c->spring_preload;
            accel[c->follower_connector_id] =
                vec2_add(accel[c->follower_connector_id],
                          vec2_scale(c->axis_dir, -c->spring_k * (s - s0)));
        }

        for (int i = 0; i < m->connector_count; i++) {
            Connector *c = &m->connectors[i];
            /* Deliberately NOT connector_is_fixed: a follower that was in
             * contact last frame still gets integrated, so that if the cam
             * has fallen away from under it this frame it is already moving.
             * resolve_cam_contact below snaps it back if it is still
             * touching. */
            if (!c->alive || connector_is_anchored_or_driven(m, i)) continue;
            /* Verlet integration: (pos - prev_pos) is an implicit velocity
             * estimate, so no separate velocity field is needed. The
             * resulting position is just a seed for solver_solve_at_current_angle's
             * Gauss-Newton projection below, which pulls it back onto
             * whatever rigid-link constraints apply to it. */
            Vec2 velocity = vec2_sub(c->pos, c->prev_pos);
            Vec2 new_pos = vec2_add(vec2_add(c->pos, vec2_scale(velocity, GRAVITY_VELOCITY_DAMPING)),
                                     vec2_scale(accel[i], dt * dt));
            c->prev_pos = c->pos;
            c->pos = new_pos;
        }
        free(accel);
    }

    resolve_cam_contact(m);

    return solver_solve_at_current_angle(m, params);
}
