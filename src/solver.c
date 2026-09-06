#include "solver.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "linalg.h"

typedef struct {
    int ci, cj;
    double rest;
} Residual;

SolverParams solver_default_params(void) {
    SolverParams p;
    p.max_iters = 30;
    p.tol = 1e-6;
    p.lambda_init = 1e-3;
    p.damping_floor = 1e-9;
    p.gravity = (Vec2){ 0.0, 0.0 };
    return p;
}

void solver_freeze(Mechanism *m) {
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

        if (l->is_driven) {
            free(l->frozen_local_offset);
            l->frozen_local_offset = malloc((size_t)k * sizeof(Vec2));
            Vec2 pivot_pos = m->connectors[l->pivot_connector_id].pos;
            for (int i = 0; i < k; i++) {
                l->frozen_local_offset[i] = vec2_sub(m->connectors[l->connector_ids[i]].pos, pivot_pos);
            }
            l->accumulated_angle_rad = 0.0;
        }
    }
}

static void pose_driven_links(Mechanism *m) {
    for (int li = 0; li < m->link_count; li++) {
        Link *l = &m->links[li];
        if (!l->alive || !l->is_driven) continue;
        Vec2 pivot_pos = m->connectors[l->pivot_connector_id].pos;
        for (int i = 0; i < l->connector_count; i++) {
            int cid = l->connector_ids[i];
            if (cid == l->pivot_connector_id) continue; /* anchor, never moves */
            m->connectors[cid].pos = vec2_add(pivot_pos, vec2_rotate(l->frozen_local_offset[i], l->accumulated_angle_rad));
        }
    }
}

static bool connector_is_fixed(const Mechanism *m, int cid) {
    if (m->connectors[cid].is_anchor) return true;
    for (int li = 0; li < m->link_count; li++) {
        const Link *l = &m->links[li];
        if (!l->alive || !l->is_driven) continue;
        for (int i = 0; i < l->connector_count; i++) {
            if (l->connector_ids[i] == cid) return true;
        }
    }
    return false;
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
        Vec2 pi = get_pos(m, free_index, xvec, res_list[k].ci);
        Vec2 pj = get_pos(m, free_index, xvec, res_list[k].cj);
        double dx = pi.x - pj.x, dy = pi.y - pj.y;
        double rr = dx * dx + dy * dy - res_list[k].rest * res_list[k].rest;
        r_out[k] = rr;
        cost += rr * rr;
    }
    return cost;
}

static void build_jacobian(const Mechanism *m, const int *free_index, const double *xvec,
                            const Residual *res_list, int nres, int n, double *J) {
    memset(J, 0, (size_t)nres * (size_t)n * sizeof(double));
    for (int k = 0; k < nres; k++) {
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

bool solver_solve_at_current_angle(Mechanism *m, SolverParams params) {
    pose_driven_links(m);

    int nconn = m->connector_count;
    int *free_index = malloc((size_t)nconn * sizeof(int));
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
        Residual *res_list = malloc((size_t)cap * sizeof(Residual));
        for (int li = 0; li < m->link_count; li++) {
            Link *l = &m->links[li];
            /* Driven links are exactly satisfied by construction; non-rigid
             * (toggled variable-length) links contribute no distance
             * constraints at all, leaving their connectors' relative
             * positions free. */
            if (!l->alive || l->is_driven || !l->rigid) continue;
            int k = l->connector_count;
            for (int i = 0; i < k; i++) {
                for (int j = i + 1; j < k; j++) {
                    int ci = l->connector_ids[i], cj = l->connector_ids[j];
                    if (free_index[ci] < 0 && free_index[cj] < 0) continue; /* both fixed: nothing to solve */
                    if (nres >= cap) {
                        cap *= 2;
                        res_list = realloc(res_list, (size_t)cap * sizeof(Residual));
                    }
                    res_list[nres].ci = ci;
                    res_list[nres].cj = cj;
                    res_list[nres].rest = l->rest_dist[mechanism_pair_index(i, j, k)];
                    nres++;
                }
            }
        }

        if (nres > 0) {
            int n = 2 * num_free;
            double *x = malloc((size_t)n * sizeof(double));
            for (int c = 0; c < nconn; c++) {
                if (free_index[c] >= 0) {
                    x[2 * free_index[c]] = m->connectors[c].pos.x;
                    x[2 * free_index[c] + 1] = m->connectors[c].pos.y;
                }
            }

            double *r = malloc((size_t)nres * sizeof(double));
            double *rnew = malloc((size_t)nres * sizeof(double));
            double *J = malloc((size_t)nres * (size_t)n * sizeof(double));
            double *Mm = malloc((size_t)n * (size_t)n * sizeof(double));
            double *rhs = malloc((size_t)n * sizeof(double));
            double *Mtmp = malloc((size_t)n * (size_t)n * sizeof(double));
            double *rhs_tmp = malloc((size_t)n * sizeof(double));
            double *delta = malloc((size_t)n * sizeof(double));
            double *xnew = malloc((size_t)n * sizeof(double));

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

                bool improved = false;
                for (int sub = 0; sub < 12 && !improved; sub++) {
                    memcpy(Mtmp, Mm, (size_t)n * (size_t)n * sizeof(double));
                    /* Additive floor alongside multiplicative damping: a free
                     * connector whose Jacobian column is exactly zero at this
                     * configuration (e.g. a dead-center singularity) would
                     * otherwise leave lambda*Mm[d][d]==0 at any lambda, making
                     * the system singular no matter how far lambda is raised. */
                    for (int d = 0; d < n; d++) Mtmp[d * n + d] += lambda * Mm[d * n + d] + params.damping_floor;
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

#define GRAVITY_VELOCITY_DAMPING 0.98

bool solver_advance(Mechanism *m, double dt, SolverParams params) {
    for (int li = 0; li < m->link_count; li++) {
        Link *l = &m->links[li];
        if (!l->alive || !l->is_driven) continue;
        l->accumulated_angle_rad += l->motor_speed_deg_s * (M_PI / 180.0) * dt;
    }

    if (params.gravity.x != 0.0 || params.gravity.y != 0.0) {
        for (int i = 0; i < m->connector_count; i++) {
            Connector *c = &m->connectors[i];
            if (!c->alive || connector_is_fixed(m, i)) continue;
            /* Verlet integration: (pos - prev_pos) is an implicit velocity
             * estimate, so no separate velocity field is needed. The
             * resulting position is just a seed for solver_solve_at_current_angle's
             * Gauss-Newton projection below, which pulls it back onto
             * whatever rigid-link constraints apply to it. */
            Vec2 velocity = vec2_sub(c->pos, c->prev_pos);
            Vec2 new_pos = vec2_add(vec2_add(c->pos, vec2_scale(velocity, GRAVITY_VELOCITY_DAMPING)),
                                     vec2_scale(params.gravity, dt * dt));
            c->prev_pos = c->pos;
            c->pos = new_pos;
        }
    }

    return solver_solve_at_current_angle(m, params);
}
