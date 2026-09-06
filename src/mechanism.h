#ifndef MECHANISM_H
#define MECHANISM_H

#include <stdbool.h>
#include "vec2.h"

/* A joint. is_anchor connectors are fixed to ground and never move. When
 * traced, its position is recorded into `path` once per simulation frame
 * (see mechanism_trace_step), so its trajectory can be drawn. `prev_pos` is
 * this connector's position on the previous simulation frame, used only for
 * Verlet-style gravity integration on free (non-anchor, non-driven)
 * connectors (see solver_advance); reset to `pos` on every solver_freeze. */
typedef struct {
    Vec2 pos;
    Vec2 prev_pos;
    bool is_anchor;
    bool selected;
    bool traced;
    Vec2 *path;
    int path_count, path_capacity;
    bool alive; /* tombstone on delete; ids (array indices) stay stable */
} Connector;

/* A body owning >=2 connectors. When `rigid` (the default), every pairwise
 * distance between its own connectors is held constant during simulation,
 * like a solid bar/plate. Toggling `rigid` off (mechanism_set_rigid) frees
 * every pairwise distance in this link from enforcement -- the connectors
 * are then free to move relative to each other (a telescoping/extensible
 * link), constrained only by whatever OTHER links or anchors touch them.
 * A driven link rotates rigidly around one of its own connectors (which
 * must be an anchor, the "pivot") at a constant angular speed; `rigid` is
 * not meaningful for a driven link (its shape is always posed directly). */
typedef struct {
    int *connector_ids;
    int connector_count; /* fixed once the link is created; no "add connector to link" in v1 */

    double *rest_dist; /* condensed upper-triangular pairwise rest distances,
                         * size connector_count*(connector_count-1)/2, frozen
                         * from current positions whenever simulation (re)starts */
    bool rigid;         /* whether rest_dist is actually enforced by the solver */

    bool is_driven;
    int pivot_connector_id;       /* one of connector_ids, must be an anchor; -1 if not driven */
    double motor_speed_deg_s;     /* signed; only meaningful if is_driven */
    double accumulated_angle_rad; /* motor state; reset to 0 whenever simulation (re)starts */
    Vec2 *frozen_local_offset;    /* size connector_count; connector pos minus pivot pos at the
                                    * moment simulation started (angle 0); only valid if is_driven */

    bool selected;
    bool alive;
} Link;

typedef struct {
    Connector *connectors;
    int connector_count, connector_capacity;
    Link *links;
    int link_count, link_capacity;
} Mechanism;

/* Condensed upper-triangular pair index for i<j among k items (0-indexed). */
int mechanism_pair_index(int i, int j, int k);

void mechanism_init(Mechanism *m);
void mechanism_free(Mechanism *m);

/* Deep-copies `src` into `dst` (which must not already own live data --
 * treat it like mechanism_init followed by populating it). Used for the
 * undo stack: clone the current state before an edit, keep the clone. */
void mechanism_clone(const Mechanism *src, Mechanism *dst);

/* Returns the new connector's id. */
int mechanism_add_connector(Mechanism *m, Vec2 pos, bool is_anchor);

/* Deletes a connector and cascades to delete every link that used it (no
 * partial link surgery in v1). No-op on an invalid/already-dead id. */
void mechanism_delete_connector(Mechanism *m, int connector_id);

void mechanism_delete_link(Mechanism *m, int link_id);

/* Creates a rigid link joining the given connectors (>=2, all alive); rest
 * lengths are captured from their current positions. Returns the new link's
 * id, or -1 on invalid input. */
int mechanism_add_link(Mechanism *m, const int *connector_ids, int count);

/* Sets/clears a connector's anchor flag. Clearing it also un-drives (and
 * clears the pivot of) any link that was pivoting on it. No-op on an
 * invalid/dead id. */
void mechanism_set_anchor(Mechanism *m, int connector_id, bool is_anchor);

/* Toggles "driven" on a link. Turning it on requires exactly one of the
 * link's own connectors to currently be an anchor (it becomes the pivot);
 * returns false without changing anything if that's not the case. Turning
 * an already-driven link off always succeeds. Returns false on an
 * invalid/dead link id. */
bool mechanism_toggle_driven(Mechanism *m, int link_id, double default_speed_deg_s);

/* Sets whether a link's pairwise distances are enforced by the solver
 * (see the Link.rigid comment above). No-op on an invalid/dead link id. */
void mechanism_set_rigid(Mechanism *m, int link_id, bool rigid);

/* Sets/clears whether a connector's position is recorded each simulation
 * frame. Clearing it discards any previously recorded path. No-op on an
 * invalid/dead id. */
void mechanism_set_traced(Mechanism *m, int connector_id, bool traced);

/* Discards recorded path points (but keeps the traced flag) for every
 * traced connector -- call when a simulation run starts so each run's
 * trace begins fresh. */
void mechanism_clear_traces(Mechanism *m);

/* Appends the current position of every traced, alive connector to its
 * path. Call once per simulation frame, after resolving positions. */
void mechanism_trace_step(Mechanism *m);

/* Nearest alive connector within `radius` of p, or -1 if none. */
int mechanism_pick_connector(const Mechanism *m, Vec2 p, double radius);

/* Nearest alive link with a pairwise edge within `dist_thresh` of p
 * (perpendicular distance to the segment, clamped to its endpoints), or -1. */
int mechanism_pick_link_edge(const Mechanism *m, Vec2 p, double dist_thresh);

#endif
