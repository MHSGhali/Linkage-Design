#ifndef MECHANISM_H
#define MECHANISM_H

#include <stdbool.h>
#include "vec2.h"
#include "cam.h"
#include "joints.h"

/* A joint. is_anchor connectors are fixed to ground and never move. When
 * traced, its position is recorded into `path` once per simulation frame
 * (see mechanism_trace_step), so its trajectory can be drawn. `path_time`
 * holds the simulation time of each of those samples -- frames are not
 * uniform in duration, so plotting position against time needs the actual
 * timestamps, not the sample index. `prev_pos` is this connector's position
 * on the previous simulation frame, used only for Verlet-style gravity
 * integration on free (non-anchor, non-driven) connectors (see
 * solver_advance); reset to `pos` on every solver_freeze. */
typedef struct {
    Vec2 pos;
    Vec2 prev_pos;
    bool is_anchor;
    bool selected;
    bool traced;
    Vec2 *path;
    double *path_time;
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

    /* True when a gear or a Geneva wheel poses this link rather than a motor:
     * its connectors are placed directly, like a driven link's, so the solver
     * treats them as fixed and leaves its own distances alone. */
    bool driven_externally;
    Vec2 frozen_pivot_pos;        /* the pivot's world position at freeze; a rack
                                    * translates from there rather than rotating */

    /* A gear wheel is a body of its own size rather than a bar between pins:
     * this is its pitch radius, and zero means "an ordinary link". A wheel is
     * still a link -- it is a rigid body that turns about a pivot, which is
     * exactly what a link is -- but it is drawn as a disc, its size is its own
     * property rather than something a mesh dictates, and it exists whether or
     * not anything is meshed with it. */
    double wheel_radius;

    bool selected;
    bool alive;
} Link;

typedef struct {
    Connector *connectors;
    int connector_count, connector_capacity;
    Link *links;
    int link_count, link_capacity;
    Cam *cams;
    int cam_count, cam_capacity;
    Slider *sliders;
    int slider_count, slider_capacity;
    Gear *gears;
    int gear_count, gear_capacity;
    Geneva *genevas;
    int geneva_count, geneva_capacity;
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

/* Makes `link_id` a motor turning about `pivot_connector_id` at
 * `speed_deg_s`. Unlike mechanism_toggle_driven the pivot need NOT be an
 * anchor: a motor can be mounted on a part that something else moves, which
 * is what a chain of rotating arms is. Returns false on invalid input. */
bool mechanism_set_driven_about(Mechanism *m, int link_id, int pivot_connector_id, double speed_deg_s);

/* Sets whether a link's pairwise distances are enforced by the solver
 * (see the Link.rigid comment above). No-op on an invalid/dead link id. */
void mechanism_set_rigid(Mechanism *m, int link_id, bool rigid);

/* Whether there is anything at all in the mechanism -- an empty canvas is
 * worth saying something about. */
bool mechanism_has_any_part(const Mechanism *m);

/* Whether any live link is currently driven by a motor. A mechanism with
 * none has nothing making it move on its own. */
bool mechanism_has_driven_link(const Mechanism *m);

/* Sets/clears whether a connector's position is recorded each simulation
 * frame. Clearing it discards any previously recorded path. No-op on an
 * invalid/dead id. */
void mechanism_set_traced(Mechanism *m, int connector_id, bool traced);

/* Discards recorded path points (but keeps the traced flag) for every
 * traced connector -- call when a simulation run starts so each run's
 * trace begins fresh. */
void mechanism_clear_traces(Mechanism *m);

/* The most samples one traced connector keeps. A run records one per frame,
 * so an unbounded path is ~3600 points a minute, every one of them redrawn
 * every frame: leave a drawing machine running and the app slows to a crawl.
 * At the cap the path is thinned by half instead, which keeps the whole
 * history -- just at coarser resolution. */
#define MECHANISM_TRACE_MAX 12000

/* Appends the current position of every traced, alive connector to its path,
 * stamped with `sim_time` (seconds since the run started). Call once per
 * simulation frame, after resolving positions. */
void mechanism_trace_step(Mechanism *m, double sim_time);

/* Adds a disc cam turning with `body_link_id` about `center_connector_id`
 * (which must be one of that link's connectors), driving the roller follower
 * at `follower_connector_id` along the radial axis through the two. Profile
 * parameters get defaults sized from the current centre-to-follower distance.
 * Returns the new cam's id, or -1 on invalid input. */
int mechanism_add_cam(Mechanism *m, int body_link_id, int center_connector_id, int follower_connector_id);

void mechanism_delete_cam(Mechanism *m, int cam_id);

/* The cam whose drawn surface passes within `dist_thresh` of p, or -1. */
int mechanism_pick_cam(const Mechanism *m, Vec2 p, double dist_thresh);

/* Cam i's current rotation in radians: how far its body link has turned since
 * solver_freeze captured the reference orientation. Zero before a run. */
double mechanism_cam_angle(const Mechanism *m, int cam_id);

/* Holds `pin_connector_id` on the line through the two rail connectors. With
 * anchors for rails that is a prismatic joint sliding on ground; with a moving
 * link's connectors it is a pin running in that link's slot. Returns the new
 * slider's id, or -1 on invalid input. */
int mechanism_add_slider(Mechanism *m, int pin_connector_id, int rail_a_id, int rail_b_id);
void mechanism_delete_slider(Mechanism *m, int slider_id);

/* Meshes `driven_link_id` to `driver_link_id`: turning the driver about its
 * centre turns the driven body about its own, in the ratio of the radii and
 * the opposite sense (the same sense when `internal`). Both centres should be
 * anchors. Returns the new gear's id, or -1. */
int mechanism_add_gear(Mechanism *m, int driver_link_id, int driver_center_id,
                        int driven_link_id, int driven_center_id, double ratio, bool internal);

/* Rack and pinion: turning `driver_link_id` about its centre slides
 * `rack_link_id` along `axis` by the arc length rolled off the pitch circle. */
int mechanism_add_rack(Mechanism *m, int driver_link_id, int driver_center_id,
                        int rack_link_id, int rack_reference_id, Vec2 axis);
void mechanism_delete_gear(Mechanism *m, int gear_id);

/* A Geneva wheel: `driver_link_id` turning about its centre indexes
 * `wheel_link_id` about its own by one slot per revolution, holding it still
 * in between. The centre distance is taken from the two centres' positions and
 * the crank radius follows from the slot count. Returns the new id, or -1. */
int mechanism_add_geneva(Mechanism *m, int driver_link_id, int driver_center_id,
                          int wheel_link_id, int wheel_center_id, int slot_count);
void mechanism_delete_geneva(Mechanism *m, int geneva_id);

/* --- Gear wheels ---------------------------------------------------------
 *
 * A wheel is a body with a size of its own, placed and resized on its own, and
 * meshed with another only when you ask. That is what lets one wheel drive
 * several of them, and it means adding a wheel never disturbs what is already
 * there. */

/* Smallest and largest pitch radius a wheel can be resized to. */
#define MECHANISM_WHEEL_MIN_RADIUS 15.0
#define MECHANISM_WHEEL_MAX_RADIUS 400.0

/* A free-standing wheel at `centre`: an anchored hub, a mark on its rim, and
 * the body joining them. Meshed with nothing. Returns its link id, or -1. */
int mechanism_add_wheel(Mechanism *m, Vec2 centre, double radius);

/* Changes a wheel's pitch radius, carrying the rim mark out with it. Any mesh
 * it is part of follows on the next refresh. Clamped to the limits above. */
void mechanism_set_wheel_radius(Mechanism *m, int link_id, double radius);

/* Meshes two wheels, `driver_link` turning `driven_link`. The driven wheel
 * slides along the line of centres until the two touch, so the mesh is real
 * whatever distance they were drawn at. Returns the new gear's id, or -1 --
 * refused if either is not a wheel, if they are meshed already, or if the
 * driven one is already turned by something else. */
int mechanism_mesh_wheels(Mechanism *m, int driver_link, int driven_link);

bool mechanism_wheels_are_meshed(const Mechanism *m, int link_a, int link_b);

/* Makes `root_link` the wheel the drive flows out from, turning round any mesh
 * in its train that pointed the other way. So a wheel can be named the driver
 * whatever order the train happened to be meshed in. False if it is not a
 * wheel. */
bool mechanism_orient_train_from(Mechanism *m, int root_link);

/* A wheel's pitch radius and the connector at its centre, or 0/-1 if that link
 * is not a wheel. */
double mechanism_gear_wheel_radius(const Mechanism *m, int link_id, int *center_out);

/* True if the link is a gear wheel. A wheel is drawn as a disc with a mark on
 * its rim -- never as a bar -- so the renderer skips its edges. */
bool mechanism_is_gear_body(const Mechanism *m, int link_id);

/* The wheel one of whose connectors is `connector_id`, as a link id, or -1. */
int mechanism_wheel_at_connector(const Mechanism *m, int connector_id);

/* The mark on a wheel's rim -- the point whose motion can be traced, so the
 * trace is a circle of the wheel's own radius. */
int mechanism_wheel_mark(const Mechanism *m, int link_id);

/* Re-reads a link's rest distances from its connectors' current positions. */
void mechanism_refresh_link_rest_lengths(Mechanism *m, int link_id);

/* Recomputes every gear's pitch radii and every Geneva's crank radius from
 * where their centres actually are now. Called before each solve, so moving a
 * part changes the mechanism's proportions rather than breaking its mesh. */
void mechanism_refresh_joint_sizes(Mechanism *m);

/* A body's rotation since solver_freeze, read off the direction from its
 * centre to `ref_connector_id`. Zero before a run. */
double mechanism_body_rotation(const Mechanism *m, int center_id, int ref_id, double frozen_angle);

/* Nearest alive connector within `radius` of p, or -1 if none. */
int mechanism_pick_connector(const Mechanism *m, Vec2 p, double radius);

/* Nearest alive link with a pairwise edge within `dist_thresh` of p
 * (perpendicular distance to the segment, clamped to its endpoints), or -1. */
int mechanism_pick_link_edge(const Mechanism *m, Vec2 p, double dist_thresh);

/* The joints themselves are pickable too, so their proportions can be edited:
 * a wheel anywhere on its face, a rack by its pitch line, a Geneva by its
 * wheel or its locking disc, a slider by its rail. Each returns an index into
 * the matching array, or -1. */
int mechanism_pick_gear(const Mechanism *m, Vec2 p, double dist_thresh);

/* The wheel whose face the point lands on (or whose rim it is near), as a link
 * id, or -1. Anywhere on the disc counts. */
int mechanism_pick_wheel(const Mechanism *m, Vec2 p, double dist_thresh);
int mechanism_pick_geneva(const Mechanism *m, Vec2 p, double dist_thresh);
int mechanism_pick_slider(const Mechanism *m, Vec2 p, double dist_thresh);

/* The radius of the disc a Geneva's wheel is drawn as -- the slots reach in
 * from here. Shared by the renderer, the picker and the export. */
double mechanism_geneva_wheel_radius(const Geneva *gv);

#endif
