#ifndef JOINTS_H
#define JOINTS_H

#include <stdbool.h>
#include "vec2.h"

/* The joint types beyond the revolute pin that connectors already provide.
 *
 * Everything here is pure geometry with no SDL dependency, so the kinematics
 * are covered by the headless tests. */

/* ---------------------------------------------------------------------------
 * Slider / pin-in-slot
 *
 * One connector held on the line through two others. That single primitive is
 * both of the sliding joints a planar mechanism needs: when the two rail
 * points are anchors it is a prismatic joint sliding on ground (a crank-slider
 * piston, a scissor lift's foot); when they belong to a moving link it is a
 * pin running in that link's slot (a Whitworth quick-return, a Scotch yoke).
 * ------------------------------------------------------------------------ */
typedef struct {
    int pin_connector_id;
    int rail_a_id, rail_b_id;
    bool selected;
    bool alive;
} Slider;

/* ---------------------------------------------------------------------------
 * Gear pair
 *
 * Meshing gears roll without slipping, so their rotations are locked in the
 * ratio of their radii -- opposite in sense for an external mesh, the same for
 * an internal one. A rack is the limiting case of a gear of infinite radius:
 * the pinion's rotation becomes a translation along the rack's axis.
 * ------------------------------------------------------------------------ */
typedef enum { GEAR_EXTERNAL, GEAR_INTERNAL, GEAR_RACK } GearKind;

typedef struct {
    GearKind kind;
    int driver_link_id, driver_center_id;
    int driven_link_id;
    int driven_center_id;    /* unused by GEAR_RACK */
    /* What you set is the RATIO -- the tooth-count ratio, in effect. The two
     * pitch radii are then whatever the live centre distance splits into at
     * that ratio, so dragging a centre resizes both wheels and they carry on
     * meshing instead of separating. Both are refreshed from the geometry
     * every frame; treat them as read-only. */
    double ratio;            /* driven radius / driver radius */
    double driver_radius;    /* the pinion's pitch radius (derived) */
    double driven_radius;    /* unused by GEAR_RACK (derived) */
    Vec2 rack_axis;          /* unit, GEAR_RACK only */

    /* Frozen when the simulation starts. */
    int driver_ref_id;
    double frozen_driver_angle;

    bool selected;
    bool alive;
} Gear;

/* How far the driven body has moved for a given rotation of the driver: an
 * angle in radians for a gear pair, a distance along the rack axis for a
 * rack and pinion. */
double gear_driven_rotation(const Gear *g, double driver_rotation);
double gear_rack_travel(const Gear *g, double driver_rotation);

/* ---------------------------------------------------------------------------
 * Geneva wheel
 *
 * Steady rotation in, interrupted rotation out: the driver's pin enters a slot
 * in the wheel, indexes it by one step, and leaves, and a locking arc holds the
 * wheel still until the pin comes round again.
 *
 * The wheel's angle is computed in closed form from the driver's, which is
 * exact and cannot jam. The proportions are the standard ones -- the pin must
 * enter and leave along the slot, with no shock, which forces the crank radius
 * to be center_distance * sin(pi/slots) and fixes everything else.
 * ------------------------------------------------------------------------ */
typedef struct {
    int driver_link_id, driver_center_id;
    int wheel_link_id, wheel_center_id;
    int slot_count;
    double crank_radius;      /* pin distance from the driver's centre */
    double center_distance;   /* driver centre to wheel centre */

    /* Frozen when the simulation starts. */
    int driver_ref_id;
    double frozen_driver_angle;

    bool engaged;             /* recomputed each frame; drives rendering */
    bool selected;
    bool alive;
} Geneva;

/* The centre distance that makes a shock-free Geneva of `slots` slots for a
 * given crank radius, and the crank radius for a given centre distance. */
double geneva_center_distance(int slots, double crank_radius);
double geneva_crank_radius(int slots, double center_distance);

/* The wheel's total rotation for a driver rotation of `driver_rotation`
 * radians, and whether the pin is in a slot right now. Advances by exactly
 * one step (2*pi/slots) per turn of the driver, and is continuous.
 *
 * The sign is opposite the driver's: an external Geneva turns its wheel the
 * other way, because the pin sweeping forwards past the line of centres swings
 * the slot it occupies backwards. */
double geneva_wheel_angle(int slots, double driver_rotation, bool *engaged_out);

/* The driver's half-angle of engagement -- the pin is in a slot for this much
 * either side of the line of centres. */
double geneva_engagement_half_angle(int slots);

#endif
