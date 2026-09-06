#ifndef CAM_H
#define CAM_H

#include <stdbool.h>
#include "vec2.h"

/* How many radii make up a cam's profile: one every two degrees. */
#define CAM_PROFILE_SAMPLES 180

/* A disc cam with a translating roller follower.
 *
 * The profile is stored as a PITCH CURVE -- the path traced by the roller
 * follower's CENTRE -- sampled as a radius every 360/CAM_PROFILE_SAMPLES
 * degrees and interpolated smoothly in between. Storing the pitch curve
 * rather than the cut surface is what makes contact exact: the follower axis
 * is radial, so the follower's distance from the cam centre is simply the
 * pitch radius whenever the two are touching. The physical surface that gets
 * drawn and printed is that curve offset inward by the roller radius.
 *
 * A profile can be produced two ways:
 *   - cam_fill_motion_law: a classic rise / high dwell / fall / low dwell
 *     cam, using the cycloidal law (zero velocity AND acceleration at every
 *     segment end, so no jerk spikes). This is what a new cam starts as.
 *   - cam_set_from_drawn_outline: whatever shape the user drew, taken as the
 *     physical surface and converted back to a pitch curve.
 * Everything downstream -- contact, rendering, export -- works off the table
 * and neither knows nor cares which produced it. */
typedef struct {
    int body_link_id;          /* the link the cam turns with */
    int center_connector_id;   /* the cam's centre of rotation */
    int follower_connector_id; /* the roller's centre */

    double pitch_r[CAM_PROFILE_SAMPLES]; /* pitch radius, uniform in phi */
    double base_radius;        /* cached min of pitch_r */
    double lift;               /* cached max - min of pitch_r */
    double roller_radius;      /* 0 gives a knife-edge follower */

    double spring_k;           /* return-spring stiffness (accel per unit) */
    double spring_preload;     /* how far the spring is compressed at base */

    /* Frozen when the simulation starts, like Link.frozen_local_offset. The
     * cam's rotation is read off its body link: how far the direction from
     * the centre to `ref_connector_id` has swung since freeze. That works for
     * a motor-driven link and for one moved by the rest of the mechanism. */
    Vec2 axis_origin;          /* the cam centre at freeze time */
    Vec2 axis_dir;             /* unit, pointing out along the follower axis */
    int ref_connector_id;      /* the body link connector used to read rotation */
    double frozen_ref_angle;   /* that direction's angle at freeze */
    double last_angle;         /* previous frame's rotation, for surface speed */

    bool in_contact;           /* recomputed each frame; drives rendering */
    bool selected;
    bool alive;
} Cam;

/* Fills in a cam with a working default profile sized for `base_radius`. */
void cam_set_defaults(Cam *c, double base_radius);

/* Replaces the profile with a rise / high dwell / fall / low dwell cam. */
void cam_fill_motion_law(Cam *c, double base_radius, double lift,
                          double rise_deg, double high_dwell_deg, double fall_deg);

/* Replaces the profile with one traced from a user-drawn closed outline,
 * given in cam-local coordinates (i.e. relative to the cam centre) and taken
 * to be the PHYSICAL cam surface. Returns false, leaving the cam untouched,
 * if the drawing is too small or too tangled to read a profile from. */
bool cam_set_from_drawn_outline(Cam *c, const Vec2 *pts, int count);

/* Scales the profile's departure from its smallest radius, i.e. makes the
 * lobes taller or shorter without moving the base circle. */
void cam_scale_lift(Cam *c, double factor);

/* Rotates the whole profile relative to the shaft -- cam timing. */
void cam_rotate_profile(Cam *c, double delta_rad);

/* Pitch radius at cam-local angle `phi` (radians, any value; wrapped). */
double cam_pitch_radius(const Cam *c, double phi);

/* d(pitch radius)/d(phi) -- the follower's velocity per unit cam rotation. */
double cam_pitch_radius_deriv(const Cam *c, double phi);

/* Samples the PHYSICAL cam surface (the pitch curve offset inward by the
 * roller radius) into `out`, `count` points evenly spaced in phi, in
 * cam-local coordinates. `count` must be >= 3. */
void cam_sample_surface(const Cam *c, Vec2 *out, int count);

/* True when that inward offset folds back on itself, i.e. the roller is
 * bigger than the profile's tightest concave radius. Such a cam cannot be cut
 * (or printed) to produce the intended motion -- the classic undercutting
 * failure. Detected by successive offset edges reversing direction. */
bool cam_is_undercut(const Cam *c);

#endif
