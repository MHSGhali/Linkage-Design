#ifndef GEARING_H
#define GEARING_H

#include <stdbool.h>
#include "vec2.h"

/* Involute spur gear and rack tooth geometry.
 *
 * This is the one place tooth shape is decided. The model quantizes wheel
 * radii through it (a wheel is a whole number of teeth on a shared module),
 * the canvas draws the outline it returns, and the STL export extrudes that
 * same outline -- so what meshes on screen is what meshes on the print bed.
 *
 * Everything is in millimetres, matching the app's 1 world unit = 1 mm.
 *
 * A gear's size is its MODULE and its TOOTH COUNT, not its radius. Two gears
 * mesh only if they share a module, and then they mesh at exactly one centre
 * distance: module * (Na + Nb) / 2. That is why the radius cannot stay a free
 * real number -- a pair drawn 0.4 mm apart looks fine and prints as two wheels
 * that either jam or never touch.
 *
 * Pure geometry, no SDL and no allocation: callers pass their own buffer.
 */

/* Fewer teeth than this and a 20-degree involute has almost no flank left
 * above its base circle; the tooth becomes a spike. */
#define GEARING_MIN_TEETH 8
#define GEARING_MAX_TEETH 400

/* Below this the flank is short enough that the pair runs roughly rather than
 * smoothly (the classic undercutting limit is 17 at 20 degrees). Printable,
 * but worth saying so in the manifest. */
#define GEARING_WEAK_TEETH 14

/* Standard ISO tooth proportions, as multiples of the module. */
#define GEARING_ADDENDUM 1.0
#define GEARING_CLEARANCE 0.25

/* The default pressure angle, in radians (20 degrees). */
#define GEARING_PRESSURE_ANGLE 0.34906585039886590

typedef struct {
    double module;          /* mm of pitch diameter per tooth */
    int teeth;
    double pressure_angle;  /* radians */
    double backlash;        /* mm shaved off THIS gear's tooth thickness at the
                              * pitch circle; a meshing pair gets the sum of
                              * the two, which is the play you can feel */
    int flank_samples;      /* involute points per flank; >= 2. The canvas uses
                              * a few, the export uses many */
} GearSpec;

/* Pitch radius, and the tooth count nearest a wanted radius (clamped to the
 * limits above). These are inverses of each other on the tooth grid. */
double gearing_pitch_radius(double module, int teeth);
int gearing_teeth_for_radius(double module, double radius);

/* The one distance at which two gears of a shared module actually mesh. */
double gearing_center_distance(double module, int teeth_a, int teeth_b);

/* Base, tip (addendum) and root (dedendum) radii of a spec. */
double gearing_base_radius(const GearSpec *s);
double gearing_tip_radius(const GearSpec *s);
double gearing_root_radius(const GearSpec *s);

/* How many points gearing_tooth_outline will write for this spec, so callers
 * can size a buffer without guessing. */
int gearing_outline_capacity(const GearSpec *s);

/* Writes the gear's closed outline into `out` as a counter-clockwise polygon
 * in gear-local coordinates (centre at the origin, one tooth centred on the
 * +x axis). Returns the number of points written, or 0 if `s` is nonsense or
 * `cap` is too small.
 *
 * The flanks are true involutes; below the base circle -- where a real cutter
 * would undercut -- the flank drops radially to the root circle, which is what
 * a generated profile looks like there. A tooth that would come to a point
 * before reaching the addendum circle is truncated to leave a top land, so a
 * small pinion prints with a tip you can actually see. */
int gearing_tooth_outline(const GearSpec *s, Vec2 *out, int cap);

/* --- Rack ---------------------------------------------------------------
 *
 * The limiting case of a gear of infinite radius: straight flanks at the
 * pressure angle. Local frame is x along the rack, pitch line at y = 0, teeth
 * pointing towards +y (i.e. towards the pinion), body hanging below. */

/* Whole teeth that fit in `length` mm of rack. */
int gearing_rack_teeth(double module, double length);

int gearing_rack_capacity(const GearSpec *s, double length);

/* Writes the rack's closed CCW outline, `back` mm of solid bar below the root
 * line. Returns points written, or 0. */
int gearing_rack_outline(const GearSpec *s, double length, double back, Vec2 *out, int cap);

#endif
