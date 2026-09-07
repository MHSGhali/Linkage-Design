#ifndef PRINT3D_H
#define PRINT3D_H

#include <stdbool.h>
#include <stddef.h>

#include "mechanism.h"

/* Writing a mechanism out as parts you can print and then bolt together.
 *
 * This is a different thing from the Blender export next door. That one writes
 * an ANIMATION: skeleton bars and marker empties that show the motion. This
 * one writes OBJECTS: a folder of STL solids with real thickness, real holes
 * and real teeth, plus a manifest telling you what each one is and how the
 * pile goes together.
 *
 * Three things have to be true for the printed pile to move the way the
 * screen does, and each is handled here:
 *
 *   - Gears must share a module and have whole tooth counts. The model
 *     already guarantees that (see Link.wheel_teeth); this reads it.
 *   - Every pin needs a hole to go through, at a matched running fit.
 *   - Two parts that share a pin must not occupy the same space. Parts are
 *     assigned LAYERS -- a graph colouring over "shares a joint" -- and any
 *     empty layer in a pin's stack gets a spacer washer, so nothing rubs and
 *     nothing floats.
 *
 * Units are millimetres, matching the app's 1 world unit = 1 mm.
 */

/* Defaults tuned for a hobby FDM printer with a 0.4 mm nozzle. */
#define PRINT_DEFAULT_PIN_DIAMETER 3.0
#define PRINT_DEFAULT_THICKNESS 3.0
#define PRINT_DEFAULT_CLEARANCE 0.4
#define PRINT_DEFAULT_LAYER_GAP 0.4
#define PRINT_DEFAULT_WALL 2.0
#define PRINT_DEFAULT_BACKLASH 0.15

typedef struct {
    double module;          /* mm of pitch diameter per tooth; taken from the
                              * mechanism unless overridden */
    double pin_diameter;    /* nominal pin or screw shank */
    double thickness;       /* how thick a plate, gear or cam is printed */
    double clearance;       /* added to the diameter of a hole that must turn
                              * on a pin -- the running fit */
    double layer_gap;       /* air left between one layer and the next */
    double wall;            /* material left around a hole */
    double backlash;        /* mm shaved off each gear's tooth thickness */
    double pressure_angle;  /* radians */
    bool m3_hardware;       /* size holes for M3 screws and list the hardware,
                              * instead of generating printed pins */
    bool baseplate;         /* emit the ground plate that fixes the anchors */
    bool assembly;          /* also write the whole machine assembled, and
                              * pulled apart layer by layer, as two more STLs */
} PrintParams;

PrintParams print_default_params(void);

/* Reads `key=value` overrides -- "m=1.5 pin=3 t=4 clr=0.3 fit=m3" -- into `p`.
 * Values are clamped to what can actually be printed. Returns false and fills
 * `err` on an unknown key or an unreadable number, leaving `p` alone. */
bool print_parse_options(PrintParams *p, const char *opts, char *err, size_t err_size);

/* Writes the whole mechanism into `dir` as one binary STL per part, plus a
 * MANIFEST.txt. The directory is created if it is not there.
 *
 * Every solid is checked for watertightness before it is written; one that
 * fails is skipped and reported rather than handed to a slicer that would
 * quietly print something else. `report` gets a one-line summary suitable for
 * the status bar, including the count of parts written and of any warnings.
 *
 * Returns false only if nothing at all could be written. */
bool print3d_export(const Mechanism *m, PrintParams p, const char *dir,
                     char *report, size_t report_size);

#endif
