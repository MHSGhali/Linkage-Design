#ifndef EXPORT_H
#define EXPORT_H

#include <stdbool.h>
#include "mechanism.h"
#include "solver.h"

/* Number of animation samples written by export_blender_script. */
#define EXPORT_FRAMES 120

/* Writes a ready-to-run Blender Python script to `filepath`: one cylinder
 * per pairwise link edge and one empty per connector (joint/anchor), plus a
 * keyframe per sample so the mechanism ANIMATES in Blender exactly as it
 * does here.
 *
 * The motion is produced by simulating a private copy of the mechanism, so
 * the live one is left untouched. A driven mechanism is sampled over one
 * full revolution of its SLOWEST motor -- with one motor the two are the same,
 * but a drawing machine's arms turn at whole multiples of a base rate and it
 * is one turn of the slowest that completes the figure -- extended to as many
 * driver turns as a gear ratio or a Geneva needs to come back round. One with
 * no motor is sampled over a few seconds of gravity. If the mechanism binds
 * partway (a fixed-length link would have to change length), sampling stops
 * there and the animation covers only the part it could actually reach.
 *
 * Convention: 1 mechanism world unit = 1 mm; the script sets the Blender
 * scene's display units to millimeters and converts coordinates to meters
 * (Blender's internal unit) accordingly. Returns false if the file could
 * not be opened for writing. */
bool export_blender_script(const Mechanism *m, SolverParams params, const char *filepath);

#endif
