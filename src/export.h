#ifndef EXPORT_H
#define EXPORT_H

#include <stdbool.h>
#include "mechanism.h"

/* Writes a ready-to-run Blender Python script to `filepath`: one cylinder
 * per rigid pairwise link edge and one empty per connector (joint/anchor),
 * positioned from the mechanism's CURRENT connector positions (not frozen
 * rest lengths, so it reflects whatever is currently drawn/edited).
 * Convention: 1 mechanism world unit = 1 mm; the script sets the Blender
 * scene's display units to millimeters and converts coordinates to meters
 * (Blender's internal unit) accordingly. Returns false if the file could
 * not be opened for writing. */
bool export_blender_script(const Mechanism *m, const char *filepath);

#endif
