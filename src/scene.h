#ifndef SCENE_H
#define SCENE_H

#include <stdbool.h>
#include <stddef.h>

#include "mechanism.h"
#include "solver.h"

/* Saving and reopening a mechanism.
 *
 * Until this existed, closing the window destroyed the work: EXPORT wrote a
 * Blender animation, which is an output, not a document you can carry on
 * editing. This is the document.
 *
 * The file is plain text, one record per line, so it diffs and can be read
 * and repaired by hand. Ids in the file are positions among the LIVE entities
 * only -- deleting a part leaves a tombstone in memory, but there is no reason
 * to write tombstones to a file, so save renumbers and load reads a dense
 * mechanism back.
 *
 * Only what was chosen is stored. Everything derived -- rest lengths, frozen
 * motor offsets, mesh pitch radii, contact flags, recorded traces -- is
 * rebuilt on load by the same calls that build a mechanism by hand.
 */

#define SCENE_FORMAT_VERSION 2
#define SCENE_EXTENSION ".linkage"

/* Writes `m` (and the parts of `params` that belong to the document, i.e.
 * gravity) to `path`. Returns false and fills `err` on failure. */
bool scene_save(const Mechanism *m, const SolverParams *params, const char *path,
                 char *err, size_t err_size);

/* Reads `path` into `m` and `params`. On success the mechanism previously in
 * `*m` is freed and replaced; on failure both are left exactly as they were
 * and `err` says why, so a bad file cannot cost you what is on screen. */
bool scene_load(Mechanism *m, SolverParams *params, const char *path,
                 char *err, size_t err_size);

/* True if `path` already names something on disk -- so EXPORT and SAVE can
 * ask before writing over it. */
bool scene_file_exists(const char *path);

#endif
