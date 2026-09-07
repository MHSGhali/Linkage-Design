#ifndef TEMPLATES_H
#define TEMPLATES_H

#include "mechanism.h"

/* Ready-made mechanisms from the standard teaching catalogue, each built out
 * of ordinary parts so it can be run, dragged, retimed and exported like
 * anything else you draw by hand. */

typedef struct {
    const char *name;     /* shown on the gallery tile */
    const char *blurb;    /* one line, shown under the gallery */
    /* Adds the mechanism centred on `centre`, sized by `scale` (1.0 gives a
     * mechanism a few hundred units across). */
    void (*build)(Mechanism *m, Vec2 centre, double scale);
} Template;

int templates_count(void);
const Template *templates_get(int index);

#endif
