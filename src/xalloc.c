#include "xalloc.h"

#include <stdio.h>
#include <stdlib.h>

/* Reported rather than silently returning NULL, so an out-of-memory death
 * is distinguishable from a crash in the geometry. */
static void out_of_memory(size_t size) {
    fprintf(stderr, "linkage_design: out of memory allocating %zu bytes\n", size);
    abort();
}

void *xmalloc(size_t size) {
    if (size == 0) size = 1;
    void *p = malloc(size);
    if (!p) out_of_memory(size);
    return p;
}

void *xcalloc(size_t count, size_t size) {
    if (count == 0 || size == 0) { count = 1; size = 1; }
    void *p = calloc(count, size);
    if (!p) out_of_memory(count * size);
    return p;
}

void *xrealloc(void *ptr, size_t size) {
    if (size == 0) size = 1;
    void *p = realloc(ptr, size);
    if (!p) out_of_memory(size);
    return p;
}
