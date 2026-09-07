#ifndef XALLOC_H
#define XALLOC_H

#include <stddef.h>

/* Allocation policy for this program, in one place.
 *
 * A mechanism is a few thousand doubles; nothing here allocates in response
 * to untrusted size input (the save-file loader bounds every count it reads
 * before allocating from it), so a failed allocation means the machine is
 * genuinely out of memory rather than that this program asked for something
 * absurd. There is no useful recovery from that in an interactive editor:
 * the alternative to dying here is threading an error return through every
 * geometry routine so the app can put up a dialog it has no memory to draw.
 *
 * So the policy is: allocation does not fail, and these wrappers make that
 * true by aborting with a diagnostic if it ever does. Callers may use the
 * result without checking it. This is a deliberate choice, not an omission
 * -- code that genuinely wants to handle exhaustion should call malloc
 * directly and check, and none of this program does.
 *
 * Each wrapper is otherwise exactly its standard counterpart, including
 * xrealloc(NULL, n) behaving as xmalloc(n). Zero-sized requests are rounded
 * up to one byte so a valid, freeable, non-NULL pointer always comes back
 * and "NULL means failure" stays unambiguous. */

void *xmalloc(size_t size);
void *xcalloc(size_t count, size_t size);
void *xrealloc(void *ptr, size_t size);

#endif
