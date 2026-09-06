#ifndef SYNTH_H
#define SYNTH_H

#include <stdbool.h>
#include "vec2.h"

/* Path synthesis: given a curve the user drew, design a mechanism that traces
 * it. Two quite different machines are on offer, and which suits depends
 * entirely on the curve.
 *
 * A FOUR-BAR LINKAGE (synth_fit_four_bar) is five parts and one motor -- by
 * far the more elegant machine, and the one you would actually build. But
 * coupler curves are a restricted family: beans, ellipses, figure-eights and
 * teardrops come out well, while a star or a heart is simply not in the set,
 * and the fit will be visibly poor.
 *
 * A CHAIN OF ROTATING ARMS (synth_fourier_fit) will trace anything at all,
 * at the cost of dozens of parts and a motor on each.
 *
 * The arm chain is a set of arms, each turning at a whole-number multiple of
 * a base speed, hung tip-to-tail from a fixed point. That is a Fourier series
 * made physical: any closed curve is a sum of circular motions
 * at integer frequencies, so with enough arms the pen at the end of the chain
 * traces the curve to whatever accuracy you like. An open stroke is handled
 * by mirroring it into a closed one, so the pen sweeps out along the stroke
 * and back again.
 *
 * The arms are ordered longest first, which is both conventional and the best
 * use of a fixed number of them: dropping the smallest coefficients is the
 * best approximation available for that many terms (Parseval). */

/* ---------------------------------------------------------------------------
 * A four-bar linkage: two ground pivots, three moving link lengths, and where
 * the traced point sits on the coupler -- `coupler_u` along the line from A to
 * B, `coupler_v` perpendicular to it. `branch` selects which of the two
 * circle-circle intersections places B, i.e. which way the linkage is
 * assembled; the two give quite different curves.
 *
 * This is path generation WITHOUT prescribed timing: only the shape of the
 * traced curve is matched, not when the point arrives at each part of it,
 * which is the standard problem and by far the more useful one.
 * ------------------------------------------------------------------------ */

typedef struct {
    Vec2 ground_a;      /* the crank's ground pivot */
    Vec2 ground_b;      /* the rocker's ground pivot */
    double crank;
    double coupler;     /* the distance from A to B */
    double rocker;
    double coupler_u;   /* traced point, measured along A -> B */
    double coupler_v;   /* traced point, measured perpendicular to A -> B */
    int branch;         /* +1 or -1 */
} FourBar;

/* Positions of the moving points at a given crank angle. False if the linkage
 * cannot be assembled there (the coupler and rocker circles fail to meet). */
bool fourbar_pose(const FourBar *fb, double theta, Vec2 *a_out, Vec2 *b_out, Vec2 *point_out);

/* Just the traced point, for cost evaluation. */
bool fourbar_coupler_point(const FourBar *fb, double theta, Vec2 *out);

/* Whether the crank can turn all the way round -- the Grashof condition with
 * the crank as the shortest link. Only such a linkage can be driven
 * continuously by a motor, so the search is restricted to these. */
bool fourbar_crank_rotates(const FourBar *fb);

/* Samples one full revolution of the coupler curve into `out` (`count`
 * points). False if the linkage fails to assemble anywhere in the turn. */
bool fourbar_coupler_curve(const FourBar *fb, Vec2 *out, int count);

typedef struct {
    int random_starts;      /* candidates drawn from the prior */
    int refine_candidates;  /* best few taken on to local refinement */
    int refine_sweeps;      /* pattern-search sweeps per candidate */
    unsigned seed;          /* fixed, so a given drawing gives a given answer */
} SynthParams;

SynthParams synth_default_params(void);

/* How far a four-bar's coupler curve sits from `target`, in world units --
 * the number reported to the user. Lower is better. */
double synth_fit_error(const FourBar *fb, const Vec2 *target, int count, bool closed);

/* Searches for the four-bar whose coupler point best follows `target`.
 * `closed` says whether the drawn path is a loop (matched shape-for-shape) or
 * an open stroke (which need only lie somewhere on the coupler curve).
 * Returns false if no linkage that assembles through a full turn was found. */
bool synth_fit_four_bar(const Vec2 *target, int count, bool closed,
                         SynthParams params, FourBar *out, double *out_error);

/* ---------------------------------------------------------------------------
 * A chain of rotating arms.
 * ------------------------------------------------------------------------ */

#define SYNTH_MAX_ARMS 48
/* How finely the drawn stroke is resampled before decomposing it. */
#define SYNTH_PATH_SAMPLES 256

typedef struct {
    int harmonic;    /* turns per cycle; signed, so arms counter-rotate */
    Vec2 amplitude;  /* the arm as a vector at cycle start; its length is the arm's */
} FourierArm;

/* Decomposes `stroke` into a chain of arms.
 *
 * Arms are added, largest first, until the RMS deviation from the drawing
 * falls below `target_rms` or `max_arms` is reached. Writes the fixed point
 * the chain hangs from, the arms themselves, and the deviation actually
 * achieved. Returns the number of arms, or 0 if the stroke is unusable. */
int synth_fourier_fit(const Vec2 *stroke, int stroke_count, bool closed,
                       int max_arms, double target_rms,
                       Vec2 *out_anchor, FourierArm *out_arms, double *out_rms);

/* Where the pen sits at cycle fraction `t` (0 to 1) -- the sum of the arms,
 * each rotated by its own harmonic. Used to check the design without having
 * to simulate it. */
Vec2 synth_fourier_point(Vec2 anchor, const FourierArm *arms, int count, double t);

/* Resamples a hand-drawn stroke to `out_count` evenly spaced points. A
 * freehand stroke bunches up wherever the hand slowed down, and that would
 * distort the decomposition badly. */
void synth_resample(const Vec2 *pts, int count, bool closed, Vec2 *out, int out_count);

/* Whether a stroke's ends are close enough (relative to its overall size) to
 * read it as a closed loop. */
bool synth_stroke_is_closed(const Vec2 *pts, int count);

/* The largest extent of a path, used to judge a deviation as a fraction of
 * the size of the thing being drawn. */
double synth_path_size(const Vec2 *pts, int count);

#endif
