#include "templates.h"

#include <math.h>
#include <stddef.h>

#define MOTOR_SPEED 90.0

/* Every template is laid out about `centre` in units that `scale` multiplies,
 * so a template dropped anywhere comes out the right size for the canvas. */
static Vec2 at(Vec2 centre, double scale, double x, double y) {
    return (Vec2){ centre.x + x * scale, centre.y + y * scale };
}

static int join(Mechanism *m, int a, int b) {
    int ids[2] = { a, b };
    return mechanism_add_link(m, ids, 2);
}

static int join3(Mechanism *m, int a, int b, int c) {
    int ids[3] = { a, b, c };
    return mechanism_add_link(m, ids, 3);
}

/* ---------------------------------------------------------------------------
 * Four-bar family
 * ------------------------------------------------------------------------ */

/* The Grashof crank-rocker: the crank turns all the way round while the far
 * rocker swings back and forth. The coupler point traces the classic bean. */
static void build_four_bar(Mechanism *m, Vec2 centre, double s) {
    int o2 = mechanism_add_connector(m, at(centre, s, -200, 60), true);
    int o4 = mechanism_add_connector(m, at(centre, s, 200, 60), true);
    int a  = mechanism_add_connector(m, at(centre, s, -100, 60), false);
    int b  = mechanism_add_connector(m, at(centre, s, 104, -224), false);
    int p  = mechanism_add_connector(m, at(centre, s, -30, -150), false);

    int crank = join(m, o2, a);
    join3(m, a, b, p);          /* ternary coupler carrying the traced point */
    join(m, b, o4);
    mechanism_toggle_driven(m, crank, MOTOR_SPEED);
    mechanism_set_traced(m, p, true);
}

/* Both grounded links turn all the way round -- Grashof with the GROUND as
 * the shortest link. Proportions 5:7:8:9, as in the standard classification. */
static void build_drag_link(Mechanism *m, Vec2 centre, double s) {
    double u = 26.0;
    int o2 = mechanism_add_connector(m, at(centre, s, -2.5 * u, 0), true);
    int o4 = mechanism_add_connector(m, at(centre, s, 2.5 * u, 0), true);
    /* B must sit where the 8u coupler off A meets the 9u rocker off O4, or
     * the built lengths won't be the 5:7:8:9 the classification needs. */
    int a  = mechanism_add_connector(m, at(centre, s, -2.5 * u, 7 * u), false);
    int b  = mechanism_add_connector(m, at(centre, s, 5.35 * u, 8.54 * u), false);

    int crank = join(m, o2, a);   /* 7u */
    join(m, a, b);                /* 8u */
    join(m, b, o4);               /* 9u */
    mechanism_toggle_driven(m, crank, MOTOR_SPEED);
    mechanism_set_traced(m, a, true);
}

/* A parallelogram: opposite links stay parallel, so the coupler translates
 * without ever rotating. */
static void build_parallelogram(Mechanism *m, Vec2 centre, double s) {
    int o2 = mechanism_add_connector(m, at(centre, s, -110, 90), true);
    int o4 = mechanism_add_connector(m, at(centre, s, 110, 90), true);
    int a  = mechanism_add_connector(m, at(centre, s, -110, -30), false);
    int b  = mechanism_add_connector(m, at(centre, s, 110, -30), false);
    int p  = mechanism_add_connector(m, at(centre, s, 0, -110), false);

    int crank = join(m, o2, a);
    join3(m, a, b, p);
    join(m, b, o4);
    mechanism_toggle_driven(m, crank, MOTOR_SPEED);
    mechanism_set_traced(m, p, true);
}

/* Hoeken's linkage: a Grashof crank-rocker whose coupler point runs very
 * nearly straight, at very nearly constant speed, over half its cycle. Watt's
 * own straight-line linkage is a double-rocker, so a constant-speed motor
 * cannot drive it round; Hoeken's does the same job and can be cranked.
 * Proportions ground:crank:coupler:rocker = 2:1:2.5:2.5, with the traced point
 * carried twice the coupler's length out from the crank pin. */
static void build_hoeken(Mechanism *m, Vec2 centre, double s) {
    double u = 60.0;
    int o2 = mechanism_add_connector(m, at(centre, s, -u, 90), true);
    int o4 = mechanism_add_connector(m, at(centre, s, u, 90), true);
    int a  = mechanism_add_connector(m, at(centre, s, 0, 90), false);
    int b  = mechanism_add_connector(m, at(centre, s, 0.5 * u, 90 - 146.97), false);
    int p  = mechanism_add_connector(m, at(centre, s, u, 90 - 293.94), false);

    int crank = join(m, o2, a);
    join3(m, a, b, p);
    join(m, b, o4);
    mechanism_toggle_driven(m, crank, MOTOR_SPEED);
    mechanism_set_traced(m, p, true);
}

/* ---------------------------------------------------------------------------
 * Sliding mechanisms
 * ------------------------------------------------------------------------ */

/* A Scotch yoke: the crank pin runs in a straight slot cut across a sliding
 * yoke, so the yoke's displacement is exactly a sine of the crank angle --
 * pure harmonic motion, with none of a connecting rod's distortion. */
static void build_scotch_yoke(Mechanism *m, Vec2 centre, double s) {
    int o   = mechanism_add_connector(m, at(centre, s, -120, 0), true);
    int pin = mechanism_add_connector(m, at(centre, s, -40, 0), false);
    int crank = join(m, o, pin);
    mechanism_toggle_driven(m, crank, MOTOR_SPEED);

    /* The yoke: a vertical slot that can only slide horizontally, held by two
     * parallel ground rails. */
    int y1 = mechanism_add_connector(m, at(centre, s, -40, -90), false);
    int y2 = mechanism_add_connector(m, at(centre, s, -40, 90), false);
    join(m, y1, y2);

    int lo1 = mechanism_add_connector(m, at(centre, s, -220, -90), true);
    int lo2 = mechanism_add_connector(m, at(centre, s, 220, -90), true);
    int hi1 = mechanism_add_connector(m, at(centre, s, -220, 90), true);
    int hi2 = mechanism_add_connector(m, at(centre, s, 220, 90), true);
    mechanism_add_slider(m, y1, lo1, lo2);
    mechanism_add_slider(m, y2, hi1, hi2);

    /* ...and the pin rides in the yoke's slot. */
    mechanism_add_slider(m, pin, y1, y2);
    mechanism_set_traced(m, y1, true);
}

/* Whitworth's quick return: the crank pin drives a slotted lever pivoted off
 * to one side, so the ram goes out slowly and comes back fast -- what you want
 * for a shaping machine, where only one stroke does any cutting. */
static void build_quick_return(Mechanism *m, Vec2 centre, double s) {
    /* Crank centre offset from the lever pivot by LESS than the crank is long,
     * which is what makes the lever swing right round rather than rock. */
    int o1  = mechanism_add_connector(m, at(centre, s, -60, 0), true);
    int pin = mechanism_add_connector(m, at(centre, s, 30, 0), false);
    int crank = join(m, o1, pin);
    mechanism_toggle_driven(m, crank, MOTOR_SPEED);

    int o2 = mechanism_add_connector(m, at(centre, s, 0, 0), true);
    int lever_end = mechanism_add_connector(m, at(centre, s, 200, 0), false);
    join(m, o2, lever_end);
    mechanism_add_slider(m, pin, o2, lever_end);

    /* The ram, on a rail far enough away and on a rod long enough that it can
     * still be reached when the lever's end swings to the far side. */
    int ram = mechanism_add_connector(m, at(centre, s, 553, -140), false);
    join(m, lever_end, ram);
    int r1 = mechanism_add_connector(m, at(centre, s, 80, -140), true);
    int r2 = mechanism_add_connector(m, at(centre, s, 620, -140), true);
    mechanism_add_slider(m, ram, r1, r2);
    mechanism_set_traced(m, ram, true);
}

/* A scissor lift: crossed arms pinned at their middles, one foot pinned to
 * ground and the other sliding along it. The platform is pinned to one arm and
 * SLIDES on the other -- pinning it to both would fix the whole assembly
 * rigid, which is why a real scissor lift slides at one top corner too. */
static void build_scissor(Mechanism *m, Vec2 centre, double s) {
    double top_y = -165.6, mid_y = -22.8;
    int base_pin = mechanism_add_connector(m, at(centre, s, -140, 120), true);
    int foot     = mechanism_add_connector(m, at(centre, s, 140, 120), false);
    int g1 = mechanism_add_connector(m, at(centre, s, -300, 120), true);
    int g2 = mechanism_add_connector(m, at(centre, s, 300, 120), true);
    mechanism_add_slider(m, foot, g1, g2);

    int mid   = mechanism_add_connector(m, at(centre, s, 0, mid_y), false);
    int top_a = mechanism_add_connector(m, at(centre, s, 140, top_y), false);
    int top_b = mechanism_add_connector(m, at(centre, s, -140, top_y), false);
    join3(m, base_pin, mid, top_a);   /* one arm, carrying the centre pin */
    join3(m, foot, mid, top_b);       /* the other, crossing it */

    int plat_far = mechanism_add_connector(m, at(centre, s, -300, top_y), false);
    join(m, top_a, plat_far);         /* the platform, pinned at top_a... */
    mechanism_add_slider(m, top_b, top_a, plat_far);   /* ...sliding at top_b */

    /* A crank on the ground line pushes the sliding foot in and out. */
    int crank_centre = mechanism_add_connector(m, at(centre, s, 300, 120), true);
    int crank_pin    = mechanism_add_connector(m, at(centre, s, 360, 120), false);
    int crank = join(m, crank_centre, crank_pin);
    mechanism_toggle_driven(m, crank, 45.0);
    join(m, crank_pin, foot);
    mechanism_set_traced(m, top_b, true);
}

/* ---------------------------------------------------------------------------
 * Gears and intermittent motion
 * ------------------------------------------------------------------------ */

/* Rack and pinion: rotation into straight-line travel, at exactly the arc
 * length rolled off the pitch circle. */
static void build_rack_pinion(Mechanism *m, Vec2 centre, double s) {
    double r = 70;
    int c = mechanism_add_connector(m, at(centre, s, 0, 60), true);
    int t = mechanism_add_connector(m, at(centre, s, r, 60), false);
    int pinion = join(m, c, t);
    mechanism_set_wheel_radius(m, pinion, r);   /* a pinion is a wheel, drawn as a disc */
    mechanism_toggle_driven(m, pinion, 60.0);

    int ra = mechanism_add_connector(m, at(centre, s, -200, 60 - r), false);
    int rb = mechanism_add_connector(m, at(centre, s, 200, 60 - r), false);
    int rack = join(m, ra, rb);
    mechanism_add_rack(m, pinion, c, rack, ra, (Vec2){ 1.0, 0.0 });
    mechanism_set_traced(m, ra, true);
}

/* A disc cam pushing a translating roller follower: the profile decides the
 * motion, which is what makes cams the tool of choice when the motion is
 * awkward. */
static void build_cam_follower(Mechanism *m, Vec2 centre, double s) {
    int c   = mechanism_add_connector(m, at(centre, s, 0, 60), true);
    int tip = mechanism_add_connector(m, at(centre, s, 70, 60), false);
    int shaft = join(m, c, tip);
    mechanism_toggle_driven(m, shaft, MOTOR_SPEED);

    int follower = mechanism_add_connector(m, at(centre, s, 0, 60 - 120), false);
    int cam = mechanism_add_cam(m, shaft, c, follower);
    if (cam >= 0) mechanism_set_traced(m, follower, true);
}

/* ------------------------------------------------------------------------ */

static const Template TEMPLATES[] = {
    { "FOUR BAR",  "crank-rocker; the coupler point traces a bean",   build_four_bar },
    { "DRAG LINK", "both grounded links turn right round",            build_drag_link },
    { "PARALLEL",  "the coupler translates without rotating",         build_parallelogram },
    { "HOEKEN",    "traces a very nearly straight line",             build_hoeken },
    { "YOKE",      "Scotch yoke: exactly sinusoidal motion",          build_scotch_yoke },
    { "QUICK RTN", "Whitworth: slow out, fast back",                  build_quick_return },
    { "SCISSOR",   "crossed arms on a sliding foot",                  build_scissor },
    { "RACK",      "rack and pinion: turn into travel",               build_rack_pinion },
    { "CAM",       "profile drives a translating roller follower",    build_cam_follower },
};

int templates_count(void) {
    return (int)(sizeof TEMPLATES / sizeof TEMPLATES[0]);
}

const Template *templates_get(int index) {
    if (index < 0 || index >= templates_count()) return NULL;
    return &TEMPLATES[index];
}
