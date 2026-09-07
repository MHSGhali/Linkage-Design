#include "export.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "xalloc.h"

/* Seconds of motion to sample when nothing is driving the mechanism and it
 * is just falling/swinging under gravity. */
#define GRAVITY_SAMPLE_SECONDS 3.0
/* Upper bound on the simulation step used while sampling. The recorded
 * frames can be far apart in time (a slow motor makes one revolution take
 * many seconds), and stepping that coarsely would let the solver lose the
 * mechanism's branch, so each sample is reached via several small steps. */
#define MAX_SUBSTEP_SECONDS (1.0 / 240.0)
/* How finely a cam's surface is written out. Dense enough that the printed
 * flank is smooth, small enough that the script stays readable. */
#define CAM_EXPORT_SAMPLES 360

/* How long the mechanism takes to come back round to where it started, or 0
 * if nothing is turning.
 *
 * That is set by the SLOWEST motor, not the fastest: with one motor the two
 * are the same, but a drawing machine's arms all turn at whole multiples of a
 * base rate, and it is one turn of the slowest that completes the drawing.
 * Sampling the fastest instead would export a fraction of the figure. */
static double driven_period_seconds(const Mechanism *m) {
    double slowest = 0.0;
    for (int li = 0; li < m->link_count; li++) {
        const Link *l = &m->links[li];
        if (!l->alive || !l->is_driven) continue;
        double speed = fabs(l->motor_speed_deg_s);
        if (speed <= 0.0) continue;
        if (slowest <= 0.0 || speed < slowest) slowest = speed;
    }
    if (slowest <= 0.0) return 0.0;
    return 360.0 / slowest;
}

bool export_blender_script(const Mechanism *m, SolverParams params, const char *filepath) {
    FILE *f = fopen(filepath, "w");
    if (!f) return false;

    /* Index the connectors we actually export, so the per-frame position
     * table lines up with the joint list regardless of deleted ids. */
    int *joint_ids = xmalloc((size_t)(m->connector_count > 0 ? m->connector_count : 1) * sizeof(int));
    int joint_count = 0;
    for (int i = 0; i < m->connector_count; i++) {
        if (m->connectors[i].alive) joint_ids[joint_count++] = i;
    }

    /* Simulate a private copy so the live mechanism is untouched. */
    Mechanism sim;
    mechanism_clone(m, &sim);
    solver_freeze(&sim);

    /* A Geneva indexes once per driver turn and only comes back round after
     * `slots` of them; a gear ratio likewise repeats over several. Sampling
     * one driver revolution would export a fraction of the cycle. */
    int cycle_turns = 1;
    for (int i = 0; i < sim.geneva_count; i++) {
        if (sim.genevas[i].alive && sim.genevas[i].slot_count > cycle_turns) {
            cycle_turns = sim.genevas[i].slot_count;
        }
    }
    for (int i = 0; i < sim.gear_count; i++) {
        const Gear *g = &sim.gears[i];
        if (!g->alive || g->kind == GEAR_RACK || !(g->driven_radius > 1e-9)) continue;
        /* Smallest whole number of driver turns that leaves the driven gear
         * back where it started. */
        double ratio = g->driver_radius / g->driven_radius;
        for (int k = 1; k <= 16; k++) {
            double turns = ratio * (double)k;
            if (fabs(turns - floor(turns + 0.5)) < 1e-6) {
                if (k > cycle_turns) cycle_turns = k;
                break;
            }
        }
    }

    double period = driven_period_seconds(&sim) * (double)cycle_turns;
    bool driven = (period > 0.0);
    double duration = driven ? period : GRAVITY_SAMPLE_SECONDS;
    if (!driven && !mechanism_has_driven_link(&sim) &&
        params.gravity.x == 0.0 && params.gravity.y == 0.0) {
        /* Nothing would move at all; match the app, which runs a motorless
         * mechanism under gravity. */
        params.gravity = (Vec2){ 0.0, 400.0 };
    }

    double frame_dt = duration / (double)EXPORT_FRAMES;
    int substeps = (int)ceil(frame_dt / MAX_SUBSTEP_SECONDS);
    if (substeps < 1) substeps = 1;
    double substep_dt = frame_dt / (double)substeps;

    /* Cams that are exportable: alive, with a live centre we can hang them on. */
    int *cam_ids = xmalloc((size_t)(m->cam_count > 0 ? m->cam_count : 1) * sizeof(int));
    int cam_count = 0;
    for (int i = 0; i < m->cam_count; i++) {
        const Cam *c = &m->cams[i];
        if (!c->alive || c->center_connector_id < 0) continue;
        if (!m->connectors[c->center_connector_id].alive) continue;
        cam_ids[cam_count++] = i;
    }

    Vec2 *samples = xmalloc((size_t)EXPORT_FRAMES * (size_t)(joint_count > 0 ? joint_count : 1) * sizeof(Vec2));
    double *cam_angles = xmalloc((size_t)EXPORT_FRAMES * (size_t)(cam_count > 0 ? cam_count : 1) * sizeof(double));
    int recorded = 0;
    bool jammed = false;

    for (int fr = 0; fr < EXPORT_FRAMES; fr++) {
        if (fr > 0) {
            for (int s = 0; s < substeps; s++) solver_advance(&sim, substep_dt, params);
            if (solver_has_length_violation(&sim, params.length_tol_abs, params.length_tol_rel)) {
                jammed = true;
                break; /* stop at the bind; don't record impossible geometry */
            }
        }
        for (int j = 0; j < joint_count; j++) {
            samples[(size_t)recorded * (size_t)joint_count + j] = sim.connectors[joint_ids[j]].pos;
        }
        for (int j = 0; j < cam_count; j++) {
            cam_angles[(size_t)recorded * (size_t)(cam_count > 0 ? cam_count : 1) + j] =
                mechanism_cam_angle(&sim, cam_ids[j]);
        }
        recorded++;
    }

    fprintf(f,
        "import bpy\n"
        "import math\n"
        "import mathutils\n"
        "\n"
        "# Generated by Linkage Design (mechanism export).\n"
        "# Convention: 1 mechanism world unit = 1 mm.\n"
        "# Press Space in Blender to play the animation.\n"
        "\n"
        "ROD_RADIUS_MM = 3.0  # link/rod cylinder radius -- edit to taste before running\n"
        "JOINT_MARKER_RADIUS_MM = ROD_RADIUS_MM * 1.5\n"
        "CAM_THICKNESS_MM = 8.0  # extrusion depth of each cam disc\n"
        "\n");

    fprintf(f, "FRAME_COUNT = %d\n", recorded);
    fprintf(f, "DURATION_SECONDS = %.6f\n", duration * ((double)(recorded > 1 ? recorded - 1 : 1) / (double)(EXPORT_FRAMES - 1)));
    if (jammed) {
        fprintf(f, "# NOTE: the mechanism bound up partway; the animation stops where it jammed.\n");
    }
    fprintf(f, "\n");

    /* Joint names, in the same order as each frame's position row. */
    fprintf(f, "JOINTS = [\n");
    for (int j = 0; j < joint_count; j++) {
        int cid = joint_ids[j];
        fprintf(f, "    (\"%s_%d\", %s),\n",
                m->connectors[cid].is_anchor ? "Anchor" : "Joint", cid,
                m->connectors[cid].is_anchor ? "True" : "False");
    }
    fprintf(f, "]\n\n");

    /* Rods, referring to joints by their index in JOINTS. */
    fprintf(f, "RODS = [\n");
    for (int li = 0; li < m->link_count; li++) {
        const Link *l = &m->links[li];
        if (!l->alive) continue;
        /* A gear wheel goes out as a disc, not as a rod between its centre and
         * the mark on its face -- the same body the editor draws. */
        if (mechanism_is_gear_body(m, li)) continue;
        int k = l->connector_count;
        for (int i = 0; i < k; i++) {
            for (int j = i + 1; j < k; j++) {
                int ci = l->connector_ids[i], cj = l->connector_ids[j];
                int slot_i = -1, slot_j = -1;
                for (int s = 0; s < joint_count; s++) {
                    if (joint_ids[s] == ci) slot_i = s;
                    if (joint_ids[s] == cj) slot_j = s;
                }
                if (slot_i < 0 || slot_j < 0) continue;
                fprintf(f, "    (\"Link%d_c%dc%d\", %d, %d),\n", li, ci, cj, slot_i, slot_j);
            }
        }
    }
    fprintf(f, "]\n\n");

    /* A slider's rail is usually two bare anchors with no link between them,
     * so nothing would be drawn for it. Emit it as a slim bar: without it the
     * piston appears to float. */
    fprintf(f, "RAILS = [\n");
    for (int si = 0; si < m->slider_count; si++) {
        const Slider *sl = &m->sliders[si];
        if (!sl->alive) continue;
        int a_slot = -1, b_slot = -1;
        for (int k = 0; k < joint_count; k++) {
            if (joint_ids[k] == sl->rail_a_id) a_slot = k;
            if (joint_ids[k] == sl->rail_b_id) b_slot = k;
        }
        if (a_slot < 0 || b_slot < 0) continue;
        fprintf(f, "    (\"Rail%d\", %d, %d),\n", si, a_slot, b_slot);
    }
    fprintf(f, "]\n\n");

    /* Gears and Geneva wheels are discs, not the single spoke their link would
     * otherwise export as. Each is given the joint it turns about and a second
     * joint on the same body, so the script can read its rotation straight out
     * of the frame table rather than needing another one. */
    fprintf(f, "DISCS = [\n");
    /* Every wheel goes out as a disc of its own size -- including one that is
     * meshed with nothing, since it is a body in the mechanism either way. */
    for (int li = 0; li < m->link_count; li++) {
        int centre = -1;
        double radius = mechanism_gear_wheel_radius(m, li, &centre);
        if (!(radius > 1e-6) || centre < 0) continue;
        const Link *l = &m->links[li];
        if (!l->alive) continue;
        int c_slot = -1, r_slot = -1;
        for (int k = 0; k < joint_count; k++) if (joint_ids[k] == centre) c_slot = k;
        for (int i = 0; i < l->connector_count && r_slot < 0; i++) {
            if (l->connector_ids[i] == centre) continue;
            for (int k = 0; k < joint_count; k++) {
                if (joint_ids[k] == l->connector_ids[i]) r_slot = k;
            }
        }
        if (c_slot < 0 || r_slot < 0) continue;
        fprintf(f, "    (\"Wheel%d\", %d, %d, %.4f),\n", li, c_slot, r_slot, radius);
    }
    fprintf(f, "]\n\n");

    /* Cams: the physical surface as a closed polygon in cam-local mm (already
     * inset from the pitch curve by the roller radius, so this is the shape
     * that actually gets cut), plus the joint it spins about and the roller
     * that rides it. */
    fprintf(f, "CAMS = [\n");
    Vec2 *profile = xmalloc((size_t)CAM_EXPORT_SAMPLES * sizeof(Vec2));
    for (int j = 0; j < cam_count; j++) {
        const Cam *c = &m->cams[cam_ids[j]];
        int center_slot = -1, follower_slot = -1;
        for (int s = 0; s < joint_count; s++) {
            if (joint_ids[s] == c->center_connector_id) center_slot = s;
            if (joint_ids[s] == c->follower_connector_id) follower_slot = s;
        }
        if (center_slot < 0) continue;

        cam_sample_surface(c, profile, CAM_EXPORT_SAMPLES);
        fprintf(f, "    (\"Cam_%d\", %d, %d, %.4f, [", cam_ids[j], center_slot, follower_slot,
                c->roller_radius);
        for (int k = 0; k < CAM_EXPORT_SAMPLES; k++) {
            fprintf(f, "(%.4f,%.4f)%s", profile[k].x, profile[k].y,
                    (k + 1 < CAM_EXPORT_SAMPLES) ? "," : "");
        }
        fprintf(f, "]),\n");
    }
    free(profile);
    fprintf(f, "]\n\n");

    /* One row of cam angles (radians) per animation frame, in CAMS order. */
    fprintf(f, "CAM_ANGLES = [\n");
    for (int fr = 0; fr < recorded; fr++) {
        fprintf(f, "    [");
        for (int j = 0; j < cam_count; j++) {
            fprintf(f, "%.6f%s", cam_angles[(size_t)fr * (size_t)(cam_count > 0 ? cam_count : 1) + j],
                    (j + 1 < cam_count) ? "," : "");
        }
        fprintf(f, "],\n");
    }
    fprintf(f, "]\n\n");

    /* One row of joint positions per animation frame. */
    fprintf(f, "FRAMES = [\n");
    for (int fr = 0; fr < recorded; fr++) {
        fprintf(f, "    [");
        for (int j = 0; j < joint_count; j++) {
            Vec2 p = samples[(size_t)fr * (size_t)joint_count + j];
            fprintf(f, "(%.4f,%.4f)%s", p.x, p.y, (j + 1 < joint_count) ? "," : "");
        }
        fprintf(f, "],\n");
    }
    fprintf(f, "]\n\n");

    fprintf(f,
        "scene = bpy.context.scene\n"
        "scene.unit_settings.system = 'METRIC'\n"
        "scene.unit_settings.length_unit = 'MILLIMETERS'\n"
        "scene.frame_start = 1\n"
        "scene.frame_end = max(FRAME_COUNT, 1)\n"
        "if FRAME_COUNT > 1 and DURATION_SECONDS > 0.0:\n"
        "    # Play back at the same rate the mechanism actually runs.\n"
        "    scene.render.fps = 60\n"
        "    scene.render.fps_base = 60.0 * DURATION_SECONDS / (FRAME_COUNT - 1)\n"
        "\n"
        "\n"
        "def mm(v):\n"
        "    return v / 1000.0\n"
        "\n"
        "\n"
        "def make_joint(name, is_anchor):\n"
        "    bpy.ops.object.empty_add(\n"
        "        type='PLAIN_AXES' if is_anchor else 'SPHERE',\n"
        "        radius=mm(JOINT_MARKER_RADIUS_MM), location=(0.0, 0.0, 0.0))\n"
        "    obj = bpy.context.object\n"
        "    obj.name = name\n"
        "    return obj\n"
        "\n"
        "\n"
        "def make_rod(name):\n"
        "    # Unit-depth cylinder: its length is set per frame via scale.z,\n"
        "    # which also lets variable-length links animate correctly.\n"
        "    bpy.ops.mesh.primitive_cylinder_add(radius=mm(ROD_RADIUS_MM), depth=1.0,\n"
        "                                        location=(0.0, 0.0, 0.0))\n"
        "    obj = bpy.context.object\n"
        "    obj.name = name\n"
        "    obj.rotation_mode = 'QUATERNION'\n"
        "    return obj\n"
        "\n"
        "\n");

    fprintf(f,
        "def make_cam(name, profile, roller_radius_mm):\n"
        "    # A solid prism swept from the cam's real surface outline: bottom\n"
        "    # ring, top ring, a quad per edge, and an n-gon cap at each end.\n"
        "    half = mm(CAM_THICKNESS_MM) / 2.0\n"
        "    n = len(profile)\n"
        "    verts = [(mm(x), mm(y), -half) for (x, y) in profile]\n"
        "    verts += [(mm(x), mm(y), half) for (x, y) in profile]\n"
        "    faces = [(i, (i + 1) %% n, n + ((i + 1) %% n), n + i) for i in range(n)]\n"
        "    faces.append(tuple(reversed(range(n))))\n"
        "    faces.append(tuple(range(n, 2 * n)))\n"
        "    mesh = bpy.data.meshes.new(name)\n"
        "    mesh.from_pydata(verts, [], faces)\n"
        "    mesh.validate()\n"
        "    mesh.update()\n"
        "    obj = bpy.data.objects.new(name, mesh)\n"
        "    bpy.context.scene.collection.objects.link(obj)\n"
        "    obj.rotation_mode = 'XYZ'\n"
        "    return obj\n"
        "\n"
        "\n"
        "def make_roller(name, radius_mm):\n"
        "    bpy.ops.mesh.primitive_cylinder_add(radius=mm(radius_mm),\n"
        "                                        depth=mm(CAM_THICKNESS_MM),\n"
        "                                        location=(0.0, 0.0, 0.0))\n"
        "    obj = bpy.context.object\n"
        "    obj.name = name\n"
        "    return obj\n"
        "\n"
        "\n"
        "def make_rail(name):\n"
        "    bpy.ops.mesh.primitive_cylinder_add(radius=mm(ROD_RADIUS_MM) * 0.55, depth=1.0,\n"
        "                                        location=(0.0, 0.0, 0.0))\n"
        "    obj = bpy.context.object\n"
        "    obj.name = name\n"
        "    obj.rotation_mode = 'QUATERNION'\n"
        "    return obj\n"
        "\n"
        "\n"
        "def make_disc(name, radius_mm):\n"
        "    # A gear or Geneva wheel: a flat disc that turns with its body.\n"
        "    bpy.ops.mesh.primitive_cylinder_add(radius=mm(radius_mm),\n"
        "                                        depth=mm(ROD_RADIUS_MM) * 1.2,\n"
        "                                        location=(0.0, 0.0, 0.0))\n"
        "    obj = bpy.context.object\n"
        "    obj.name = name\n"
        "    obj.rotation_mode = 'XYZ'\n"
        "    return obj\n"
        "\n"
        "\n"
        "joints = [make_joint(name, is_anchor) for (name, is_anchor) in JOINTS]\n"
        "rods = [(make_rod(name), i, j) for (name, i, j) in RODS]\n"
        "cams = [(make_cam(name, profile, roller), center, follower)\n"
        "        for (name, center, follower, roller, profile) in CAMS]\n"
        "rollers = [(make_roller(name + '_Roller', roller), follower)\n"
        "           for (name, _c, follower, roller, _p) in CAMS if follower >= 0]\n"
        "rails = [(make_rail(name), i, j) for (name, i, j) in RAILS]\n"
        "discs = [(make_disc(name, r), c, ref) for (name, c, ref, r) in DISCS]\n"
        "\n"
        "\n");

    fprintf(f,
        "for frame_index, positions in enumerate(FRAMES):\n"
        "    frame = frame_index + 1\n"
        "    angles = CAM_ANGLES[frame_index] if frame_index < len(CAM_ANGLES) else []\n"
        "    for obj, (x, y) in zip(joints, positions):\n"
        "        obj.location = (mm(x), mm(y), 0.0)\n"
        "        obj.keyframe_insert('location', frame=frame)\n"
        "    for cam_index, (obj, center, _follower) in enumerate(cams):\n"
        "        cx, cy = positions[center]\n"
        "        obj.location = (mm(cx), mm(cy), 0.0)\n"
        "        obj.rotation_euler = (0.0, 0.0, angles[cam_index] if cam_index < len(angles) else 0.0)\n"
        "        obj.keyframe_insert('location', frame=frame)\n"
        "        obj.keyframe_insert('rotation_euler', frame=frame)\n"
        "    for obj, follower in rollers:\n"
        "        fx, fy = positions[follower]\n"
        "        obj.location = (mm(fx), mm(fy), 0.0)\n"
        "        obj.keyframe_insert('location', frame=frame)\n"
        "    for obj, i, j in rails:\n"
        "        p0 = mathutils.Vector((mm(positions[i][0]), mm(positions[i][1]), 0.0))\n"
        "        p1 = mathutils.Vector((mm(positions[j][0]), mm(positions[j][1]), 0.0))\n"
        "        direction = p1 - p0\n"
        "        length = direction.length\n"
        "        obj.location = (p0 + p1) / 2.0\n"
        "        if length > 1e-12:\n"
        "            obj.rotation_quaternion = direction.to_track_quat('Z', 'Y')\n"
        "        obj.scale = (1.0, 1.0, max(length, 1e-9))\n"
        "        obj.keyframe_insert('location', frame=frame)\n"
        "        obj.keyframe_insert('rotation_quaternion', frame=frame)\n"
        "        obj.keyframe_insert('scale', frame=frame)\n"
        "    for obj, c, ref in discs:\n"
        "        cx, cy = positions[c]\n"
        "        rx, ry = positions[ref]\n"
        "        obj.location = (mm(cx), mm(cy), 0.0)\n"
        "        obj.rotation_euler = (0.0, 0.0, math.atan2(ry - cy, rx - cx))\n"
        "        obj.keyframe_insert('location', frame=frame)\n"
        "        obj.keyframe_insert('rotation_euler', frame=frame)\n"
        "    for obj, i, j in rods:\n"
        "        p0 = mathutils.Vector((mm(positions[i][0]), mm(positions[i][1]), 0.0))\n"
        "        p1 = mathutils.Vector((mm(positions[j][0]), mm(positions[j][1]), 0.0))\n"
        "        direction = p1 - p0\n"
        "        length = direction.length\n"
        "        obj.location = (p0 + p1) / 2.0\n"
        "        if length > 1e-12:\n"
        "            obj.rotation_quaternion = direction.to_track_quat('Z', 'Y')\n"
        "        obj.scale = (1.0, 1.0, max(length, 1e-9))\n"
        "        obj.keyframe_insert('location', frame=frame)\n"
        "        obj.keyframe_insert('rotation_quaternion', frame=frame)\n"
        "        obj.keyframe_insert('scale', frame=frame)\n"
        "\n");

    fprintf(f,
        "def iter_fcurves(action):\n"
        "    # Blender <4.4 exposes action.fcurves directly; 4.4+ moved them\n"
        "    # into slotted actions (layers -> strips -> channelbags).\n"
        "    fcurves = getattr(action, 'fcurves', None)\n"
        "    if fcurves is not None:\n"
        "        for fcurve in fcurves:\n"
        "            yield fcurve\n"
        "        return\n"
        "    for layer in getattr(action, 'layers', []):\n"
        "        for strip in getattr(layer, 'strips', []):\n"
        "            for channelbag in getattr(strip, 'channelbags', []):\n"
        "                for fcurve in channelbag.fcurves:\n"
        "                    yield fcurve\n"
        "\n"
        "\n"
        "# Constant-speed motion should not ease in and out between samples.\n"
        "# The motion is already baked into the keyframes, so treat this as a\n"
        "# nicety: never let an API change here break the whole import.\n"
        "try:\n"
        "    animated = (joints + [rod for (rod, _i, _j) in rods]\n"
        "               + [cam for (cam, _c, _f) in cams] + [r for (r, _f) in rollers]\n"
        "               + [r for (r, _i, _j) in rails] + [d for (d, _c, _r) in discs])\n"
        "    for obj in animated:\n"
        "        action = obj.animation_data.action if obj.animation_data else None\n"
        "        if action is None:\n"
        "            continue\n"
        "        for fcurve in iter_fcurves(action):\n"
        "            for kp in fcurve.keyframe_points:\n"
        "                kp.interpolation = 'LINEAR'\n"
        "except Exception as exc:\n"
        "    print('Linkage Design: could not set linear interpolation (%%s); '\n"
        "          'the animation is still correct.' %% exc)\n"
        "\n"
        "scene.frame_set(1)\n"
        "print('Linkage Design: built %%d joints, %%d rods, %%d rails, %%d discs, '\n"
        "      '%%d cams, %%d frames.'\n"
        "      %% (len(joints), len(rods), len(rails), len(discs), len(cams), FRAME_COUNT))\n");

    fclose(f);
    free(samples);
    free(cam_angles);
    free(cam_ids);
    free(joint_ids);
    mechanism_free(&sim);
    return true;
}
