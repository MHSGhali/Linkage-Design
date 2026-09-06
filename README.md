# Linkage-Design

A general-purpose 2D mechanism editor and simulator, in the spirit of David
Rector's *Linkage*: build a planar mechanism by hand out of connectors
(joints), anchors (grounded connectors), and rigid links (each owning any
number of connectors, not just simple two-point bars), mark one link as a
constant-speed motor, then forward-simulate the resulting motion. Every
link's length is shown live in the canvas, and the mechanism can be exported
as a ready-to-run Blender Python script for 3D printing.

## Build

Requires SDL2 (`brew install sdl2` on macOS, `apt install libsdl2-dev` on
Debian/Ubuntu).

```
make        # builds ./linkage_design
make test   # builds and runs the headless kinematics regression tests
```

## Run

```
./linkage_design
```

- **Edit mode** (mouse):
  - Click empty space: place a connector and select it
  - Drag from empty space: rubber-band select connectors in the box
  - Click a connector: select it (Shift: add/remove from selection)
  - Click a link's edge: select that link
  - Drag a selected connector: move the whole selection (reshapes any
    attached links — rest lengths are only frozen when you press `R`)
- **Edit mode** (keyboard):
  - `L`: link the selected connectors (rigid; needs 2+ selected)
  - `A`: toggle anchor (grounded) on the selected connectors
  - `M`: toggle the selected link as a driven motor (it must have exactly
    one anchor connector, which becomes the pivot); `+`/`-` adjust its speed
  - `V`: toggle the selected link's length between fixed (rigid, the
    default) and variable — a variable-length link is drawn green and is no
    longer enforced by the solver, so its two connectors can move freely
    relative to each other (a telescoping/extensible link), constrained
    only by whatever else touches them. Not available on a driven link.
  - `T`: toggle path tracing on the selected connectors (traced connectors
    are drawn cyan; their path is recorded and drawn while the simulation
    runs, and reset each time you press `R`)
  - `E`: export the mechanism to `linkage_export.py`, a ready-to-run Blender
    Python script (see below)
  - Delete / Backspace: delete the selection
  - Escape: clear the selection
  - Cmd/Ctrl+Z: undo the last edit (placing/moving/deleting a connector,
    linking, toggling anchor/motor/length/tracing, changing motor speed)
- Scroll wheel: zoom in/out, centered on the cursor
- `C`: clear all recorded traces (works in edit mode or mid-simulation,
  without untracing anything)
- `G`: toggle gravity (works in edit mode or mid-simulation). Pulls every
  unconstrained connector downward while the simulation runs; rigid links
  stay exactly rigid throughout — a free connector hanging off an anchor by
  a single link will swing like a pendulum. Off by default; has no effect
  on connectors that are anchors or driven.
- `R`: run the simulation / stop and return to the pre-run layout

Every link's length is drawn as a live numeral next to it (a small hand-drawn
seven-segment display — no font dependency), reflecting its connectors'
current positions.

## Exporting to Blender

Pressing `E` writes `linkage_export.py` in the current directory: a script
that, run inside Blender (Scripting tab, or `blender --python
linkage_export.py`), creates one cylinder per rigid link edge and one empty
per joint (anchors and regular connectors are named `Anchor_N` /
`Joint_N`), positioned from the mechanism's current connector coordinates.
Convention: **1 world unit = 1 mm** — the script sets the scene's display
units to millimeters accordingly. Rod radius is a constant
(`ROD_RADIUS_MM`) at the top of the generated script, since this 2D tool
doesn't track link thickness; edit it before running if you want a
different diameter. This gets the mechanism's skeleton into Blender for you
to build real printable geometry around (fillets, pin holes, wall
thickness, etc.).

## How it works

`src/mechanism.c` owns the data model (connectors, links, rigidity as
pairwise rest-distance constraints) and editing/hit-testing operations.
`src/solver.c` poses driven links directly each frame (rotating their frozen
shape around their anchor pivot) and solves every other connector with a
damped Gauss-Newton iteration (reusing `src/linalg.c`'s linear solver),
warm-started from the previous frame so the motion tracks a continuous
branch without an explicit case for four-bar-style branch ambiguity. This
generalizes to arbitrary topology — open chains, multi-loop mechanisms,
ternary+ links — not just a hardcoded four-bar case. Gravity is a Verlet
integration step applied to free connectors before that same Gauss-Newton
solve, which then projects them back onto the rigid-link constraint
manifold — so gravity is just an external force on otherwise-unconstrained
degrees of freedom, not a separate physics engine. `src/export.c` writes
the Blender script; undo (in `src/main.c`) is a bounded stack of full
`mechanism_clone()` snapshots taken before each edit.
