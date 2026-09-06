# Linkage-Design

A general-purpose 2D mechanism editor and simulator: build a planar
mechanism by hand out of connectors (joints), anchors (grounded connectors),
and rigid links (each owning any number of connectors, not just simple
two-point bars), mark one link as a constant-speed motor, then
forward-simulate the resulting motion. Every link's length is shown live in
the canvas, and the mechanism can be exported as a ready-to-run Blender
Python script for 3D printing.

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

Every command has a button in the toolbar down the left edge **and** a
keyboard shortcut — the two are interchangeable, and each button shows its
shortcut on the right. Buttons grey out when they don't apply (LINK needs two
connectors selected, MOTOR needs a link with exactly one anchor, and so on)
and light up yellow when the thing they control is on (GRAVITY while gravity
is in force, MOTOR on a driven link, VARY on a variable-length link, TRACE on
a traced connector). While the simulation runs, the editing buttons are
disabled, RUN becomes STOP, and GRAVITY and CLEAR keep working.

- **Edit mode** (mouse):
  - Click empty space: place a connector and select it
  - Drag from empty space: rubber-band select connectors in the box
  - Click a connector: select it (Shift: add/remove from selection)
  - Click a link's edge: select that link
  - Drag a selected connector: move the whole selection (reshapes any
    attached links — rest lengths are only frozen when you press `R`)
- **Edit mode** (keyboard):
  - `J`: place a connector at the centre of the view and select it (the
    JOINT button does the same, for when you'd rather not aim a click)
  - `L`: link the selected connectors (rigid; needs 2+ selected)
  - `A`: toggle anchor (grounded) on the selected connectors
  - `M`: toggle the selected link as a driven motor (it must have exactly
    one anchor connector, which becomes the pivot); `+`/`-` adjust its speed
  - `V`: toggle the selected link's length between fixed (rigid, the
    default) and variable. A **fixed** link genuinely cannot change length:
    if the motor would have to stretch or compress it to keep turning, the
    simulation locks up and stops there rather than deforming it (see
    below). A **variable** link is drawn green and still holds its length
    like any other link — it only stretches or compresses when the rest of
    the mechanism leaves it no choice, and then only by as much as the
    geometry actually demands. Not available on a driven link.
  - `T`: toggle path tracing on the selected connectors (traced connectors
    are drawn cyan; their path is recorded and drawn while the simulation
    runs, and reset each time you press `R`)
  - `E`: export the mechanism to `linkage_export.py`, a ready-to-run Blender
    Python script (see below)
  - Delete / Backspace: delete the selection
  - Escape: clear the selection
  - Cmd/Ctrl+Z: undo the last edit (placing/moving/deleting a connector,
    linking, toggling anchor/motor/length/tracing, changing motor speed)
  - Shift+Cmd/Ctrl+Z or Cmd/Ctrl+Y: redo what you just undid. Making a fresh
    edit after undoing discards the redo history, as usual.
- Scroll wheel: zoom in/out, centered on the cursor
- `C`: clear all recorded traces (works in edit mode or mid-simulation,
  without untracing anything)
- `G`: toggle gravity (works in edit mode or mid-simulation). Pulls every
  unconstrained connector downward while the simulation runs; rigid links
  stay exactly rigid throughout — a free connector hanging off an anchor by
  a single link will swing like a pendulum. Has no effect on connectors
  that are anchors or driven.

  **A mechanism with no motor runs under gravity automatically**, since
  otherwise nothing would make it move at all. Pressing `G` takes over from
  there: once you've set it yourself, your choice applies either way.
- `R`: run the simulation / stop and return to the pre-run layout

Every link's length is drawn as a live numeral alongside it (a small
hand-drawn seven-segment display — no font dependency), rotated to run
parallel to the link and offset just clear of it, reflecting its connectors'
current positions.

## Binding (lock-up)

Fixed-length links are treated as genuinely rigid. If the motor reaches a
position the mechanism cannot physically assume without a fixed link
changing length — a non-Grashof linkage hitting its limit position, say —
the simulation rolls that step back and stops, leaving every link at exactly
its rest length, and prints a message. It stays locked until you stop the
simulation (STOP / `R`), so you can see precisely where it bound up. To let it
through, either change the geometry or hit VARY (`V`) on the link that needs
to give.

## Exporting to Blender

Pressing `E` writes `linkage_export.py` in the current directory. Run it
inside Blender (Scripting tab, or `blender --python linkage_export.py`) and
it builds one cylinder per link edge and one empty per joint (named
`Anchor_N` / `Joint_N`) — **and animates them**: press Space in Blender to
watch the mechanism run exactly as it does here.

The motion is produced by simulating a private copy of the mechanism, so
exporting never disturbs what's on screen. A driven mechanism is sampled
over one full revolution of its fastest motor, so the animation loops
cleanly; a motorless one is sampled over a few seconds of gravity. If the
mechanism binds partway, sampling stops at the bind and the animation covers
only what it could actually reach. Playback is set to real time via the
scene's frame rate.

Each rod is a unit-depth cylinder scaled along its local Z per frame, so
variable-length links animate their length correctly too. (Apply the scale
in Blender before exporting for print.)

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
ternary+ links — not just a hardcoded four-bar case.

Variable-length links are handled by solving each frame twice when any are
present: first holding every link at its rest length (variable ones
included), and only if that can't be satisfied, re-solving with just the
genuinely rigid links enforced. That's what makes a variable link stretch
only when the geometry forces it rather than whenever it is permitted to.

Gravity is a Verlet integration step applied to free connectors before that
same Gauss-Newton solve, which then projects them back onto the rigid-link
constraint manifold — so gravity is just an external force on
otherwise-unconstrained degrees of freedom, not a separate physics engine.
`src/export.c` writes the Blender script; undo/redo (in `src/main.c`) is a
pair of bounded stacks of full `mechanism_clone()` snapshots — one pushed
before each edit, the other filled by undoing and discarded by the next edit.

`src/ui.c` owns the toolbar: its layout, hit-testing, and the rules deciding
which buttons are enabled and lit. It deliberately has no SDL dependency, so
those rules are covered by the headless tests alongside the kinematics ones;
`render.c` draws it, including a small stroke-drawn alphabet that keeps the
app free of any font dependency (the same reason link dimensions are drawn
as hand-rolled seven-segment numerals). Every button and its hotkey call the
same function in `main.c`, so the two can't drift apart.
