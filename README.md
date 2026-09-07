# Linkage-Design

A general-purpose 2D mechanism editor and simulator: build a planar
mechanism by hand out of connectors (joints), anchors (grounded connectors),
rigid links (each owning any number of connectors, not just simple two-point
bars), sliders and pins-in-slots, gears, racks, disc cams and Geneva wheels,
mark a link as a constant-speed motor, then forward-simulate the resulting
motion. Or start from the **template gallery** of famous mechanisms and take
one apart.
Or work the other way round: **draw a path you want a point to follow and the
program designs a machine that traces it** — either a neat four-bar linkage,
or, for shapes no linkage can manage, a chain of rotating arms that will
redraw anything you can draw.
Every link's length is shown live in the canvas, traced points are plotted
against time, and the mechanism can be exported either as a ready-to-run
Blender Python script or as a folder of printable STL parts.

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

The window is resizable, and **`H` lists every key inside it** — there is
nothing you have to read here first.

Every command has a button in the toolbar down the left edge **and** a
keyboard shortcut — the two are interchangeable, and each button shows its
shortcut on the right. Buttons grey out when they don't apply (LINK needs two
connectors selected, MOTOR needs a link with exactly one anchor, and so on)
and light up yellow when the thing they control is on (GRAVITY while gravity
is in force, MOTOR on a driven link, VARY on a variable-length link, TRACE on
a traced connector). Pressing a key runs it through exactly the same rules the
button obeys: a command that isn't available says what it wants selected
first, in the same words its tooltip uses. While the simulation runs, the
editing buttons are disabled, RUN becomes STOP, and GRAVITY, CLEAR, FIT and
PAUSE keep working.

**Everything the program says appears on the canvas**, bottom-left, and fades
after a few seconds; a condition that is still true (a jam) stays up as a
banner across the top until it isn't. The same text is still printed to the
terminal for anyone running it from one.

- **Edit mode** (mouse):
  - Click empty space: place a connector and select it
  - Drag from empty space: rubber-band select connectors in the box
  - Click a connector: select it (Shift: add/remove from selection)
  - Click a link's edge: select that link
  - Drag a selected connector: move the whole selection (reshapes any
    attached links — rest lengths are only frozen when you press `R`)
  - Middle-drag: pan the view. Arrow keys do the same, `0` puts it back to
    1:1, and `F` (FIT) frames the whole mechanism — use it whenever something
    has wandered off the edge.
- Hovering a toolbar button for a moment shows a **tooltip** saying what it
  does and what it wants selected first — the buttons that behave differently
  depending on the selection (GEAR above all) say so there.
- **Edit mode** (keyboard):
  - `S`, `O`, `W`: **SLIDER**, **GEAR**, **GENEVA**. Press one with nothing
    selected and it draws a whole working assembly into the middle of the view
    — motor, bars, joint and a traced point, running the moment you press `R`.
    You do not have to know what to arrange first. Everything about it is then
    editable by hand: see **Editing a joint** below.

    If you *have* the right parts selected it joins those instead, so a joint
    can still be added to something you built yourself:

    - `S`: three connectors — the two farthest apart become the rail and the
      third the pin that runs along it. Rails on anchors give a slide on
      ground; rails on a moving link give a pin in that link's slot.
    - `O`: two grounded centres to mesh a gear pair, or a grounded centre plus
      a point on a free bar for a rack and pinion.
    - `W`: the motor's centre and the wheel's centre.
  - `N`: open the **TEMPLATE gallery** — a grid of famous mechanisms; click one
    to drop it on the canvas, running and ready. See **The template gallery**
    below.
  - `J`: place a connector at the centre of the view and select it (the
    JOINT button does the same, for when you'd rather not aim a click)
  - `L`: link the selected connectors (rigid; needs 2+ selected)
  - `A`: toggle anchor (grounded) on the selected connectors
  - `M`: toggle the selected link as a driven motor (it must have exactly
    one anchor connector, which becomes the pivot); `+`/`-` adjust its speed.
    On a **gear wheel** `+`/`-` resize the wheel instead, so `Alt`+`+`/`-` is
    the speed there — it always means the motor, whatever is selected.
  - `K`: the cam tool. Arm it, then **drag on the canvas to draw the cam's
    outline** — the shape you draw becomes the cam. Nothing needs selecting
    first: the shaft, its motor, and the roller follower are all created for
    you, seated and ready to run. Click once instead of dragging for a
    default cam. Escape cancels. Afterwards, click a cam's outline to select
    it: `+`/`-` scale its lift, `[` and `]` shift its timing. See **Cams and
    followers** below.
  - `P`: **LINKAGE** — draw a curve and a four-bar is fitted to it. Five parts
    and one motor, but only the curves four-bars can trace.
  - `B`: **ARMS** — draw *any* curve and a chain of rotating arms is built to
    redraw it exactly. Dozens of parts, but nothing is out of reach.

    Both build a complete, running, editable mechanism from one stroke. See
    **Designing from a path** below for which to reach for.
  - `V`: toggle the selected link's length between fixed (rigid, the
    default) and variable. A **fixed** link genuinely cannot change length:
    if the motor would have to stretch or compress it to keep turning, the
    simulation locks up and stops there rather than deforming it (see
    below). A **variable** link is drawn green and still holds its length
    like any other link — it only stretches or compresses when the rest of
    the mechanism leaves it no choice, and then only by as much as the
    geometry actually demands. Not available on a driven link.
  - `T`: toggle path tracing on the selected connectors. Each traced
    connector takes a colour of its own; its path is drawn in the canvas and
    its x and y are plotted against time below (see **The motion plot**).
    Both reset each time you press `R`.
  - `Shift+E`: write a folder of printable STL parts — gears with real teeth,
    plates with pin holes, a baseplate and the pins to join them (see
    **Printing it** below).
  - `E`: export the mechanism to a ready-to-run Blender Python script. You are
    asked for the file name, and told if one of that name already exists (see
    **Exporting to Blender** below).
  - Cmd/Ctrl+`S`: **save the mechanism** so you can come back to it, and
    Cmd/Ctrl+`O` to open one again. Shift+Cmd/Ctrl+`S` saves under a new name.
    This is the machine itself, not the animation EXPORT writes — see
    **Saving your work** below.
  - `H` or `?`: the list of every key. `F`: fit the view to the mechanism.
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
- Space: **pause** where it is, without putting anything back. While paused,
  `.` advances exactly one frame — which is how you watch a linkage go through
  a dead centre. `<` and `>` run it anywhere from a tenth of real time to four
  times it, live.
- While it runs you can still click a part to select it and use `+`/`-` to
  change a motor's speed, so "what if it turned faster" needs no stopping.

## The motion plot

Below the canvas is a plot of every traced connector's **x and y against
time**. Each traced connector gets a colour, used for its dot, its path in the
canvas and its two curves, so a curve can be matched to the point that drew
it; x is the brighter of the pair and y the dimmer, each labelled where it
ends. Both axes auto-scale, and x and y share one value axis so their
magnitudes stay comparable.

Frames are not uniform in length, so each trace sample is stamped with the
simulation time it was taken at rather than being assumed evenly spaced --
the time axis is real seconds. `C` (CLEAR) empties it, and it refills as the
simulation runs.

Every link's length is drawn as a live numeral alongside it (a small
hand-drawn seven-segment display — no font dependency), rotated to run
parallel to the link and offset just clear of it, reflecting its connectors'
current positions.

## The template gallery

TEMPLATE (`N`) opens a grid of nine standard mechanisms — the ones a theory
of machines course works through. Click one and it lands on the canvas as
ordinary parts: drag them, retime the motor, trace different points, export
them. Nothing in a template is special-cased.

Each tile is a real miniature, built by running the very same code the button
inserts and scaling the result to fit, so a tile can never drift out of step
with what you actually get.

**Every one of them can also be built by hand.** The templates use no private
back door: each is assembled from the same connectors, links, motors, sliders,
gears, cams and Geneva wheels the toolbar gives you, so a template is a
starting point or a worked example rather than a black box. Drop one in and
take it apart, or build your own from nothing.

The crank-slider, the gear train and the Geneva are **not** in the gallery, and
that is deliberate: their own toolbar buttons draw them outright. Press SLIDER,
GEAR or GENEVA on an empty canvas and the whole assembly appears, ready to be
dragged into whatever proportions you wanted — a template would only be a worse
copy of what the button already gives you.

| | |
|---|---|
| FOUR BAR | Grashof crank-rocker; the coupler point traces the classic bean |
| DRAG LINK | both grounded links turn right round (proportions 5:7:8:9) |
| PARALLEL | a parallelogram: the coupler translates without ever rotating |
| HOEKEN | traces a very nearly straight line at nearly constant speed |
| YOKE | Scotch yoke: exactly sinusoidal motion, no rod distortion |
| QUICK RTN | Whitworth's quick return: slow cutting stroke, fast return |
| SCISSOR | crossed arms on a sliding foot |
| RACK | rack and pinion: rotation into travel |
| CAM | a profile driving a translating roller follower |

Two notes on what is and isn't here. **Watt's own straight-line linkage is a
double-rocker** — a constant-speed motor cannot drive it round, it would just
bind — so the gallery carries Hoeken's linkage, which does the same job and can
be cranked. And a **scissor lift pinned at both top corners has zero mobility**;
the template slides at one of them, which is how real ones are built too.

## Joints beyond the pin

Connectors are revolute joints: two links sharing one turns about it. Three
more kinds of joint make the rest of the catalogue possible.

**Sliders and pins-in-slots** are one constraint: a connector held on the line
through two others. When the two rail points are anchors that is a prismatic
joint sliding on ground — a piston, a scissor lift's foot. When they belong to
a moving link it is a pin running in that link's slot — the Whitworth's slotted
lever, the Scotch yoke. A slider dragged off its rail binds the mechanism, the
same as stretching a fixed-length link.

**Gears** roll without slipping, so their rotations are locked in the ratio of
their radii: opposite in sense for an external mesh, the same for an internal
one. A **rack** is the limiting case of a gear of infinite radius, so the
pinion's rotation becomes travel along the rack's axis — exactly the arc length
rolled off the pitch circle.

**Geneva wheels** turn steady rotation into interrupted rotation: the driver's
pin enters a slot, indexes the wheel one step, and leaves, and a locking disc
holds it still until the pin comes round again. The wheel turns the **opposite**
way to the driver, as an external Geneva does -- the pin sweeping forwards past
the line of centres swings the slot it occupies backwards. The wheel's angle is computed
in closed form from the driver's, which is exact and cannot jam. The
proportions are the standard shock-free ones — the pin has to enter and leave
*along* the slot, which forces the crank radius to be the centre distance times
sin(pi/slots) and fixes everything else. Trace the wheel and the plot shows the
staircase: a step, a long dwell, a step.

Gears and Geneva wheels are *posed* from their driver rather than solved, in
the same dependency-ordered pass that places motors — so a gear can drive a
gear that drives another, and each is placed only once its driver has been.

### A gear is a wheel, not a bar

Every other body here is a bar between pins, but a gear is a disc, and drawing
a rod from its centre out to some point on it would be a spoke nobody asked
for. So a wheel is exactly that: a hub, a pitch circle, and **a mark on its
rim** — and that mark is a connector like any other, so TRACE plots its path
and the time-series panel shows it. The mark sits *on* the pitch circle rather
than somewhere inside it, so tracing a wheel draws a circle of the wheel's own
radius: the trace is the gear, not a smaller circle of no significance. It
exports to Blender as a disc, meshed or not, and to STL as a real involute
gear of its own tooth count.

A wheel is also a body with **a size of its own**, not a share of the gap
between two centres. Click anywhere on its face to select it and `+`/`-`
resize that wheel alone — a whole tooth at a time, since that is what a wheel's
size is (see **Printing it**). The number beside a wheel is its tooth count.

### Building a gear train

**To mesh two gears: click one wheel's face, shift-click the other, press
GEAR.** Shift-click is what adds a second wheel to the selection.

GEAR does one of two things, and which one is plain from what is selected:

| selected | GEAR does |
|---|---|
| nothing | puts down **one** wheel, on its own, clear of whatever is already there |
| two or more wheels | **meshes** them, in the order they were made |

So wheels get placed and sized first and connected afterwards. One press is one
wheel — pressing it repeatedly never stacks wheels on the same spot, and never
connects anything you did not ask it to.

Meshing makes the mesh real: two wheels of fixed size touch at exactly one
distance, so the driven one **slides along the line of centres until they
touch**, however far apart they were drawn. Afterwards, dragging a meshed wheel
swings it round its partner instead of pulling the teeth apart, and resizing
either one closes the gap again by itself.

Because meshing names the wheels rather than guessing, **one wheel can drive
several** — mesh a hub to one wheel, then to another, then to a third. A wheel
takes its motion from one place, so meshing onto a wheel that is already driven
is refused.

Press `M` on a wheel and it becomes the one that drives. The rest of its train
is re-pointed to flow outwards from it, so any wheel can be named the driver
whatever order you happened to mesh things in. The driving wheel is drawn in
the motor's colour, since it has no red bar to give it away.

### Editing a joint### Editing a joint

A joint is a thing you can click, not just a rule you declared once. Click a
Geneva wheel's rim or its locking disc, or a slider's rail, and that joint is
selected. Then:

- `+`/`-` change **a Geneva's slot count** (and, on a wheel, its tooth count). The
  same keys still change motor speed and cam lift when one of those is what's
  selected.
- DELETE removes it and leaves the bars standing. Deleting a wheel takes any
  mesh it was part of with it.

A Geneva's proportions are still read off the geometry before every solve: its
crank radius follows its centre distance and slot count, and a rack's pitch
radius is the whole number of teeth nearest how far the pinion's centre stands
off the bar, so sliding the bar retimes it a tooth at a time. What is drawn on
the canvas — each size, the slot count — is what the solver uses.

## Designing from a path

Everything else in this program works forwards: you build a mechanism and see
what it does. The path tools work backwards. Draw a curve and one of them
designs a machine that traces it, then builds it out of ordinary parts you can
drag, retime and export like anything else.

There are two, because there is a real trade to make.

**LINKAGE (`P`)** fits a **four-bar**: two grounded pivots, a motorised crank,
a ternary coupler carrying the traced point, and a rocker closing the loop.
Five parts, one motor — the machine you would actually build. But coupler
curves are a restricted family. Beans, ellipses, figure-eights, teardrops and
D-shapes come out well; a star or a heart is simply not in the set. The tool
still returns its closest attempt and prints the average miss as a percentage
of your drawing, and if that is more than a few percent it says outright that
the path is beyond a four-bar and points you at ARMS.

Fitting is a search, and an honest one: four-bar synthesis is badly
multi-modal, so it draws hundreds of thousands of candidates from a prior
scaled to your drawing, discards the great majority that cannot turn a full
revolution, keeps the best few dozen, and polishes each with a derivative-free
pattern search. That takes a few seconds. The seed is fixed, so the same
drawing always gives the same linkage.

**ARMS (`B`)** builds a **chain of rotating arms**. Any closed curve is a sum
of circular motions at whole-number frequencies, so arms hung tip to tail,
each turning at its own multiple of a base speed, have a pen at the end that
traces it — a Fourier series made out of parts. Because the fit is a transform
rather than a search it is instant, exact in the limit, and works on anything:
a star takes about 35 arms and a heart about 29, both landing near a quarter
of a percent. The cost is the part count and a motor on every arm.

Arms are added longest first, which is the best use of however many you get:
dropping the smallest terms is provably the closest approximation for that
count. The tool keeps adding until the average miss falls below a quarter of a
percent of your drawing, or it hits 48 arms.

Both tools read your stroke the same way. Finish near where you started and it
is a **closed loop**, traced over and over. Leave it open and it is an **open
stroke**: for a linkage that means the drawn part need only lie somewhere on
the coupler curve, and for arms the stroke is **mirrored** so the pen sweeps
out along it and back. Mirroring rather than just joining the ends matters — a
straight jump from end to start is a step, and a step needs endless harmonics,
so it would ripple along the whole curve.

Only the *shape* is matched, never the timing: where along the curve the pen
sits at any moment is whatever the machine gives. That is the standard problem
and by far the more useful one.

Whichever you use, the path you drew stays on screen behind the result in a
dim outline, so you can judge the fit by eye as well as by the number printed
to the terminal. CLEAR (`C`) removes it.

Two things worth knowing about arm chains. The whole figure takes one turn of
the *slowest* arm — eight seconds by default — so let it run a full cycle
before judging it. And a drawing is one continuous stroke: a face with
separate eyes and a mouth would need the pen to lift, which no single chain
can do, so draw it without lifting or build it from several paths.

## Cams and followers

A cam is a shaped disc that turns with a link (usually a motor) and pushes a
roller follower along a fixed axis. **You draw the shape**: arm the cam tool
(CAM / `K`) and drag a loop on the canvas. The whole rig is then built in one
go — the shaft grounded at the outline's centroid, a motor to turn it, and the
roller follower seated on the profile — so there is nothing to select and
nothing to assemble by hand. A single click instead of a drag gives a default
rise-dwell-fall cam of the classic kind.

The shaft sits at the **centroid** of what you drew, which is what makes the
cam roughly balanced. It also means an off-centre-looking blob gives less lift
than you might expect: lift comes from how far the outline swings about its
own centroid, not about the middle of the screen. Select the cam and use
`+`/`-` to scale the lift up or down, and `[` / `]` to rotate the profile
against the shaft — the same motion, earlier or later in the turn, which is
cam timing.

Internally the profile is kept as the **pitch curve** — the path of the
roller's *centre* — as a radius every two degrees, splined between samples.
Your drawn outline is the physical surface, so the pitch curve is that surface
offset outward by the roller's radius along its normal. Keeping the pitch
curve is what makes contact exact: the follower axis is radial, so the
follower's distance from the centre is simply the pitch radius whenever the
two are touching. A default (undrawn) cam fills the same table from the
**cycloidal** rise / high dwell / fall / low dwell law, which has zero
velocity *and* acceleration at both ends of every segment, so the segments
join smoothly and the follower sees no jerk spike.

A freehand outline routinely has a concave stretch tighter than the roller
first guessed for it. Forcing the roller in anyway would **undercut**: it
physically cannot reach into the notch, and the cut cam would not give the
intended motion. Rather than hand back a profile unlike your drawing, the
roller is shrunk until it fits the shape you actually drew — which is what a
cam designer would do. Only a genuinely sharp notch still warns.

**Contact is one-sided.** The cam can push the follower out but never pull it
back, so what holds the follower down is a preloaded return spring. Turn the
cam slowly and the follower tracks the profile exactly; spin it fast enough
and the profile falls away quicker than the spring can push the follower after
it, and the follower leaves the cam and drops back on -- real cam float. The
roller is drawn green while it is touching and red while it is airborne, so
this is visible rather than something to infer. (Contact never *jams*, being
one-sided; the follower's guide axis is a hard constraint like any other, so
dragging the follower off its axis does lock up -- see below.)

Scope: the follower axis is radial, i.e. it passes through the cam centre.
That is the textbook arrangement; offset followers and a general-purpose
slider joint would be the natural next step.

## Binding (lock-up)

Fixed-length links are treated as genuinely rigid. If the motor reaches a
position the mechanism cannot physically assume without a fixed link
changing length — a non-Grashof linkage hitting its limit position, say —
the simulation rolls that step back and stops, leaving every link at exactly
its rest length, and says so in a banner across the top of the canvas (RUN
also changes to read JAMMED). It stays locked until you stop the
simulation (STOP / `R`), so you can see precisely where it bound up. To let it
through, either change the geometry or hit VARY (`V`) on the link that needs
to give.

## Saving your work

Cmd/Ctrl+`S` writes the mechanism to a `.linkage` file: a plain-text document
listing the pins, bodies, joints and cam profiles, which reopens with
Cmd/Ctrl+`O` exactly as you left it — same geometry, same motors, same motion.
Everything that can be worked out again (rest lengths, pitch radii, recorded
traces) is left out and rebuilt on opening, so the file stays small and
readable.

The title bar shows the file's name and marks it with `*` while there are
unsaved changes, and closing the window with changes outstanding asks first.
A file that turns out to be damaged is refused with a message saying which
line failed — what is already on screen is never disturbed by a failed open.

Names without a directory go in the working directory, or in your home
directory when that isn't writable (which is what happens when the app is
launched from a file manager). Either way the message tells you the full path
it wrote.

## Exporting to Blender

Pressing `E` asks for a name and writes a Blender script — it used to write
`linkage_export.py` over whatever was already called that, without asking. Run it
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

Sliders, gears and Geneva wheels come across too: a slider's rail becomes a
slim bar (without it the piston would appear to float, since a rail is often
just two bare anchors), and gears and Geneva wheels become discs at their pitch
radius, keyframed from the joints already in the frame table rather than
needing one of their own.

**Cams are exported as real solids**, not skeletons: each one becomes a closed
watertight mesh swept from its actual cut profile (already offset inward by
the roller radius) to `CAM_THICKNESS_MM`, keyframed with the rotation it has
in the simulation, and its roller comes along as a cylinder. That is the part
you would send to a printer.

Convention: **1 world unit = 1 mm** — the script sets the scene's display
units to millimeters accordingly. Rod radius is a constant
(`ROD_RADIUS_MM`) at the top of the generated script, since this 2D tool
doesn't track link thickness; edit it before running if you want a
different diameter. This gets the mechanism's skeleton into Blender to look
at and to render. For geometry you can actually print and assemble — real
teeth, pin holes, a baseplate — use `Shift+E` instead; see **Printing it**.

## Printing it

`E` writes an animation. **`Shift+E` writes the machine**: a folder of binary
STL parts you can print, plus a `MANIFEST.txt` saying what each one is and how
the pile goes together. You are asked for a folder name, and you can put print
settings on the same line:

```
gearbox m=1.5 t=4 pin=3 fit=m3
```

| key | what it sets | default |
| --- | --- | --- |
| `m=` | gear module, mm of pitch diameter per tooth | the mechanism's own |
| `pin=` | nominal pin or screw diameter | 3.0 |
| `t=` | how thick a plate, gear or cam is printed | 3.0 |
| `clr=` | running clearance added to a turning hole | 0.4 |
| `gap=` | air between one layer and the next | 0.4 |
| `wall=` | material left around a hole | 2.0 |
| `bl=` | backlash shaved off each gear's teeth | 0.15 |
| `pa=` | pressure angle, degrees | 20 |
| `fit=` | `pin` for printed pins, `m3` for M3 hardware | `pin` |
| `base=` | `on` or `off` for the baseplate | `on` |

Nonsense values are clamped rather than refused; an unknown key stops the
export and says which one it was. Every setting actually used is written at the
top of the manifest.

### Gears have whole teeth now

Two gears mesh only if they share a module and have whole tooth counts, and
then they mesh at exactly one centre distance — `module * (Na + Nb) / 2`. A
pair drawn a third of a tooth apart looks fine on screen and prints as two
wheels that jam or never touch. So **a wheel's size is now a tooth count**:
`+`/`-` step it a whole tooth at a time, meshing sets the centre distance
exactly, and the number beside a wheel is its tooth count rather than its
radius. The canvas draws the real involute outline, from the same generator the
STL export extrudes, so the mesh you judge by eye is the mesh that prints.

A pitch radius is `module * teeth / 2`, so at the default module of 2 a 30 mm
wheel is a 30-tooth wheel. Below 14 teeth the flanks get short and the manifest
says so; below 8 a 20-degree involute has no usable flank left and the wheel is
refused.

### What comes out

- **`link_N.stl`** — a plate covering the link's pins: a dogbone for two, a
  rounded plate for three or more, with a running-fit hole at each pin. Where
  a motor drives the body, that hole is a press fit instead, so a shaft turns
  it.
- **`gear_N.stl`** — a real involute spur gear, correct tooth count, with a
  bore.
- **`rack_N.stl`** — a rack on the pinion's module, with two slots to slide on.
- **`cam_N.stl`** and **`roller_N.stl`** — the cam's actual cut surface (the
  pitch curve already inset by the roller), with a bore and, where there is
  room, a key pin so it turns with its shaft rather than on it.
- **`geneva_wheel_N.stl`** and **`geneva_driver_N.stl`** — a slotted wheel and
  its crank arm. It indexes correctly; it is **not** locked between steps,
  because the locking disc and its matching rim scallops need a second plane
  this exporter does not generate. The manifest says so.
- **`rail_N.stl`** — a slider's rail as a slotted bar, with a mounting hole
  past each end.
- **`baseplate.stl`** — the ground plate, with a hole at every anchor. This is
  the part that actually holds the gear centres the right distance apart, so
  it is the one to print carefully.
- **`pin_*.stl`** / **`cap_*.stl`** / **`spacer_*.stl`** — the fasteners, cut
  to the lengths the stack needs. In `fit=m3` mode the pins are replaced by a
  shopping list in the manifest.

### Layers, so nothing prints into itself

Two bodies that share a pin cannot both sit at z = 0, and two gears that mesh
*must*. So meshed groups are contracted to one node, nodes that share a joint
are joined, and the result is greedily coloured — the colour is the layer. Any
empty layer in a pin's stack gets a spacer washer, so nothing rubs and nothing
floats. Parts are written flat at z = 0 ready for the bed; the manifest records
which layer each belongs on.

### Watertightness

Every solid is checked before it is written: each edge must be shared by
exactly two triangles wound opposite ways. A part that fails is skipped and
reported rather than handed to a slicer that would quietly print something
else. `make test` covers this end to end — it exports a mechanism with gears, a
four-bar and a slider, reads every STL back and checks each one.

The load-bearing test is `test_printed_gears_actually_mesh`: it takes the two
outlines the exporter would print, places them at the centre distance the
manifest states, and turns them through several teeth, asserting that they
never overlap (or the printed pair jams) and stay within a backlash of contact
(or there is no drive at all, just two discs spinning past each other).


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
`src/gearing.c` owns involute tooth geometry -- the one place tooth shape is
decided, shared by the model, the canvas and the export. `src/mesh3d.c` turns
flat outlines into watertight prisms (ear clipping with hole bridging, then
extrusion) and writes binary STL. `src/print3d.c` builds the parts, assigns
layers and writes the manifest. `src/export.c` writes the Blender script; undo/redo (in `src/main.c`) is a
pair of bounded stacks of full `mechanism_clone()` snapshots — one pushed
before each edit, the other filled by undoing and discarded by the next edit.

`src/joints.c` owns the kinematics of the sliding, gear and Geneva joints, and
`src/templates.c` the catalogue. Neither has an SDL dependency, so the Geneva's
indexing law and every template in the gallery are covered by the headless
tests -- including one that builds all nine, runs each for six seconds, and
fails if any binds or fails to move the point it traces.

`src/synth.c` owns both path tools. Both start by resampling the stroke to even
arc length -- a freehand stroke bunches up wherever the hand slowed, which
would otherwise distort the result badly.

The four-bar side has closed-form four-bar kinematics, the Grashof test that
decides whether a candidate can be driven round at all, and the search. Its
cost is the symmetric distance between the drawn points and the candidate's
coupler curve, measured to the curve's *segments* rather than its samples, so
a coarse sampling still measures the true distance.

The arm-chain side mirrors the stroke if it is open and takes a discrete
Fourier transform; each coefficient becomes one arm. Which arms to keep is
decided by Parseval's theorem: the energy in the terms you drop *is* the mean
squared deviation, so the error of every possible truncation is known without
reconstructing anything.

Each arm is an ordinary driven link, pivoting on the tip of the one before it.
That needed one generalisation in `src/solver.c`: a motor's pivot no longer has
to be an anchor, so driven links are posed in dependency order -- anchors
first, then whatever their motion has settled -- rather than in array order,
which would pose a child arm from its parent's stale position. Because each
link's accumulated angle is absolute rather than relative to its parent, an arm
turning at k times the base rate sweeps k turns per cycle in world terms, which
is exactly what makes the chain sum a Fourier series.

`src/ui.c` owns the toolbar: its layout, hit-testing, and the rules deciding
which buttons are enabled and lit. It deliberately has no SDL dependency, so
those rules are covered by the headless tests alongside the kinematics ones;
`render.c` draws it, including a small stroke-drawn alphabet that keeps the
app free of any font dependency (the same reason link dimensions are drawn
as hand-rolled seven-segment numerals). Every button and its hotkey call the
same function in `main.c`, so the two can't drift apart.
