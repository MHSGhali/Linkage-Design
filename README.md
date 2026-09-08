# Linkage Design

A 2D mechanism editor and simulator. Build a planar machine out of pins,
links, sliders, gears, cams and Geneva wheels, drive it with a motor, and
watch it move — or draw the path you want a point to follow and let the
program design the machine that traces it. Then export it as a Blender
animation, or as a folder of printable STL parts that bolt together.

<p align="center">
  <img src="docs/images/four-bar.gif" width="100%"
       alt="A four-bar linkage running: the coupler point traces its curve while x and y are plotted against time below">
</p>

## Build

Requires SDL2 (`brew install sdl2` on macOS, `apt install libsdl2-dev` on
Debian/Ubuntu). macOS and Linux only — the STL export uses POSIX directory
calls, so Windows needs MSYS2/Cygwin or WSL.

```
make          # builds ./linkage_design
make test     # runs the headless kinematics regression tests
./linkage_design
```

## Every command is a key and a button

`H` lists the lot inside the window, so there is nothing to memorise from
here. Buttons grey out when they don't apply and say what they want selected
first; anything the program has to say appears on the canvas.

<p align="center">
  <img src="docs/images/help.png" width="100%" alt="The in-app key list, opened with H">
</p>

## Draw a path, get a machine

Draw a curve and the program designs something that traces it. `P` fits a
**four-bar** — five parts and one motor, but only the curves a four-bar can
manage. `B` builds a **chain of rotating arms**, which will redraw anything at
all, at the cost of dozens of parts. Both come out running and editable.

<table>
<tr>
<td width="50%"><img src="docs/images/arms-star.gif" width="100%" alt="A chain of rotating arms redrawing a hand-drawn star"></td>
<td width="50%"><img src="docs/images/linkage-oval.gif" width="100%" alt="A four-bar linkage fitted to a drawn oval"></td>
</tr>
<tr>
<td align="center"><code>B</code> — arms redrawing a star</td>
<td align="center"><code>P</code> — a four-bar fitted to an oval</td>
</tr>
</table>

## Joints beyond the pin

Press one with nothing selected and a whole working assembly appears, running
the moment you press `R`. Press it with the right parts selected and it joins
those instead.

<table>
<tr>
<td width="50%"><img src="docs/images/gear.gif" width="100%" alt="Two meshed gears with involute teeth"></td>
<td width="50%"><img src="docs/images/geneva.gif" width="100%" alt="A Geneva wheel indexing one slot per turn of the driver"></td>
</tr>
<tr>
<td align="center"><code>O</code> — gears, meshed on a real tooth grid</td>
<td align="center"><code>W</code> — a Geneva wheel, indexing</td>
</tr>
<tr>
<td width="50%"><img src="docs/images/cam.gif" width="100%" alt="A disc cam lifting a roller follower"></td>
<td width="50%"><img src="docs/images/slider.gif" width="100%" alt="A slider-crank turning rotation into a stroke"></td>
</tr>
<tr>
<td align="center"><code>K</code> — draw a cam, get a follower</td>
<td align="center"><code>S</code> — sliders and pins in slots</td>
</tr>
</table>

<p align="center">
  <img src="docs/images/rack.gif" width="60%" alt="A rack and pinion, the pinion drawn with real involute teeth">
  <br><em>a rack and pinion — the teeth you see are the teeth that get printed</em>
</p>

## Or start from a famous one and take it apart

`N` opens the gallery. Everything in it is made of the same pins and bars you
draw by hand, so any of it can be pulled apart, retimed and rebuilt.

<p align="center">
  <img src="docs/images/gallery.png" width="100%" alt="The template gallery: four bar, drag link, parallel, Hoeken, yoke, quick return, scissor, rack, cam">
</p>

## Rigid means rigid

A fixed-length link never stretches. If the motor would have to stretch one to
keep turning, the mechanism **jams and says so** — which is what the real one
would do. `V` marks a link as free to change length when you want the other
behaviour.

## Getting it out

`E` writes a ready-to-run Blender script with the animation baked in.
`Shift+E` writes a folder of **printable STL parts** — gears with a whole
number of involute teeth, plates with pin holes, a baseplate, the pins to join
them, and two views of the whole thing assembled. Parts that share a pin are
stacked on separate layers so nothing prints into anything else.

## The manual

[**docs/manual.md**](docs/manual.md) has all of it: every joint and what it
wants selected, designing from a path, cams and followers, lock-up, saving,
the Blender export, printing (tooth counts, layers, watertightness), and how
the solver works.

## The pictures in this README

None of them are screenshots. They come out of the program:

```
make docs-images    # needs ffmpeg
```

which runs `./linkage_design --capture <subject> <frames> <dir>` to render
frames offscreen through the real drawing code, then encodes them — so they
can be regenerated whenever the drawing changes.

## License

MIT — see [LICENSE](LICENSE).
