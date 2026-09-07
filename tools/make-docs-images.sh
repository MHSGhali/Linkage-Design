#!/bin/sh
# Regenerates the images in docs/images/ from the program itself.
#
# Every picture in the README comes out of `linkage_design --capture`, which
# renders a template mechanism through the real drawing code into a folder of
# BMP frames with no window on screen. ffmpeg turns those into the GIF and the
# stills. So when the drawing changes, the documentation is one command behind
# rather than however long it takes someone to remember to retake a screenshot.
#
# Needs ffmpeg (brew install ffmpeg / apt install ffmpeg).

set -e

root=$(cd "$(dirname "$0")/.." && pwd)
bin="$root/linkage_design"
out="$root/docs/images"
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

[ -x "$bin" ] || { echo "build it first: make" >&2; exit 1; }
command -v ffmpeg >/dev/null || { echo "ffmpeg not found" >&2; exit 1; }

mkdir -p "$out"

# An animation of a four-bar tracing its coupler curve: the whole idea of the
# program in one picture. Two passes so the GIF gets a palette of its own --
# a 256-colour default palette turns the anti-aliased strokes to mud.
frames=150
rm -rf "$work/anim"; mkdir -p "$work/anim"
"$bin" --capture "four bar" "$frames" "$work/anim" >/dev/null
# Thinned to 15fps and undithered on the way out: at 30fps with dithering the
# same animation is three times the size, and the extra is noise in the flat
# background rather than anything you can see moving.
ffmpeg -loglevel error -y -framerate 30 -i "$work/anim/frame_%03d.bmp" \
    -vf "fps=15,scale=800:-1:flags=lanczos,palettegen=stats_mode=diff" "$work/palette.png"
ffmpeg -loglevel error -y -framerate 30 -i "$work/anim/frame_%03d.bmp" -i "$work/palette.png" \
    -lavfi "fps=15,scale=800:-1:flags=lanczos[s];[s][1:v]paletteuse=dither=none" \
    -loop 0 "$out/four-bar.gif"

# Stills: one frame late enough in the run that the traced path has been drawn.
still() {
    name=$1; template=$2; n=$3
    rm -rf "$work/$name"; mkdir -p "$work/$name"
    "$bin" --capture "$template" "$n" "$work/$name" >/dev/null
    last=$(printf 'frame_%03d.bmp' $((n - 1)))
    ffmpeg -loglevel error -y -i "$work/$name/$last" -vf "scale=900:-1:flags=lanczos" "$out/$name.png"
}

still rack-and-pinion "rack"      120
still cam-and-follower "cam"      120
still quick-return     "quick rtn" 120

ls -l "$out"
