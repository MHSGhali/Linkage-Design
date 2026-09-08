#!/bin/sh
# Regenerates every picture in docs/images/ from the program itself.
#
# Nothing here is a hand-taken screenshot. `linkage_design --capture` renders a
# subject through the real drawing code into BMP frames with a hidden window
# and a software renderer; ffmpeg turns those into the GIFs and stills. So when
# the drawing changes the documentation is one command behind, rather than
# however long it takes someone to remember to retake a screenshot.
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

# The capture is 1180x820 with the toolbar down the left and the plot along the
# bottom. The whole window is what you want for a hero shot or a picture of a
# panel; for a close-up of one joint it is mostly chrome, so those are cropped
# to the canvas.
CANVAS_CROP="crop=1040:640:140:0"

# gif <name> <subject> <frames> <width> <fps> [extra filter]
# Two ffmpeg passes so each GIF gets a palette built from its own frames: the
# default 256-colour web palette turns these thin strokes to mud. Undithered
# on the way out, because dithering a flat dark background triples the file
# size to encode noise nobody can see.
gif() {
    name=$1; subject=$2; n=$3; width=$4; fps=$5; extra=$6
    rm -rf "$work/$name"; mkdir -p "$work/$name"
    "$bin" --capture "$subject" "$n" "$work/$name" >/dev/null
    chain="${extra:+$extra,}fps=$fps,scale=$width:-1:flags=lanczos"
    ffmpeg -loglevel error -y -framerate 30 -i "$work/$name/frame_%03d.bmp" \
        -vf "$chain,palettegen=stats_mode=diff" "$work/palette.png"
    ffmpeg -loglevel error -y -framerate 30 -i "$work/$name/frame_%03d.bmp" -i "$work/palette.png" \
        -lavfi "$chain[s];[s][1:v]paletteuse=dither=none" -loop 0 "$out/$name.gif"
}

# still <name> <subject> <width> [extra filter]
still() {
    name=$1; subject=$2; width=$3; extra=$4
    rm -rf "$work/$name"; mkdir -p "$work/$name"
    "$bin" --capture "$subject" 1 "$work/$name" >/dev/null
    ffmpeg -loglevel error -y -i "$work/$name/frame_000.bmp" \
        -vf "${extra:+$extra,}scale=$width:-1:flags=lanczos" "$out/$name.png"
}

# The hero: the whole window, so the toolbar, the canvas and the motion plot
# are all visible at once.
gif four-bar "four bar" 150 800 15

# The two path tools, side by side in the README: same gesture, two answers.
gif arms-star    arms    150 520 15 "$CANVAS_CROP"
gif linkage-oval linkage 150 520 15 "$CANVAS_CROP"

# One per joint, cropped to the canvas.
gif gear   gear     110 400 14 "$CANVAS_CROP"
gif geneva geneva   110 400 14 "$CANVAS_CROP"
gif cam    cam      110 400 14 "$CANVAS_CROP"
gif slider slider   110 400 14 "$CANVAS_CROP"
gif rack   rack     110 400 14 "$CANVAS_CROP"

# The panels, full window -- the key list IS the keyboard reference.
still help    help    800
still gallery gallery 800

# Superseded by the GIFs above.
rm -f "$out/rack-and-pinion.png" "$out/cam-and-follower.png" "$out/quick-return.png"

du -h "$out"/* | sort -k2
echo "total: $(du -sh "$out" | cut -f1)"
