#!/usr/bin/env bash
# Regenerates every STL and preview PNG from magnet_effector.scad.
# Usage: OSC=/path/to/openscad ./render_all.sh   (needs a 2024+ build for --backend=Manifold)
cd "$(dirname "$0")"
OSC="${OSC:-openscad}"
mkdir -p stl preview
for p in carrier sleeve adapter_horn adapter_bolt rigid_cup; do
  "$OSC" --backend=Manifold -D "part=\"$p\"" -o "stl/$p.stl" magnet_effector.scad
  "$OSC" -D "part=\"$p\"" -o "preview/$p.png" --imgsize=700,600 --colorscheme=Tomorrow \
         --camera=0,0,0,55,0,30,0 --autocenter --viewall --projection=p magnet_effector.scad
done
"$OSC" -D 'part="assembly"' -o preview/assembly.png --imgsize=900,1000 --colorscheme=Tomorrow \
       --camera=0,0,0,70,0,30,0 --autocenter --viewall --projection=p magnet_effector.scad
"$OSC" -D 'part="section"' -o preview/section.png --imgsize=900,1000 --colorscheme=Tomorrow \
       --camera=0,0,0,90,0,180,0 --autocenter --viewall --projection=o magnet_effector.scad
