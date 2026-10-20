# Electromagnet end effector

Spring-loaded housing for the **KK-P20/15** 12 V lifting electromagnet (Ø20 × 15 mm, up to
3 kg on flat steel). It replaces the arm's gripper. Everything comes from one parametric
file, `magnet_effector.scad`. The STLs in `stl/` are rendered with the default numbers.

![section](preview/section.png)

```
 wrist servo horn
 ├─ adapter_horn   (or adapter_bolt)     5 mm disk, 4 × M3 nuts trapped on the servo side
 ├─ sleeve         flange + tube, Ø29.6 mm, lead channel on top, M3 cross pin
 │    spring       inside the carrier, pushes it down
 ├─ carrier        slides 8 mm up inside the sleeve, held by the cross pin in two slots
 └─ magnet         screwed into the carrier through its rear threaded hole, face 2 mm proud
```

At rest, the distance from the top of the adapter to the magnet face is **52 mm**. When pressed
on a part, the carrier slides up as much as 8 mm.

## Why a spring

The magnet only reaches its rated force when its face sits flat against the part. A
servo arm misses by a few millimetres, so the software drives the tool 3 mm below the
part's top (`PRESS_DEPTH` in `host/config.py`). The spring takes that up and the face
seats flat every time. If you don't want the spring, `rigid_cup.stl` bolts straight to
the adapter, but then pick height has to be exact.

## Measure before printing

These dimensions could not be confirmed from the product listings:

| Parameter | Default | What to measure |
|---|---|---|
| `mag_d`, `mag_h` | 20, 15 | Magnet body. Sellers agree on 20 × 15. |
| `mag_screw_d` | 4.4 | Rear threaded hole. Sellers list M4; 4.4 clears both M4 and M3. |
| `wire_hole_r` | 5.5 | Distance from the centre to where the leads leave the rear face. |
| `horn_len`, `horn_w`, `horn_t` | 32.5, 6.5, 1.8 | Your wrist servo's cross horn: tip to tip, arm width, thickness. |
| `wrist_holes` | 16 × 16 | Only for `adapter_bolt`: spacing of the screw holes the gripper used. |

Change the numbers at the top of `magnet_effector.scad`, then re-export with
`render_all.sh` or OpenSCAD → F6 → Export STL with `part` set to that part.

## Which adapter

* **`adapter_horn`**: the gripper sat on a servo horn, usually on the wrist roll servo.
  The cross horn drops into the pocket, and 4 small self-tapping screws go through the
  horn's outer holes into the pilot holes. Fit the horn to the servo with its centre screw
  through the 8 mm hole before bolting on the sleeve.
* **`adapter_bolt`**: the gripper bolted to a bracket. Set `wrist_holes` to that hole
  pattern and use M3 screws.

## Bill of materials

| Qty | Part |
|---|---|
| 1 | KK-P20/15 12 V electromagnet |
| 1 | Compression spring, OD 16–19 mm, free length 28–35 mm, light (spring assortment box) |
| 4 | M3 × 10 screws + M3 nuts (sleeve to adapter) |
| 1 | M3 × 35 screw + M3 nut (cross pin) |
| 1 | M4 × 8 screw for the magnet's rear hole (M3 × 8 if yours is M3) |
| 4 | M2 × 6 self-tapping screws (horn to adapter) |
| 1 | Logic-level MOSFET module or IRLZ44N, 1N4007 / 1N5819 diode, 220 Ω, 10 kΩ |
| 1 | 12 V supply, ≥ 0.5 A |

## Printing

* **PETG**, not PLA. The coil runs warm, and PLA softens around 55 °C.
* 0.2 mm layers, 4 walls, 40 % infill.
* No supports. Print the carrier pocket-down, the sleeve flange-down and the adapters flat.
* If the carrier binds in the sleeve, raise `fit_slide` from 0.5 to 0.6–0.7 and reprint
  the carrier. Sand lightly if needed.

## Assembly

1. Screw the magnet into the carrier through the centre hole. Feed its leads up through
   the 3.5 mm hole beside it.
2. Drop the spring into the carrier, around the screw head and the leads.
3. Push the carrier into the sleeve, line up the slots with the side holes, and slide the
   M3 × 35 cross pin through. Put the nut in its hexagon trap. The carrier should now
   spring back after being pushed up about 8 mm.
4. Route the leads up through the sleeve's centre hole and out sideways along the channel
   on top of the flange.
5. Fix the adapter to the wrist, then bolt the sleeve to it with 4 × M3 (nuts sit in the
   adapter's traps).
6. Set `L4` in `host/config.py` to the distance from the wrist pitch axis to the adapter's
   top face, plus 52 mm.

## Wiring

```
+12V ──────┬──────────── magnet lead A
           │
     1N4007 / 1N5819   (stripe towards +12V)
           │
MOSFET D ──┴──────────── magnet lead B
MOSFET S ── GND  (12 V GND, servo GND and Arduino GND all joined)
MOSFET G ── 220 Ω ── Nano D7  /  ESP32 GPIO25      (+ 10 kΩ from G to GND)
```

Never drive the coil straight from a pin, and never leave out the diode.

## Limits of a magnet

* **It only lifts steel.** Plain or zinc-plated steel works. Stainless A2/A4, brass and
  aluminium will not lift. Test every part with a fridge magnet first.
* **Contact area.** Holding force drops a lot on small or curved contact. A flat washer or
  nut is easy. A bolt lying on its side touches only along a line, so it lifts with far
  less force, but a few grams is still well within reach.
* **Neighbours.** The magnet attracts parts within a few mm, so the sorter picks isolated
  parts first (`ISOLATION_MM`).
* **Residual magnetism** can hold a light washer for a moment after switch-off. The sorter
  waits `RELEASE_DWELL_MS` over the bin before moving away.
