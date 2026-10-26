// magnet_hanger.scad - free-swinging hanger for the KK-P20/15 electromagnet
// =============================================================================
// The arm has no wrist servo, so a magnet bolted to the forearm tilts with it.
// This U-shaped hanger hooks into two Ø6 mm holes drilled across the forearm end
// and swings freely, so gravity keeps the magnet face pointing straight down.
//
//          side view of forearm end              front view (looking along the arm)
//      ______________________                    hook ->|  |<- hook (pegs point inward)
//     |                      |  35               |  |=======|  |
//     |            (o)<-hole |  tall             |  |  arm  |  |   arm end 27 wide
//     |____________^_________|                   |  |=======|  |
//                  17.5 from front, 17.5 up      |  |       |  |
//                                                |  |_______|  |  <- hanger, swings
//                                                |_____[o]_____|  <- screw into magnet
//                                                    [magnet]
//
// Print: PETG, lying flat exactly as exported (no supports). 0.2 mm layers,
// 4 walls, 60 % infill. Pegs use a teardrop profile so they print without support;
// the hanger hangs on the round top of the peg, so the point never touches the hole.
//
// Fit: slide one peg into its hole, then spread the other arm a little and push
// its peg in until the barb clicks through the wall into the hollow. The split
// barb squeezes through the Ø6 hole and opens up again inside.

part = "hanger";      // hanger | peg_test | assembly
$fn = 72;

// ---- forearm end (measured, ±1-2 mm) -------------------------------------------
arm_width   = 27.0;   // across the arm, side face to side face        [measured]
wall        = 6.0;    // side-wall thickness before the hollow inside  [measured]
hole_d      = 6.0;    // drill size                                    [measured]
pivot_up    = 17.5;   // hole centre above the arm's bottom edge       [measured]
pivot_back  = 17.5;   // hole centre back from the arm's front edge    [measured]

// ---- magnet -------------------------------------------------------------------------
mag_d       = 20.0;
mag_h       = 15.0;
screw_d     = 4.4;    // passes M3 (use a washer) and M4

// ---- hanger ------------------------------------------------------------------------
side_gap    = 0.75;   // clearance between each hanger arm and the forearm side face
drop        = 40.0;   // pivot centre -> top of the bottom bar (swing clearance, >= 35)
bar_t       = 6.0;    // bottom bar thickness
arm_t       = 3.5;    // hanger arm thickness (thin enough to spread for fitting)
depth       = 10.0;   // hanger thickness along the forearm
top_above   = 6.0;    // hanger arm continues this far above the pivot

peg_d       = hole_d - 0.4;   // turns freely in the hole
peg_play    = 0.5;            // axial play so it swings without binding
barb_len    = 2.0;
barb_d      = hole_d + 1.0;   // 1 mm bigger than the hole: it can't pull back out
barb_tip_d  = hole_d - 1.4;   // pointed tip so it can be pushed through
split_w     = 1.2;            // slot that lets the barb squeeze through the hole
split_len   = barb_len + 3.0;

inner = arm_width + 2 * side_gap;                 // gap between the two hanger arms
outer = inner + 2 * arm_t;
shaft_len = side_gap + wall + peg_play;          // arm face -> inner side of the wall

// 2D teardrop pointing toward -Y (print bed after the export rotation)
module teardrop(d) {
  r = d / 2;
  hull() {
    circle(r = r);
    translate([0, -r * sqrt(2)]) rotate(45) square(0.01, center = true);
  }
}

// One peg: shaft + barb, built along +X from the origin.
module peg() {
  rotate([0, 90, 0]) rotate([0, 0, 90]) {
    linear_extrude(shaft_len) teardrop(peg_d);
    translate([0, 0, shaft_len])
      linear_extrude(barb_len, scale = barb_tip_d / barb_d) teardrop(barb_d);
  }
}

module peg_with_split() {
  difference() {
    peg();
    // vertical slot (in print orientation) through the barb end
    translate([shaft_len + barb_len - split_len, -split_w / 2, -barb_d])
      cube([split_len + 0.1, split_w, 2 * barb_d]);
  }
}

// Hanger in use orientation: X = pivot axis (across the arm), Z = up, Y = along the arm.
// Pivot at the origin.
module hanger_use() {
  bar_top = -drop;
  difference() {
    union() {
      // two arms
      for (s = [-1, 1])
        translate([s * (inner / 2 + arm_t / 2) - arm_t / 2, -depth / 2, bar_top - bar_t])
          cube([arm_t, depth, drop + bar_t + top_above - depth / 2]);
      // rounded arm tops around the pivot
      for (s = [-1, 1])
        translate([s * (inner / 2 + arm_t / 2), 0, top_above - depth / 2])
          rotate([0, 90, 0]) cylinder(d = depth, h = arm_t, center = true);
      // bottom bar
      translate([-outer / 2, -depth / 2, bar_top - bar_t]) cube([outer, depth, bar_t]);
      // inner fillets between arms and bar
      for (s = [-1, 1])
        translate([s * (inner / 2 - 2), 0, bar_top + 2]) cube([4, depth, 4], center = true);
    }
    // screw hole into the magnet (vertical in use)
    translate([0, 0, bar_top - bar_t - 1]) cylinder(d = screw_d, h = bar_t + 2);
  }
  // pegs pointing inward from each arm
  translate([-inner / 2, 0, 0]) peg_with_split();
  translate([inner / 2, 0, 0]) mirror([1, 0, 0]) peg_with_split();
}

// Print orientation: lay the hanger flat (its 10 mm thickness becomes the height).
module hanger_print() {
  translate([0, 0, depth / 2]) rotate([90, 0, 0]) hanger_use();
}

// A small coupon: one peg on a tab, to test the fit in a drilled hole first.
module peg_test() {
  translate([0, 0, depth / 2]) rotate([90, 0, 0]) {
    translate([-arm_t, -depth / 2, -8]) cube([arm_t, depth, 16]);
    peg_with_split();
  }
}

// ---- assembly preview with a ghost forearm end and magnet -------------------------
module assembly() {
  color("SteelBlue") hanger_use();
  // forearm end (ghost), pivot at origin
  %translate([-arm_width / 2, -pivot_back - 25, -pivot_up]) cube([arm_width, pivot_back + 25, 35]);
  color("silver") translate([0, 0, -drop - bar_t - mag_h]) cylinder(d = mag_d, h = mag_h);
}

if (part == "hanger") hanger_print();
else if (part == "peg_test") peg_test();
else if (part == "assembly") assembly();

echo(str("Pivot -> magnet face = ", drop + bar_t + mag_h, " mm  (use as TOOL_DOWN)"));
echo(str("Hanger outside width = ", outer, " mm, inside gap = ", inner, " mm"));
echo(str("Peg shaft = ", shaft_len, " mm, barb Ø", barb_d, " -> Ø", barb_tip_d));
