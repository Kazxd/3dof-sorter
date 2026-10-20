// magnet_effector.scad - spring-loaded housing for a KK-P20/15 electromagnet
// =============================================================================
// Replaces the gripper of a 5-DOF hobby arm. Four printed parts:
//
//   adapter_horn  disk that screws onto the wrist servo's cross horn
//   adapter_bolt  same disk, but with a 4-hole pattern for the gripper's old screw holes
//   sleeve        flanged tube bolted under the adapter (4x M3)
//   carrier       cup that holds the magnet, slides 8 mm inside the sleeve on an M3 cross pin
//   rigid_cup     no-spring alternative: magnet cup with its own flange
//
// Stack (top = wrist side, bottom = table side):
//   servo horn -> adapter -> sleeve flange -> sleeve tube -> spring -> carrier -> magnet face
//
// Why a spring: the magnet only holds at full force when its face sits flat on the
// part. Hobby-servo arms are off by a few mm, so the arm is driven 3 mm "too low"
// (config.PRESS_DEPTH) and the spring absorbs it.
//
// Render one part:  openscad -D 'part="carrier"' -o carrier.stl magnet_effector.scad
// or run cad/render_all.cmd. Print in PETG (the coil runs warm; PLA softens ~55 C).
// Every dimension below is a parameter; MEASURE the ones marked [measure].

part = "assembly";   // assembly | adapter_horn | adapter_bolt | sleeve | carrier | rigid_cup | section
$fn = 96;

// ---- electromagnet KK-P20/15 ---------------------------------------------------
mag_d       = 20.0;   // body diameter                              [measure]
mag_h       = 15.0;   // body height                                [measure]
mag_screw_d = 4.4;    // clearance for the rear threaded hole: M4 (4.4) also passes M3 [measure]
wire_hole_d = 3.5;    // lead exit through the cup ceiling (2 thin leads)
wire_hole_r = 5.5;    // distance of the lead exit from the centre; must sit inside the spring [measure]
mag_proud   = 2.0;    // magnet face sticks out of the cup by this much

// ---- print fits ------------------------------------------------------------------
fit_press   = 0.30;   // diametral clearance magnet -> cup (snug, screw holds it)
fit_slide   = 0.50;   // diametral clearance carrier -> sleeve (sliding)
m3_clear    = 3.4;
m3_nut_af   = 5.7;    // M3 nut across flats + clearance
m3_nut_h    = 2.6;

// ---- carrier --------------------------------------------------------------------
wall        = 2.0;
ceiling_t   = 3.0;
skirt_h     = 16.0;   // tube above the ceiling that carries the pin slots
travel      = 8.0;    // spring travel
pin_d       = 3.3;    // M3 bolt used as the cross pin

pocket_d    = mag_d + fit_press;
pocket_h    = mag_h - mag_proud;
carrier_od  = pocket_d + 2 * wall;
carrier_h   = pocket_h + ceiling_t + skirt_h;
slot_top    = 2.0;    // slot starts this far below the skirt rim
slot_len    = travel + pin_d;

// ---- sleeve ---------------------------------------------------------------------
sleeve_wall = 2.4;
sleeve_id   = carrier_od + fit_slide;
sleeve_od   = sleeve_id + 2 * sleeve_wall;
plate_t     = 4.0;    // top plate / flange thickness
wire_chan_w = 4.5;    // side channel on top of the sleeve plate: leads exit between sleeve and adapter
wire_chan_d = 2.5;
spring_od   = 18.0;   // compression spring, OD 16-19 mm, free length 28-35 mm, light   [buy]
flange_d    = 46.0;
bolt_r      = 18.0;   // radius of the 4 flange bolts (at 45 deg, clear of the horn arms and the tube)
centre_hole = 8.0;    // wires + screwdriver for the horn screw
rest_gap    = travel + 1.0;           // carrier rim to sleeve ceiling at rest
sleeve_len  = 30.0;                   // tube length below the plate
pin_z       = -(rest_gap + slot_top + pin_d / 2);   // pin centre, below the sleeve ceiling

// ---- adapters -------------------------------------------------------------------
adapter_t   = 5.0;
horn_len    = 32.5;   // SG90/MG90S cross horn, tip to tip           [measure]
horn_w      = 6.5;    // arm width at its widest                     [measure]
horn_t      = 1.8;    // arm thickness = pocket depth                [measure]
horn_head_d = 6.0;    // clearance for the horn screw head
wrist_holes = [16, 16];  // adapter_bolt: X/Y spacing of the gripper's old screw holes [measure]

// ================================================================================
module ring(od, id, h) { difference() { cylinder(d = od, h = h); translate([0, 0, -0.1]) cylinder(d = id, h = h + 0.2); } }

module bolt_circle(r = bolt_r, a0 = 45) {
  for (a = [0 : 90 : 270]) rotate(a0 + a) translate([r, 0, 0]) children();
}

// carrier: pocket opening at z=0 (magnet face side), skirt rim at the top
module carrier() {
  difference() {
    cylinder(d = carrier_od, h = carrier_h);
    translate([0, 0, -0.1]) cylinder(d = pocket_d, h = pocket_h + 0.1);                // magnet pocket
    translate([0, 0, pocket_h + ceiling_t]) cylinder(d = pocket_d, h = skirt_h + 0.1);  // spring space
    translate([0, 0, pocket_h - 0.1]) cylinder(d = mag_screw_d, h = ceiling_t + 0.2);   // magnet screw
    translate([wire_hole_r, 0, pocket_h - 0.1]) cylinder(d = wire_hole_d, h = ceiling_t + 0.2);
    // two opposite slots for the cross pin
    translate([0, 0, carrier_h - slot_top - slot_len])
      hull() for (z = [pin_d / 2, slot_len - pin_d / 2])
        translate([0, 0, z]) rotate([0, 90, 0]) cylinder(d = pin_d + 0.2, h = carrier_od + 1, center = true);
    // finger notch to pull the magnet out
    translate([0, -carrier_od / 2, -0.1]) cube([6, 4, 4], center = true);
  }
}

// sleeve: flange/plate at z = 0..plate_t, tube going down to -sleeve_len
module sleeve() {
  difference() {
    union() {
      translate([0, 0, 0]) cylinder(d = flange_d, h = plate_t);
      translate([0, 0, -sleeve_len]) cylinder(d = sleeve_od, h = sleeve_len + 0.01);
      // bosses around the pin holes
      translate([0, 0, pin_z]) rotate([0, 90, 0]) cylinder(d = 9, h = sleeve_od + 4, center = true);
    }
    translate([0, 0, -sleeve_len - 0.1]) cylinder(d = sleeve_id, h = sleeve_len + 0.1);   // bore
    translate([0, 0, -0.1]) cylinder(d = centre_hole, h = plate_t + 0.2);
    bolt_circle() translate([0, 0, -0.1]) cylinder(d = m3_clear, h = plate_t + 0.2);
    // lead channel: from the centre hole out to the edge, on the adapter side
    translate([0, -wire_chan_w / 2, plate_t - wire_chan_d]) cube([flange_d, wire_chan_w, wire_chan_d + 0.1]);
    translate([0, 0, pin_z]) rotate([0, 90, 0]) cylinder(d = pin_d, h = sleeve_od + 4, center = true);
    // nut trap on one side of the pin
    translate([sleeve_od / 2 + 1.6, 0, pin_z]) rotate([0, 90, 0]) cylinder(d = m3_nut_af / cos(30), h = m3_nut_h + 0.4, $fn = 6);
  }
}

module adapter_base() {
  difference() {
    cylinder(d = flange_d, h = adapter_t);
    translate([0, 0, -0.1]) cylinder(d = centre_hole, h = adapter_t + 0.2);
    // 4x M3 to the sleeve, nuts trapped on the servo side
    bolt_circle() {
      translate([0, 0, -0.1]) cylinder(d = m3_clear, h = adapter_t + 0.2);
      translate([0, 0, adapter_t - m3_nut_h]) cylinder(d = m3_nut_af / cos(30), h = m3_nut_h + 0.1, $fn = 6);
    }
  }
}

// horn version: the cross horn's outer face lies in a pocket on the servo side (top)
module adapter_horn() {
  difference() {
    adapter_base();
    for (a = [0, 90]) rotate(a)
      translate([0, 0, adapter_t - horn_t]) hull() for (s = [-1, 1])
        translate([s * (horn_len / 2 - horn_w / 2), 0, 0]) cylinder(d = horn_w + 0.4, h = horn_t + 0.1);
    translate([0, 0, adapter_t - horn_t - 2]) cylinder(d = horn_head_d, h = 2.1);   // horn screw head
    // pilot holes for 2 mm self-tapping screws through the horn's outer holes
    for (a = [0 : 90 : 270]) rotate(a) translate([horn_len / 2 - 3, 0, -0.1]) cylinder(d = 1.6, h = adapter_t);
  }
}

// bolt version: 4 holes matching where the gripper used to bolt on
module adapter_bolt() {
  difference() {
    adapter_base();
    for (sx = [-1, 1], sy = [-1, 1])
      translate([sx * wrist_holes[0] / 2, sy * wrist_holes[1] / 2, -0.1]) {
        cylinder(d = m3_clear, h = adapter_t + 0.2);
        cylinder(d = 6.2, h = 2.6);   // screw head recess on the sleeve side
      }
  }
}

// no-spring alternative: cup + flange in one piece
module rigid_cup() {
  difference() {
    union() {
      cylinder(d = carrier_od, h = pocket_h + ceiling_t);
      translate([0, 0, pocket_h + ceiling_t]) cylinder(d = flange_d, h = plate_t);
    }
    translate([0, 0, -0.1]) cylinder(d = pocket_d, h = pocket_h + 0.1);
    translate([0, 0, pocket_h - 0.1]) cylinder(d = mag_screw_d, h = ceiling_t + plate_t + 0.2);
    translate([wire_hole_r, 0, pocket_h - 0.1]) cylinder(d = wire_hole_d, h = ceiling_t + plate_t + 0.2);
    translate([0, 0, pocket_h + ceiling_t]) cylinder(d = 9, h = plate_t + 0.1);   // screw head recess
    bolt_circle() translate([0, 0, -0.1]) cylinder(d = m3_clear, h = 40);
  }
}

module magnet() {   // reference body, not printed
  color("silver") difference() {
    cylinder(d = mag_d, h = mag_h);
    translate([0, 0, mag_h - 4]) cylinder(d = 3.3, h = 4.1);
  }
  color("dimgray") translate([0, 0, -0.01]) ring(10, 6, 0.3);
}

// ---- assembly, at rest, magnet face down ------------------------------------------
module cut_half(cut) {   // section view: keep the half with y < 0
  if (cut) intersection() { children(); translate([-60, -120, -150]) cube([120, 120, 300]); }
  else children();
}

module assembly(cut = false) {
  carrier_z = -rest_gap - carrier_h;                          // carrier bottom (pocket opening)
  color("SteelBlue") cut_half(cut) translate([0, 0, plate_t]) adapter_horn();
  color("Orange") cut_half(cut) sleeve();
  color("Khaki") cut_half(cut) translate([0, 0, carrier_z]) carrier();
  cut_half(cut) translate([0, 0, carrier_z - mag_proud]) magnet();
  color("DimGray") translate([0, 0, pin_z]) rotate([0, 90, 0]) cylinder(d = 3, h = sleeve_od + 8, center = true);
  color("Crimson") cut_half(cut) translate([0, 0, carrier_z + pocket_h + ceiling_t])
    ring(spring_od, spring_od - 2, rest_gap + skirt_h - 0.2); // spring (illustration)
}

// ---- print orientations --------------------------------------------------------------
if (part == "assembly") assembly();
else if (part == "section") assembly(cut = true);
else if (part == "carrier") carrier();                               // pocket opening on the bed
else if (part == "sleeve") translate([0, 0, plate_t]) rotate([180, 0, 0]) sleeve();   // flange on the bed
else if (part == "adapter_horn") adapter_horn();
else if (part == "adapter_bolt") adapter_bolt();
else if (part == "rigid_cup") rigid_cup();

echo(str("TOOL LENGTH (adapter top -> magnet face, at rest) = ",
         adapter_t + plate_t + rest_gap + carrier_h + mag_proud, " mm"));
echo(str("carrier OD ", carrier_od, "  sleeve ID ", sleeve_id, "  sleeve OD ", sleeve_od));
