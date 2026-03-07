// PROJECT: 5kg Vertical Panel Actuator
// PART: Solid D-Profile Hex Coupler (Scratch Build)
// REQ: 14mm Hex + 7.9mm M8 Hole + 5mm D-Profile Hole

/* [Dimensions] */
total_h = 55;
hex_size = 14;       // 14mm across flats for spanner
m8_hole_dia = 7.9;   // Tight fit for M8 rod self-tapping
m8_depth = 35;       // Depth for threaded rod
motor_shaft_dia = 5.2; // 5mm + 0.2mm tolerance
motor_flat_dist = 4.5; // NEMA 17 standard flat-to-curve
motor_depth = 20;

/* [Calculated] */
hex_outer = hex_size / cos(30);

module hex_coupler_v2() {
    difference() {
        // 1. MAIN BODY (Solid Hexagon)
        cylinder(d = hex_outer, h = total_h, $fn = 6);

        // 2. M8 THREADED ROD HOLE (Top)
        translate([0, 0, total_h - m8_depth])
            cylinder(d = m8_hole_dia, h = m8_depth + 1, $fn = 32);

        // 3. MOTOR SHAFT D-PROFILE (Bottom)
        translate([0, 0, -1])
            intersection() {
                // The circular part of the 5mm shaft
                cylinder(d = motor_shaft_dia, h = motor_depth + 1, $fn = 64);
                
                // The Flat Face (creates the D-shape)
                translate([-motor_shaft_dia/2, -motor_shaft_dia/2, 0]) 
                    cube([motor_shaft_dia, motor_flat_dist, motor_depth + 2]); 
            }
    }
}

// Render
hex_coupler_v2();

