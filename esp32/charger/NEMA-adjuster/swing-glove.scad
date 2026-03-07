// PROJECT: 5kg Vertical Panel Actuator
// PART: Bearing-Pivot Timber Glove (38x21mm Timber)
// HARDWARE: 2x 608RS Bearings (Stacked) + M8 Pivot Bolt

timber_w = 38.6; // +0.6 tolerance for PETG
timber_t = 21.6; 
sleeve_depth = 80;
wall_thick = 5;      // Beefier walls for bearing housing
bearing_od = 22.2;    // 608RS OD
bearing_thick = 7.1;  // Single bearing thickness
tongue_w = 14.8;     // Fits your 15mm slider gap (2x bearings)

module bearing_glove() {
    difference() {
        union() {
            // Main Sleeve Body
            translate([-(timber_w + wall_thick*2)/2, 0, 0])
                cube([timber_w + wall_thick*2, timber_t + wall_thick*2, sleeve_depth]);
            
            // Structural Neck to Tongue
            translate([-tongue_w/2, (timber_t + wall_thick*2)/2 - 15, sleeve_depth])
                cube([tongue_w, 30, 30]); // Extended neck for 5kg support
            
            // Rounded Bearing Housing
            translate([0, (timber_t + wall_thick*2)/2, sleeve_depth + 30])
                rotate([0, 90, 0])
                    cylinder(d=bearing_od + 10, h=tongue_w, center=true, $fn=64);
        }

        // Internal Timber Pocket
        translate([-timber_w/2, wall_thick, -1])
            cube([timber_w, timber_t, sleeve_depth + 1]);

        // BEARING POCKET (Through the tongue)
        translate([0, (timber_t + wall_thick*2)/2, sleeve_depth + 30])
            rotate([0, 90, 0]) {
                // Pocket for 2x 608RS Bearings
                cylinder(d=bearing_od, h=tongue_w + 1, center=true, $fn=64);
                // M8 Clear hole for the axle bolt
                cylinder(d=8.5, h=tongue_w + 10, center=true, $fn=32);
            }

        // Adjustment Screw Holes (M4/M5)
        for(z = [20, 60]) {
            for(side = [-1, 1]) {
                translate([side * (timber_w/2 + 5), 10, z])
                    rotate([0, 90, 0]) cylinder(d=4.5, h=20, center=true, $fn=24);
            }
        }
    }
}

bearing_glove();
