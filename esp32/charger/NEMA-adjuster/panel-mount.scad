
// PROJECT: 5kg Vertical Panel Actuator
// PART: Twin-Lug Panel Bracket (Matching 14.8mm Bearing Glove)
// REQ: 15.2mm Gap + M8 Pivot + 4x Mounting Holes

base_w = 60;
base_h = 50;
base_t = 6;
lug_gap = 15.2; // Clearance for the 14.8mm bearing glove
lug_thick = 10;
lug_height = 35; // Height from base to pivot center
pivot_dia = 8.2;

module panel_bracket() {
    difference() {
        union() {
            // Main Base Plate
            translate([-base_w/2, -base_h/2, 0])
                cube([base_w, base_h, base_t]);
            
            // The Twin Lugs
            for(side = [-1, 1]) {
                translate([(side * (lug_gap/2 + lug_thick/2)) - lug_thick/2, -base_h/2, base_t])
                    cube([lug_thick, base_h, lug_height]);
            }
        }

        // M8 PIVOT HOLE (Through both lugs)
        translate([-base_w/2 - 5, 0, base_t + lug_height - 10])
            rotate([0, 90, 0])
                cylinder(d=pivot_dia, h=base_w + 10, $fn=32);

        // MOUNTING SCREW HOLES (4x 5.2mm for Wood Screws)
        for(x = [-1, 1]) {
            for(y = [-1, 1]) {
                translate([x * (base_w/2 - 8), y * (base_h/2 - 8), -1])
                    cylinder(d=5.2, h=base_t + 2, $fn=24);
            }
        }
        
        // Relief Cut for Arm Swing (Ensures the glove doesn't hit the base at 65°)
        translate([-lug_gap/2, -base_h/2 - 1, base_t])
            cube([lug_gap, base_h + 2, lug_height - 5]);
    }
}

panel_bracket();
