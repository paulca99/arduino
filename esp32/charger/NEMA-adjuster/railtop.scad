// PROJECT: 5kg Vertical Panel Actuator
// PART 2: Top Track (Modified with 3mm Deep Recess)

segment_length = 200; 
top_width = 24;         
web_width = 12.4;       
top_flange_thick = 6;

difference() {
    union() {
        // --- ORIGINAL GEOMETRY ---
        // Top Slider Surface
        translate([-top_width/2, 16, 0]) 
            cube([top_width, top_flange_thick, segment_length]);
            
        // Upper half of Central Web
        translate([-web_width/2, 16.0, 0]) 
            cube([web_width, 1, segment_length]);

// 5.8mm DOWELs - 5x Deeper (25mm)
for(z = [50, 90, 150]) {
    translate([0, 16.0, z]) 
        rotate([90, 0, 0]) 
        cylinder(d=5.8, h=10, $fn=24);
}
    }

    // --- NEW: DEEP MOUNTING HOLES ---
    for(z = [25.4, 100, 174.6]) {
        translate([0, 22.1, z]) rotate([90, 0, 0]) {
            
            // 1. Main 4mm Through Hole
            cylinder(d=4, h=30, $fn=32); 
            
            // 2. Extra Deep Recess
            // This creates a 3mm deep straight 'well' before the chamfer starts
            // to ensure the head is buried 3mm below the top surface.
            translate([0, 0, -0.1]) {
                // Straight side-wall for the first 3mm
                cylinder(d=9.5, h=3.1, $fn=32);
                // The actual conical chamfer starting 3mm down
                translate([0,0,3]) cylinder(d1=9.5, d2=4, h=3, $fn=32);
            }
        }
    }
}
