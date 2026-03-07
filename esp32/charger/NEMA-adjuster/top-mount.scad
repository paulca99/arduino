// --- TODMORDEN TOP ALIGNMENT MOUNT (OUTBOARD HOLES) ---
$fn = 64;

// Parameters
bearing_od = 28.2;   
rod_spacing = 35.0;  
timber_w = 75.0;     

module top_alignment_mount_v2() {
    difference() {
        union() {
            // 1. BACKPLATE (120mm wide)
            translate([-60, -10, 0]) cube([120, 10, timber_w]);
            
            // 2. M12 BEARING HOUSING
            translate([-20, 0, 0]) cube([40, 35, 30]);
            
            // 3. M6 GUIDE ROD SUPPORT
            translate([rod_spacing - 10, 0, 0]) cube([20, 20, 32]);
        }

        // --- M12 ROD & BEARING SEAT ---
        translate([0, 20, -1]) cylinder(d=14, h=50); 
        translate([0, 20, 19]) cylinder(d=bearing_od, h=12); // Bearing at top

        // --- M6 GUIDE ROD ANCHOR ---
        translate([rod_spacing, 10, -1]) cylinder(d=6.5, h=50);
        translate([rod_spacing, 10, 20]) cylinder(d=11.5, h=13, $fn=6); // Nut Trap

        // --- 4x TIMBER MOUNTING HOLES (Shifted 10mm Outward) ---
        // Holes moved from +/- 45mm to +/- 55mm
        for(z=[15, 60], x=[-55, 55])
            translate([x, -11, z]) rotate([-90, 0, 0]) cylinder(d=4.5, h=25);
    }
}

top_alignment_mount_v2();
