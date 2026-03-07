// --- DOUBLE-NUT TRAP BOTTOM MOUNT ---
bearing_od = 28.2;   
rod_spacing = 35.0;  
standoff_h = 42.0;   
motor_bolt_dist = 31.04; 
$fn = 64;

module final_bottom_mount() {
    difference() {
        union() {
            translate([-45, -10, -10]) cube([110, 10, 70]); // Backplate
            translate([-22, 0, 0]) cube([44, 44, standoff_h + 10]); // Motor Tower
            translate([rod_spacing - 10, 0, 0]) cube([20, 20, standoff_h + 10]); // Guide Tower
        }

        // M12 & Bearing
        translate([0, 22, -20]) cylinder(d=14, h=100); 
        translate([0, 22, standoff_h + 2]) cylinder(d=bearing_od, h=9);

        // M6 Guide Rod + DOUBLE NUT TRAP
        translate([rod_spacing, 10, -20]) cylinder(d=6.5, h=100);
        // Deeper 12mm hex trap for two M6 nuts (Standard M6 nut is ~5mm thick)
        translate([rod_spacing, 10, -11]) cylinder(d=11.5, h=12, $fn=6);

        // NEMA17 Bolts
        for(x=[-1,1], z=[-1,1])
            translate([x*motor_bolt_dist/2, 22 + z*motor_bolt_dist/2, -11])
                cylinder(d=3.5, h=15);

        // 4mm Mounting Holes (Accessible Wings)
        for(z=[10, 50], x=[-35, 55])
            translate([x, -11, z]) rotate([-90, 0, 0]) cylinder(d=4.5, h=20);

        // Access Window
        translate([-25, 10, 10]) cube([20, 40, 25]);
    }
}
final_bottom_mount();
