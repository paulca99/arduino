// PROJECT: 5kg Vertical Panel Actuator - MODIFIED BOTTOM
segment_length = 200; 
base_width = 45;        
web_width = 12;         
flange_height = 6;      
total_height = 22;      
joint_depth = 12;       

difference() {
    union() {
        translate([-base_width/2, 0, 0]) cube([base_width, flange_height, segment_length]);
        translate([-web_width/2, flange_height, 0]) cube([web_width, 8, segment_length]);
        translate([-web_width/2 + 1, 0, segment_length]) cube([web_width - 2, 14, joint_depth]);
    }

    // Female Tongue Pocket
    translate([-web_width/2 + 0.9, -0.1, -0.1]) cube([web_width - 1.8, 14.2, joint_depth + 0.2]);

    // NEW: 4mm Through Holes (Middle and 1" from ends)
    for(z = [25.4, 100, 174.6]) {
        translate([0, -0.1, z]) rotate([-90, 0, 0]) {
            cylinder(d=4, h=total_height + 0.2, $fn=32); // Main 4mm hole
        }
    }

    // Original Wall Mounting Holes (Side flanges)
    for(z = [25, 75, 125, 175]) {
        for(side = [-1, 1]) {
            translate([side * (base_width/2 - 6), -0.1, z]) rotate([-90, 0, 0]) {
                cylinder(d=4.5, h=flange_height + 0.2, $fn=24); 
                translate([0, 0, flange_height - 3.9]) cylinder(d1=9, d2=4.5, h=4, $fn=24); 
            }
        }
    }
    

    // 6.2mm DOWEL HOLES
    
    // 6.2mm DOWEL HOLES
for(z = [50, 90, 150]) {
    // Position it along the Z axis, then move it INTO the web height (Y)
    translate([0, 10, z]) rotate([90, 0, 0]) 
        cylinder(d=6.2, h=20, $fn=24, center=true); 
}

}
