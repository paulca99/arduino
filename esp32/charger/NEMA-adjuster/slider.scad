// PROJECT: NEMA17 Solar Adjuster (Todmorden)
// PART: Narrow Reinforced Slider (4mm RADIUS ENTRY/EXIT CHAMFER)

$fn = 64;

/* [Rail & Body Dimensions] */
slider_length = 50;     
rail_top_width = 25.5;  
rail_flange_thick = 7.0; 
web_gap = 14.5;         
rail_total_depth = 22.4; 
flare_size = 4.0;       // INCREASED TO 4mm RADIUS CHAMFER

/* [Nut & Rod Settings] */
nut_flat_to_flat = 13.4;   
nut_length = 24.5;         
rod_diameter = 8.5;        
print_tolerance = 0.4;     

/* [Lugs & Pivot] */
lug_width = 10;      
lug_gap = 17.0;      
pivot_z_pos = 16;    
lug_radius = 16;     
hole_y_offset = lug_radius; 

// Helper to render the T-shape for the hull-chamfer
module render_t_profile(h_val, flare=0) {
    // Top of T (Flange)
    translate([-(rail_top_width+flare*2)/2, 6-flare, 0]) 
        cube([rail_top_width+flare*2, rail_flange_thick+flare*2, h_val]);
    // Stem of T (Web)
    translate([-(web_gap+flare*2)/2, -0.1-flare, 0]) 
        cube([web_gap+flare*2, 10+flare*2, h_val]);
}

module final_chamfered_slider() {
    body_depth = 32; 
    body_width = 37; 
    
    difference() {
        union() {
            // MAIN BODY
            translate([-body_width/2, 0, 0]) 
                cube([body_width, body_depth, slider_length]); 
            
            // COMPACT PIVOT LUGS
            for(side = [-1, 1]) {
                hull() {
                    translate([(side * (lug_gap/2 + lug_width/2)) - lug_width/2, body_depth - 1, 0])
                        cube([lug_width, 1, 32]);
                    
                    translate([(side * (lug_gap/2 + lug_width/2)), body_depth + hole_y_offset, pivot_z_pos])
                        rotate([0, 90, 0])
                            cylinder(d=lug_radius*2, h=lug_width, center=true);
                }
            }
        }

        // 1. RAIL CHANNEL WITH 4mm RADIUS CHAMFERS
        union() {
            // Main Constant Rail Profile (Horizontal axis)
            translate([0, 0, -1]) {
                translate([-rail_top_width/2, 6, 0]) 
                    cube([rail_top_width, rail_flange_thick, slider_length + 2]);
                translate([-web_gap/2, -0.1, 0]) 
                    cube([web_gap, 10, slider_length + 2]);
            }
            
            // Chamfers at both ends (Entrance and Exit)
            for(z_pos = [0, slider_length]) {
                translate([0, 0, z_pos])
                mirror([0, 0, (z_pos == 0 ? 0 : 1)])
                hull() {
                    // Internal Start (Standard size)
                    translate([0, 0, 0.1])
                        render_t_profile(0.1);
                    
                    // External Flare (4mm flare over 4mm depth)
                    translate([0, 0, -flare_size])
                        render_t_profile(0.1, flare=flare_size);
                }
            }
        }

        // 2. ROD & NUT TRAP (Top Loading)
        translate([0, 25, slider_length/2]) { 
            cylinder(d=rod_diameter, h=slider_length + 10, center=true);
            rotate([0, 0, 0])
                cylinder(d=(nut_flat_to_flat + print_tolerance) / cos(30), h=nut_length, $fn=6, center=true);
            translate([0, 10, 0])
                cube([nut_flat_to_flat + print_tolerance +3.1, 20, nut_length], center=true);
        }
        
        // 3. PIVOT HOLE
        translate([-50, body_depth + hole_y_offset, pivot_z_pos])
            rotate([0, 90, 0])
                cylinder(d=8.2, h=100, $fn=32);
    }
}

final_chamfered_slider();
