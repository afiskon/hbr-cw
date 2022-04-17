W = 178.5;
H = 62;
t = 4;

pot_dia = 7; // volume control
jack_dia = 6; // 3.5 mm jack
jack_nut_dia = 9;
jack_nut_height = t-1;
button_dia = 7; // button
toggle_dia = 6; // toggle switch
corner_dia = 7;
screw_dia = 3;
screw_offset = 4;
screw2_dia = 6;
screw2_height = 0.5;
eps = 0.01;

main_enc_dia = 21;
main_enc_screws_dia = 15;
main_enc_screws_dist = 7;
main_enc_rotate_delta = 12;
main_enc_screw_dia = 3.5;
multi_enc_dia = 10;

// LCD info
LCD_W = 58;
LCD_H = 32;
lcd_offset_y = 8;
lcd_offset_x = 6;
lcd_h = LCD_H - lcd_offset_x*2;
lcd_w = LCD_W - lcd_offset_y*2;

hole_dia = 3;
hole_offset_x = 1+hole_dia/2;
hole_offset_y = 1+hole_dia/2;

module rounded_cube( width, height, dia, t ) {
    translate([-width/2, -height/2, -t/2])
        linear_extrude(height = t)
            hull() {
                translate([dia/2, dia/2, 0])
                    circle(d = dia, $fn = 25, center = true);
                translate([dia/2, height - dia/2, 0 ])
                    circle(d = dia, $fn = 25, center = true);
                translate([width - dia/2, height - dia/2, 0])
                    circle(d = dia, $fn = 25, center = true);
                translate([width - dia/2, dia/2, 0])
                    circle(d = dia, $fn = 25, center = true);
            }
}

difference() {
    rounded_cube(W, H, corner_dia, t);
    
    // screws
    translate([-W/2+screw_offset, H/2-screw_offset, 0]) {
        cylinder(d = screw_dia, h = t*2, $fn = 25, center = true);
        translate([0, 0, (t-screw2_height)/2+eps])
            cylinder(d = screw2_dia, h = screw2_height+eps, $fn = 25, center = true);
    }
    translate([-W/2+screw_offset, -H/2+screw_offset, 0]) {
        cylinder(d = screw_dia, h = t*2, $fn = 25, center = true);
        translate([0, 0, (t-screw2_height)/2+eps])
            cylinder(d = screw2_dia, h = screw2_height+eps, $fn = 25, center = true);
    }
    translate([W/2-screw_offset, H/2-screw_offset, 0]) {
        cylinder(d = screw_dia, h = t*2, $fn = 25, center = true);
        translate([0, 0, (t-screw2_height)/2+eps])
            cylinder(d = screw2_dia, h = screw2_height+eps, $fn = 25, center = true);
    }
    
    translate([W/2-screw_offset, -H/2+screw_offset, 0]) {
        cylinder(d = screw_dia, h = t*2, $fn = 25, center = true);
        translate([0, 0, (t-screw2_height)/2+eps])
            cylinder(d = screw2_dia, h = screw2_height+eps, $fn = 25, center = true);
    }
    
    // volume control and MULTI knob    
    translate([-74, 15, 0])
        cylinder(d = pot_dia, h = t*2, $fn = 50, center = true);
    translate([-74, -15, 0])
        cylinder(d = multi_enc_dia, h = t*2, $fn = 50, center = true);
    
    // LCD
    translate([-35, 5, 0]) rotate([0, 0, 90]) {
        cube([lcd_h, lcd_w, t*2], center=true);
        /*translate([LCD_H/2-hole_offset_x, LCD_W/2-hole_offset_y, 0]) {
            cylinder(h = t*2, d = hole_dia, center = true, $fn = 25);
            translate([0, 0, (t-screw2_height)/2+eps])
                cylinder(d = screw2_dia, h = screw2_height+eps, $fn = 25, center = true);
        }*/
        translate([-LCD_H/2+hole_offset_x, LCD_W/2-hole_offset_y, 0]) {
            cylinder(h = t*2, d = hole_dia, center = true, $fn = 25);
            translate([0, 0, (t-screw2_height)/2+eps])
                cylinder(d = screw2_dia, h = screw2_height+eps, $fn = 25, center = true);
        }
        translate([LCD_H/2-hole_offset_x, -LCD_W/2+hole_offset_y, 0]) {
            cylinder(h = t*2, d = hole_dia, center = true, $fn = 25);
            translate([0, 0, (t-screw2_height)/2+eps])
                cylinder(d = screw2_dia, h = screw2_height+eps, $fn = 25, center = true);
        }
        /* translate([-LCD_H/2+hole_offset_x, -LCD_W/2+hole_offset_y, 0]) {
            cylinder(h = t*2, d = hole_dia, center = true, $fn = 25);
            translate([0, 0, (t-screw2_height)/2+eps])
                cylinder(d = screw2_dia, h = screw2_height+eps, $fn = 25, center = true);
        } */
    }
    
    // 3.5 mm jacks
    translate([-24, -15, 0]) {
        cylinder(d = jack_dia, h = t*2, $fn = 50, center = true);
        translate([0, 0, (t - jack_nut_height)/2+eps])
            cylinder(d = jack_nut_dia, h = jack_nut_height, $fn = 50, center = true);
    }
    translate([-47, -15, 0]) {
        cylinder(d = jack_dia, h = t*2, $fn = 50, center = true);
        translate([0, 0, (t - jack_nut_height)/2+eps])
            cylinder(d = jack_nut_dia, h = jack_nut_height, $fn = 50, center = true);
    }
    
    // Main dial
    translate([15, 0, 0]) {
        cylinder(d = main_enc_dia, h = t*2, $fn = 50, center = true);
        
        rotate([0,0,120*0 + main_enc_rotate_delta]) {
            translate([main_enc_screws_dist/2, main_enc_screws_dia, 0]) {
                cylinder(d = main_enc_screw_dia, h = t*2, $fn = 25, center = true);
                translate([0, 0, (t-screw2_height)/2+eps])
                    cylinder(d = screw2_dia, h = screw2_height+eps, $fn = 25, center = true);
            }
            
            /*translate([-main_enc_screws_dist/2, main_enc_screws_dia,0]) {
                cylinder(d = main_enc_screw_dia, h = t*2, $fn = 25, center = true);
                translate([0, 0, (t-screw2_height)/2+eps])
                    cylinder(d = screw2_dia, h = screw2_height+eps, $fn = 25, center = true);
            }*/
        }
        
        rotate([0,0,120*1 + main_enc_rotate_delta]) {
            translate([main_enc_screws_dist/2, main_enc_screws_dia, 0]) {
                cylinder(d = main_enc_screw_dia, h = t*2, $fn = 25, center = true);
                translate([0, 0, (t-screw2_height)/2+eps])
                    cylinder(d = screw2_dia, h = screw2_height+eps, $fn = 25, center = true);
            }
            
            /*translate([-main_enc_screws_dist/2, main_enc_screws_dia,0]) {
                cylinder(d = main_enc_screw_dia, h = t*2, $fn = 25, center = true);
                translate([0, 0, (t-screw2_height)/2+eps])
                    cylinder(d = screw2_dia, h = screw2_height+eps, $fn = 25, center = true);
            }*/
        }
        
        rotate([0,0,120*2 + main_enc_rotate_delta]) {
            translate([main_enc_screws_dist/2, main_enc_screws_dia, 0]) {
                cylinder(d = main_enc_screw_dia, h = t*2, $fn = 25, center = true);
                translate([0, 0, (t-screw2_height)/2+eps])
                    cylinder(d = screw2_dia, h = screw2_height+eps, $fn = 25, center = true);
            }
            
            /*translate([-main_enc_screws_dist/2, main_enc_screws_dia,0]) {
                cylinder(d = main_enc_screw_dia, h = t*2, $fn = 25, center = true);
                translate([0, 0, (t-screw2_height)/2+eps])
                    cylinder(d = screw2_dia, h = screw2_height+eps, $fn = 25, center = true);
            }*/
        }
    }
    
    // Buttons and toggles
    translate([74, 0, 0]) {
        cylinder(d = button_dia, h = t*2, $fn = 50, center = true);
        
        translate([0, 15, 0])
            cylinder(d = button_dia, h = t*2, $fn = 50, center = true);
        
        translate([0, -15, 0])
            cylinder(d = toggle_dia, h = t*2, $fn = 50, center = true);
    }
    
    translate([74-15, 0, 0]) {
        cylinder(d = button_dia, h = t*2, $fn = 50, center = true);
        
        translate([0, 15, 0])
            cylinder(d = button_dia, h = t*2, $fn = 50, center = true);
        
        translate([0, -15, 0])
            cylinder(d = toggle_dia, h = t*2, $fn = 50, center = true);
    }
    
    translate([74-15*2, 0, 0]) {
        cylinder(d = button_dia, h = t*2, $fn = 50, center = true);
        
        translate([0, 15, 0])
            cylinder(d = button_dia, h = t*2, $fn = 50, center = true);
        
        translate([0, -15, 0])
            cylinder(d = toggle_dia, h = t*2, $fn = 50, center = true);
    }
}