union() {
    translate([-0.005, 0.1, 0.1])
        rotate(a=[90,0,-90]) {
                color("Black")
                resize(newsize=[0.1,0.07,0])
                linear_extrude(0.01)
                    text("UP",size=1, halign = "center",valign= "center");
        }        
    cube([0.02,0.4,0.4], center = true);      
    translate([-0.35, 0, 0])
        difference() {
            scale(0.02)
                sphere(r = 20);
             translate([-0.15, 0, 0])
               cube([1.0,2.0,2.0], center = true);     
        }
    translate([-0.01, -0.1, -0.1])
        rotate(a=[0,90,0]) {
            scale(0.02)
                cylinder( h=2,r1= 0.5,r2=0.5,center= true);
        }
}