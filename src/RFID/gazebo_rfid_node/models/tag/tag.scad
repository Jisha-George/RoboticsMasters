function turtle(x, y, angle) = [[x, y], angle];

function get_x(turtle) = turtle[0][0];
function get_y(turtle) = turtle[0][1];
function get_xy(turtle) = turtle[0];
function get_angle(turtle) = turtle[1];

function set_point(turtle, point) = [point, get_angle(turtle)];

function set_x(turtle, x) = [[x, get_y(turtle)], get_angle(turtle)];
function set_y(turtle, y) = [[get_x(turtle), y], get_angle(turtle)];
function set_angle(turtle, angle) = [get_xy(turtle), angle];

function forward(turtle, leng) = 
    turtle(
        get_x(turtle) + leng * cos(get_angle(turtle)), 
        get_y(turtle) + leng * sin(get_angle(turtle)), 
        get_angle(turtle)
    );

function turn(turtle, angle) = [get_xy(turtle), get_angle(turtle) + angle];

module line(point1, point2, width = 1, cap_round = true) {
    angle = 90 - atan((point2[1] - point1[1]) / (point2[0] - point1[0]));
    offset_x = 0.5 * width * cos(angle);
    offset_y = 0.5 * width * sin(angle);

    offset1 = [-offset_x, offset_y];
    offset2 = [offset_x, -offset_y];

    if(cap_round) {
        translate(point1) circle(d = width, $fn = 24);
        translate(point2) circle(d = width, $fn = 24);
    }

    polygon(points=[
        point1 + offset1, point2 + offset1,  
        point2 + offset2, point1 + offset2
    ]);
}

module polyline(points, width = 1) {
    module polyline_inner(points, index) {
        if(index < len(points)) {
            line(points[index - 1], points[index], width);
            polyline_inner(points, index + 1);
        }
    }
    linear_extrude(height = 2, center = true, convexity = 10, twist = 0)
    polyline_inner(points, 1);
}

// The above is the implementation of turtle graphics.



module turtle_spiral(t_before, side_leng, d_step, min_leng, angle, width) {
    if(side_leng > min_leng) {
        t_after = forward(turn(t_before, angle), side_leng);
        polyline([get_xy(t_before), get_xy(t_after)], width);
        turtle_spiral(t_after, side_leng - d_step, d_step, min_leng, angle, width);
    }

}

side_leng = 25;
d_step = 2;
min_leng = 1;
angle = 90;
width = 2;


translate([0,-side_leng/2, -side_leng/2 ])
rotate(a=[0,90,0])
turtle_spiral(
    turtle(0, 0, 0), 
    side_leng, 
    d_step, 
    min_leng, 
    angle, 
    width
);



