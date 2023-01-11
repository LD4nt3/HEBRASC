function R = Ry(theta)

    theta = deg2rad(theta);
    R = [cos(theta) 0 sin(theta); 
               0 1 0;
               -sin(theta) 0 cos(theta)];

end