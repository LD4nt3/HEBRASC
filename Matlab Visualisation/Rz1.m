function R = Rz1(theta)
    theta = deg2rad(theta);
    R = [cos(theta) -sin(theta) 0; 
               sin(theta) cos(theta) 0;
               0 0 1];

    
end