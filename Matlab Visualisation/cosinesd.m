function theta = cosinesd(u,v)
    if proy(u/norm(u),v/norm(v)) == -1
        theta = 180;
    else
        theta = acos((dot(u,v))/(norm(u)*norm(v)));
        dg = sign(ones(1,3)*cross(u,v)); %Direccion de giro
        theta = dg*theta;
        theta = rad2deg(theta);
    end
end