function MRz = Rz(angz)
%Matriz de rotacion en z
angz = deg2rad(angz);
MRz = [cos(angz) -sin(angz) 0 0;
        sin(angz) cos(angz) 0 0;
        0 0 1 0; 0 0 0 1];
end