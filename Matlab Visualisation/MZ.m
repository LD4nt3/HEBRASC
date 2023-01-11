function MRZ = MZ(angz)
%Matriz de rotacion en z
angz = deg2rad(angz);
MRZ = [cos(angz) -sin(angz) 0; sin(angz) cos(angz) 0; 0 0 1];
end