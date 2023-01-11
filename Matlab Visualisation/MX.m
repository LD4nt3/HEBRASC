function MRX = MX(angx)
%Matriz de rotacion en z
angx = deg2rad(angx);
MRX = [1 0 0; 0 cos(angx) -sin(angx); 0 sin(angx) cos(angx)];
end