function MRz = Rz_S(An)
angz=str2sym(An);
    %syms A_r;
MRz = [cos(angz) -sin(angz) 0 0; sin(angz) cos(angz) 0 0; 0 0 1 0; 0 0 0 1];

    
end