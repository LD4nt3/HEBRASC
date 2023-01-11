function R = Rx_S(An)
angx=str2sym(An);
    %syms A_r;

    R = [1 0 0 0; 0 cos(angx) -sin(angx) 0; 0 sin(angx) cos(angx) 0; 0 0 0 1];

end