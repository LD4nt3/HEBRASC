%%vec3([1 1 0],[1 1 1],0.05,'b')
function vec3(o0_1,o1_2,a,color)
    if size(o0_1,2) ~= 1
        o0_1 = o0_1';
        o1_2 = o1_2';
    end
    if length(o0_1) < 3
        o0_1 = [o0_1;0];
        o1_2 = [o1_2,0];
    end
% =====================================
    if color == 'o'
        color = [0.8500 0.3250 0.0980]; % Naranja "mecanica" XD
    end
    % =====================================
    N = norm(o1_2);
    o1_4 = [N;0;0];
    if abs(proy(o1_4/N,o1_2/N)) == 1
        C1_3 = MZ(0);
    else 
        [C1_3,~] = GS([o1_4 o1_2 cross(o1_4,o1_2)]); %Gram Schimdt
    end
    C3_1 = C1_3';
    o3_4 = C3_1 * o1_4;
    o3_2 = C3_1 * o1_2;
    theta = cosinesd(o3_4,o3_2); %Angulo entre vectores (grados)
    % ========================================
    b = 4;
    o4_p = zeros(3,b);
    for i = 1:b
        o4_p(:,i) = MX(i*(360/b))*MZ(30)*[-a;a;0];
    end
% ========================================
    R3_3 = MZ(theta);
    o3_4 = R3_3 * o3_4;
    o0_3 = o0_1;
    C4_3 = C1_3;
    C0_3 = C1_3;
    o3_p = o3_4 + R3_3 * C4_3' * o4_p;
    o3_p = [o3_4 o3_p]; % Concatenación de puntos
    o0_4 = o0_3 + C0_3 * o3_4;
    o0_p = o0_3 + C0_3 * o3_p;
% ======================================
    hold on
    grid on
    S.FaceColor = color;
    S.EdgeColor = color;
    S.LineWidth = 1;
    S.FaceVertexCData = [0;1];
    o0_p = o0_p';
    for i = 1:b+1
        if i <= (b+1)-2
            S.Faces = [1 2 3];
            S.Vertices = [o0_p(1,:);o0_p(i+1,:);o0_p(i+2,:)];
        else
            S.Faces = 1:b;
            S.Vertices = o0_p(2:b+1,:);
        end
        patch(S)
        if i == b+1
            S.Faces = [1 2 3];
            S.Vertices = [o0_p(1,:);o0_p(b+1,:);o0_p(2,:)];
            patch(S)
        end
    end
    plot3([o0_1(1) o0_4(1)],[o0_1(2) o0_4(2)],[o0_1(3) o0_4(3)],'Color',color,'LineWidth',a*2);
end

