function [e,u] = GS(v)
    if size (v,2) == 2
        v = [v cross(v(:,1),v(:,2))];
    end
    u = zeros(size(v)); %Base ortogonal
    e = u; %Base Ortogonormal
    for i = 1 : size(v,2)
        if i == 1
            u(:,i) = v(:,i);
            e(:,i) = u(:,i)/norm(u(:,i));
        else
            u(:,i) = v(:,i);
            for j = 1:i-1
                u(:,i) = u(:,i) - (dot(v(:,i),u(:,j)) / (dot(u(:,j),u(:,j)))).*u(:,j);
            end
            e(:,i) = u(:,i)/norm(u(:,i));
        end
    end
end
