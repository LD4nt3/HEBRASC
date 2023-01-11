function [ab,ab_v] = proy(a,b)
ab = (dot(a,b)/(norm(b)));
ab_v = (dot(a,b)/(norm(b)^2))*b;
end