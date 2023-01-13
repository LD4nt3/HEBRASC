close all; clear; clc;
%%%%%%% Variables/Inverse kinematics %%%%%%%%
%Coordinadas objetivo/Target coordinates
x=452;
y=395;
z=1;
%%%%%%% Fijos/fixed(home) %%%%%%%%%
theta1=0;
theta2=0;
theta3=0;
theta5=0;
theta4=-90;
%Links sizes and starting denavit angles
d1=60;
db=3.8;%offset1 value from the base link
db2=9.8;%offset2 
d1=db2+d1
d2=126
d4=20;
d3=90
d5=27;
dc6=60;
d5=dc6+d5
%%%%%%% 3D visualisation %%%%%%%%%
%{
*Cyan link= Base

*Black link= Hombro/shoulder

*Yellow link= Codo/elbow

*Purple link= Muñeca-efector final/wrist-end effector
%}

%%%%%%% Denavit-Hartenberg %%%%%%%%%
kk=0.05;
A0 = [0 1 0 0;0 0 1 0;0 0 0 1];

Xmy=abs((x-310)*1.45)
Ymx=abs((y-444)*1.09)

if x<310
Xmy=-Xmy;
end


R06 = [0 1 0;
    0 0 -1;
    -1 0 0];


o06 =[Xmy*kk;% Eje X
    Ymx*kk;% Eje y
    z*kk;]% Eje z

    

o0c=o06-d5*kk*R06;
o0c=o0c*[1;0;0]

A1s = Rz_S('A_r1')*Tz(d1*kk)*Rx_S('pi/2');
T1s = A1s;%k<--0- a - 1;
A22=Tz(-d4*kk);
T2x = T1s*A22;%k<--2- a - 3;

A2s = Rz_S('pi/2 +A_r2')*Tx(d2*kk);

T2s = T2x*A2s;%k<--1- a - 2;
A3s = Rz_S('-pi/2 +A_r3')*Tx(d3*kk);
T3s = T2s*A3s;%k<--2- a - 3;

syms A_r1 A_r2 A_r3 
    px=o0c(1);
    py=o0c(2);
    pz=o0c(3);
    eqn1 = T3s(1:1,4) == px;
    eqn2 = T3s(2:2,4) ==  py;%1.5
    eqn3 = T3s(3:3,4) == pz;

    [A_r1, A_r2, A_r3]= vpasolve(eqn1,eqn2,eqn3,A_r1,A_r2,A_r3);
    Ar1=double(A_r1);
    Ar2=double(A_r2);
    Ar3=double(A_r3);
    theta1=wrapTo360(rad2deg(Ar1));
    theta2=wrapTo360(rad2deg(Ar2));
    theta3=wrapTo360(rad2deg(Ar3));



A1 = Rz(theta1)*Tz(d1*kk)*Rx(90);
T1 = A1;%k<--0- a - 1;
A22=Tz(-d4*kk);
T2x = T1*A22;%k<--2- a - 3;
A2 = Rz(90+theta2+0)*Tx(d2*kk);
T2 = T2x*A2;%k<--1- a - 2;Tz(0)*
A3 = Rz(-90+theta3+0)*Tx(d3*kk);
T3 = T2*A3;%k<--2- a - 3;
%v1=T3(1:3,4)-T2(1:3,4)
%v2=o06-T3(1:3,4)
%a = atan2d(norm(cross(v1,v2)),dot(v1,v2));
R36=T3(1:3,1:3)'*R06;
a1=wrapTo360(atan2d(R36(8),R36(7)));
a2=wrapTo360(atan2d(sqrt(R36(8)^2+R36(7)^2),R36(9)));
a3=wrapTo360(atan2d(-R36(9),R36(3)));

for x=-90:45:180
    t4=theta4+a1+x;
    A4 = Rz(t4);%*Tz(0*kk);
    T4 = T3*A4;%k<--3- a - 4;
    A5 = Tx(d5*kk)*Rx(theta5);
    T = T4*A5;%R36=T3(1:3,1:3)'*T(1:3,1:3)
   if round(T(1:3,4))==round(o06)
       theta4=t4;
       break
   end
end

t4=wrapTo360(t4);
if  t4>180
   t4=360-t4;
end

t1=theta1
if  theta1>180
t1=theta1-180
end

error=0;
if theta2>180
    t2=180-(360-theta2);
    t3= wrapTo360(abs(315-180-theta3+361))-1;
    men=135-theta3;%rango 315-360--0-135
    if men<0 && men>-179.99
        error='theta3 fuera de rango';
    end

else

    t2=180-theta2;
    t3= abs(225-180-theta3)
    men=225-theta3;
    if men<0||men>180
        error='theta3 fuera de rango';
    end
end

error

%Output angles
t1
t2
t3
t4

hold on
axis equal
grid minor

xlabel('$ X $', 'Interpreter','latex','fontsize',14)
ylabel('$ Y $', 'Interpreter','latex','fontsize',14)
zlabel('$ Z $', 'Interpreter','latex','fontsize',14)

vec3([A0(:,1)],[A1(1:3,4)],.1,'c')
vec3([T1(1:3,4)],T1(1:3,1:3)*A2(1:3,4),.1,'black')
vec3([T2(1:3,4)],T2(1:3,1:3)*A3(1:3,4),.1,'yellow')
vec3([T3(1:3,4)],T3(1:3,1:3)*A4(1:3,4),.1,'m')
vec3([T4(1:3,4)],T4(1:3,1:3)*A5(1:3,4),.1,'m')

frame(A0,0,0.05);%<---- identidad real!!

frame([T1(1:3,4),T1(1:3,1:3)],1,0.05);
frame([T2(1:3,4),T2(1:3,1:3)],2,0.05);
frame([T3(1:3,4),T3(1:3,1:3)],3,0.05);
frame([T4(1:3,4),T4(1:3,1:3)],4,0.05);
frame([T(1:3,4),T(1:3,1:3)],5,0.05);
figuresk(0,20,1,12);


%view(-90,68)
view(0,88)
