clc; close all; clear;
%%%%%%% Variables/Direct kinematics %%%%%%%%
%Valores para las articulaciones en angulos/Values for joints in angle units
theta1=0
theta2=0;
theta3=0;
theta4=-90;
theta5=0;

%%%%%%% Fijos/fixed %%%%%%%%%
%Links sizes and starting denavit angles

d1=60;
db=3.8;%offset1 value from the base link
db2=9.8;%offset2 
d1=db2+d1
d2=126
d4=-20;
d3=90
d5=27;
dc6=60;
d5=dc6+d5

kk=0.05;

%%%%%%% 3D visualisation %%%%%%%%%

%{
*Cyan link= Base

*Black link= Hombro/shoulder

*Yellow link= Codo/elbow

*Purple link= Muñeca-efector final/wrist-end effector
%}

%%%%%%% Denavit-Hartenberg %%%%%%%%%
A0 = [0 1 0 0;0 0 1 0;0 0 0 1];
%%A= [d r'ó'a a~ 0;tz tx rx Rz;];
R06 = [0 1 0;
    0 0 -1;
    -1 0 0];

%o06=[1;0;0];
%o0c=o06-d5*kk*R06;
%o0c=o0c*[1;0;0]
%vec3([A0(:,1)],o06,.1,'c')


A1 = Rz(theta1)*Tz(d1*kk)*Rx(90)
T1 = A1;%k<--0- a - 1;
A22=Tz(d4*kk);
T2x = T1*A22;%k<--2- a - 3;
A2 = Rz(90+theta2)*Tx(d2*kk)
T2 = T2x*A2;%k<--1- a - 2;
A3 = Rz(-90+theta3)*Tx(d3*kk)
T3 = T2*A3

O0C=T3(1:3,4:4)
o06=O0C+d5*kk*R06*[1;0;0];

R36=T3(1:3,1:3)'*R06;
a1=atan2d(R36(8),R36(7));
a2=atan2d(sqrt(R36(8)^2+R36(7)^2),R36(9));
a3=atan2d(-R36(9),R36(3));
%theta4=theta4+a1+180

A4 = Rz(theta4)
T4 = T3*A4%k<--3- a - 4;
A5 = Tx(d5*kk)*Rx(theta5);
T = T4*A5


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


view(0,88)



 