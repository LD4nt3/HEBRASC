import numpy as np  
import math as m
import time
import sys

from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit


import sympy as sym
from sympy import sin, cos, lambdify, nsolve, Add, pi

i2c = busio.I2C(SCL, SDA)

pca = PCA9685(i2c)

kit = ServoKit(channels=16)
kit.servo[1].set_pulse_width_range(465, 2420)
kit.servo[3].set_pulse_width_range(560, 2480)
kit.servo[4].set_pulse_width_range(525, 2550)
kit.servo[3].set_pulse_width_range(525, 2550)
kit.servo[4].set_pulse_width_range(525, 2550)
kit.servo[5].set_pulse_width_range(525, 2550)
kit.servo[9].set_pulse_width_range(670, 2460)

theta1=0
theta2=0
theta3=0
theta5=0
theta4=-90

#link lengths
d1=60
db=3.8
db2=9.8
d1=db2+d1
d2=126
d4=20
d3=90
d5=27
dc6=60
d5=dc6+d5

kk=1

#target coordinatesx=352;
y=200
e=0.1
s=0.1
print(e)
Xmy=abs((x-304)*(1-(e)))
Ymx=abs((y-402)*(1-s))

Xmy=abs((x-304)*(1-(0.1)))
Ymx=abs((y-402)*(1+(0.05)))

zd=-36
if x<304:
  Xmy=-Xmy

k=0.7
o06 =np.array([[Xmy*k],# Eje X
               [Ymx*k],# Eje y
               [zd]])# Eje z


R06 = np.array([[0.0, 1.0, 0.0],
                        [0.0, 0.0, -1.0],
                        [-1.0, 0.0, 0.0]])
o0c=o06-d5*kk*R06
#o0c=o0c*np.array([[1],[0],[0]])
print(o0c)

px=o0c[0,0]
py=o0c[1,0]
pz=o0c[2,0]
print(px,py,pz)


a1 = d1 
a2 = d2 
a3 = d3 
a4 = 0 
a5 = d5 
ax = d4  
 
theta1 = 0 
theta2 = 0

theta3 = 0
theta4 = -90 
theta5 = 0 
 

# theta, alpha, r, and d

#rz rx tx tz?
sym.init_printing()
A_r1,A_r2,A_r3 = sym.symbols('A_r1,A_r2,A_r3')

eqn1 = sym.Eq(126*cos(A_r1)*cos(A_r2 + pi/2) - 20*sin(A_r1) + 90*cos(A_r1)*cos(A_r2 + pi/2)*cos(A_r3 - pi/2) - 90*cos(A_r1)*sin(A_r2 + pi/2)*sin(A_r3 - pi/2),px)
eqn2 = sym.Eq(20*cos(A_r1) + 126*cos(A_r2 + pi/2)*sin(A_r1) + 90*cos(A_r2 + pi/2)*cos(A_r3 - pi/2)*sin(A_r1) - 90*sin(A_r1)*sin(A_r2 + pi/2)*sin(A_r3 - pi/2),py)
eqn3 = sym.Eq(126*sin(A_r2 + pi/2) + 90*cos(A_r2 + pi/2)*sin(A_r3 - pi/2) + 90*cos(A_r3 - pi/2)*sin(A_r2 + pi/2) + 349/5, pz)


xx=nsolve([eqn1, eqn2, eqn3], [A_r1,A_r2,A_r3], [0,-1,0])
theta1=np.rad2deg(float(xx[0,0]))%360
theta2=np.rad2deg(float(xx[1,0]))%360
theta3=np.rad2deg(float(xx[2,0]))%360

print(theta1,theta2,theta3)
print()
d_h_table = np.array([[np.deg2rad(theta1), np.deg2rad(90), 0 ,a1],
                      [np.deg2rad(90+theta2), 0, a2, 0],
                      [np.deg2rad(-90+theta3), 0,  a3,0],
                      [np.deg2rad(theta4), 0, a4,0],
                      [0, np.deg2rad(theta5), a5, 0],
                      [0,0, 0, -ax]]) 
 
i = 0
homgen_0_1 = np.array([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), d_h_table[i,2] * np.cos(d_h_table[i,0])],
                      [np.sin(d_h_table[i,0]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), d_h_table[i,2] * np.sin(d_h_table[i,0])],
                      [0, np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3]],
                      [0, 0, 0, 1]])  

i = 5
homgen_0_x = np.array([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), d_h_table[i,2] * np.cos(d_h_table[i,0])],
                      [np.sin(d_h_table[i,0]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), d_h_table[i,2] * np.sin(d_h_table[i,0])],
                      [0, np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3]],
                      [0, 0, 0, 1]])   
i = 1
homgen_1_2 = np.array([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), d_h_table[i,2] * np.cos(d_h_table[i,0])],
                      [np.sin(d_h_table[i,0]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), d_h_table[i,2] * np.sin(d_h_table[i,0])],
                      [0, np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3]],
                      [0, 0, 0, 1]])  
 

i = 2
homgen_2_3 = np.array([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), d_h_table[i,2] * np.cos(d_h_table[i,0])],
                      [np.sin(d_h_table[i,0]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), d_h_table[i,2] * np.sin(d_h_table[i,0])],
                      [0, np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3]],
                      [0, 0, 0, 1]])  
T3=homgen_0_1 @ homgen_0_x @homgen_1_2 @ homgen_2_3
a=T3[0:3,0:3]
a=np.transpose(a) @ R06
a1=(np.rad2deg(m.atan2(a[1,2],a[0,2])))%360
a2=(np.rad2deg(m.atan2(np.sqrt(a[1,2]**2+a[0,2]**2),a[2,2])))%360
a3=(np.rad2deg(m.atan2(-a[2,2],a[2,0])))%360
print(a1)
print()

i = 4
A5= np.array([[np.cos(d_h_table[i,0]), -np.sin(d_h_table[i,0]) * np.cos(d_h_table[i,1]), np.sin(d_h_table[i,0]) * np.sin(d_h_table[i,1]), d_h_table[i,2] * np.cos(d_h_table[i,0])],
                    [np.sin(d_h_table[i,0]), np.cos(d_h_table[i,0]) * np.cos(d_h_table[i,1]), -np.cos(d_h_table[i,0]) * np.sin(d_h_table[i,1]), d_h_table[i,2] * np.sin(d_h_table[i,0])],
                    [0, np.sin(d_h_table[i,1]), np.cos(d_h_table[i,1]), d_h_table[i,3]],
                    [0, 0, 0, 1]])  

o0x=        [round(float(o06[0]),1),
            round(float(o06[1]),1),
            round(float(o06[2]),1)]
for x in range (-90,181,45):
  d_table = np.array([[np.deg2rad(theta4), 0, a4,0]]) 
  t4=theta4+a1+x
  angz = np.deg2rad(t4)
  MRz = np.array([[cos(angz),-sin(angz), 0, 0],
                  [sin(angz), cos(angz), 0, 0],
                  [0, 0, 1, 0,],
                  [0,0, 0, 1]])
  A4 = MRz  
  T4 = T3 @ A4
  T =T4 @ A5
  Ts=T[0:3,3][0]
  Ts =        [round(float(T[0:3,3][0]),1),
                  round(float(T[0:3,3][1]),1),
                  round(float(T[0:3,3][2]),1)]
  
  if Ts==o0x:
    print(t4)
    break

t4=t4%360
if  t4>180:
   t4=360-t4


t1=theta1
if  theta1>180:
  t1=theta1-180

error=0
if theta2>180:
    t2=180-(360-theta2)
    t3=((abs(315-180-theta3+361))%360)-1
    men=135-theta3
    if men<0 and men>(-179.99):
        error='theta3 fuera de rango'
   

else:
  t2=180-theta2
  t3= abs(225-180-theta3)
  men=225-theta3
  if men<0 or men>180:
      error='theta3 fuera de rango'


print("Homogeneous Matrix Frame 0 to Frame 5:")
print(t1,t2,t3,t4)
print(T)
if error!=0:
    print(error)
    sys.exit()
kit.servo[3].angle = 145
time.sleep(1.8)
kit.servo[4].angle = 125
time.sleep(2)
kit.servo[1].angle = 180
time.sleep(2.8)
kit.servo[3].angle = 180
time.sleep(1.8)
kit.servo[9].angle = 0
time.sleep(2)
kit.servo[8].angle = 170

time.sleep(2)
angle_i=90
zz=180
angle1 = t1
angle2 = t2
angle3 = t3
angle4 = t4
angle5 = 70
try:
    while True:

        kit.servo[1].angle = angle1
        time.sleep(2)
        si=angle3+45
        kit.servo[4].angle = angle3
        time.sleep(2)                
        kit.servo[9].angle = angle4
        time.sleep(2)
        dif=(zz-angle2)+1
        print(zz)
        print(dif)
        if dif<0:
            paso=10
        else:
            paso=-10
        for x in range (int(zz),int(angle2+1),paso):
            
            print(x)
            kit.servo[3].angle = x
            time.sleep(1.2)
        kit.servo[4].angle = angle3
        time.sleep(2)
        kit.servo[8].angle = angle5
        time.sleep(2)
        sys.exit()  
finally:
    kit.servo[8].angle = 90

    time.sleep(1.2)
    pca.deinit()
    print("Adios")
    





 
