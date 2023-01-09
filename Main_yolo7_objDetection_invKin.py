import torch
import numpy as np
import cv2
import imutils
import sys
import time
import yaml
import os
import json
import freenect

import math as m
import sympy as sym
from sympy import sin, cos, lambdify, nsolve, Add, pi
import warnings
from time import sleep

from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit

warnings.filterwarnings("ignore")


sym.init_printing()
i2c = busio.I2C(SCL, SDA)

pca = PCA9685(i2c)

kit = ServoKit(channels=16)
kit.servo[2].set_pulse_width_range(465, 2420)
kit.servo[3].set_pulse_width_range(560, 2480)
kit.servo[4].set_pulse_width_range(525, 2550)
kit.servo[3].set_pulse_width_range(525, 2550)
kit.servo[4].set_pulse_width_range(525, 2550)
kit.servo[5].set_pulse_width_range(525, 2550)
kit.servo[9].set_pulse_width_range(670, 2460)

theta1=0;
theta2=0;
theta3=0;
theta5=0;
theta4=-90;

d1=60;
db=3.8;
db2=9.8;
d1=db2+d1
d2=126
d4=10;
d3=90
d5=27;
dc6=60;
d5=dc6+d5

a1 = d1 
a2 = d2 
a3 = d3 
a4 = 0 
a5 = d5 
ax = d4  
kk=1
kz=0.7

R06 = np.array([[0.0, 1.0, 0.0],
                        [0.0, 0.0, -1.0],
                        [-1.0, 0.0, 0.0]])
Rball=[]
Rbox=[]

# https://github.com/ultralytics/yolov5/issues/6460#issuecomment-1023914166
# https://github.com/ultralytics/yolov5/issues/36


# Loading Model
#model = torch.hub.load('C:/Users/17R4/Desktop/proyecto/yolov5', 'custom', path="C:/Users/17R4/Desktop/proyecto/yolov5/runs/train/exp2/weights/best.pt", source='local')  # local repoprint(cv2.__version__)
_model = torch.hub.load('/home/chugv2/Desktop/Proyecto/yolov7',
                      'custom', "bestv0.pt", source='local')  # local repo
model = torch.load("bestv0.pt")
#model = torch.hub.load("yolov5", 'custom', path="yolov5/runs/train/exp/weights/yolo_weights.pt", source='local', force_reload=True)  # local repo
#model = torch.hub.load("C:/Users/17R4/Desktop/proyecto/yolov5/runs/train/exp2/weights/", 'custom', path="best.pt", source='local')

# Configuring Model

'''

model = torch.hub.load('ultralytics/yolov5', 'yolov5x')
'''

#model.classes =66
#model.cpu()  #  .cpu() ,or .cuda()
_model.conf = 0.7  # NMS confidence threshold
#model.iou = 0.45  # NMS IoU threshold
#model.agnostic = False  # NMS class-agnostic
#model.multi_label = False  # NMS multiple labels per box
_model.classes = 2  # filter by class, i.e. = [0, 15, 16] 
#model.max_det = 10  # maximum number of detections per image
#model.amp = False  # Automatic Mixed Precision (AMP) inference

def get_video():
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    return array
 
#function to get depth image from kinect
def get_depth():
    array,_ = freenect.sync_get_depth()
    #array = array.astype(np.uint8)
    return array

# Function to draw Centroids on the deteted objects and returns updated image
def draw_centroids_on_image(output_image, json_results, depth):   
    data = json.loads(json_results) 
    for objects in data:
        xmin = objects["xmin"]
        ymin = objects["ymin"]
        xmax = objects["xmax"]
        ymax = objects["ymax"]

        cx = int((xmin+xmax)/2.0)
        cy = int((ymin+ymax)/2.0)   

        print(cx,cy)
        #z = depth[int(cy),int(cx)]
        #depV = ((4-0.8)/2048)*(depth[xVd,yVd]+1)+0.8
        #depV = 1/(depth[cy,cx]*(-0.0028642) + 3.15221)
        #para obtener dato es en coordenada (y,x)->(480x640)
        depV = depth[cy,cx]
        depV = round(depV,4)
        print(depV)
        if objects["class"]==2:
            Rball.append(cx)
            Rball.append(cy)
            _model.classes =1
            _model.conf = 0.3
            #Rball.append(depV)
            
            print(_model.classes)
        if objects["class"]==1:
            Rbox.append(cx)
            Rbox.append(cy)
            #Rbox.append(depV)
            
        
        cv2.circle(output_image, (cx,cy), 2, (0, 0, 255), 2, cv2.FILLED) #draw center dot on detected object
        cv2.putText(output_image, str(str(cx)+" , "+str(cy)), (int(cx)-40, int(cy)+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)

    return (output_image)
        



if __name__== "__main__":    
    while(1):
        
        #Start reading camera feed (https://answers.opencv.org/question/227535/solvedassertion-error-in-video-capturing/))
        #cap = cv2.VideoCapture(0)
        #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        #Now Place the base_plate_tool on the surface below the camera.
        while(1):
            frame = get_video()
            depth = get_depth()
            
            #frame = undistortImage(frame)
            #cv2.imshow("Live" , frame)
            k = cv2.waitKey(1)
            if k == 27: #exit by pressing Esc key
                cv2.destroyAllWindows()
                sys.exit()
            #if k == 13: #execute detection by pressing Enter key           
            #image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # OpenCV image (BGR to RGB)
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # OpenCV image (BGR to RGB)
            # Inference
            results = _model(image) #includes NMS
            # Results
            #results.print()  # .print() , .show(), .save(), .crop(), .pandas(), etc.
            #results.show()
            results.xyxy[0]  # im predictions (tensor)
            results.pandas().xyxy[0]  # im predictions (pandas)
            #      xmin    ymin    xmax   ymax  confidence  class    name
            # 0  749.50   43.50  1148.0  704.5    0.874023      0  person
            # 2  114.75  195.75  1095.0  708.0    0.624512      0  person
            # 3  986.00  304.00  1028.0  420.0    0.286865     27     tie
            
            #Results in JSON
            json_results = results.pandas().xyxy[0].to_json(orient="records") # im predictions (JSON)
            print(json_results)
            if json_results!='[]':
            #print(type(frame))
            #cv2.imshow('Frame from Webcam', image)
            #cv2.waitKey(0)
                results.render()  # updates results.imgs with boxes and labels                    
                output_image = results.imgs[0] #output image after rendering
                output_image = cv2.cvtColor(output_image, cv2.COLOR_RGB2BGR)
                
                output_image = draw_centroids_on_image(output_image, json_results, depth) # Draw Centroids on the deteted objects and returns updated image
                cv2.imshow("Output", output_image) #Show the output image after rendering
                #if cv2.waitKey(20) & 0xFF == ord('q'):
                #    break
                
                cv2.waitKey(1)
                print(Rball,Rbox)#reset falta
                if Rball and Rbox:
                    kit.servo[3].angle = 145
                    time.sleep(1.8)
                    kit.servo[4].angle = 125
                    time.sleep(2)
                    kit.servo[2].angle = 180
                    time.sleep(2.8)
                    kit.servo[3].angle = 180
                    time.sleep(1.8)
                    kit.servo[9].angle = 0
                    time.sleep(2)
                    kit.servo[8].angle = 170
                    time.sleep(1.4)
                        #home
                    while Rball:
                        #home
                        xc=[int(Rball[0])]
                        yc=[int(Rball[1])]
                        #dv=[int(Rball[2])]
                        xc.append(int(Rbox[0]))
                        yc.append(int(Rbox[1]))
                        #dv.append(int(Rbox[2]))
                        
                        del Rball[0]
                        del Rball[0]
                        #del Rball[2]
                        
                        while xc:
                            print(xc)
                            Xmy=abs((xc[0]-304)*1.436)
                            Ymx=abs((yc[0]-402)*1.06)
                            zd=-26
                            
                            
                            if xc[0]<304:
                                Xmy=-Xmy
                        
                            o06 =np.array([[Xmy*kz],# Eje X
                                           [Ymx*kz],# Eje y
                                           [zd]])# Eje z
                            o0c=o06-d5*kk*R06;
                            #o0c=o0c*np.array([[1],[0],[0]])
                            #print(o0c)
                            px=o0c[0,0];
                            py=o0c[1,0];
                            pz=o0c[2,0];
                            #print(px,py,pz)
                            a1 = d1 
                            a2 = d2 
                            a3 = d3 
                            a4 = 0 
                            a5 = d5 
                            ax = d4  
                            # theta, alpha, r, and d
                            #rz rx tx tz?
                            #sym.init_printing()
                            A_r1,A_r2,A_r3 = sym.symbols('A_r1,A_r2,A_r3')
                            eqn1 = sym.Eq(126*cos(A_r1)*cos(A_r2 + pi/2) - 20*sin(A_r1) + 90*cos(A_r1)*cos(A_r2 + pi/2)*cos(A_r3 - pi/2) - 90*cos(A_r1)*sin(A_r2 + pi/2)*sin(A_r3 - pi/2),px)
                            eqn2 = sym.Eq(20*cos(A_r1) + 126*cos(A_r2 + pi/2)*sin(A_r1) + 90*cos(A_r2 + pi/2)*cos(A_r3 - pi/2)*sin(A_r1) - 90*sin(A_r1)*sin(A_r2 + pi/2)*sin(A_r3 - pi/2),py)
                            eqn3 = sym.Eq(126*sin(A_r2 + pi/2) + 90*cos(A_r2 + pi/2)*sin(A_r3 - pi/2) + 90*cos(A_r3 - pi/2)*sin(A_r2 + pi/2) + 349/5, pz)
                            try:
                                xx=nsolve([eqn1, eqn2, eqn3], [A_r1,A_r2,A_r3], [0,0,0])
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
                                  t4=theta4+a1+x;
                                  angz = np.deg2rad(t4);
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
                                error=0;
                                if theta2>180:
                                    t2=180-(360-theta2);
                                    t3=((abs(315-180-theta3+361))%360)-1;
                                    men=135-theta3;#rango 315-360--0-135
                                    if men<0 and men>(-179.99):
                                        error='theta3 fuera de rango';
                                else:
                                  t2=180-theta2;
                                  t3= abs(225-180-theta3)
                                  men=225-theta3;
                                  if men<0 or men>180:
                                      error='theta3 fuera de rango';
                                print("Matrix Frame 0 to 5:")
                                print(t1,t2,t3,t4)
                                print(T)
                                print()
                                print(error)
                                print()
                                print(x)
                                if error==0:
                                    angle_i=90
                                    zz=180
                                    angle1 = t1
                                    angle2 = t2
                                    angle3 = t3
                                    angle4 = t4
                                    angle5 = 70
                                    kit.servo[2].angle = angle1
                                    time.sleep(2)
                                    kit.servo[4].angle = angle3
                                    time.sleep(2)                
                                    kit.servo[9].angle = angle4
                                    time.sleep(2)
                                    dif=(zz-angle2)+1
                                    print(zz)
                                    print(dif)
                                    print(x)
                                    if dif<0:
                                        paso=10
                                    else:
                                        paso=-10
                                    for x in range (int(zz),int(angle2+1),paso):
                                        #zz=x
                                        print(x)
                                        kit.servo[3].angle = x
                                        time.sleep(1.2)
                                        #0print(zz)
                                    kit.servo[3].angle = angle2
                                    time.sleep(1.1)
                                    kit.servo[8].angle = angle5
                                    time.sleep(2)
                                    
                                    kit.servo[3].angle = 135
                                    time.sleep(1.8)
                                    kit.servo[4].angle = 45
                                    time.sleep(2)
                                    kit.servo[2].angle = 90
                                    time.sleep(1.8)
                                    kit.servo[3].angle = 170
                                    time.sleep(1.8)
                                
                                    print(x)
                                    del xc[0]
                                    del yc[0]
                                    zd=40
                                    if not xc:
                                        kit.servo[8].angle = 170
                                        time.sleep(1.8)
                                    #print(x)

                                #else:
                                 #   if x:
                                  #      x.clear()
                                   #     y.clear()#si no puede llegar a caja que no la agarre
                            except Exception as e:
                                print("Error in Main Loop\n",e)
                                #x.clear()
                                #y.clear()
                                print("no hay solucion")
                                kit.servo[3].angle = 145
                                time.sleep(1.8)
                                kit.servo[4].angle = 125
                                time.sleep(2)
                                kit.servo[2].angle = 180
                                time.sleep(2.8)
                                kit.servo[3].angle = 180
                                time.sleep(1.8)
                                kit.servo[9].angle = 0
                                time.sleep(2)
                                kit.servo[8].angle = 170
                                time.sleep(1.4)
            
                            #sleep(300)
                            print("fin")
                            #sleep(60)
                
                 
                    
        
    #cv2.destroyAllWindows()
    
    
    
