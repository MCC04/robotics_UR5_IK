#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-
from __future__ import division
import time
import sys
from math import *
from array import *
import numpy as np
from numpy.linalg import inv,norm

import roslib; roslib.load_manifest('ur_driver') 
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

#vision
import cv2
import image_geometry
from Camera import Camera
from cv_bridge import CvBridge
bridge = CvBridge()

# OpenCV2 for saving an image
from collections import deque
import glob
from termcolor import colored
from cv_bridge import CvBridge, CvBridgeError

#msgs
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState

# define the list of boundaries of colours
boundaries = [([0, 0, 100], [50, 56, 200])]

#GLOBAL
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [2.2,0,-1.57,0,0,0]
Q2 = [1.5,0,-1.57,0,0,0]
Q3 = [1.5,-0.2,-1.57,0,0,0]
client = None
joints = None

####################################
##########__ARM MOVES__#############
####################################
def move1():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
        #JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        #JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

####################################
##########IMAGE CONVERTER###########
####################################
class image_converter:
  def __init__(self):
    print "INIT OF IMAGE CONVERTER"
    self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
    self.camera = Camera()
    self.bridge = CvBridge()
    self.realRadius=0.02
    self.knownWidth=self.realRadius*2

    #TEST1    
    #self.dist=(2 + 0.2027 + self.realRadius)
    #self.focal=(4.048*2*self.dist)/self.knownWidth #445.8
    #TEST2   
    #self.dist=(2 - (0.6001 )+ self.realRadius*2)    
    #self.focal=(6.7801*2*self.dist)/self.knownWidth #445.8
    #TEST3    
    self.dist=(2 + 0.4157 + self.realRadius)
    self.focal=(3.6056*2*self.dist)/self.knownWidth #445.8


    self.image_sub = rospy.Subscriber("ur5/camera1/image_raw",Image,self.callback)

  """compute and return the distance, from marker to camera, as:
     (knownWidth * focalLength) / pixelWidth"""
  def distance_to_camera(self, pixelWidth):
    return (self.knownWidth*self.focal)/ pixelWidth

  def get3DpointCamera(self,x,y,radius,img):
    height, width, channels = img.shape
    rho=self.distance_to_camera(radius*2)
    print "Focal: ",self.focal
    print "Rho:", rho

    azimuth,elevation=self.camera.pixel2angle(x,y)
    elevRad=(pi/2)-elevation
    azimuthRad=(pi/2)+azimuth
    print "azimuthRad: ",azimuthRad
    print "elevRad: ",elevRad

    newx=rho*sin(elevRad)*cos(azimuthRad)
    newy=rho*sin(elevRad)*sin(azimuthRad)
    newz=rho*cos(elevRad)
    print "NEW x,y,z: ",newx,newy,newz

    cm=cos(-pi/2)
    sm=sin(-pi/2)
    cp=cos(pi/2)
    sp=sin(pi/2)
    c0=cos(0)
    s0=sin(0)
    c180=cos(radians(180))
    s180=sin(radians(180))

    rotX=np.matrix([[1, 0, 0, 0],
                    [0, c180, -s180, 0],
                    [0, s180, c180, 0],
                    [0, 0, 0, 1]])
    rotZ=np.matrix([[cp, -sp, 0, 0],
                    [sp, cp, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    rotY=np.matrix([[cp, 0, sp, 0],
                    [0, 1, 0, 0],
                    [-sp, 0, cp, 0],
                    [0, 0, 0, 1]])
    rotations=rotZ*rotX

    transform=np.array([-2,0,1,1])
    worldSpace=rotations
    worldSpace[[0,1,2,3],3]=transform
    #print "world space: ", worldSpace    
    coordWorldSpace=np.dot(worldSpace,[newx,newy,newz,1])
    print colored(("World coord ",coordWorldSpace),'blue')

    return coordWorldSpace.item(0,0),coordWorldSpace.item(0,1),coordWorldSpace.item(0,2)


  def callback(self,data):
    print "CALLBACK"
    x=0
    y=0
    radius=0
    try:
        start_time = time.time()

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        for (lower, upper) in boundaries:
            #initialize the list of tracked points, 
            #the frame counter and the coordinate deltas
            pts = deque()
            #create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            mask = cv2.inRange(cv_image, lower, upper)
            output = cv2.bitwise_and(cv_image, cv_image, mask = mask)
            imgray = cv2.cvtColor(output,cv2.COLOR_BGR2GRAY)
            #find contours in the mask and initialize the current (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None
            #only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)

                if radius>0  :
                    M = cv2.moments(c)
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                    #only proceed if radius meets a minimum size
                    if radius > 0.1:
                        #draw the circle and centroid on the frame, then update the list of tracked points
                        cv2.circle(cv_image, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                        cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
                        print "CIRCLE: ",x,y,radius
                        pts.appendleft(center)
                        print radius
                else :
                    print "Object not found!"

    except CvBridgeError as e:
      print e 
    
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print e 
    if(radius>0) :
        #cv2.imshow("Camera", cv_image)
        offs=0.04
        px,py,pz=self.get3DpointCamera(x,y,radius,cv_image)
        key = cv2.waitKey(1) & 0xFF

        #TEST1
        #print "BALL POSITION: 0.2027; 0.1435; 0.5012" 
        #f=pi+2.5
        #t=0.135
        #p=-0.85

        #TEST2
        #print "BALL POSITION: -0.6001; 0.4103; 0.3021" 
        #f=(3*pi/2)
        #t=0.5
        #p=0
        #px=px+offs
        #pz=pz+offs

        #TEST3
        print "BALL POSITION: 0.4157; -0.0989; 0.6257" 
        f=(pi/2)+0.6
        t=0.5
        p=pi
        px=px#-offs-0.08
        pz=pz+0.22

        #TEST4
        #print "BALL POSITION: -0.3022; -0.3247; 0.6305" 
        #f=pi
        #t=pi/4
        #p=0.4
        #pz=pz+0.12

        print colored(("px,py,pz: " ,px,py,pz),'yellow')
        px=px+offs
        py=py+offs
        pz=pz+offs  

        print("--- %s seconds IMAGE PROC---" % (time.time() - start_time))

        start_time = time.time()
        InverseKinematicUR5(px,py,pz,f,t,p)
        print("------ %s seconds IK------" % (time.time() - start_time))
        
        #handGrasp()    
    else :
        print "IK not possible"

####################################
##########__HAND GRASP__############
####################################
def handGrasp():
    tpcNames= [
	#thumb
	'/hand/hollie_real_left_hand_j3/cmd_pos', #neg: nail thumb move
	'/hand/hollie_real_left_hand_j4/cmd_pos', #neg: distal thumb move
	'/hand/hollie_real_left_hand_Thumb_Flexion/cmd_pos', #neg: proxal thumb move
   
	'/hand/hollie_real_left_hand_Thumb_Opposition/cmd_pos',#pos: lateral thumb move
	#proximals
        '/hand/hollie_real_left_hand_Index_Finger_Proximal/cmd_pos',
	'/hand/hollie_real_left_hand_Middle_Finger_Proximal/cmd_pos',
	'/hand/hollie_real_left_hand_Ring_Finger/cmd_pos',
	'/hand/hollie_real_left_hand_Pinky/cmd_pos',
	#distals
	'/hand/hollie_real_left_hand_Index_Finger_Distal/cmd_pos',
	'/hand/hollie_real_left_hand_Middle_Finger_Distal/cmd_pos',
	'/hand/hollie_real_left_hand_j12/cmd_pos', 
	'/hand/hollie_real_left_hand_j13/cmd_pos',
	#nails
	'/hand/hollie_real_left_hand_j14/cmd_pos', 
	'/hand/hollie_real_left_hand_j15/cmd_pos', 
	'/hand/hollie_real_left_hand_j16/cmd_pos', 
	'/hand/hollie_real_left_hand_j17/cmd_pos'
   ]
    
    moves=[
	#thumb
	-1,-1,-1,0.6,
	#prox
	1.2,1.2,1.2,1.3,
	#dist
	0.8,0.8,0.8,0.8,
	#nails
	0.5,0.5,0.5,0.5
	]

    for n,m in zip(tpcNames,moves):
        pub = rospy.Publisher(n, Float64, latch=True,queue_size=20)
        rospy.loginfo(m)
        pub.publish(Float64(m))
        rospy.sleep(0.2)

####################################
#########UR5_FWD_KINEMATICS#########
####################################
def ForwardKinematicUR5(th):
    #DH_params
    a1,a2,a3,a4,a5,a6 = 0, -0.42500, -0.39225, 0, 0, 0
    d1,d2,d3,d4,d5,d6 = 0.089159, 0, 0, 0.10915, 0.09465, 0.0823+0.012

    th= np.array(th) 

    nx=cos(th[1])*cos(th[2]+th[3]+th[4])*cos(th[5])*cos(th[6]) + sin(th[1])* sin(th[5])*cos(th[6]) - cos(th[1])*sin(th[2]+th[3]+th[4])*sin(th[6])

    ny=sin(th[1])*cos(th[2]+th[3]+th[4])*cos(th[5])*cos(th[6]) - cos(th[1])* sin(th[5])*cos(th[6]) - sin(th[1])*sin(th[2]+th[3]+th[4])*sin(th[6])

    nz=sin(th[2]+th[3]+th[4])*cos(th[5])*cos(th[6]) + cos(th[2]+th[3]*th[4])* sin(th[6])

    ox=-cos(th[1])*cos(th[2]+th[3]+th[4])*cos(th[5])*sin(th[6]) - sin(th[1])* sin(th[5])*sin(th[6]) - cos(th[1])*sin(th[2]+th[3]+th[4])*cos(th[6])

    oy=-sin(th[1])*cos(th[2]+th[3]+th[4])*cos(th[5])*sin(th[6]) + cos(th[1])* sin(th[5])*sin(th[6]) - sin(th[1])*sin(th[2]+th[3]+th[4])*cos(th[6])

    oz=-sin(th[2]+th[3]+th[4])*cos(th[5])*sin(th[6]) + cos(th[2]+th[3]+th[4])*cos(th[6])

    ax=-cos(th[1])*cos(th[2]+th[3]+th[4])* sin(th[5]) + sin(th[1])*cos(th[5])
    ay=-sin(th[1])*cos(th[2]+th[3]+th[4])*sin(th[5]) - cos(th[1])*cos(th[5])
    az=-sin(th[2]+th[3]+th[4])*sin(th[5])

    px=cos(th[1])*cos(th[2])*a2 +cos(th[1])*cos(th[2]+th[3])*a3 + sin(th[1])*d4 + cos(th[1])*sin(th[2]+th[3]+th[4])*d5 + (-cos(th[1])*cos(th[2]+th[3]+th[4])* sin(th[5]) + sin(th[1])*cos(th[5]))*d6

    py=sin(th[1])*cos(th[2])*a2 + sin(th[1])*cos(th[2]+th[3])*a3 - cos(th[1])*d4+ sin(th[1])*sin(th[2]+th[3]+th[4])*d5 - (sin(th[1])*cos(th[2]+th[3]+th[4])*sin(th[5]) + cos(th[1])*cos(th[5]) )*d6

    pz=d1 + sin(th[2])*a2 + sin(th[2]+th[3])*a3 - cos(th[2]+th[3]+th[4])*d5 - sin(th[2]+th[3]+th[4])* sin(th[5])*d6

    T06=[[nx, ox, ax, -px], 
         [ny, oy, ay, -py],
	 [nz, oz, az, pz],
	 [0,  0,  0,  1]]

    print "------FWD_KIN_T06: \n", T06

####################################
###########__UR5_IK__###############
####################################

def InverseKinematicUR5(eeX,eeY,eeZ,f,t,p):
    start_time = time.time()


    global joints       
    solutions = np.zeros(shape=(6,8)) #SOLUTION matrix: rows=(th1,..., th6); cols=solns
    #handOffs=0.09

    #DH_params
    a1,a2,a3,a4,a5,a6 = 0, -0.42500, -0.39225, 0, 0, 0
    d1,d2,d3,d4,d5 = 0.089159, 0, 0, 0.10915, 0.09465
    d6= 0.0823 
    #eeX=eeX+0.06
    eeX=eeX-0.04
    eeZ=eeZ-0.06
    alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = pi/2, 0, 0, pi/2, -pi/2, 0

    #Transform from Robot Frame to World Frame
    Trw=[[cos(pi),   -sin(pi), 0,0],
         [sin(pi),   cos(pi),  0,0],
         [0,         0,        1,0],
         [0,         0,        0,1]]
    print "Trw : ", Trw
    P=[[0,0,0, eeX],
       [0,0,0, eeY],
       [0,0,0, abs(eeZ)],
       [0,0,0, 1]]
    Trw=np.matrix(Trw)
    P=np.matrix(P)
    P=Trw*P
    P=np.array(P[:,3])
    print "P vector : ", P

    #ZYZ angles-compute rotation matrix and pos vector
    cf=cos(f)
    ct=cos(t)
    cp=cos(p)
    sf=sin(f)
    st=sin(t)
    sp=sin(p)
    T06=[[(cf*ct*cp)-(sf*sp), -(cf*ct*sp)-(sf*cp), cf*st,  P[0]], 
         [(sf*ct*cp)+(cf*sp), -(sf*ct*sp)+(cf*cp), sf*st,  P[1]],
    	 [-(st*cp),           (st*sp),             ct,     P[2]],
    	 [0,                  0,                   0,      1]]

    print "T06: ", T06
    print "********"

    #***************THETA1***************
    T06=np.matrix(T06)    
    V=[[0,   0,0,0],
       [0,   0,0,0],
       [-d6, 0,0,0],
       [1,   0,0,0]]
    V= np.matrix(V)

    p05=T06*V    
    p05=np.array(p05[:,0])
    psi=atan2(p05[1],p05[0])
    arg1= d4/(sqrt(pow(p05[0],2)+pow(p05[1],2)))
    phi=acos(arg1)
    #shoulder left/right
    theta1_a=psi+phi+(pi/2) 
    theta1_b=psi-phi+(pi/2) 
    print "theta1_a: ",theta1_a
    print "theta1_b: ",theta1_b
    print "*********"

    solutions[0,0:4]=theta1_a;   
    solutions[0,4:8]=theta1_b;   

    #***************THETA5***************
    c1a=cos(theta1_a)
    s1a=sin(theta1_a)

    c1b=cos(theta1_b)
    s1b=sin(theta1_b)

    #Theta5 using theta1_a
    p61z= (T06[0,3]*s1a)-(T06[1,3]*c1a) #p06x*s1 - p06y*c1
    arg5_a=(p61z-d4)/d6
    if arg5_a>1:
	print "theta5_a not well defined: th5 just set to zero \n"
	theta5_a=0; 
        theta5_b=0; 
    else:
	theta5_a= acos(arg5_a) #wrist up
        theta5_b= -theta5_a  #wrist down

    #Theta5 using theta1_b
    p61z= (T06[0,3]*s1b)-(T06[1,3]*c1b) #p06x*s1 - p06y*c1
    arg5_b=(p61z-d4)/d6
    if arg5_b>1:
	print "theta5_b not well defined: th5 just set to zero \n"
	theta5_c=0; 
        theta5_d=0; 
    else:
	theta5_c= acos(arg5_b) #wrist up
        theta5_d= -theta5_c   #wrist down

    print "theta5_a: ", theta5_a
    print "theta5_b: ", theta5_b
    print "theta5_c: ", theta5_c
    print "theta5_d: ", theta5_d
    print "*********"

    solutions[4,[0,1]]=theta5_a;   
    solutions[4,[2,3]]=theta5_b;
    solutions[4,[4,5]]=theta5_c;   
    solutions[4,[6,7]]=theta5_d;

    #***************THETA6*************** 
    #theta6 using theta1_a  
    c_a1=cos(alpha1) 
    s_a1=sin(alpha1)

    T01_1=[[c1a, -s1a*c_a1, s1a*s_a1,  a1*c1a], 
         [s1a, c1a*c_a1,  -c1a*s_a1, a1*s1a],
	 [0,  s_a1,     c_a1,     d1],
	 [0,  0,        0,        1]]
    T01_1=np.matrix(T01_1)
    T01Inv_a = np.linalg.inv(T01_1) 

    T16_a =T01Inv_a*T06    
    T16_a=np.matrix(T16_a)
    T61_a=np.linalg.inv(T16_a)

    #theta6 using theta1_b
    T01_2=[[c1b, -s1b*c_a1, s1b*s_a1,  a1*c1b], 
         [s1b, c1b*c_a1,  -c1b*s_a1, a1*s1b],
	 [0,  s_a1,     c_a1,     d1],
	 [0,  0,        0,        1]]
    T01_2=np.matrix(T01_2)
    T01Inv_b = np.linalg.inv(T01_2) 

    T16_b =T01Inv_b*T06    
    T16_b=np.matrix(T16_b)
    T61_b=np.linalg.inv(T16_b)
 
    ###NOW using theta5_a1
    s5a = sin(theta5_a); 
    if s5a==0:
	print "theta6_a not well defined - set th6 = 0"
        theta6_a=0
    else:
        firstArg = -T61_a[1,2]/s5a
        secArg = T61_a[0,2]/s5a
        theta6_a=atan2(firstArg, secArg)

    ###NOW using theta5_a2
    s5b = sin(theta5_b); 
    if s5b==0:
	print "theta6 not well defined - set th6 = 0"
        theta6_b=0
    else:	
        firstArg = -T61_a[1,2]/s5b
        secArg = T61_a[0,2]/s5b
        theta6_b=atan2(firstArg, secArg)

    ###NOW using theta5_b1
    s5c = sin(theta5_c); 
    if s5c==0:
	print "theta6 not well defined - set th6 = 0"
        theta6_c=0
    else:	
        firstArg = -T61_b[1,2]/s5c
        secArg = T61_b[0,2]/s5c
        theta6_c=atan2(firstArg, secArg)

    ###NOW using theta5_b2
    s5d = sin(theta5_d); 
    if s5d==0:
	print "theta6 not well defined - set th6 = 0"
        theta6_d=0
    else:	
        firstArg = -T61_b[1,2]/s5d
        secArg = T61_b[0,2]/s5d
        theta6_d=atan2(firstArg, secArg)

    print "theta6_a: ",theta6_a
    print "theta6_b: ",theta6_b
    print "theta6_c: ",theta6_c
    print "theta6_d: ",theta6_d
    print "*********"
    solutions[5,[0,1]]=theta6_a;   
    solutions[5,[2,3]]=theta6_b;
    solutions[5,[4,5]]=theta6_c;   
    solutions[5,[6,7]]=theta6_d;

    #***************THETA3***************
    c_a5=cos(alpha5) 
    s_a5=sin(alpha5)
    c_a6=cos(alpha6) 
    s_a6=sin(alpha6)    
    #COS 5
    c5a=cos(theta5_a)
    c5b=cos(theta5_b) 
    c5c=cos(theta5_c)
    c5d=cos(theta5_d)
    #COS 6
    c6a=cos(theta6_a)
    c6b=cos(theta6_b)
    c6c=cos(theta6_c)
    c6d=cos(theta6_d)
    #SIN 6
    s6a=sin(theta6_a)
    s6b=sin(theta6_b)
    s6c=sin(theta6_c)
    s6d=sin(theta6_d)

#####theta5_A 
    T45_A=[[c5a, -s5a*c_a5, s5a*s_a5,  a5*c5a], 
         [s5a, c5a*c_a5,  -c5a*s_a5, a5*s5a],
	 [0 , s_a5,     c_a5,     d5],
	 [0 , 0 ,       0,        1 ]]
    #theta5_B
    T45_B=[[c5b, -s5b*c_a5, s5b*s_a5,  a5*c5b], 
         [s5b, c5b*c_a5,  -c5b*s_a5, a5*s5b],
	 [0 , s_a5,     c_a5,     d5],
	 [0 , 0 ,       0,        1 ]]
    #theta5_C
    T45_C=[[c5c, -s5c*c_a5, s5c*s_a5,  a5*c5c], 
         [s5c, c5c*c_a5,  -c5c*s_a5, a5*s5c],
	 [0 , s_a5,     c_a5,     d5],
	 [0 , 0 ,       0,        1 ]]
    #theta5_D
    T45_D=[[c5d, -s5d*c_a5, s5d*s_a5,  a5*c5d], 
         [s5d, c5d*c_a5,  -c5d*s_a5, a5*s5d],
	 [0 , s_a5,     c_a5,     d5],
	 [0 , 0 ,       0,        1 ]]

#####theta6_A
    T56_A=[[c6a, -s6a*c_a6, s6a*s_a6, a6*c6a], 
         [s6a, c6a*c_a6, -c6a*s_a6, a6*s6a],
	 [0 , s_a6, c_a6, d6],
	 [0 , 0 ,  0, 1 ]]
    #theta6_B
    T56_B=[[c6b, -s6b*c_a6, s6b*s_a6, a6*c6b], 
         [s6b, c6b*c_a6, -c6b*s_a6, a6*s6b],
	 [0 , s_a6, c_a6, d6],
	 [0 , 0 ,  0, 1 ]]
    #theta6 _C
    T56_C=[[c6c, -s6c*c_a6, s6c*s_a6, a6*c6c], 
         [s6c, c6c*c_a6, -c6c*s_a6, a6*s6c],
	 [0 , s_a6, c_a6, d6],
	 [0 , 0 ,  0, 1 ]]
    #theta6_D
    T56_D=[[c6d, -s6d*c_a6, s6d*s_a6, a6*c6d], 
         [s6d, c6d*c_a6, -c6d*s_a6, a6*s6d],
	 [0 , s_a6, c_a6, d6],
	 [0 , 0 ,  0, 1 ]]

    T45_A=np.matrix(T45_A)
    T45_B=np.matrix(T45_B)
    T45_C=np.matrix(T45_C)
    T45_D=np.matrix(T45_D)

    T56_A=np.matrix(T56_A)
    T56_B=np.matrix(T56_B)
    T56_C=np.matrix(T56_C)
    T56_D=np.matrix(T56_D)

    
    T46_A=T45_A*T56_A #using T45_A   
    T46_B=T45_B*T56_B #using T45_B  
    T46_C=T45_C*T56_C #using T45_C 
    T46_D=T45_D*T56_D #using T45_D

    T46_A=np.matrix(T46_A)
    T46_B=np.matrix(T46_B)
    T46_C=np.matrix(T46_C)
    T46_D=np.matrix(T46_D)
 
    T46A_inv=np.linalg.inv(T46_A)
    T46B_inv=np.linalg.inv(T46_B)
    T46C_inv=np.linalg.inv(T46_C)
    T46D_inv=np.linalg.inv(T46_D)

    #mul by T16_a    
    T14_A=T16_a*T46A_inv  
    T14_B=T16_a*T46B_inv
    #mul by T16_b
    T14_C=T16_b*T46C_inv
    T14_D=T16_b*T46D_inv

    Y= np.array([0,-d4,0,1])
    T14_A = np.array(T14_A)
    T14_B = np.array(T14_B)
    T14_C = np.array(T14_C)
    T14_D = np.array(T14_D)

    #print "T14_A : ",T14_A
    #print "T14_B : ",T14_B 
    #print "T14_C : ",T14_C 
    #print "T14_D : ",T14_D 
 
    e4=np.array([0,0,0,1])
    p13 = np.zeros(shape=( 4,4))
    p13=np.array(p13)

    #####T14_A
    temp=T14_A.dot(Y)
    p13[0,:]=temp-e4
    #####T14_B
    temp=T14_B.dot(Y)
    p13[1,:]=temp-e4
    #####T14_C
    temp=T14_C.dot(Y)
    p13[2,:]=temp-e4
    #####T14_D
    temp=T14_D.dot(Y)
    p13[3,:]=temp-e4

    #print "p13 \n :",p13 
    p13_norm2=np.zeros(4)
    arg3=np.zeros(4)

    try:
        for i in range(len(p13)):#r->4
            for j in range(len(p13)):#c->4
                p13_norm2[i]+=pow(p13[i,j],2) #square 2-norm
        tmp_theta3=np.zeros(4)
        theta3_ab=np.zeros(8)
        k=0
        for el in p13_norm2:
            arg3[k]=(el -pow(a2,2)-pow(a3,2))/(2*a2*a3)	
            if arg3[k] <= 1 and arg3[k] >= -1:#acos domain = [-1; 1] 
                tmp_theta3[k]=acos(arg3[k]); 
            else: 
                tmp_theta3[k]=-1000;#dummy value
	    k+=1
	
        theta3_ab[[0,2,4,6]]= tmp_theta3[:] #elbow up
        theta3_ab[[1,3,5,7]]= -tmp_theta3[:] #elbow down
    except KeyboardInterrupt:
        print "if math domain error, it will be set with a dummy value"        
        raise

    print "theta3 : \n",theta3_ab
    print "*********"
    solutions[2,:]=theta3_ab; 

    #***************THETA2***************
    theta2_ab=np.zeros(8)
    epsilon=np.zeros(8)
    delta=np.zeros(4)
    j=0

    for i in range(len(p13)):
	delta[i]=atan2(p13[i,1], -p13[i,0])
        epsilon[j]=asin(a3*sin(theta3_ab[j])/sqrt(p13_norm2[i]))
        theta2_ab[j] = - delta[i] + epsilon[j]-0.2
        epsilon[j+1]=asin(a3*sin(theta3_ab[j+1])/sqrt(p13_norm2[i])) 
	#print "delta: ",delta[i]
	#print "epsilon1: ",epsilon[j]
	#print "eps2: ",epsilon[j+1]
        theta2_ab[j+1] = - delta[i] + epsilon[j+1]-0.2	
        j=j+2

    print "theta2ab: \n",theta2_ab
    print "*********"        
 
    solutions[1,:]=theta2_ab; 

    #***************THETA4***************
    c_a3=cos(alpha3) 
    s_a3=sin(alpha3)
    c_a2=cos(alpha2) 
    s_a2=sin(alpha2)

    theta4=np.zeros(8)

    for i in range(len(theta2_ab)):
        c2=cos(theta2_ab[i])
        s2=sin(theta2_ab[i])
        c3=cos(theta3_ab[i])
        s3=sin(theta3_ab[i])

        T12=[[c2, -s2*c_a2, s2*s_a2,  a2*c2], 
             [s2, c2*c_a2,  -c2*s_a2, a2*s2],
	     [0,  s_a2,     c_a2,     d2],
	     [0,  0,        0,        1]]
        T23=[[c3, -s3*c_a3, s3*s_a3,  a3*c3], 
             [s3, c3*c_a3,  -c3*s_a3, a3*s3],
	     [0,  s_a3,     c_a3,     d3],
	     [0,  0,        0,        1]]
        T12=np.mat(T12)
        T23=np.mat(T23)

        T13=T12*T23
        T31 = np.linalg.inv(T13) 

        if i==0 or i==1:
            T34 = T31*T14_A
        if i==2 or i==3:
            T34 = T31*T14_B
        if i==4 or i==5:
            T34 = T31*T14_C
        if i==6 or i==7:
            T34 = T31*T14_D

        T34 = np.matrix(T34)
        theta4[i]=atan2(T34[1,0],T34[0,0])

    print "theta4: \n",theta4
    print "*********"
    solutions[3,:]=theta4;

    print "*********"
    print "SOLUTION MATRIX:\n",solutions
    print "*********"

    #***************BEST SOLUTION***************
    distance=np.zeros(8)
    for i in range(len(solutions[0])):#col->8
        for j in range(len(solutions)):#row->6
            if solutions[j,i] >=1000 or solutions[j,i] <=-1000:
                #discard the solution
                distance[i]=10000
                break;
            else:
                distance[i]+=pow((abs(joints[j])-abs(solutions[j,i])),2)
        if distance[i]<10000:
            distance[i]=np.sqrt(distance[i])
    
    minIndex = np.argmin(distance)
    if distance[minIndex] >=10000:
        print "Chosen solution is impossible, maybe ball isn't in Workspace "
        print "IMPOSSIBLE TO EXECUTE IK. Press a button to end"
        print "*********"
    else:
        print "****Chosen solution number: ",minIndex
        print "*********"
        joints=solutions[:,minIndex] #best solution        
        print "Joints:   ", joints
        print "*********"
        print "IK COMPUTING END"
        print "*********"

        #ForwardKinematicUR5([0,theta1,theta2,theta3,theta4,theta5,theta6])

        print("------ %s seconds EFFECTIVE IK------" % (time.time() - start_time))

        ForwardKinematicUR5([0,joints[0],joints[1],joints[2],joints[3],joints[4],joints[5]])

        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        g.trajectory.points = [JointTrajectoryPoint(positions=joints, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]

        client.send_goal(g)
        try:
	    client.wait_for_result()
	    print "RESULT:  " ,client.get_result()
            print "STATE:  " ,client.get_state()
            print "STATUS:  " ,client.get_goal_status_text()
        except KeyboardInterrupt:
            client.cancel_goal()
            raise

####################################
#############__INIT__###############
####################################
def zeroInit():
    global joints
    Q0=[0,0,0,0,0,0]
    #Q0=[1.67, -0.39, 0.66, -0.21, 1.1,  -1.5]
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q0, velocities=[0]*6, time_from_start=rospy.Duration(2.0))]

    client.send_goal(g)
    try:
	joints = Q0
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

joints=np.zeros(6)
####################################
#############__MAIN__###############
####################################
def main():
    global client
    global joints

    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
	zeroInit()
        print "Joints positions: ", joints


	#TEST1
	#print "BALL POSITION: 0.2027; 0.1435; 0.5012" 
	#f=pi+2.5
	#t=0.135
	#p=-0.85
	#px=0.2027
	#py=0.1435
	#pz= 0.5012
	#px=px+offs


	#TEST2
	#print "BALL POSITION: -0.6001; 0.4103; 0.3021" 
	#f=(3*pi/2)
	#t=0.5
	#p=0
	#px=-0.6001
	#py=0.4103
	#pz=0.3021
	#px=px+offs
	#pz=pz+offs


	#TEST3
	#f=-(pi/2)-0.6
	#t=-0.5
	#p=-pi
	#f=(pi/2)+0.6
	#t=0.5
	#p=pi
	#px=px#-offs-0.08
	#pz=pz+0.22
	#px=0.4157
	#py=-0.0989
	#pz=0.6257

	#start_time = time.time()
	#InverseKinematicUR5(px,py,pz,f,t,p)
	#print("------ %s seconds IK------" % (time.time() - start_time))

	#print colored(("px,py,pz: " ,px,py,pz),'yellow')
	#px=px+offs
	#py=py+offs
	#pz=pz+offs  


        

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


    ic = image_converter()

    # rospy.init_node('image_converter', anonymous=True)
    try:
       rospy.spin()
    except KeyboardInterrupt:
       print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__': main()



