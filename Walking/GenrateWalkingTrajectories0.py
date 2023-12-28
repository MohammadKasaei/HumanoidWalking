#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 22 21:56:03 2020

@author: mohammad
"""

import matplotlib.pyplot as plt
import numpy 
import math
import statistics

# import rospy
# import time
# import tf
# import std_msgs


# import threading

# from getkey import getkey, keys

from .FootStepPlanner import FootStepPlanner
from .ZMPGenerator import ZMPGenerator
from .COMGenerator import COMGenerator
from .FeetGenerator import FeetGenerator


# from std_msgs.msg import String
# from geometry_msgs.msg import PoseArray
# from geometry_msgs.msg import Pose
# from rosgraph_msgs.msg import Clock
# from gazebo_msgs.msg import ModelStates
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import WrenchStamped 
# from nav_msgs.msg import Odometry
# from gazebo_msgs.srv import ApplyBodyWrench 
# from geometry_msgs.msg import Wrench
# from geometry_msgs.msg import Point
# from geometry_msgs.msg import Vector3

# from std_srvs.srv import Empty # reset model pose()





class GenrateWalkingTrajectories:
    g = 9.81
    NumberOfStep = 5
    FootLength = 0.0
    FootWidth = 0.
    FootHeelToe = 0
    
    SamplingTime = 0.01
    DSRatio = 0.
    DistanceBetweenFeet = 0.17
    CoMHeightAmp = 0.0
    StepTime = 1
    
    StepX = 0
    StepY = 0
    SwingStepZ = 0
    FirstStepIsRight = 0

    Z0 = 0.35
    
    FR0X = 0
    FR0Y = -DistanceBetweenFeet/2
    FL0X = 0
    FL0Y = DistanceBetweenFeet/2
    COM_X0 = (FR0X+FL0X)/2
    COM_Y0 = (FR0Y+FL0Y)/2
    RightLegX = []
    RightLegY = []
    RightLegZ = []
    
    LeftLegX = []
    LeftLegY = []
    LeftLegZ = []
    SmoothHeightIndex = 0
    ZLeg  = 0.375
    
    
    
    def __init__(self,FR0X,FR0Y,FL0X,FL0Y,StepX,StepY,SwingStepZ,StepTime,SamplingTime,FirstStepIsRight,ZLeg,Zoffset):
        
        self.SamplingTime = SamplingTime
        self.DSRatio = 0.
        self.StepTime = StepTime
        self.DSTime = (self.DSRatio * StepTime)
        self.SSTime =  StepTime - self.DSTime
        self.FR0X = FR0X
        self.FR0Y = FR0Y
        self.FL0X = FL0X
        self.FL0Y = FL0Y
        self.COM_X0 = (FR0X+FL0X)/2
        self.COM_Y0 = (FR0Y+FL0Y)/2   
        self.StepX = StepX
        self.StepY = StepY
        self.SwingStepZ = SwingStepZ
        self.FirstStepIsRight = FirstStepIsRight

        self.ZLeg = ZLeg + Zoffset
        #self.Z0 -= Zoffset
        
    
        self.RightLegX = []
        self.RightLegY = []
        self.RightLegZ = []
        
        self.LeftLegX = []
        self.LeftLegY = []
        self.LeftLegZ = []        
        
    def Generate(self):   
        FP = FootStepPlanner(self.FR0X,self.FR0Y,self.FL0X,self.FL0Y,self.DistanceBetweenFeet)
        FP.PlanSteps(self.NumberOfStep,self.StepX,self.StepY,self.FirstStepIsRight)
        
        #%% ZMP Generator     
        ZMP = ZMPGenerator()
        ZMP.Generate(FP.SupportPositionsX,FP.SupportPositionsY,self.FootHeelToe,self.StepTime,self.DSRatio,self.SamplingTime)
        
        #%% COM Generator
        self.COM = COMGenerator()
        self.COM.Generate(self.COM_X0,self.COM_Y0,FP.SupportPositionsX,FP.SupportPositionsY,ZMP.zmp_x,ZMP.zmp_y,self.StepTime,self.SamplingTime,self.NumberOfStep,self.DSRatio,self.CoMHeightAmp,self.Z0)
        #%% FeetGenerator
        self.FT = FeetGenerator()
        self.FT.Generate(self.StepX,self.StepY,self.SwingStepZ,self.StepTime,self.SamplingTime,self.NumberOfStep,ZMP.zmp_x,ZMP.zmp_y,self.FR0X,self.FR0Y,self.FL0X,self.FL0Y,self.FirstStepIsRight,self.SmoothHeightIndex)
        

        for i in range(0,len(self.COM.COM_X)-1): 
            self.RightLegX.append(self.FT.RightX[i] - self.COM.COM_X[i])
            self.RightLegY.append(self.FT.RightY[i] - self.COM.COM_Y[i])
            self.RightLegZ.append(self.FT.RightZ[i] - self.ZLeg )
            
            self.LeftLegX.append(self.FT.LeftX[i] - self.COM.COM_X[i])
            self.LeftLegY.append(self.FT.LeftY[i] - self.COM.COM_Y[i])
            self.LeftLegZ.append(self.FT.LeftZ[i] - self.ZLeg )
            


# SamplingTime=0.01
# StepTime=1

# FootHeelToe=0
# DistanceBetweenFeet=0.2

# NumberOfStep=6 
# StepX=0.
# StepY=0.
# SwingStepZ = 0.35 #0.04

# FirstStepIsRight=1
# DSRatio=0.
# CoMHeightAmp = 0
# Z0 = 1
# NewZMPGenerator=0
# SmoothHeightIndex=0

# ##
# FR0X=-StepX/2
# FR0Y=DistanceBetweenFeet/2
# FL0X=0
# FL0Y=-DistanceBetweenFeet/2

# COM_X0 = (FR0X+FL0X)/2
# COM_Y0 = (FR0Y+FL0Y)/2
# ZLeg = 0.385


# Walking = GenrateWalkingTrajectories(FR0X,FR0Y,FL0X,FL0Y,StepX,StepY,SwingStepZ,StepTime,SamplingTime,FirstStepIsRight,ZLeg)
# Walking.Generate()

# fig = plt.figure()
# plt.plot(Walking.RightLegX)   
# plt.hold(True)
# plt.grid(True)
# plt.plot(Walking.LeftLegX)   
# plt.show(False)

# fig = plt.figure()
# plt.plot(Walking.RightLegY)   
# plt.hold(True)
# plt.grid(True)
# plt.plot(Walking.LeftLegY)   
# plt.show(False)
    

# fig = plt.figure()
# plt.plot(Walking.RightLegZ)   
# plt.hold(True)
# plt.grid(True)
# plt.plot(Walking.LeftLegZ)   
# plt.show(True)



##########################################################################################3
# State = ModelStates()
# IMU = Imu()
# RAS = WrenchStamped()
# LAS = WrenchStamped()
# ROD = Odometry()
# LOD = Odometry()                self.JointPos = self.UpdateJointPositions(self.RfootPosition,self.LfootPosition)

# gyro = [0,0,0]


# NewCommand = 0
# NewStepX = 0
# NewStepY = 0
# NewStepTheta = 0
# Stop = 1
# WriteData =0
# file = []



# def read_kbd_input(name):
#     #print('Ready for keyboard input:')
#     global NewCommand
#     global NewStepX
#     global NewStepY
#     global NewStepTheta
#     global exit
#     global Stop
#     global WriteData
#     global file
#     exit = 0
#     while (exit == 0):
#         #input_str = input()
#         #inputQueue.put(input_str)
#         key = getkey()
#         if (key == 'z' or key == 'Z'):
#             exit =1
#             print "EXIT"
#         elif (key == keys.UP):
#             if (NewStepX<0.2):               
#                 NewStepX = NewStepX + 0.01
#                 print "Speed ", str(NewStepX)
#                 NewCommand = 1
#         elif (key == keys.DOWN):
#             if (NewStepX>-0.2):               
#                 NewStepX = NewStepX - 0.01
#                 print "Speed ", str(NewStepX)
#                 NewCommand = 1
#         elif (key == keys.RIGHT):
#             if (NewStepTheta<15):               
#                 NewStepTheta = NewStepTheta + 1
#                 print "Theta ", str(NewStepTheta)
#                 NewCommand = 1
#         elif (key == keys.LEFT):
#             if (NewStepTheta>-15):               
#                 NewStepTheta = NewStepTheta - 1
#                 print "Theta ", str(NewStepTheta)
#                 NewCommand = 1
#         elif (key == 'a' or key == 'A' ):
#             if (NewStepY>-0.025):               
#                 NewStepY = NewStepY - 0.005
#                 print "StepY ", str(NewStepY)
#                 NewCommand = 1
#         elif (key == 'd' or key == 'D' ):
#             if (NewStepY<0.025):               
#                 NewStepY = NewStepY + 0.005
#                 print "StepY ", str(NewStepY)
#                 NewCommand = 1
#         elif (key == ' '):
#             if(Stop == 1):
#                 NewStepX = 0
#                 NewStepY = 0
#                 NewStepTheta = 0
#                 NewCommand = 1
#                 Stop = 0
#             else:
#                 Stop = 1
#         elif (key == 's' or key == 'S'):
#             if (WriteData == 1):
#                 WriteData = 0
#                 file.close() 
#                 print "file is closed"
#             else:
#                 file = open('log.csv','w') 
#                 WriteData = 1
#                 print "file is opened"
#         elif (key == 'f' or key == 'F'):
#                 body_name = 'coman_1_1::base_link'
#                 reference_frame = 'coman_1_1::base_link'
#                 reference_point = Point(x = 0, y = 0, z = 0)
#                 wrench = Wrench(force = Vector3( x = 0, y = 200, z = 0), torque = Vector3( x = 0, y = 0, z = 0))
#                 start_time = rospy.Time(secs = 0, nsecs = 0)
#                 duration = rospy.Duration(secs = 0, nsecs = 10000000)
#                 applyForce(body_name, reference_frame, reference_point, wrench, start_time, duration)
#         elif (key == 'g' or key == 'g'):
#                 body_name = 'coman_1_1::base_link'
#                 reference_frame = 'coman_1_1::base_link'
#                 reference_point = Point(x = 0, y = 0, z = 0)
#                 wrench = Wrench(force = Vector3( x = 500, y = 0, z = 0), torque = Vector3( x = 0, y = 0, z = 0))
#                 start_time = rospy.Time(secs = 0, nsecs = 0)
#                 duration = rospy.Duration(secs = 0, nsecs = 10000000)
#                 applyForce(body_name, reference_frame, reference_point, wrench, start_time, duration)
                
#         elif (key == 'r' or key == 'R'):
#                print "Reset Model Pose \n"
#                Stop = 1
#                Reset_Model_Pose()
            


# def read_State(data):
#     global State
#     global IMU  
#     global gyro
#     idx = 2 # robot index
#     State = data.pose[idx].position
#     #print data


#     q = [data.pose[idx].orientation.x, data.pose[idx].orientation.y, data.pose[idx].orientation.z, data.pose[idx].orientation.w]
#     IMU = tf.transformations.euler_from_quaternion(q)
#     gyro = data.twist[idx].angular

#     #print IMU
#     #print gyro
#     #print data.twist[1].angular

#     #print round(numpy.rad2deg(IMU2[2])*1000)/1000.0

#     #print State.pose[1].position, "\n\n"
    
#     #print  State.x

# # def read_IMU(data):
# #     global IMU
# #     global gyro
# #     #print data.orientation
# #     gyro = [data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z]
    
# #     # zprint gyro
# #     q = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
# #     IMU = tf.transformations.euler_from_quaternion(q)
# #     IMUdeg = numpy.rad2deg(IMU)
# #     #print round(numpy.rad2deg(IMU [0])*1000)/1000.0, "\t", round(numpy.rad2deg(IMU[1])*1000)/1000.0 , "\t", round(numpy.rad2deg(IMU[2])*1000)/1000.0 
# #     #print  round(IMUdeg[0],2),"\t",round(IMUdeg[1],2),"\t",round(IMUdeg[2],2)

# def read_LAS(data):
#     global LAS
#     LAS = data
#     #print LAS.wrench.force.x

    
   
# def read_RAS(data):
#     global RAS
#     RAS = data

# def read_LOD(data):
#     global LOD
#     #LOD = data
#     LOD = data.pose.pose
#     #print LOD.position.x

    
# def read_ROD(data):
#     global ROD
#     ROD = data.pose.pose

# def sat(inp, UB,LB):
#     if inp>UB:
#         return UB
#     elif inp<LB:
#         return LB
#     else:
#         return inp

# def applyForce(body_name, reference_frame, reference_point, wrench, start_time, duration): 
#     #print "waiting for service\n"
#     rospy.wait_for_service('/island_1/gzserver/apply_body_wrench')
#     try:
#         apply_body_wrench = rospy.ServiceProxy('/island_1/gzserver/apply_body_wrench', ApplyBodyWrench)
#         apply_body_wrench(body_name, reference_frame, reference_point, wrench, start_time, duration)
#         print "Force applied", wrench
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e


# def PDControl(err, derr,PGain, DGain,Saturation):
#     return sat((PGain*(err)) + (DGain*derr),Saturation[0],Saturation[1])


# def Reset_Model_Pose():
#     rospy.wait_for_service('/island_1/gzserver/reset_world')
#     reset_world = rospy.ServiceProxy('/island_1/gzserver/reset_world', Empty)
#     reset_world()


# SamplingTime=0.01
# StepTime=1

# FootHeelToe=0
# DistanceBetweenFeet=0.2

# NumberOfStep=6 
# StepX=0.
# StepY=0.
# SwingStepZ = 0.35 #0.04

# FirstStepIsRight=1
# DSRatio=0.
# CoMHeightAmp = 0
# Z0 = 1
# NewZMPGenerator=0
# SmoothHeightIndex=0

# ##
# FR0X=-StepX/2
# FR0Y=DistanceBetweenFeet/2
# FL0X=0
# FL0Y=-DistanceBetweenFeet/2

# COM_X0 = (FR0X+FL0X)/2
# COM_Y0 = (FR0Y+FL0Y)/2
# ZLeg = 0.385


# Walking = GenrateWalkingTrajectories(FR0X,FR0Y,FL0X,FL0Y,StepX,StepY,SwingStepZ,StepTime,SamplingTime,FirstStepIsRight,ZLeg)
# Walking.Generate()

# # fig = plt.figure()
# # plt.plot(Walking.RightLegX)   
# # plt.hold(True)
# # plt.grid(True)
# # plt.plot(Walking.LeftLegX)   
# # plt.show(False)

# # fig = plt.figure()
# # plt.plot(Walking.RightLegY)   
# # plt.hold(True)
# # plt.grid(True)
# # plt.plot(Walking.LeftLegY)   
# # plt.show(False)
    

# # fig = plt.figure()
# # plt.plot(Walking.RightLegZ)   
# # plt.hold(True)
# # plt.grid(True)
# # plt.plot(Walking.LeftLegZ)   
# # plt.show(False)
    

# rospy.init_node('PythonWalking')
# time.sleep(1)

# pub = rospy.Publisher('/island_1/robot_1/matlab', PoseArray,queue_size=10)
# rate = rospy.Rate(100) # 100hz

# rospy.Subscriber('/island_1/gzserver/model_states',ModelStates,read_State,queue_size=1)
# #rospy.Subscriber('/island_1/robot_1/imu',Imu,read_IMU,queue_size=1)
# rospy.Subscriber('/island_1/robot_1/ft_sensor/LAS',WrenchStamped,read_LAS,queue_size=1)
# rospy.Subscriber('/island_1/robot_1/ft_sensor/RAS',WrenchStamped,read_RAS,queue_size=1)
# rospy.Subscriber('/island_1/robot_1/odom/foot_l',Odometry,read_LOD,queue_size=1)
# rospy.Subscriber('/island_1/robot_1/odom/foot_r',Odometry,read_ROD,queue_size=1)





# time.sleep(1)


# poseLeft = Pose()
# poseRight = Pose()
# RotLeft = Pose()
# RotRight = Pose()

# UpperBody = Pose()
# LArm = Pose()
# RArm = Pose()
# LElbow = Pose ()
# RElbow = Pose ()

# LHipOffset = Pose()
# RHipOffset = Pose()

# LAnkleOffset = Pose()
# RAnkleOffset = Pose()

# IK = PoseArray()

# WalkingInit =0

# t0walk =0
# twalk =0

# DistanceBetweenFeet = 0.17
# ZLeg = 0.385


# LHipOffset.position = Point(0,0,0)
# RHipOffset.position = Point(0,0,0)

# poseLeft.position  = Point(0,DistanceBetweenFeet/2,-ZLeg)
# RotLeft.position = Point(0,0,0)

# poseRight.position = Point(0,-DistanceBetweenFeet/2,-ZLeg)
# RotRight.position  = Point(0,0,0)

# LAnkleOffset.position = Point(0,0,0)
# RAnkleOffset.position = Point(0,0,0)

# ARM_X_OFFSET  = 30
# ARM_Y_OFFSET  = 10
# ELBOW_OFFSET  = 70
# BODY_X_OFFSET = 5 
# BODY_Y_OFFSET = 0

# UpperBody.position  = Point(BODY_X_OFFSET,BODY_Y_OFFSET,0)
# LArm.position = Point(ARM_X_OFFSET,-ARM_Y_OFFSET,0)

# RArm.position = Point(ARM_X_OFFSET,ARM_Y_OFFSET,0)

# LElbow.position.x = -ELBOW_OFFSET
# RElbow.position.x = -ELBOW_OFFSET


# IK.poses=[poseLeft,poseRight,RotLeft,RotRight,UpperBody,LArm,RArm,LElbow,RElbow,LHipOffset,RHipOffset,LAnkleOffset,RAnkleOffset]
# pub.publish(IK)


# gtime =0
# gtime0 = rospy.get_rostime().to_sec()
# t0 = gtime0
# exit = 0
# IMU = [0,0,0]

# inputThread = threading.Thread(target=read_kbd_input, args=(1,))
# inputThread.start()


# StepX=0
# StepY=0
# SwingStepZ = 0.035
# StepTime=0.33 
# StepTheta = -0
# StopForOneStep =  1
# ZLeg = 0.385

# FirstStepIsRight = 0

# if (StepY<0 or StepTheta>0):
#     FirstStepIsRight = 1

# if (FirstStepIsRight):
#     FR0X = -StepX/2
#     FR0Y = -DistanceBetweenFeet/2
#     FL0X = 0
#     FL0Y = DistanceBetweenFeet/2
# else:
#     FL0X = -StepX/2    
#     FL0Y = DistanceBetweenFeet/2
#     FR0X = 0
#     FR0Y = -DistanceBetweenFeet/2    

            
# Walking = GenrateWalkingTrajectories(FR0X,FR0Y,FL0X,FL0Y,StepX,StepY,SwingStepZ,StepTime,SamplingTime,FirstStepIsRight,ZLeg)
# Walking.Generate()


# while ( exit ==0):   
#         gtime =  rospy.get_rostime().to_sec()-t0
#         print gtime,"\t",numpy.rad2deg(IMU[0]),"\n"
#         if (WalkingInit==0):
#             WalkingInit =1
#             twalk0 = gtime
        
#         if (WriteData==1):
#             file.write(str(gtime)+","+str(State.x)+","+str(State.y)+","+str(State.z)+",")#1-4
#             file.write(str(IMU[0])+","+str(IMU[1])+","+str(IMU[2])+",") #5-7
#             file.write(str(LAS.wrench.force.x)+","+str(LAS.wrench.force.y)+","+str(LAS.wrench.force.z)+",") #8-10
#             file.write(str(LAS.wrench.torque.x)+","+str(LAS.wrench.torque.y)+","+str(LAS.wrench.torque.z)+",") #11-13
#             file.write(str(RAS.wrench.force.x)+","+str(RAS.wrench.force.y)+","+str(RAS.wrench.force.z)+",")#14-16
#             file.write(str(RAS.wrench.torque.x)+","+str(RAS.wrench.torque.y)+","+str(RAS.wrench.torque.z)+",")#17-19
#             file.write(str(LOD.position.x)+","+str(LOD.position.y)+","+str(LOD.position.z)+",") #20-22
#             file.write(str(ROD.position.x)+","+str(ROD.position.y)+","+str(ROD.position.z)) # 23-25
            
#             file.write("\n")

#         # if (WalkingInit==0):
#         #     WalkingInit =1
#         #     twalk0 = gtime

#         #     StepX=0
#         #     StepY=0
#         #     SwingStepZ = 0.03
#         #     StepTime=0.35
#         #     StepTheta = -0
#         #     StopForOneStep =0
#         #     ZLeg = 0.385

#         #     FirstStepIsRight = 0

#         #     if (StepY<0 or StepTheta>0):
#         #         FirstStepIsRight = 1
            
#         #     if (FirstStepIsRight):
#         #         FR0X = -StepX/2
#         #         FR0Y = -DistanceBetweenFeet/2
#         #         FL0X = 0
#         #         FL0Y = DistanceBetweenFeet/2
#         #     else:
#         #         FL0X = -StepX/2    
#         #         FL0Y = DistanceBetweenFeet/2
#         #         FR0X = 0
#         #         FR0Y = -DistanceBetweenFeet/2    
            
                        
#         #     Walking = GenrateWalkingTrajectories(FR0X,FR0Y,FL0X,FL0Y,StepX,StepY,SwingStepZ,StepTime,SamplingTime,FirstStepIsRight,ZLeg)
#         #     Walking.Generate()

#         # else:
#         if (gtime-twalk0 >= (2*StepTime)-SamplingTime):
#                 dt = gtime-twalk0
#                 twalk0 = gtime
        
#                 if (NewCommand): 
#                     StepX = -NewStepX
#                     StepY = NewStepY
#                     StepTheta = NewStepTheta
#                     NewCommand = 0

#                 if (Stop == 1):
#                     StopForOneStep = 1
#                 else:
#                     StopForOneStep = 0

#                 if (StepY<0 or StepTheta>0):
#                     if (FirstStepIsRight==0): # robot should stop
#                         StopForOneStep =1
#                     FirstStepIsRight = 1
#                 else:
#                     if (FirstStepIsRight==1): # robot should stop
#                         StopForOneStep =1
#                     FirstStepIsRight = 0
                
#                 if (FirstStepIsRight):
#                     FR0X = -StepX/2
#                     FR0Y = -DistanceBetweenFeet/2
#                     FL0X = 0
#                     FL0Y = DistanceBetweenFeet/2
#                 else:
#                     FL0X = -StepX/2    
#                     FL0Y = DistanceBetweenFeet/2
#                     FR0X = 0
#                     FR0Y = -DistanceBetweenFeet/2    
                            
#                 Walking = GenrateWalkingTrajectories(FR0X,FR0Y,FL0X,FL0Y,StepX,StepY,SwingStepZ,StepTime,SamplingTime,FirstStepIsRight,ZLeg)
#                 Walking.Generate()   
        
#         else:

#                 twalk = gtime - twalk0
#                 idx = int(round(twalk/SamplingTime))
#                 if (Stop == 1):
#                     StopForOneStep = 1
#                 else:
#                     StopForOneStep = 0

#                 # if (StepY<0 or StepTheta>0):
#                 #     if (FirstStepIsRight==0): # robot should stop
#                 #         StopForOneStep =1
#                 #     FirstStepIsRight = 1
#                 # else:
#                 #     if (FirstStepIsRight==1): # robot should stop
#                 #         StopForOneStep =1
#                 #     FirstStepIsRight = 0
                
#                 # if (FirstStepIsRight):
#                 #     FR0X = -StepX/2
#                 #     FR0Y = -DistanceBetweenFeet/2
#                 #     FL0X = 0
#                 #     FL0Y = DistanceBetweenFeet/2
#                 # else:
#                 #     FL0X = -StepX/2    
#                 #     FL0Y = DistanceBetweenFeet/2
#                 #     FR0X = 0
#                 #     FR0Y = -DistanceBetweenFeet/2    
                            
#                 # Walking = GenrateWalkingTrajectories(FR0X,FR0Y,FL0X,FL0Y,StepX,StepY,SwingStepZ,StepTime,SamplingTime,FirstStepIsRight,ZLeg)
#                 # Walking.Generate()   

#                 PDUpperBodyX = PDControl(BODY_X_OFFSET - numpy.rad2deg(IMU[1]),numpy.rad2deg(gyro.x),0.5 ,0,[10,-10])
#                 PDArmX   = PDControl(BODY_X_OFFSET - numpy.rad2deg(IMU[1]),numpy.rad2deg(gyro.x),0.8,-0.0,[10,-10])
#                 PDHipX   = PDControl(BODY_X_OFFSET - numpy.rad2deg(IMU[1]),numpy.rad2deg(gyro.x),0.25,-0.,[10,-10])
#                 PDAnkleX = PDControl(BODY_X_OFFSET - numpy.rad2deg(IMU[1]),numpy.rad2deg(gyro.x),0.25,0.,[10,-10])
                

#                 PDUpperBodyY = PDControl(BODY_Y_OFFSET - numpy.rad2deg(IMU[0]),numpy.rad2deg(gyro.y),0.5 ,0.0,[10,-10])
#                 PDArmY   = PDControl(BODY_Y_OFFSET - numpy.rad2deg(IMU[0]),numpy.rad2deg(gyro.y),0.2,-0.02,[10,-10])
#                 PDHipY   = PDControl(BODY_Y_OFFSET - numpy.rad2deg(IMU[0]),numpy.rad2deg(gyro.y),0.15,-0.,[10,-10])
#                 PDAnkleY = PDControl(BODY_Y_OFFSET - numpy.rad2deg(IMU[0]),numpy.rad2deg(gyro.y),0.15,-0.0,[8,-8])
#                 ####################

#                # print  BODY_X_OFFSET - numpy.rad2deg(IMU[1])


#                 EN_UpperX = 1
#                 EN_ArmX = 1
#                 EN_HipX =  -1
#                 EN_AnkleX = -0

#                 EN_UpperY = 1   
#                 EN_ArmY = 1
#                 EN_HipY =  1
#                 EN_AnkleY = 0

#                 if math.fabs(numpy.rad2deg(IMU[0]))>20 or math.fabs(numpy.rad2deg(IMU[1]))>20:
#                     EN_UpperX = 0
#                     EN_ArmX   = 0
#                     EN_HipX   = 0
#                     EN_AnkleX = 0

#                     EN_UpperY = 0
#                     EN_ArmY   = 0
#                     EN_HipY   = 0
#                     EN_AnkleY = 0
#                     StopForOneStep = 1


#                 UpperBody.position.x = EN_UpperX * PDUpperBodyX
#                 UpperBody.position.y = EN_UpperY * PDUpperBodyY

#                 #print str(StopForOneStep)
#                 if (StopForOneStep == 1):
#                     poseLeft.position.x  = 0
#                     poseLeft.position.y  = DistanceBetweenFeet/2
#                     poseLeft.position.z  = -ZLeg

#                     RotLeft.position.x  = 0
#                     RotLeft.position.y  = 0
#                     RotLeft.position.z  = 0

#                     poseRight.position.x  = 0
#                     poseRight.position.y  = -DistanceBetweenFeet/2
#                     poseRight.position.z  = -ZLeg

#                     RotRight.position.x  = 0
#                     RotRight.position.y  = 0
#                     RotRight.position.z  = 0

#                     LArm.position.x  = ARM_X_OFFSET - EN_ArmX * PDArmX
#                     LArm.position.y  = -ARM_Y_OFFSET - EN_ArmY * PDArmY
                    
#                     RArm.position.x  = ARM_X_OFFSET - EN_ArmX * PDArmX
#                     RArm.position.y  = ARM_Y_OFFSET - EN_ArmY * PDArmY

#                     LHipOffset.position.x =  EN_HipX * PDHipX
#                     LHipOffset.position.y =  EN_HipY * PDHipY
                    
#                     RHipOffset.position.x =  EN_HipX * PDHipX
#                     RHipOffset.position.y =  EN_HipY * PDHipY

#                     LAnkleOffset.position.x = EN_AnkleX * PDAnkleX
#                     LAnkleOffset.position.y = -EN_AnkleY * PDAnkleY
                    
#                     RAnkleOffset.position.x = EN_AnkleX * PDAnkleX
#                     RAnkleOffset.position.y = EN_AnkleY * PDAnkleY
                    
                    

#                     IK.poses=[poseLeft,poseRight,RotLeft,RotRight,UpperBody,LArm,RArm,LElbow,RElbow,LHipOffset,RHipOffset,LAnkleOffset,RAnkleOffset]
#                     pub.publish(IK)
#                 elif (idx>=0 and idx<len(Walking.LeftLegX)):

#                     amp_arm = 5#+(math.fabs(StepX)*100)/1.5 
#                     if (not FirstStepIsRight):
#                         LArm.position.x = ARM_X_OFFSET + amp_arm*math.sin(twalk*2*math.pi/(2*StepTime)) - EN_ArmX * 2 * PDArmX
#                         RArm.position.x = ARM_X_OFFSET - amp_arm*math.sin(twalk*2*math.pi/(2*StepTime)) - EN_ArmX * 2 * PDArmX
#                     else:
#                         LArm.position.x = ARM_X_OFFSET - amp_arm*math.sin(twalk*2*math.pi/(2*StepTime)) - EN_ArmX * 2 * PDArmX
#                         RArm.position.x = ARM_X_OFFSET + amp_arm*math.sin(twalk*2*math.pi/(2*StepTime)) - EN_ArmX * 2 * PDArmX
                
#                     LArm.position.y  = -ARM_Y_OFFSET + EN_ArmY * PDArmY
#                     RArm.position.y  = ARM_Y_OFFSET - EN_ArmY * PDArmY
                    
#                     poseLeft.position.x =  Walking.LeftLegX[idx]
#                     poseLeft.position.y =  Walking.LeftLegY[idx]
#                     poseLeft.position.z =  Walking.LeftLegZ[idx]
#                     poseRight.position.x = Walking.RightLegX[idx]
#                     poseRight.position.y = Walking.RightLegY[idx]
#                     poseRight.position.z = Walking.RightLegZ[idx]
                    
                    
#                     RotLeft.position.z  =  (twalk/(2*StepTime))*StepTheta
#                     RotRight.position.z = (twalk/(2*StepTime))*StepTheta
                    
#                     #UpperBody.position.z  = -StepTheta
                   
#                     LHipOffset.position.x =  EN_HipX * PDHipX
#                     LHipOffset.position.y =  EN_HipY * PDHipY
                    
#                     RHipOffset.position.x =  EN_HipX * PDHipX
#                     RHipOffset.position.y =  EN_HipY * PDHipY

#                     LAnkleOffset.position.x = EN_AnkleX * PDAnkleX
#                     LAnkleOffset.position.y = -EN_AnkleY * PDAnkleY
                    
#                     RAnkleOffset.position.x = EN_AnkleX * PDAnkleX
#                     RAnkleOffset.position.y = EN_AnkleY * PDAnkleY
                    
                    


#                     IK.poses=[poseLeft,poseRight,RotLeft,RotRight,UpperBody,LArm,RArm,LElbow,RElbow,LHipOffset,RHipOffset,LAnkleOffset,RAnkleOffset]
#                     pub.publish(IK)
#                     #rate.sleep()
#         rate.sleep()     
            
# poseLeft.position.x  = 0
# poseLeft.position.y  = DistanceBetweenFeet/2
# poseLeft.position.z  = -ZLeg

# RotLeft.position.x  = 0
# RotLeft.position.y  = 0
# RotLeft.position.z  = 0

# poseRight.position.x  = 0
# poseRight.position.y  = -DistanceBetweenFeet/2
# poseRight.position.z  = -ZLeg

# RotRight.position.x  = 0
# RotRight.position.y  = 0
# RotRight.position.z  = 0

# UpperBody.position.x  = BODY_X_OFFSET
# UpperBody.position.y  = BODY_Y_OFFSET
# UpperBody.position.z  = 0

# LArm.position.x =ARM_X_OFFSET
# LArm.position.y =-ARM_Y_OFFSET
# LArm.position.z =0

# RArm.position.x =ARM_X_OFFSET
# RArm.position.y =ARM_Y_OFFSET
# RArm.position.z =0

# LElbow.position.x = -ELBOW_OFFSET
# RElbow.position.x = -ELBOW_OFFSET


# LHipOffset.position.x = 0
# LHipOffset.position.y = 0
# LHipOffset.position.z = 0


# RHipOffset.position.x = 0
# RHipOffset.position.y = 0
# RHipOffset.position.z = 0

# IK.poses=[poseLeft,poseRight,RotLeft,RotRight,UpperBody,LArm,RArm,LElbow,RElbow,LHipOffset,RHipOffset,LAnkleOffset,RAnkleOffset]
# pub.publish(IK)


# if (WriteData == 1):
#     file.close() 
#     print "file closed"
