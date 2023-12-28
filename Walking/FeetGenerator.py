#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 22 14:51:52 2020

@author: mohammad
"""


from .FootStepPlanner import FootStepPlanner
from .ZMPGenerator import ZMPGenerator
from .COMGenerator import COMGenerator

import matplotlib.pyplot as plt
import numpy
import math
import statistics


class FeetGenerator:
    RightX  = []
    RightY  = []
    RightZ  = []

    LeftX  = []
    LeftY  = []
    LeftZ  = []
    
    def __init__(self):
        self.RightX  = []
        self.RightY  = []
        self.RightZ  = []
    
        self.LeftX  = []
        self.LeftY  = []
        self.LeftZ  = []

        
    def Generate(self,StepX,StepY,SwingStepZ,StepTime,SamplingTime,NumberOfStep,zmp_x,zmp_y,FR0X,FR0Y,FL0X,FL0Y,FirstStepIsRight,SmoothHeightIndex):

            t = 0
            index =0
            swing=[]
            RightIsMoving = FirstStepIsRight
            LeftLeg = []
            RightLeg = []
            
            TFinalSimulation = (NumberOfStep)*StepTime    
            for GlobalTime in numpy.arange(0,TFinalSimulation,SamplingTime):
                if (GlobalTime==0 or t > StepTime-(SamplingTime) ):           
                    if (index==0):
                        RightIsMoving = FirstStepIsRight
                        if (FirstStepIsRight):
                            Xp = FR0X 
                            Yp = FR0Y
                        else:
                            Xp = FL0X 
                            Yp = FL0Y
                        
                    else:
                        if (RightIsMoving):
                            RightIsMoving = 0
                        else:
                            RightIsMoving = 1
                        
                        Xp = ZMPX
                        Yp = ZMPY
                    
                    l = int(index*round(StepTime/SamplingTime))
                    ZMPX = statistics.mean(zmp_x[ l:l+int(round(0.1*StepTime/SamplingTime))]); 
                    ZMPY = statistics.mean(zmp_y[l:l+int(round(0.1*StepTime/SamplingTime))]); 
                    
                    index=index+1
                    t = 0    
                else:
                    t = t + SamplingTime
                
                
                #ZMP = [ZMPX ,ZMPY , 0];    
                ZMPZ = 0
                t0 = 0.
                if (SmoothHeightIndex>1 and index<SmoothHeightIndex):
                    if (t<t0):
                        SWINGX = Xp+(StepX*(t/(StepTime)))
                        SWINGY = Yp+(StepY*(t/StepTime))
                        SWINGZ = 0*(index/SmoothHeightIndex)*SwingStepZ*(math.sin(math.pi*t/StepTime))        
                    else:
                        SWINGX = Xp+(StepX*(t/(StepTime)))
                        SWINGY = Yp+(StepY*(t/StepTime))
                        SWINGZ = (index/SmoothHeightIndex)*SwingStepZ*(math.sin(math.pi*(t-t0)/(StepTime-t0)))        
                else:
                    if (t<t0):            
                        SWINGX = Xp+(StepX*(t/(StepTime)))
                        SWINGY = Yp+(StepY*(t/StepTime))
                        SWINGZ = 0        
                    else:
                         SWINGX = Xp+(StepX*(t/(StepTime)))
                         SWINGY = Yp+(StepY*(t/StepTime))
                         SWINGZ = SwingStepZ*(math.sin(math.pi*(t-t0)/(StepTime-t0)));
                    
                
                
                
                #swing = [swing; SWING ];
                
                if (RightIsMoving):    
                   self.RightX.append(SWINGX)
                   self.RightY.append(SWINGY)
                   self.RightZ.append(SWINGZ)
                   
                   self.LeftX.append(ZMPX)
                   self.LeftY.append(ZMPY)
                   self.LeftZ.append(ZMPZ)
                else:
                   self.LeftX.append(SWINGX)
                   self.LeftY.append(SWINGY)
                   self.LeftZ.append(SWINGZ)
                   
                   self.RightX.append(ZMPX)
                   self.RightY.append(ZMPY)
                   self.RightZ.append(ZMPZ)
            
            
            
#
#
#SamplingTime=0.01
#StepTime=1
#
#FootHeelToe=0
#DistanceBetweenFeet=0.2
#
#NumberOfStep=10
#StepX=0.1
#StepY=0.1
#SwingStepZ =0.04
#
#FirstStepIsRight=1
#DSRatio=0.
#CoMHeightAmp = 0
#Z0 = 1
#NewZMPGenerator=0
#SmoothHeightIndex=0
#
###
#FR0X=-StepX/2
#FR0Y=DistanceBetweenFeet/2
#FL0X=0
#FL0Y=-DistanceBetweenFeet/2
#
#COM_X0 = (FR0X+FL0X)/2;
#COM_Y0 = (FR0Y+FL0Y)/2;
#
##%% FootStep planner
#FP = FootStepPlanner(FR0X,FR0Y,FL0X,FL0Y,DistanceBetweenFeet)
#FP.PlanSteps(NumberOfStep,StepX,StepY,FirstStepIsRight)
#
##%% ZMP Generator     
#ZMP = ZMPGenerator()
#ZMP.Generate(FP.SupportPositionsX,FP.SupportPositionsY,FootHeelToe,StepTime,DSRatio,SamplingTime)
#
##%% COM Generator
#COM = COMGenerator()
#COM.Generate(COM_X0,COM_Y0,FP.SupportPositionsX,FP.SupportPositionsY,ZMP.zmp_x,ZMP.zmp_y,StepTime,SamplingTime,NumberOfStep,DSRatio,CoMHeightAmp,Z0)
#
#
##%% FeetGenerator
#FT = FeetGenerator()
#FT.Generate(StepX,StepY,SwingStepZ,StepTime,SamplingTime,NumberOfStep,ZMP.zmp_x,ZMP.zmp_y,FR0X,FR0Y,FL0X,FL0Y,FirstStepIsRight,SmoothHeightIndex)
##%% plotting
#time = numpy.arange(0,NumberOfStep*StepTime,SamplingTime)
#time2 = numpy.arange(0,(NumberOfStep-1)*StepTime,SamplingTime)
#
#fig = plt.figure()
#plt.plot(FP.SupportPositionsX,FP.SupportPositionsY,'o--',label='FootSteps')
#plt.legend()
#plt.grid(True)
#
#fig = plt.figure()
#ax1 = fig.add_subplot(211)
#ax1.plot(time,ZMP.zmp_x,linestyle='dashed',color = 'darkgreen',linewidth=2,label='ZMP_X')
#ax1.hold(True)
#ax1.grid(True)
#ax1.plot(time2,COM.COM_X,color = 'red',linewidth=1,label='COM_X')
#ax1.legend()      
#
#ax2 = fig.add_subplot(212)
#ax2.plot(time,ZMP.zmp_y,linestyle='dashed',color = 'darkgreen',linewidth=2,label='ZMP_Y')
#ax2.hold(True)
#ax2.grid(True)
#ax2.plot(time2,COM.COM_Y,color = 'red',linewidth=1,label='COM_Y')
#ax2.legend()
#
#fig = plt.figure()
#plt.plot(time,FT.LeftX)
#plt.hold(True)
#plt.grid(True)
#plt.plot(time,FT.RightX)
#
#fig = plt.figure()
#plt.plot(time,FT.LeftY)
#plt.hold(True)
#plt.grid(True)
#plt.plot(time,FT.RightY)
#
#fig = plt.figure()
#plt.plot(time,FT.LeftZ)
#plt.hold(True)
#plt.grid(True)
#plt.plot(time,FT.RightZ)
