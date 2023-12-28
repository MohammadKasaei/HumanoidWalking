#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 22 14:51:52 2020

@author: mohammad
"""


from .FootStepPlanner import FootStepPlanner
from .ZMPGenerator import ZMPGenerator
import matplotlib.pyplot as plt
import numpy
import math


class COMGenerator:
    COM_X  = []
    COM_Y  = []
    COM_Z  = []
    
    def __init__(self):
        self.COM_X = []
        self.COM_Y = []
        self.COM_Z = []
        
        
    def Generate(self,COM_X0,COM_Y0,SupportPositionsX,SupportPositionsY,zmp_x,zmp_y,StepTime,SamplingTime,NumberOfStep,DSRatio,CoMHeightAmp,Z0):
            DSTime = (DSRatio * StepTime)
           
            COM_X = []
            COM_Y = []
            COM_Z = []
            
            
            gravity = 9.81
            
            Zc = Z0
            ddotzc = 0
            a = (gravity+ddotzc)/Zc
            w = math.sqrt(a)
            
            i=0
            
            #A = CoMHeightAmp
            
            for GlobalTime in numpy.arange(0,StepTime*(NumberOfStep-1),SamplingTime):
                t=GlobalTime
                COM_Z.append(Zc)
                w = math.sqrt(Zc/(gravity))
                   
                #%%%%%%%%%%%%%%%%%% Search for Initial Condition
                k = NumberOfStep+1
                while( k>0):    
                    if (t <= (k*StepTime) - DSTime/2 and t >= ((k-1)*StepTime) -DSTime/2):
                        break;
                    k=k-1
                #print("k is :"+ str(k)+"\n")
                #%*****************************************************************************************************
                if(k>1):
                    t0=(k-1)*StepTime - DSTime/2
                    tf=k*StepTime - DSTime/2
                else:
                    t0=0
                    tf=k*StepTime - DSTime/2    
                
                #%*****************************************************************************************************
                z=zmp_y[i]
                if (i<len(zmp_y)):
                    i=i+1;           
                
                if (t0==0):
                 y0 = COM_Y0
                 yf = (SupportPositionsY[k-1] + SupportPositionsY[k])/ 2           
                else:
                 y0 = (SupportPositionsY[k-2] + SupportPositionsY[k-1])/ 2                   
                 yf = (SupportPositionsY[k-1] + SupportPositionsY[k])/ 2                   

                y = z + (1/math.sinh((t0 - tf)/w))*((yf - z)*math.sinh((t0 - t)/w) + (-y0 + z)*math.sinh((tf - t)/w))
                COM_Y.append(y)
            
                #%*****************************************************************************************************
                z=zmp_x[i]
            
                if (t0==0):
                     x0 = COM_X0
                     xf = (SupportPositionsX[k-1] + SupportPositionsX[k])/ 2                     
                else:
                     x0 = (SupportPositionsX[k-2] + SupportPositionsX[k-1])/ 2                   
                     xf = (SupportPositionsX[k-1] + SupportPositionsX[k])/ 2                 
                
            
                x = z + (1/math.sinh((t0 - tf)/w))*((xf - z)*math.sinh((t0 - t)/w) + (-x0 + z)*math.sinh((tf - t)/w));
                COM_X.append(x);    
                
            
            
            self.COM_X = COM_X
            self.COM_Y = COM_Y
            self.COM_Z = COM_Z
            
#            
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
#FirstStepIsRight=1
#DSRatio=0.
#CoMHeightAmp = 0;
#Z0 = 1;
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
