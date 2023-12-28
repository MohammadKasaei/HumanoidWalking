#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 22 14:07:46 2020

@author: mohammad
"""

from .FootStepPlanner import FootStepPlanner
import matplotlib.pyplot as plt
import numpy
import math


class ZMPGenerator:
    zmp_x  = []
    zmp_y  = []
    
    def __init__(self):
        self.zmp_x = []
        self.zmp_y = []
        
    def Generate(self,SupportPositionsX,SupportPositionsY,FootHeelToe,StepTime,DSRatio,SamplingTime):
                
        NumberOfStep = len(SupportPositionsY)-2
        
        DSTime = (DSRatio * StepTime)
        SSTime =  StepTime - DSTime
        
        zmp_x = []
        zmp_y = []
        
        index=0
        t=0
        for GlobalTime in numpy.arange(0,StepTime*(NumberOfStep),SamplingTime):
            if ( t <= SSTime): #SS        
                if (index==1):                        
                    ex0 = SupportPositionsX[index] +(FootHeelToe*t/SSTime)           
                else:
                    ex0 = SupportPositionsX[index] - (FootHeelToe)+(2*FootHeelToe*t/SSTime)           
                
                zmp_x.append(ex0);
                zmp_y.append(SupportPositionsY[index]);        
                #zmp_z.append(SupportPositionsZ[index]);
                LastZmpX =  ex0;
                LastZmpY =  SupportPositionsY[index];    
            else: #%%DS
                #dx = SupportPositionsX(2+floor(GlobalTime/StepTime)) - SupportPositionsX(1+floor(GlobalTime/StepTime))
                dx =( SupportPositionsX[1+math.floor(GlobalTime/StepTime)] - SupportPositionsX[math.floor(GlobalTime/StepTime)]) - (2*FootHeelToe)
                
                dy = SupportPositionsY[1+math.floor(GlobalTime/StepTime)] - SupportPositionsY[math.floor(GlobalTime/StepTime)]
                
                zmp_x.append(LastZmpX+(t-SSTime)*dx/(DSTime))
                zmp_y.append(LastZmpY+(t-SSTime)*dy/(DSTime))    
                #zmp_z = [zmp_z SupportPositionsZ(index)]
            
        
            if (t >= StepTime-SamplingTime):    
                index=index+1
                t=0
            else:
                t = t+SamplingTime;#%mod(GlobalTime,StepTime)            
            
        
        self.zmp_x = zmp_x
        self.zmp_y = zmp_y


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
#DSRatio=0.2
#
###
#FR0X=-StepX/2
#FR0Y=DistanceBetweenFeet/2
#FL0X=0
#FL0Y=-DistanceBetweenFeet/2
#
#
#FP = FootStepPlanner(FR0X,FR0Y,FL0X,FL0Y,DistanceBetweenFeet)
#FP.PlanSteps(NumberOfStep,StepX,StepY,FirstStepIsRight)
##print "X:",FP.SupportPositionsX,"\nY:",FP.SupportPositionsY
#
#
#fig = plt.figure()
#plt.scatter(FP.SupportPositionsX,FP.SupportPositionsY,label='FootSteps',marker = '*',color = 'darkgreen',linewidth=3)
#plt.legend()
#plt.plot(FP.SupportPositionsX,FP.SupportPositionsY,label='FootSteps',color = 'darkred',linewidth=1)
#      
#ZMP = ZMPGenerator()
#ZMP.Generate(FP.SupportPositionsX,FP.SupportPositionsY,FootHeelToe,StepTime,DSRatio,SamplingTime)
#
#
### plotting
#time = numpy.arange(0,NumberOfStep*StepTime,SamplingTime)
#fig = plt.figure()
#ax1 = fig.add_subplot(121)
#ax1.plot(time,ZMP.zmp_x,color = 'darkgreen',linewidth=1)
#      
#ax2 = fig.add_subplot(122)
#ax2.plot(time,ZMP.zmp_y,color = 'darkgreen',linewidth=1)
#
