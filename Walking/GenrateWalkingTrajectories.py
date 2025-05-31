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


class GenrateWalkingTrajectories:
    g = 9.81
    NumberOfStep = 5
    FootLength = 0.0
    FootWidth = 0.
    FootHeelToe = 0
    
    SamplingTime = 0.01
    DSRatio = 0.0
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
        self.FP = FootStepPlanner(self.FR0X,self.FR0Y,self.FL0X,self.FL0Y,self.DistanceBetweenFeet)
        self.FP.PlanSteps(self.NumberOfStep,self.StepX,self.StepY,self.FirstStepIsRight)
        
        #%% ZMP Generator     
        self.ZMP = ZMPGenerator()
        self.ZMP.Generate(self.FP.SupportPositionsX,self.FP.SupportPositionsY,self.FootHeelToe,self.StepTime,self.DSRatio,self.SamplingTime)
        
        #%% COM Generator
        self.COM = COMGenerator()
        self.COM.Generate(self.COM_X0,self.COM_Y0,self.FP.SupportPositionsX,self.FP.SupportPositionsY,self.ZMP.zmp_x,self.ZMP.zmp_y,self.StepTime,self.SamplingTime,self.NumberOfStep,self.DSRatio,self.CoMHeightAmp,self.Z0)
        #%% FeetGenerator
        self.FT = FeetGenerator()
        self.FT.Generate(self.StepX,self.StepY,self.SwingStepZ,self.StepTime,self.SamplingTime,self.NumberOfStep,self.ZMP.zmp_x,self.ZMP.zmp_y,self.FR0X,self.FR0Y,self.FL0X,self.FL0Y,self.FirstStepIsRight,self.SmoothHeightIndex)
        

        for i in range(0,len(self.COM.COM_X)-1): 
            self.RightLegX.append(self.FT.RightX[i] - self.COM.COM_X[i])
            self.RightLegY.append(self.FT.RightY[i] - self.COM.COM_Y[i])
            self.RightLegZ.append(self.FT.RightZ[i] - self.ZLeg )
            
            self.LeftLegX.append(self.FT.LeftX[i] - self.COM.COM_X[i])
            self.LeftLegY.append(self.FT.LeftY[i] - self.COM.COM_Y[i])
            self.LeftLegZ.append(self.FT.LeftZ[i] - self.ZLeg )
            
