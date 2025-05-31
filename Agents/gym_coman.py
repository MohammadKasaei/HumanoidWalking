import pybullet as p
import time
import pybullet_data as pd
import math
import numpy as np
from gym import spaces
import os, json
import random
import platform

from getkey import getkey, keys
import matplotlib.pyplot as plt

from Agents import Utilities as Ut

__cwd__ = os.path.realpath( os.path.join(os.getcwd(), os.path.dirname(__file__)))

class HumanoidGym():

    #============================================================================================ Analytic controller
    #================================================================================================================

    def IK(self, targetPosition):

        Lu = 0.205
        Ld = 0.205

        targetPosition[0] = targetPosition[0] * math.cos(math.radians(targetPosition[3])) - targetPosition[1] * math.sin(math.radians(targetPosition[3]))
        targetPosition[1] = targetPosition[1] * math.sin(math.radians(targetPosition[3])) + targetPosition[1] * math.cos(math.radians(targetPosition[3]))

        sqrLx =  (targetPosition[0]* targetPosition[0])+(targetPosition[2]* targetPosition[2])
        sqrLy =  (targetPosition[1]* targetPosition[1])+(targetPosition[2]* targetPosition[2])
        Lx = math.sqrt(sqrLx)
        Ly = math.sqrt(sqrLy)

        sqrLu = Lu * Lu
        sqrLd = Ld * Ld

        cosX = (sqrLx+sqrLu-sqrLd) / (2*Lx*Lu)
        cosX = 1 if cosX>1 else -1 if cosX<-1 else cosX
        alpha = math.acos(cosX)
        hipX = alpha + math.atan(targetPosition[0] / Lx)
        hipY = math.atan(targetPosition[1] / Ly)
        hipTheta = targetPosition[3]

        cosKnee = (sqrLu+sqrLd-sqrLx) / (2.0*Lu*Ld)
        cosKnee = 1 if cosKnee>1 else -1 if cosKnee<-1 else cosKnee
        kneeX = math.pi-(math.acos(cosKnee))
        ankleX = hipX-(kneeX)
        ankleY = -hipY

        JointPoses = [hipX,hipY,hipTheta,kneeX,ankleX,ankleY]
        return JointPoses 

    def UpdateJointPositions(self, RfootPosition,LfootPosition):
        JPR = self.IK(RfootPosition)
        RHipX = -JPR[0]
        RHipY = JPR[1]
        RHipZ = JPR[2]
        RKneeX = JPR[3]
        RAnkleX = JPR[4]
        RAnkleY = JPR[5]

        JPL = self.IK(LfootPosition)
        LHipX = -JPL[0]
        LHipY = JPL[1]
        LHipZ = JPL[2]
        LKneeX = JPL[3]
        LAnkleX = JPL[4]
        LAnkleY = JPL[5]
        
        return ([RHipX,RHipY,RHipZ,RKneeX,RAnkleX,RAnkleY,LHipX,LHipY,LHipZ,LKneeX,LAnkleX,LAnkleY])


    def PDControl(self, err, derr,PGain, DGain,Saturation):
        return np.clip((PGain*(err)) + (DGain*derr),Saturation[1],Saturation[0])


    def UpdateZMP(self, init=False):

        lCoP, rCoP = self.COMAN.pos_3d_lfoot_CoP, self.COMAN.pos_3d_rfoot_CoP #x: positive=front, y: positive=left, z: positive=up

        xCopL = lCoP[1]
        yCopL = lCoP[0]
        xCopR = rCoP[1]
        yCopR = rCoP[0]


        if (init):
            self.CoP_L = [xCopL,yCopL] #mohammad, what are these variables for?
            self.CoP_R = [xCopR,yCopR]
        else:
            a = 0.975
            b = 0.025
            self.CoP_L = [(a*self.CoP_L[0]+b*xCopL),(a*self.CoP_L[1]+b*yCopL)]
            self.CoP_R = [(a*self.CoP_R[0]+b*xCopR),(a*self.CoP_R[1]+b*yCopR)]

            #Reset each foot immediately if it's in the air
            if(np.all(lCoP==0)): self.CoP_L = [xCopL, yCopL]
            if(np.all(rCoP==0)): self.CoP_R = [xCopR, yCopR]

    def wsin(self,time, period, period_shift, mag, mag_shift):
        return mag * np.sin(2 * np.pi / period * time - period_shift) + mag_shift

    def update_time_param(self):

        self.DSP_Ratio      = 0.1
        self.PELVIS_OFFSET  = 0.0
        self.ARM_SWING_GAIN = 3.5
        
        
        SSP_Ratio              = 1 - self.DSP_Ratio

        self.X_Swap_PeriodTime = self.StepTime
        self.X_Move_PeriodTime = 2 * self.StepTime * SSP_Ratio

        self.Y_Swap_PeriodTime = 2 * self.StepTime
        self.Y_Move_PeriodTime = 2 * self.StepTime * SSP_Ratio
        
        self.Z_Swap_PeriodTime = self.StepTime
        self.Z_Move_PeriodTime = self.StepTime * SSP_Ratio
        
        self.A_Move_PeriodTime = 2 * self.StepTime * SSP_Ratio

        self.SSP_Time          = 2 * self.StepTime * SSP_Ratio
        self.SSP_Time_Start_L  = (1 - SSP_Ratio) * self.StepTime / 2
        self.SSP_Time_End_L    = (1 + SSP_Ratio) * self.StepTime / 2
        self.SSP_Time_Start_R  = (3 - SSP_Ratio) * self.StepTime / 2
        self.SSP_Time_End_R    = (3 + SSP_Ratio) * self.StepTime / 2

        self.Phase_Time1       = (self.SSP_Time_End_L + self.SSP_Time_Start_L) / 2
        self.Phase_Time2       = (self.SSP_Time_Start_R + self.SSP_Time_End_L) / 2
        self.Phase_Time3       = (self.SSP_Time_End_R + self.SSP_Time_Start_R) / 2

        self.Pelvis_Offset     = self.PELVIS_OFFSET
        self.Pelvis_Swing      = self.Pelvis_Offset * 0.35
        self.Arm_Swing_Gain    = self.ARM_SWING_GAIN    


    def update_move_param(self):
        # Forward/Back
        self.X_Move_Amplitude = self.StepX / 2
        self.X_Swap_Amplitude = self.StepX / 4
        
        # Right/Left
        self.Y_Move_Amplitude = self.StepY / 2
        if ( self.Y_Move_Amplitude > 0):
            self.Y_Move_Amplitude_Shift = self.Y_Move_Amplitude
        else:
            self.Y_Move_Amplitude_Shift = -self.Y_Move_Amplitude
        
        self.Y_SWAP_AMPLITUDE = 0.025
        self.Y_Swap_Amplitude = self.Y_SWAP_AMPLITUDE + (self.Y_Move_Amplitude_Shift * 0.04)

        
        # self.Theta_Move_Amplitude = math.radians(self.StepTheta) / 2

        self.Z_Move_Amplitude = self.SwingStepZ / 1
        self.Z_SWAP_AMPLITUDE = -0.0005
        self.Z_Move_Amplitude_Shift = self.Z_Move_Amplitude / 2
        self.Z_Swap_Amplitude = self.Z_SWAP_AMPLITUDE
        self.Z_Swap_Amplitude_Shift = self.Z_Swap_Amplitude

        # Theta movement
        self.A_MOVEMENT_ON = False
        self.A_MOVE_AMPLITUDE = math.radians(self.StepTheta) / 2
        if(self.A_MOVEMENT_ON == False): 
            self.A_Move_Amplitude = self.A_MOVE_AMPLITUDE / 2
            if(self.A_Move_Amplitude > 0):
                self.A_Move_Amplitude_Shift = self.A_Move_Amplitude
            else:
                self.A_Move_Amplitude_Shift = -self.A_Move_Amplitude
        else:    
            self.A_Move_Amplitude = -self.A_MOVE_AMPLITUDE / 2
            if( self.A_Move_Amplitude > 0 ):
                self.A_Move_Amplitude_Shift = -self.A_Move_Amplitude
            else:
                self.A_Move_Amplitude_Shift = self.A_Move_Amplitude
    
        
    def generate_cpg(self,Time):

        TIME_UNIT = self.SamplingTime / 2

        self.X_Swap_Phase_Shift     = np.pi
        self.X_Swap_Amplitude_Shift = 0
        self.X_Move_Phase_Shift     = np.pi / 2
        self.X_Move_Amplitude_Shift = 0
        
        self.Y_Swap_Phase_Shift     = 0
        self.Y_Swap_Amplitude_Shift = 0
        self.Y_Move_Phase_Shift     = np.pi / 2
            
        self.Z_Swap_Phase_Shift = np.pi * 3 / 2
        self.Z_Move_Phase_Shift = np.pi / 2
            
        self.A_Move_Phase_Shift = np.pi / 2

        if (Time <= TIME_UNIT):
            self.update_time_param()
        
        elif(Time >= (self.Phase_Time1 - TIME_UNIT) and Time < (self.Phase_Time1 + TIME_UNIT)):    
           self.update_move_param()
        
        elif(Time >= (self.Phase_Time2 - TIME_UNIT) and Time < (self.Phase_Time2 + TIME_UNIT)):
            self.update_time_param()
            # Time = self.Phase_Time2

        elif(Time >= (self.Phase_Time3 - TIME_UNIT) and Time < (self.Phase_Time3 + TIME_UNIT)):    
            self.update_move_param()
        
        x_swap = self.wsin(Time, self.X_Swap_PeriodTime, self.X_Swap_Phase_Shift, self.X_Swap_Amplitude, self.X_Swap_Amplitude_Shift)
        y_swap = self.wsin(Time, self.Y_Swap_PeriodTime, self.Y_Swap_Phase_Shift, self.Y_Swap_Amplitude, self.Y_Swap_Amplitude_Shift)
        z_swap = self.wsin(Time, self.Z_Swap_PeriodTime, self.Z_Swap_Phase_Shift, self.Z_Swap_Amplitude, self.Z_Swap_Amplitude_Shift)
        c_swap = z_swap

        if (Time <= self.SSP_Time_Start_L): 
            x_move_l = self.wsin(self.SSP_Time_Start_L, self.X_Move_PeriodTime, self.X_Move_Phase_Shift + 2 * np.pi / self.X_Move_PeriodTime * self.SSP_Time_Start_L, self.X_Move_Amplitude, self.X_Move_Amplitude_Shift)
            y_move_l = self.wsin(self.SSP_Time_Start_L, self.Y_Move_PeriodTime, self.Y_Move_Phase_Shift + 2 * np.pi / self.Y_Move_PeriodTime * self.SSP_Time_Start_L, self.Y_Move_Amplitude, self.Y_Move_Amplitude_Shift)
            z_move_l = self.wsin(self.SSP_Time_Start_L, self.Z_Move_PeriodTime, self.Z_Move_Phase_Shift + 2 * np.pi / self.Z_Move_PeriodTime * self.SSP_Time_Start_L, self.Z_Move_Amplitude, self.Z_Move_Amplitude_Shift)
            c_move_l = self.wsin(self.SSP_Time_Start_L, self.A_Move_PeriodTime, self.A_Move_Phase_Shift + 2 * np.pi / self.A_Move_PeriodTime * self.SSP_Time_Start_L, self.A_Move_Amplitude, self.A_Move_Amplitude_Shift)

            x_move_r = self.wsin(self.SSP_Time_Start_L, self.X_Move_PeriodTime, self.X_Move_Phase_Shift + 2 * np.pi / self.X_Move_PeriodTime * self.SSP_Time_Start_L, -self.X_Move_Amplitude, -self.X_Move_Amplitude_Shift)
            y_move_r = self.wsin(self.SSP_Time_Start_L, self.Y_Move_PeriodTime, self.Y_Move_Phase_Shift + 2 * np.pi / self.Y_Move_PeriodTime * self.SSP_Time_Start_L, -self.Y_Move_Amplitude, -self.Y_Move_Amplitude_Shift)
            z_move_r = self.wsin(self.SSP_Time_Start_R, self.Z_Move_PeriodTime, self.Z_Move_Phase_Shift + 2 * np.pi / self.Z_Move_PeriodTime * self.SSP_Time_Start_R,  self.Z_Move_Amplitude,   self.Z_Move_Amplitude_Shift)
            c_move_r = self.wsin(self.SSP_Time_Start_L, self.A_Move_PeriodTime, self.A_Move_Phase_Shift + 2 * np.pi / self.A_Move_PeriodTime * self.SSP_Time_Start_L, -self.A_Move_Amplitude, -self.A_Move_Amplitude_Shift)

            pelvis_offset_l = 0
            pelvis_offset_r = 0            
        
        elif (Time <= self.SSP_Time_End_L):    
            x_move_l = self.wsin(Time, self.X_Move_PeriodTime, self.X_Move_Phase_Shift + 2 * np.pi / self.X_Move_PeriodTime * self.SSP_Time_Start_L, self.X_Move_Amplitude, self.X_Move_Amplitude_Shift)
            y_move_l = self.wsin(Time, self.Y_Move_PeriodTime, self.Y_Move_Phase_Shift + 2 * np.pi / self.Y_Move_PeriodTime * self.SSP_Time_Start_L, self.Y_Move_Amplitude, self.Y_Move_Amplitude_Shift)
            z_move_l = self.wsin(Time, self.Z_Move_PeriodTime, self.Z_Move_Phase_Shift + 2 * np.pi / self.Z_Move_PeriodTime * self.SSP_Time_Start_L, self.Z_Move_Amplitude, self.Z_Move_Amplitude_Shift)
            c_move_l = self.wsin(Time, self.A_Move_PeriodTime, self.A_Move_Phase_Shift + 2 * np.pi / self.A_Move_PeriodTime * self.SSP_Time_Start_L, self.A_Move_Amplitude, self.A_Move_Amplitude_Shift)

            x_move_r = self.wsin(Time, self.X_Move_PeriodTime, self.X_Move_Phase_Shift + 2 * np.pi / self.X_Move_PeriodTime * self.SSP_Time_Start_L, -self.X_Move_Amplitude, -self.X_Move_Amplitude_Shift)
            y_move_r = self.wsin(Time, self.Y_Move_PeriodTime, self.Y_Move_Phase_Shift + 2 * np.pi / self.Y_Move_PeriodTime * self.SSP_Time_Start_L, -self.Y_Move_Amplitude, -self.Y_Move_Amplitude_Shift)
            z_move_r = self.wsin(self.SSP_Time_Start_R, self.Z_Move_PeriodTime, self.Z_Move_Phase_Shift + 2 * np.pi / self.Z_Move_PeriodTime * self.SSP_Time_Start_R, self.Z_Move_Amplitude, self.Z_Move_Amplitude_Shift)
            c_move_r = self.wsin(Time, self.A_Move_PeriodTime, self.A_Move_Phase_Shift + 2 * np.pi / self.A_Move_PeriodTime * self.SSP_Time_Start_L, -self.A_Move_Amplitude, -self.A_Move_Amplitude_Shift)
            
            pelvis_offset_l = self.wsin(Time, self.Z_Move_PeriodTime, self.Z_Move_Phase_Shift + 2 * np.pi / self.Z_Move_PeriodTime * self.SSP_Time_Start_L, self.Pelvis_Swing / 2, self.Pelvis_Swing / 2)
            pelvis_offset_r = self.wsin(Time, self.Z_Move_PeriodTime, self.Z_Move_Phase_Shift + 2 * np.pi / self.Z_Move_PeriodTime * self.SSP_Time_Start_L, -self.Pelvis_Offset / 2, -self.Pelvis_Offset / 2)
        
        elif (Time <= self.SSP_Time_Start_R):    
            x_move_l = self.wsin(self.SSP_Time_End_L, self.X_Move_PeriodTime, self.X_Move_Phase_Shift + 2 * np.pi / self.X_Move_PeriodTime * self.SSP_Time_Start_L, self.X_Move_Amplitude, self.X_Move_Amplitude_Shift)
            y_move_l = self.wsin(self.SSP_Time_End_L, self.Y_Move_PeriodTime, self.Y_Move_Phase_Shift + 2 * np.pi / self.Y_Move_PeriodTime * self.SSP_Time_Start_L, self.Y_Move_Amplitude, self.Y_Move_Amplitude_Shift)
            z_move_l = self.wsin(self.SSP_Time_End_L, self.Z_Move_PeriodTime, self.Z_Move_Phase_Shift + 2 * np.pi / self.Z_Move_PeriodTime * self.SSP_Time_Start_L, self.Z_Move_Amplitude, self.Z_Move_Amplitude_Shift)
            c_move_l = self.wsin(self.SSP_Time_End_L, self.A_Move_PeriodTime, self.A_Move_Phase_Shift + 2 * np.pi / self.A_Move_PeriodTime * self.SSP_Time_Start_L, self.A_Move_Amplitude, self.A_Move_Amplitude_Shift)
            
            x_move_r = self.wsin(self.SSP_Time_End_L, self.X_Move_PeriodTime, self.X_Move_Phase_Shift + 2 * np.pi / self.X_Move_PeriodTime * self.SSP_Time_Start_L, -self.X_Move_Amplitude, -self.X_Move_Amplitude_Shift)
            y_move_r = self.wsin(self.SSP_Time_End_L, self.Y_Move_PeriodTime, self.Y_Move_Phase_Shift + 2 * np.pi / self.Y_Move_PeriodTime * self.SSP_Time_Start_L, -self.Y_Move_Amplitude, -self.Y_Move_Amplitude_Shift)
            z_move_r = self.wsin(self.SSP_Time_Start_R, self.Z_Move_PeriodTime, self.Z_Move_Phase_Shift + 2 * np.pi / self.Z_Move_PeriodTime * self.SSP_Time_Start_R, self.Z_Move_Amplitude, self.Z_Move_Amplitude_Shift)
            c_move_r = self.wsin(self.SSP_Time_End_L, self.A_Move_PeriodTime, self.A_Move_Phase_Shift + 2 * np.pi / self.A_Move_PeriodTime * self.SSP_Time_Start_L, -self.A_Move_Amplitude, -self.A_Move_Amplitude_Shift)
            
            pelvis_offset_l = 0
            pelvis_offset_r = 0    
        
        elif( Time <= self.SSP_Time_End_R):    

            x_move_l = self.wsin(Time, self.X_Move_PeriodTime, self.X_Move_Phase_Shift + 2 * np.pi / self.X_Move_PeriodTime * self.SSP_Time_Start_R + np.pi, self.X_Move_Amplitude, self.X_Move_Amplitude_Shift)
            y_move_l = self.wsin(Time, self.Y_Move_PeriodTime, self.Y_Move_Phase_Shift + 2 * np.pi / self.Y_Move_PeriodTime * self.SSP_Time_Start_R + np.pi, self.Y_Move_Amplitude, self.Y_Move_Amplitude_Shift)
            z_move_l = self.wsin(self.SSP_Time_End_L, self.Z_Move_PeriodTime, self.Z_Move_Phase_Shift + 2 * np.pi / self.Z_Move_PeriodTime * self.SSP_Time_Start_L, self.Z_Move_Amplitude, self.Z_Move_Amplitude_Shift)
            c_move_l = self.wsin(Time, self.A_Move_PeriodTime, self.A_Move_Phase_Shift + 2 * np.pi / self.A_Move_PeriodTime * self.SSP_Time_Start_R + np.pi, self.A_Move_Amplitude, self.A_Move_Amplitude_Shift)
            
            x_move_r = self.wsin(Time, self.X_Move_PeriodTime, self.X_Move_Phase_Shift + 2 * np.pi / self.X_Move_PeriodTime * self.SSP_Time_Start_R + np.pi, -self.X_Move_Amplitude, -self.X_Move_Amplitude_Shift)
            y_move_r = self.wsin(Time, self.Y_Move_PeriodTime, self.Y_Move_Phase_Shift + 2 * np.pi / self.Y_Move_PeriodTime * self.SSP_Time_Start_R + np.pi, -self.Y_Move_Amplitude, -self.Y_Move_Amplitude_Shift)
            z_move_r = self.wsin(Time, self.Z_Move_PeriodTime, self.Z_Move_Phase_Shift + 2 * np.pi / self.Z_Move_PeriodTime * self.SSP_Time_Start_R, self.Z_Move_Amplitude, self.Z_Move_Amplitude_Shift)
            c_move_r = self.wsin(Time, self.A_Move_PeriodTime, self.A_Move_Phase_Shift + 2 * np.pi / self.A_Move_PeriodTime * self.SSP_Time_Start_R + np.pi, -self.A_Move_Amplitude, -self.A_Move_Amplitude_Shift)
            
            pelvis_offset_l = self.wsin(Time, self.Z_Move_PeriodTime, self.Z_Move_Phase_Shift + 2 * np.pi / self.Z_Move_PeriodTime * self.SSP_Time_Start_R, self.Pelvis_Offset / 2, self.Pelvis_Offset / 2)
            pelvis_offset_r = self.wsin(Time, self.Z_Move_PeriodTime, self.Z_Move_Phase_Shift + 2 * np.pi / self.Z_Move_PeriodTime * self.SSP_Time_Start_R, -self.Pelvis_Swing / 2, -self.Pelvis_Swing / 2)
        
        else:    
        
            x_move_l = self.wsin(self.SSP_Time_End_R, self.X_Move_PeriodTime, self.X_Move_Phase_Shift + 2 * np.pi / self.X_Move_PeriodTime * self.SSP_Time_Start_R + np.pi, self.X_Move_Amplitude, self.X_Move_Amplitude_Shift)
            y_move_l = self.wsin(self.SSP_Time_End_R, self.Y_Move_PeriodTime, self.Y_Move_Phase_Shift + 2 * np.pi / self.Y_Move_PeriodTime * self.SSP_Time_Start_R + np.pi, self.Y_Move_Amplitude, self.Y_Move_Amplitude_Shift)
            z_move_l = self.wsin(self.SSP_Time_End_L, self.Z_Move_PeriodTime, self.Z_Move_Phase_Shift + 2 * np.pi / self.Z_Move_PeriodTime * self.SSP_Time_Start_L, self.Z_Move_Amplitude, self.Z_Move_Amplitude_Shift)
            c_move_l = self.wsin(self.SSP_Time_End_R, self.A_Move_PeriodTime, self.A_Move_Phase_Shift + 2 * np.pi / self.A_Move_PeriodTime * self.SSP_Time_Start_R + np.pi, self.A_Move_Amplitude, self.A_Move_Amplitude_Shift)
            
            x_move_r = self.wsin(self.SSP_Time_End_R, self.X_Move_PeriodTime, self.X_Move_Phase_Shift + 2 * np.pi / self.X_Move_PeriodTime * self.SSP_Time_Start_R + np.pi, -self.X_Move_Amplitude, -self.X_Move_Amplitude_Shift)
            y_move_r = self.wsin(self.SSP_Time_End_R, self.Y_Move_PeriodTime, self.Y_Move_Phase_Shift + 2 * np.pi / self.Y_Move_PeriodTime * self.SSP_Time_Start_R + np.pi, -self.Y_Move_Amplitude, -self.Y_Move_Amplitude_Shift)
            z_move_r = self.wsin(self.SSP_Time_End_R, self.Z_Move_PeriodTime, self.Z_Move_Phase_Shift + 2 * np.pi / self.Z_Move_PeriodTime * self.SSP_Time_Start_R, self.Z_Move_Amplitude, self.Z_Move_Amplitude_Shift)
            c_move_r = self.wsin(self.SSP_Time_End_R, self.A_Move_PeriodTime, self.A_Move_Phase_Shift + 2 * np.pi / self.A_Move_PeriodTime * self.SSP_Time_Start_R + np.pi, -self.A_Move_Amplitude, -self.A_Move_Amplitude_Shift)
            
            pelvis_offset_l = 0
            pelvis_offset_r = 0
        
        if (self.X_Move_Amplitude == 0):
            arm_r = 0 
            arm_l = 0 
        else:
            arm_r = self.wsin(Time, self.StepTime * 2, np.pi * 1.5, -self.X_Move_Amplitude * self.Arm_Swing_Gain, 0)
            arm_l = self.wsin(Time, self.StepTime * 2, np.pi * 1.5, self.X_Move_Amplitude * self.Arm_Swing_Gain, 0)


        xl  = x_swap + x_move_l
        yl  = y_swap + y_move_l  
        zl  = self.Zleg + z_swap + z_move_l
        tl  = c_move_l + c_swap
            
        xr  = x_swap + x_move_r
        yr  = y_swap + y_move_r
        zr  = self.Zleg  + z_swap + z_move_r
        tr  = c_move_r + c_swap
        
        l_pos = np.array([xl,yl,zl,tl,arm_l])
        r_pos = np.array([xr,yr,zr,tr,arm_r])   

        return l_pos, r_pos

    def updateOmniJoints_CPG(self):

        if (abs(self.gtime-self.twalk0) > (2*self.StepTime)-(self.SamplingTime/2)):
            self.twalk0 = self.gtime
            self.StopForOneStep =0


            if (self.NewCommand): 
                self.StepTheta = self.NewStepTheta
                self.StepX = self.NewStepX*math.cos(math.radians(self.StepTheta))-self.NewStepY*math.sin(math.radians(self.StepTheta))
                self.StepY = self.NewStepX*math.sin(math.radians(self.StepTheta))+self.NewStepY*math.cos(math.radians(self.StepTheta))
                # self.StepTime = self.NewStepTime
                self.NewCommand = 0

            self.idx = -1

            if (self.StepTime<0.5):
                self.StepTime += 0.05
            else:
                self.StepTime = 0.3
            

            print("step time{0}".format(self.StepTime))

            self.update_time_param()
            self.update_move_param()
            self.generate_cpg(0)

        else:
            
            twalk = self.gtime - self.twalk0
            self.idx = self.idx+1 #int(round(twalk/SamplingTime))
            
            vA =  self.COMAN.wAngVel_3d_torso #mohammad, why do we use world coordinates for angular velocity?

            self.PD_Wx = (self.action[0] * 0.30) + 1*self.PDControl(-self.COMAN.ori_3d_torso_rad[1], vA[1], 1, -0.05, [0.3,-0.3])
            self.PD_Wy = (self.action[1] * 0.05) + 1*self.PDControl(-self.COMAN.ori_3d_torso_rad[0], vA[0], 0.5, 0.0, [0.1,-0.1])

            self.PD_HLx = (self.action[2] * 0.25) + -1*self.PDControl(0.0-self.COMAN.ori_3d_torso_rad[1], vA[1], 2, -0.05, [0.25,-0.25])
            self.PD_HRx = (self.action[3] * 0.25) + -1*self.PDControl(0.0-self.COMAN.ori_3d_torso_rad[1], vA[1], 2, -0.05, [0.25,-0.25])

            self.PD_HLy = (self.action[4] * 0.25) + -1*self.PDControl(0.0-self.COMAN.ori_3d_torso_rad[0], vA[0], .7, -0.05, [0.,-0.2])
            self.PD_HRy = (self.action[5] * 0.25) + -1*self.PDControl(0.0-self.COMAN.ori_3d_torso_rad[0], vA[0], .7, -0.05, [0.2,-0.])

            self.PD_ALx = (self.action[6] * 0.2) + -self.PD_HLx
            self.PD_ARx = (self.action[7] * 0.2) + -self.PD_HRx

            self.PD_ALy = (self.action[8] * 0.2) + -self.PD_HLy
            self.PD_ARy = (self.action[9] * 0.2) + -self.PD_HRy

            self.PD_Wx =  np.clip(self.PD_Wx , -0.30, 0.30 )
            self.PD_Wy =  np.clip(self.PD_Wy , -0.20, 0.20 )
            self.PD_HLx = np.clip(self.PD_HLx, -0.25, 0.25 )
            self.PD_HRx = np.clip(self.PD_HRx, -0.25, 0.25 )
            self.PD_HLy = np.clip(self.PD_HLy, -0.00, 0.25 )
            self.PD_HRy = np.clip(self.PD_HRy, -0.25, 0.00 )
            self.PD_ALx = np.clip(self.PD_ALx, -0.20, 0.20 ) #foot front rotation
            self.PD_ARx = np.clip(self.PD_ARx, -0.20, 0.20 ) #foot front rotation
            self.PD_ALy = np.clip(self.PD_ALy, -0.30, 0.30 ) #foot side rotation
            self.PD_ARy = np.clip(self.PD_ARy, -0.30, 0.30 ) #foot side rotation

            self.PD_Wx  += self.action[0] * 0.3
            self.PD_Wy  += self.action[1] * 0.05
            self.PD_HLx += self.action[2] * 0.25
            self.PD_HRx += self.action[3] * 0.25
            self.PD_HLy += self.action[4] * 0.25
            self.PD_HRy += self.action[5] * 0.25
            self.PD_ALx += self.action[6] * 0.2
            self.PD_ARx += self.action[7] * 0.2
            self.PD_ALy += self.action[8] * 0.2
            self.PD_ARy += self.action[9] * 0.2

            self.PD_Wx  = np.clip(self.PD_Wx , -0.30, 0.30 )
            self.PD_Wy  = np.clip(self.PD_Wy , -0.20, 0.20 )
            self.PD_HLx = np.clip(self.PD_HLx, -0.25, 0.25 )
            self.PD_HRx = np.clip(self.PD_HRx, -0.25, 0.25 )
            self.PD_HLy = np.clip(self.PD_HLy, -0.00, 0.25 )
            self.PD_HRy = np.clip(self.PD_HRy, -0.25, 0.00 )
            self.PD_ALx = np.clip(self.PD_ALx, -0.20, 0.20 ) #foot front rotation
            self.PD_ARx = np.clip(self.PD_ARx, -0.20, 0.20 ) #foot front rotation
            self.PD_ALy = np.clip(self.PD_ALy, -0.30, 0.30 ) #foot side rotation 
            self.PD_ARy = np.clip(self.PD_ARy, -0.30, 0.30 ) #foot side rotation

            lpos, rpos = self.generate_cpg(twalk)
            self.RfootPosition = rpos
            self.LfootPosition = lpos
            
            arm_offset = math.radians(20)
            self.LShX  = arm_offset+lpos[4]
            self.RShX  = arm_offset+rpos[4]
            self.JointPos = self.UpdateJointPositions(self.RfootPosition,self.LfootPosition)
            self.JointPositions = [self.WaistX + self.PD_Wx, self.WaistY + self.PD_Wy, self.WaistZ,
                            self.RShX,self.RShY,self.RShZ,self.REB,
                            self.LShX,self.LShY,self.LShZ,self.LEB,
                            self.JointPos[0] + self.PD_HRx, self.JointPos[1] + self.PD_HRy, self.JointPos[2],self.JointPos[3],self.JointPos[4]  + self.PD_ARx, self.JointPos[5]  + self.PD_ARy,
                            self.JointPos[6] + self.PD_HLx, self.JointPos[7] + self.PD_HLy, self.JointPos[8],self.JointPos[9],self.JointPos[10] + self.PD_ALx, self.JointPos[11] + self.PD_ALy]

    # #====================================== Scenario specific stuff
  

    def create_obstacle_course(self): #obst course

        scale = 0.3     #scale of blocks
        self.course_grid_size = grid_size = 0.5
        height_scale = 0.6 #height = scale * height_scale
        density = 20      #kg / m^3

        # positive number is the obs (number represents the scale) 
        # -10 position target, -01 is the visual target  
        #  the start is always at left middle
        

        self.course = course = np.array(
                   [[  0,  0,  0,  0, 10,  0,  0,  0,  0,  0,  0,  0],
                    [  0,  0,  0,  0,-30,  0,  0,  0,-40,  0, 20, -4],
                    [  0,  0,  0, -7,  0, 10, 10, 10,  0,  0,  0,  0],
                    [  0,  0,  0,-70,  0, 10, 10,  0,  0,  0,  0,  0],
                    [  0,  0,  0,  0,-20, 10, 10,  0,  0,  0,  0,  0],
                    [ 10, 10, 10, 10,  0, 10,  0,  0,  0,  0,  0, -5],
                    [  0,  0,  0,  0,  0,  0,  0,  0,-10, -1,  0,  0],
                    [ 10, 10, 10, 10, 10, 10,  0,  0,  0,  0,  0,  0],
                    [  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
                    [  0,  0,  10,  0,  0, 0,  0,  0,  0,  0,  0,  0],
                    [  0,  0,  0,  0,  0,  0,-50,  0,  0, -60, 0,  0],
                    [  0,  0,  0,  0, -3,  0,  0,  0,  0,  0, -6,  0],
                    [  0,  0,  0,  0, -2,  0,  0,  0,  0,  0,  0,  0]])

        ylen, xlen = course.shape
        self.course_ydev = ydev = (ylen-1)/2 * grid_size

        for y in range(ylen):
            for x in range(xlen):
                if course[y][x] > 0:
                    size = course[y][x] / 10.0 * scale
                    halfsize = size / 2
                    box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[halfsize, halfsize, halfsize*height_scale])
                    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[halfsize, halfsize, halfsize*height_scale], rgbaColor=[1,0.5,0,1], specularColor=[1,1,1])
                    p.createMultiBody(density * size**3, box, vis, [x*grid_size,ydev-y*grid_size,halfsize*height_scale], [0,0,0,1])

        return 0

    def generate_obstacle_course_walk_command(self): #obst course

        if self.obstCourse_progress is None:
            self.obstCourse_progress = 1

        target = None
        vis = None

        ylen, xlen = self.course.shape
        for y in range(ylen):
            for x in range(xlen):
                if self.course[y][x] < 0:
                    if -self.course[y][x] % 10  == self.obstCourse_progress: vis    = [x*self.course_grid_size,self.course_ydev-y*self.course_grid_size,0]
                    if -self.course[y][x] // 10 == self.obstCourse_progress: target = [x*self.course_grid_size,self.course_ydev-y*self.course_grid_size,0]

        if vis is not None and target is not None: #generate walk command

            #current pos
            my_x, my_y, _ = self.COMAN.pos_3d_feet

            #get walking target and visual target
            self.NewStepX_raw, self.NewStepY_raw, _ = Ut.world_to_local_transform(target,[0,0,0,1],[my_x,my_y,0],self.COMAN.ori_4d_torso)[0]
            vis_localx, vis_localy, _ = Ut.world_to_local_transform(vis,[0,0,0,1],[my_x,my_y,0],self.COMAN.ori_4d_torso)[0]

            self.NewStepTheta_raw = -math.degrees( math.atan2(vis_localy, vis_localx) ) 

            self.NewStepX_raw *= 0.7 
            self.NewStepY_raw *= 0.7

            #check if walking target was reached
            if(abs(self.NewStepX_raw) < 0.13 and abs(self.NewStepY_raw) < 0.13):
                self.obstCourse_progress += 1
            
            #draw targets
            p.addUserDebugLine([my_x, my_y, 0.2], target, [0,0,1], lineWidth=3, lifeTime=1)
            p.addUserDebugLine([my_x, my_y, 0.4], vis, [0,1,0], lineWidth=2, lifeTime=1)

        else:
            self.NewStepX_raw, self.NewStepY_raw, self.NewStepTheta_raw = 0,0,0

        self.restrict_raw_walk_command(False)
        print(self.NewStepX_raw, self.NewStepY_raw, self.NewStepTheta_raw)
        self.apply_walk_command()


    def generate_8_figure_target(self):

        target = self.figure8_progress + self.figure8_stepsize
        scale = 2

        x = scale * math.cos(math.pi * 4 * target) - scale
        y = scale * math.sin(math.pi * 4 * target)


        return [x,y] if target < 0.5 else [-x,y]

    def draw_trajectory(self):
        scale = 2

        for i in range(0,100):

            i *= 0.01
            
            target = i + 0.01
            
            x_now = scale * math.cos(math.pi * 4 * i) - scale
            y_now = scale * math.sin(math.pi * 4 * i)
            if target > 0.5: x_now *= -1

            x_tar = scale * math.cos(math.pi * 4 * target) - scale
            y_tar = scale * math.sin(math.pi * 4 * target)
            if target > 0.5: x_tar *= -1

            p.addUserDebugLine([x_now, y_now, 0.05], [x_tar, y_tar, 0.05], [0,0,1], lineWidth=3, lifeTime=0)

    

    def generate_8_figure_walk_command(self):

        #current pos
        x, y, _ = self.COMAN.pos_3d_torso

        #check if target should be updated
        old_dist = (x - self.figure8_oldtarget[0])**2 + (y - self.figure8_oldtarget[1])**2
        new_dist = (x - self.figure8_target[0])**2 + (y - self.figure8_target[1])**2

        if(new_dist < old_dist):
            self.figure8_oldtarget = self.figure8_target
            self.figure8_progress += self.figure8_stepsize
            if self.figure8_progress > 1: self.figure8_progress -= 1
            self.figure8_target = self.generate_8_figure_target()

        #generate walk command
        target = [self.figure8_target[0], self.figure8_target[1], 0]
        self.NewStepX_raw, self.NewStepY_raw, _ = Ut.world_to_local_transform(target,[0,0,0,1],[x,y,0],self.COMAN.ori_4d_torso)[0]
        self.NewStepTheta_raw = -math.degrees( math.atan2(self.NewStepY_raw, self.NewStepX_raw) ) / 2
        
        self.restrict_raw_walk_command(False)
        self.apply_walk_command()


        oldtar = [self.figure8_oldtarget[0], self.figure8_oldtarget[1], 0.02]
        p.addUserDebugLine(oldtar, target, [1,0,0], lineWidth=6, lifeTime=4)
   

    def apply_walk_command(self):
    
        self.NewStepX +=     max( min( self.NewStepX_raw - self.NewStepX, 0.1) , -0.1)
        self.NewStepY +=     max( min( self.NewStepY_raw - self.NewStepY, 0.05) , -0.05)
        self.NewStepTheta += max( min( self.NewStepTheta_raw - self.NewStepTheta, 10) , -10)

        self.NewCommand = 1

        #print("Apply:", self.NewStepX, self.NewStepY, self.NewStepTheta)


    def restrict_raw_walk_command(self, isNormalized=False):

        stepx_lim = [-0.5,0.5]
        stepy_lim = [-0.3, 0.3]
        steptheta_lim = [-15, 15]

        #Normalize if not already
        if not isNormalized:
            x = Ut.translate( self.NewStepX_raw, stepx_lim, [-0.5,0.5] ) #map to relative values [-0.5, 0.5]
            y = Ut.translate( self.NewStepY_raw, stepy_lim, [-0.5,0.5] ) #map to relative values [-0.5, 0.5]
            t = Ut.translate( self.NewStepTheta_raw, steptheta_lim, [-0.5,0.5] ) #map to relative values [-0.5, 0.5]
        else:
            x = self.NewStepX_raw
            y = self.NewStepY_raw
            t = self.NewStepTheta_raw

        #avoid transition stop
        if abs(y) < 0.03: 
            y = 0
        if abs(t) < 0.2: 
            t = 0

        
        #limit overall maximum (only affects the command when the limit is actually exceeded)
        manhatDist = abs(x) + abs(y) + abs(t)

        if manhatDist > 0.5:
            factor = 1.0 #1 means maximum performance (<1 means safe parameters)
            norm = manhatDist / (0.5*factor)

            x /= norm
            y /= norm
            t /= norm

        #denormalize
        self.NewStepX_raw = np.interp( x, [-0.5,0.5], stepx_lim ) #map to absolute commands
        self.NewStepY_raw = np.interp( y, [-0.5,0.5], stepy_lim ) #map to absolute commands
        self.NewStepTheta_raw = np.interp( t, [-0.5,0.5], steptheta_lim ) #map to absolute commands


    def generate_random_walk_command(self): #WALK-MAIN

        self.NewStepX_raw     = Ut.translate( np.random.rand(), [0.1,0.9], [-0.5,0.5] ) #creates NO bias and saturation boundaries
        self.NewStepY_raw     = Ut.translate( np.random.rand(), [0.1,0.9], [-0.5,0.5] ) #creates saturation boundaries
        self.NewStepTheta_raw = Ut.translate( np.random.rand(), [0.1,0.9], [-0.5,0.5] ) #creates saturation boundaries

        self.restrict_raw_walk_command(True)
        self.apply_walk_command()

    def auto_push_routine(self):

        if self.push_countdown == 0:

            forcepos = np.add(self.COMAN.pos_3d_torso, [0,0,0.3])

            if self.CONFIG["clientMode"]!="GUI" or p.readUserDebugParameter(self.param_switchf) % 2 == 1:

                if self.push_duration == 5:
                    ang = np.random.rand()*2*math.pi
                    extraF = 0#self.gtime*5*4

                    wp = np.array(Ut.local_to_world_transform([math.sin(ang),math.cos(ang),0],[0,0,0,1],[0,0,0],self.COMAN.ori_4d_torso)[0][0:2])  
                    wp /= np.linalg.norm(wp) #2D world unit vector

                    self.push_force = [wp[0]*np.random.randint( self.push_min_force_norm+extraF, self.push_max_force_norm+extraF ), 
                                    wp[1]*np.random.randint( self.push_min_force_norm+extraF, self.push_max_force_norm+extraF ), 0]

                    if(self.CONFIG["deterministicPushes"]):
                        print("Deterministic push: f=", self.push_force, "d=", self.push_duration)


                p.applyExternalForce(self.COMAN.ID, -1, self.push_force, forcepos, p.WORLD_FRAME)
                self.time_since_last_push = 0
                fnorm = np.linalg.norm(self.push_force)

                if self.CONFIG["clientGUI"]:
                    if self.arrow_life==0:
                        self.arrow_life = 0.2 / self.SamplingTime
                        self.arrow_vec = np.multiply(self.push_force, (1+100/fnorm)/1000) #np.divide(self.push_force, 600)

                    if self.push_duration == 5:
                        tpos1 = np.add(self.COMAN.pos_3d_torso, [0,0,0.60])
                        tpos2 = np.add(tpos1, [0,0,-0.06])
                        p.addUserDebugText(text="F={}N".format(int(fnorm)) ,textPosition=tpos1, textSize=1.1, textColorRGB=[0,0,0], lifeTime=1)
                        p.addUserDebugText(text="dt=25ms" ,textPosition=tpos2, textSize=1.0, textColorRGB=[0,0,0], lifeTime=1)


                self.push_duration -= 1
                if self.push_duration == 0:
                    self.push_countdown = np.random.randint(self.push_min_cooldown, self.push_max_cooldown )
                    self.push_duration = 5

                    #self.push_force = [math.sin(ang)*np.random.randint( self.push_min_force_norm+extraF, self.push_max_force_norm+extraF ), 
                    #                math.cos(ang)*np.random.randint( self.push_min_force_norm+extraF, self.push_max_force_norm+extraF ), 0]
                    
        else:
            self.push_countdown -= 1


    #================================================================================================ Everything Else
    #================================================================================================================


    def load_map(self,mapX):

        p.setGravity(0,0,-9.81)

        if(mapX==0):
            z_difference = 0
            stabilization_steps = 100
        elif(mapX==5):   
            z_difference = self.create_obstacle_course()
            stabilization_steps = 100
            

        p.setAdditionalSearchPath(pd.getDataPath())
        floor = p.loadURDF("plane.urdf",[0,0,-z_difference])
        #floor = p.loadURDF("plane_ice.urdf",[0,0,-z_difference])

        p.setAdditionalSearchPath("")
        self.COMAN = Ut.bullet_agent( p.loadURDF(__cwd__ + "/models/coman/model_org.urdf",[0,0,0.515]), floor )

        #Stabilize robot for some iterations
        p.setTimeStep(self.SamplingTime)
        for _ in range(stabilization_steps): 
            p.stepSimulation()



    def __init__(self, *args, **kwargs):

        __cwd__ = os.path.realpath( os.path.join(os.getcwd(), os.path.dirname(__file__)))

        try:
            with open(os.path.join(__cwd__, 'config_' + platform.node() + '.txt'),'r') as fp:
                self.CONFIG = json.loads(fp.read()) 
        except:
            assert False, "You need to have a configuration file with the name of your host machine: config_" + platform.node() + '.txt'

        # self.thread_no = args[0]
        # self.mpi_size = args[1]
        # self.network_name = args[2]

        ########################################################### Initialize variables
        self.SamplingTime=0.005
      
        self.StepTime = 0.30
        self.StopForOneStep =  0
        self.NewStepTime = self.StepTime

        self.NewCommand = 0
        self.NewStepX = 0
        self.NewStepY = 0
        self.NewStepTheta = 0

        self.WaistX_DOF, self.WaistY_DOF, self.WaistZ_DOF = 3, 2, 4
        self.RShX_DOF,   self.RShY_DOF,   self.RShZ_DOF   = 7, 8, 9
        self.REB_DOF                                      = 10
        self.LShX_DOF,   self.LShY_DOF,   self.LShZ_DOF   = 21, 22, 23
        self.LEB_DOF                                      = 24

        self.RHipX_DOF,  self.RHipY_DOF,   self.RHipZ_DOF   = 35, 36, 37
        self.RKneeX_DOF, self.RAnkleX_DOF, self.RAnkleY_DOF = 38, 40, 39
        self.LHipX_DOF,  self.LHipY_DOF,   self.LHipZ_DOF   = 49, 50, 51
        self.LKneeX_DOF, self.LAnkleX_DOF, self.LAnkleY_DOF = 52, 54, 53

        self.JointIDOrder = [self.WaistX_DOF,self.WaistY_DOF,self.WaistZ_DOF,
                            self.RShX_DOF,self.RShY_DOF,self.RShZ_DOF,self.REB_DOF,
                            self.LShX_DOF,self.LShY_DOF,self.LShZ_DOF,self.LEB_DOF,
                            self.RHipX_DOF,self.RHipY_DOF,self.RHipZ_DOF,self.RKneeX_DOF,self.RAnkleX_DOF,self.RAnkleY_DOF,
                            self.LHipX_DOF,self.LHipY_DOF,self.LHipZ_DOF,self.LKneeX_DOF,self.LAnkleX_DOF,self.LAnkleY_DOF]


        self.WaistX = math.radians(5)
        self.WaistY = math.radians(0)
        self.WaistZ = math.radians(0)

        self.RShX = math.radians(20)
        self.RShY = math.radians(-20)
        self.RShZ = math.radians(0)
        self.REB = math.radians(-70)

        self.LShX = math.radians(20)
        self.LShY = math.radians(20)
        self.LShZ = math.radians(0)
        self.LEB = math.radians(-70)

        
        self.Zleg = 0.38
        self.JPR = self.IK([0,0,self.Zleg,0])
        self.RHipX = -self.JPR[0]
        self.RHipY = self.JPR[1]
        self.RHipZ = self.JPR[2]
        self.RKneeX = self.JPR[3]
        self.RAnkleX = self.JPR[4]
        self.RAnkleY = self.JPR[5]

        self.JPL = self.IK([0,0,self.Zleg,0])
        self.LHipX = -self.JPL[0]
        self.LHipY = self.JPL[1]
        self.LHipZ = self.JPL[2]
        self.LKneeX = self.JPL[3]
        self.LAnkleX = self.JPL[4]
        self.LAnkleY = self.JPL[5]

        self.RfootPosition = [0,0,self.Zleg,0]
        self.LfootPosition = [0,0,self.Zleg,0]
        self.JointPos = self.UpdateJointPositions(self.RfootPosition,self.LfootPosition)
        self.JointPositions = [ self.WaistX,self.WaistY,self.WaistZ,
                            self.RShX,self.RShY,self.RShZ,self.REB,
                            self.LShX,self.LShY,self.LShZ,self.LEB,
                            self.RHipX,self.RHipY,self.RHipZ,self.RKneeX,self.RAnkleX,self.RAnkleY,
                            self.LHipX,self.LHipY,self.LHipZ,self.LKneeX,self.LAnkleX,self.LAnkleY ]


        ####################################################################################

        #Initialize server
        self.physicsClient = p.connect(getattr(p, "GUI" if self.CONFIG["clientGUI"] else "DIRECT"))#p.GUI / p.DIRECT

        p.configureDebugVisualizer( p.COV_ENABLE_RENDERING, 0)

        #Load scenario (if self.CONFIG["scenario"] == -1 load one scenario per thread)
        self.scenario = self.CONFIG["scenario"]
        self.load_map(0)

        #======================================================= Change stuff and save state in RAM
            
        #Lock joints in place
        numJoints = p.getNumJoints(self.COMAN.ID)
        for j in range(numJoints):
            p.setJointMotorControl2( bodyIndex = self.COMAN.ID, jointIndex = j, controlMode = p.POSITION_CONTROL, targetPosition = 0 )

        #fix undefined joints
        for i in range(p.getNumJoints(self.COMAN.ID)):
            if(p.getDynamicsInfo(self.COMAN.ID, i)[0] == 1):
                p.changeDynamics(self.COMAN.ID, i, mass=0.001, localInertiaDiagonal=[0.0,0.0,0.0])

        #feet traction
        p.changeDynamics(self.COMAN.ID, 54, lateralFriction=1) #LAnkSag
        p.changeDynamics(self.COMAN.ID, 40, lateralFriction=1) #RAnkSag

        #Save state in RAM for future resets
        self.resetState = p.saveState()

        #=======================================================

        self.CoP_L = []
        self.CoP_R = []

        self.twalk0 = 0
        self.gtime = 0
        self.idx = 0

        self.episode_number = 0
        self.time_since_last_push = 10
        self.action = None #np.zeros(self.action_size, np.float32)

        #------------------------------------------ WALK/RUN

        self.StepX= 0.0
        self.StepY= 0.0
        self.StepTheta = 0.0
        self.figure8_stepsize = 0.035
        #------------------------------------------ PUSH
        self.ENABLE_PERTURBATIONS = self.CONFIG["perturbations"] #flags


        self.push_min_force_norm = 100 #400
        self.push_max_force_norm = 300 #500

        self.push_min_cooldown = 5.0 / self.SamplingTime #2.5 / self.SamplingTime
        self.push_max_cooldown = 6.0 / self.SamplingTime #3.0 / self.SamplingTime
        self.push_countdown = np.random.randint(self.push_min_cooldown, self.push_max_cooldown )
        self.push_duration = 5
        
        self.arrow_life = 0
        self.arrow_vec = None
        self.arrow_ID = -1

        self.radial_push_angle = 0
        self.radial_push_force = 0

        self.cinematic_pushNo = -1

        #------------------------------------------ Define objective

        self.COMAN.update()
        self.objective = self.Objective(self)

        #------------------------------------------


        #self.UpdateZMP(init=True) # just to get the number of observations
        self.action_size = 22
        self.action_space = spaces.Box(low=np.array([0] * self.action_size), high=np.array([0] * self.action_size), dtype="float32")
        observation_size = len(self.observe())
        self.observation_space = spaces.Box(low=np.zeros(observation_size), high=np.zeros(observation_size), dtype="float32")

        

        #------------------------------------------ GUI Configuration

        if(self.CONFIG["clientGUI"]):
            self.param_force = p.addUserDebugParameter(paramName="Force", rangeMin=0, rangeMax=1000, startValue=280)
            self.param_angle = p.addUserDebugParameter(paramName="Angle", rangeMin=-180, rangeMax=180, startValue=0)
            self.param_apply = p.addUserDebugParameter(paramName="Apply", rangeMin=1, rangeMax=0,   startValue=1)
            self.param_apply_val = 1
            self.param_switchf = p.addUserDebugParameter(paramName="Toggle Force", rangeMin=1, rangeMax=0,   startValue=1)
            self.param_switchcam = p.addUserDebugParameter(paramName="Toggle Camera", rangeMin=1, rangeMax=0,   startValue=1)
            #p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) #stop rendering
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0)
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,0)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
            p.setDebugObjectColor(self.COMAN.ID,15,objectDebugColorRGB=[1,0,0])
            p.configureDebugVisualizer( p.COV_ENABLE_RENDERING, 1)


    def reset(self):
        p.restoreState(stateId=self.resetState)
        self.COMAN.update()
        self.episode_number += 1

        if(self.CONFIG["deterministicPushes"]):# and self.episode_number == 1: #changed for tilting platform sync
            np.random.seed(self.episode_number)
            

        self.push_countdown = np.random.randint(self.push_min_cooldown, self.push_max_cooldown )
        self.push_duration = 5


        self.FirstStepIsRight = 0#np.random.randint(0,2)
        self.Zleg = -0.385
        self.NewCommand = 0
        self.NewStepX = 0
        self.NewStepY = 0
        self.NewStepTheta = 0
        self.NewStepTime = self.StepTime

            

        self.energy_penalty_sum = 0
        self.energy_penalty_sum_cnt = 0
        self.WALK_DURATION_MAX = 5
        self.walk_duration = self.WALK_DURATION_MAX / self.SamplingTime


        self.twalk0 = 0
        self.gtime = 0
        self.gstep = 0
        self.rew_stats = np.zeros(7, np.float32)
        self.time_stats = time.time()

        self.idx = 0
        self.time_since_last_push = 10

        self.action = None #np.zeros(self.action_size, np.float32)

        self.UpdateZMP(init=True)

        #Generate walking trajectory
        self.StepX= 0.0
        self.StepY= 0.0
        self.StepTheta = 0.0
        self.SwingStepZ = 0.03

        self.tiltplat_joints = np.array([0,0],np.float32)


        self.Zleg = -0.365
        self.update_time_param()
        self.update_move_param()   
        self.generate_cpg(0)

        return self.observe()


    class Objective():

        def __init__(self, gym):
            self.gym = gym
            self.x = 0
            self.y = 0
            self.xy0 = np.array([0,0,0], np.float32)
            self.t = 180
            self.conquer_time = 1 / self.gym.SamplingTime
            self.last_dist = self.get_2d_dist()
            self.last_ang_dist = abs(Ut.normalize_deg(-self.gym.COMAN.ori_z_torso_deg))
            self.current_conquer_time = 0


        def update(self, x, y, t, conquer_time):

            self.x = x
            self.y = y
            self.xy0 = np.array([x,y,0], np.float32)
            self.t = t
            self.conquer_time = conquer_time
            self.last_dist = self.get_2d_dist()
            self.last_ang_dist = abs(Ut.normalize_deg(t - self.gym.COMAN.ori_z_torso_deg))
            self.current_conquer_time = 0



        def get_2d_rel_pos(self):
            '''
            Get 2D relative positions (discards torso inclination)
            '''
            x,y,_ = Ut.world_to_local_transform([self.x, self.y, 0], [0,0,0,1], self.gym.COMAN.pos_3d_torso_proj, self.gym.COMAN.ori_4d_torso_z_only)[0]

            return np.array([x,y],np.float32)

        def get_2d_dist(self):
            return np.linalg.norm(self.get_2d_rel_pos())


        def get_agent_obs(self):
            '''
            Saturate agent obs (to simplify training)
            :return: [x,y],t
            '''
            rel_pos_2d = np.clip(self.get_2d_rel_pos(),-2,2) #saturate relative position (max: 2 meters)
            rel_theta = np.clip(Ut.normalize_deg(self.t - self.gym.COMAN.ori_z_torso_deg), -45, 45)

            return rel_pos_2d, rel_theta


    def create_new_objective(self):

        max_dist = 6

        x = self.COMAN.pos_3d_torso[0] + np.random.rand()*max_dist*2-max_dist
        y = self.COMAN.pos_3d_torso[1] + np.random.rand()*max_dist*2-max_dist
        
        t = np.random.rand()*360.0 - 180.0

        conquer_time = 1.5 / self.SamplingTime

        self.objective.update(x,y,t,conquer_time)

    def step(self, ppo_action):
        """
        Step function

        :param ppo_action: PD[10], zleg, stime, arms[8]  (Old version)
        :param ppo_action: PD[10], zleg, arms[8], xstep, ystep, tstep
        :return: obs, reward, terminal, info
        """     

        master_control = 0.0
        ppo_action *= master_control

        self.NewStepX     = 0.0     #ppo_action[19] * 0.5
        self.NewStepY     = 0       #ppo_action[20] * 0.3
        self.NewStepTheta = 0   #ppo_action[21] * 15.0
        self.NewCommand   = 1
        ppo_action       *= 0 * master_control


        if self.action is None:
            self.action = ppo_action
        else:
            self.action = ppo_action * 0.1 + self.action * 0.9

        self.gtime += self.SamplingTime
        self.gstep += 1
        self.UpdateZMP()

        if self.walk_duration > 0:
            self.walk_duration -= 1

        #------------------------------------------- Control omniwalk without constraints or lp-filter (using unfiltered actions from ppo)
        # self.NewStepX = ppo_action[19] * 0.5
        # self.NewStepY = ppo_action[20] * 0.3
        # self.NewStepTheta = ppo_action[21] * 15.0
        # self.NewCommand = 1

        #------------------------------------------- Control PD, zleg
        self.updateOmniJoints_CPG()
        #------------------------------------------- Control Arms
        pi_1 = math.pi
        pi_2 = math.pi/2
        pi_3 = math.pi/3
        pi_6 = math.pi/6
        zero = 0.0

        #Change arms with PPO and saturate resulting action
        self.JointPositions[3]  = np.clip(self.JointPositions[3]  + self.action[11] * pi_3, -pi_3, pi_3 ) # self.RShX 
        self.JointPositions[4]  = np.clip(self.JointPositions[4]  + self.action[12] * pi_6, -pi_3, zero ) # self.RShY 
        self.JointPositions[5]  = np.clip(self.JointPositions[5]  + self.action[13] * pi_3, -pi_3, pi_3 ) # self.RShZ 
        self.JointPositions[6]  = np.clip(self.JointPositions[6]  + self.action[14] * pi_2, -pi_1, zero ) # self.REB 
        self.JointPositions[7]  = np.clip(self.JointPositions[7]  + self.action[15] * pi_3, -pi_3, pi_3 ) # self.LShX 
        self.JointPositions[8]  = np.clip(self.JointPositions[8]  + self.action[16] * pi_6,  zero, pi_3 ) # self.LShY 
        self.JointPositions[9]  = np.clip(self.JointPositions[9]  + self.action[17] * pi_3, -pi_3, pi_3 ) # self.LShZ 
        self.JointPositions[10] = np.clip(self.JointPositions[10] + self.action[18] * pi_2, -pi_1, zero ) # self.LEB 

        #------------------------------------------- Apply action to all joints
        for j in range(len(self.JointIDOrder)):
            if(j in range(3,11)): f=20
            else: f=60
            p.setJointMotorControl2(bodyIndex=self.COMAN.ID, jointIndex=self.JointIDOrder[j], controlMode=p.POSITION_CONTROL,
                                                targetPosition=self.JointPositions[j], force=f)

        p.stepSimulation()
        self.COMAN.update()

        #------------------------------------------- Change camera
        if(self.CONFIG["clientGUI"]):
            w = 0.98

            robotWorldOri = Ut.local_to_world_transform([0,0,0],[0,0,0,1],self.COMAN.pos_3d_torso,self.COMAN.ori_4d_torso)[1] 
            yaw = np.degrees(p.getEulerFromQuaternion(robotWorldOri)[2])+80 #yaw
            if self.gtime == self.SamplingTime: w=0

            pitch = 10 if self.scenario == 4 else -15
            dist = 1.4
            rotSpeed = 2

            Ut.change_bullet_camera(dist, 1, self.gtime*rotSpeed + yaw, 1, pitch, 1, [self.COMAN.pos_3d_torso[0],self.COMAN.pos_3d_torso[1], 0.5 ], w)

        #------------------------------------- energy penalty

        #PD[10], zleg, arms[8]
        abs_ppo_action = np.abs(ppo_action)

        for i in range(2):      abs_ppo_action[i] *= 0.5    #PD.wx PD.wy
        for i in range(2, 6):   abs_ppo_action[i] *= 0.25   #PD ankle
        for i in range(6, 10):  abs_ppo_action[i] *= 0.25   #PD hip
        for i in range(11, 19): abs_ppo_action[i] *= 0.25   #Arms

        en_penalty = np.sum(abs_ppo_action) / 6 #6 Groups: PD:rotation/ankle/hip, zleg, arms(it's not fair, so the arms are 2 groups now)

        #------------------------------------- basic objective routine (can be used in different contexts)

        obj_rel_t = abs(Ut.normalize_deg(self.objective.t-self.COMAN.ori_z_torso_deg))
        obj_dist = self.objective.get_2d_dist()

        if obj_dist < 0.1: #Conquer objective
            self.objective.current_conquer_time += 1
            rew_dist_bonus = 1
            if self.objective.current_conquer_time > self.objective.conquer_time: self.create_new_objective()
        else:
            self.objective.current_conquer_time = 0
            rew_dist_bonus = 0

        rew_ang_bonus = 1 if abs(obj_rel_t) < 3 else 0 #reward agent for being close to the objective rotation

        rew_distance = self.objective.last_dist - obj_dist #reward agent for getting closer to objective position
        self.objective.last_dist = obj_dist #save last distance

        rew_ang_dist = self.objective.last_ang_dist - obj_rel_t #reward agent for getting closer to objective rotation
        self.objective.last_ang_dist = obj_rel_t #save last angular distance
        
        #------------------------------------- reward, terminal, camera

        rew_distance *= 200.0 #theoretic max: 1
        rew_ang_dist *= 6.666 #theoretic max: 1
        rew_dist_bonus *= 0.5 #max: 0.5
        rew_ang_bonus *= 0.5  #max: 0.5

        reward = rew_distance + rew_ang_dist + rew_dist_bonus + rew_ang_bonus - 0.1*en_penalty

        terminal = (self.COMAN.pos_z_torso < 0.37 or self.COMAN.flag_hands_touch_ground)

        if(self.CONFIG["clientGUI"]):
            if (self.gstep%100==0):
                s = self.rew_stats
                performance = self.SamplingTime / ((time.time() - self.time_stats) / s[0] )
                self.time_stats = time.time()
                print("Rew: {:6.3f} -En: {:6.3f} +Dist: {:6.3f}+{:6.3f} +Ang: {:6.3f}+{:6.3f}   {:6.3f}%".format(s[1]/s[0], s[2]/s[0], s[3]/s[0], s[4]/s[0], s[5]/s[0], s[6]/s[0], performance))
                self.rew_stats = np.zeros(7, np.float32)
                p.addUserDebugLine(self.COMAN.pos_3d_torso, self.objective.xy0, [1,0,0], lineWidth=3, lifeTime=1)
                rot_vec = np.array([math.cos(np.radians(self.objective.t)), math.sin(np.radians(self.objective.t)), 0.0], np.float32)
                p.addUserDebugLine(self.COMAN.pos_3d_torso, self.COMAN.pos_3d_torso+rot_vec, [0,1,0], lineWidth=3, lifeTime=1)
            else:
                self.rew_stats[0] += 1
                self.rew_stats[1] += reward
                self.rew_stats[2] += en_penalty
                self.rew_stats[3] += rew_distance
                self.rew_stats[4] += rew_dist_bonus
                self.rew_stats[5] += rew_ang_dist
                self.rew_stats[6] += rew_ang_bonus


        return self.observe(), reward*0.01, terminal, "INFO"

        
        

    def observe(self):

        ob=[]
        symOb=[0]*len(self.JointIDOrder)*3

        #------------------------------------------------------------------ Joints

        #Joint        Symmetry       Joint Effect with positive torque
        #(0)waistX   - same          (tilt front)
        #(1)waistY   - invert        (tilt right)
        #(2)waistZ   - invert        (rotate left)
        #(3)RShX     = (7)LShX       (shoulder back)
        #(4)RShY     = -(8)LShY      (shoulder inside)
        #(5)RShZ     = -(9)LShZ      (shoulder thumb inside)
        #(6)REB      = (10)LEB       (elbow extend)
        #(7)LShX     = (3)RShX       (shoulder back)
        #(8)LShY     = -(4)RShY      (shoulder outside)
        #(9)LShZ     = -(5)RShZ      (shoulder thumb outside)
        #(10)LEB     = (6)REB        (elbow extend)
        #(11)RHipX   = (17)LHipX     (hip back)
        #(12)RHipY   = -(18)LHipY    (hip inside)
        #(13)RHipZ   = -(19)LHipZ    (hip toes inside)
        #(14)RKneeX  = (20)LKneeX    (knee back)
        #(15)RAnkleX = (21)LAnkleX   (foot down)
        #(16)RAnkleY = -(22)LAnkleY  (foot twist outside)
        #(17)LHipX   = (11)RHipX     (hip back)
        #(18)LHipY   = -(12)RHipY    (hip outside)
        #(19)LHipZ   = -(13)RHipZ    (hip toes outside)
        #(20)LKneeX  = (14)RKneeX    (knee back)
        #(21)LAnkleX = (15)RAnkleX   (foot down)
        #(22)LAnkleY = -(16)RAnkleY  (foot twist inside)

        jointSymmetry = [0,-1,-2,7,-8,-9,10,3,-4,-5,6,17,-18,-19,20,21,-22,11,-12,-13,14,15,-16]
        assert len(jointSymmetry) == len(self.JointIDOrder)

        for i in range(len(self.JointIDOrder)):
            posi, vel, _, torque = p.getJointState(self.COMAN.ID,self.JointIDOrder[i]) #pos, vel, reaction force, applied torque
            ob.append(posi)    #pos
            ob.append(vel)     #vel
            ob.append(torque)  #torque

            indx = jointSymmetry[i]
            if indx >= 0:
                symOb[indx*3]   = posi
                symOb[indx*3+1] = vel
                symOb[indx*3+2] = torque
            else:
                symOb[-indx*3]   = -posi
                symOb[-indx*3+1] = -vel
                symOb[-indx*3+2] = -torque

        #------------------------------------------------------------------ Sensors

        #Sensor     Symmetry   Axis positive direction
        #LinSpeedX - same      (front)
        #LinSpeedY - invert    (left)
        #LinSpeedZ - same      (up)
        #AngSpeedX - invert    (tilt right)
        #AngSpeedY - same      (tilt forward)
        #AngSpeedZ - invert    (rotate left)
        #Height    - same      (up)
        #GravityX  - same      (front)
        #GravityY  - invert    (left)
        #GravityZ  - same      (up)
        #LCoPX     = RCoPX     (front)
        #LCoPY     = -RCoPY    (outside)
        #RCoPX     = LCoPX     (front)
        #RCoPY     = -LCoPY    (inside)
        #LFootF    = RFootF    (down)
        #RFootF    = LFootF    (down)   

        linVel = self.COMAN.rLinVel_3d_torso
        angVel = self.COMAN.rAngVel_3d_torso

        ob.extend(linVel)
        symOb.extend([  linVel[0],-linVel[1],linVel[2]  ])
        ob.extend(angVel)
        symOb.extend([  -angVel[0],angVel[1],-angVel[2]  ])

        ob.append(self.COMAN.pos_z_torso)    #base height
        symOb.append(self.COMAN.pos_z_torso) #base height

        gravity = Ut.world_to_local_transform([0,0,-1],[0,0,0,1],[0,0,0],self.COMAN.ori_4d_torso)[0] #gravity vector in local coordinates
        ob.extend(gravity)   
        symOb.extend([  gravity[0],-gravity[1],gravity[2]  ])

        lcop, rcop, lcopf, rcopf = self.COMAN.pos_3d_lfoot_CoP, self.COMAN.pos_3d_rfoot_CoP, self.COMAN.force_3d_lfoot_CoP, self.COMAN.force_3d_rfoot_CoP
        ob.extend(lcop[0:2]) #Center of pressure L
        ob.extend(rcop[0:2]) #Center of pressure R
        ob.append(lcopf) #LFoot Force
        ob.append(rcopf) #RFoot Force

        symOb.extend([  rcop[0],-rcop[1]  ]) #Center of pressure R
        symOb.extend([  lcop[0],-lcop[1]  ]) #Center of pressure L
        symOb.append(rcopf) #RFoot Force
        symOb.append(lcopf) #LFoot Force

        #------------------------------------------------------------------ Commands
        #Sensor     Symmetry   Axis positive direction
        #ObjRelX     - same      (front)
        #ObjRelY     - invert    (side)
        #ObjRelT     - invert    (z)

        [cx,cy],ct = self.objective.get_agent_obs()

        ob.append(cx)
        ob.append(cy)
        ob.append(ct)

        symOb.append(cx)
        symOb.append(-cy)
        symOb.append(-ct)

        #------------------------------------------------------------------ Counter (to learn phase of gait but also value due to sparse reward)
        ob.append(self.idx)
        symOb.append(self.idx)


        return np.asarray(ob, dtype=np.float32)

    
    def close(self):
        p.disconnect()

if __name__ == "__main__":
    agent = HhumanoidGym()
    agent.reset()
    while True:
        obs, reward, terminal, _ = agent.step(np.zeros(30))
        if terminal:
            agent.reset()