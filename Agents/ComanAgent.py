from numpy.core.defchararray import array
import pybullet as p
import pybullet_data as pd
import numpy as np
from gym import spaces
import matplotlib.pyplot as plt

import os, json
import random
import platform
import math
import time

from Walking.GenrateWalkingTrajectories import GenrateWalkingTrajectories

__cwd__ = os.path.realpath( os.path.join(os.getcwd(), os.path.dirname(__file__)))

class Coman():
    def Test(self):
        pass

    def __init__(self, *args, **kwargs):

        __cwd__ = os.path.realpath( os.path.join(os.getcwd(), os.path.dirname(__file__)))

        try:
            with open(os.path.join(__cwd__, 'config.txt'),'r') as fp:
                self.CONFIG = json.loads(fp.read()) 
        except:
            assert False, "You need to have a configuration file with the name of your host machine: config_" + platform.node() + '.txt'

        # self.thread_no = args[0]
        # self.mpi_size = args[1]
        # self.network_name = args[2]
        self.thread_no    = 0
        self.mpi_size     = 0
        self.network_name = 0

        ########################################################### Initialize variables
        self.SamplingTime=0.005
        self.FootHeelToe=0
        self.DistanceBetweenFeet = 0.17
        self.NumberOfStep=10

        self.StepTime = 0.30
        self.StopForOneStep =  0
        self.NewStepTime = self.StepTime

        self.NewCommand = 0
        self.NewStepX = 0
        self.NewStepY = 0
        self.NewStepTheta = 0
        self.SwingStepZ = 0.03

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

        self.Zleg = -0.365
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
        self.physicsClient = p.connect(getattr(p, self.CONFIG["clientMode"]))#p.GUI / p.DIRECT

        p.configureDebugVisualizer( p.COV_ENABLE_RENDERING, 0)

        #Load scenario (if self.CONFIG["scenario"] == -1 load one scenario per thread)
        self.scenario = self.CONFIG["scenario"]
        
        if self.scenario == -1: #walk in place + tilting platform
            self.scenario = (self.thread_no % 2) * 3

        if self.scenario == 0: #flat terrain
            self.load_map(0)
        elif self.scenario == 1: #balancing platform
            self.load_map(1)
        elif self.scenario == 2: #rotating platform    
            self.load_map(2)
        elif self.scenario == 3: #tilting platform (changes gravity instead)
            self.load_map(0) 
        elif self.scenario == 4: #uneven terrain
            self.load_map(3) 
        elif self.scenario == 5: #flat radial test
            self.load_map(0)
        elif self.scenario == 6: #uneven radial test
            self.load_map(3) 
        elif self.scenario == 7: #tilting platform
            self.load_map(4)
        elif self.scenario == 8: #cinematic push
            self.load_map(0)
        elif self.scenario == 9: #obstacle course
            self.load_map(5)
        elif self.scenario == 10: #obstacle course
            self.load_map(0)
            
        
        if(self.CONFIG["clientMode"]=="GUI"):
            time.sleep(0.123) #wait for recording

        if self.scenario == 5 or self.scenario == 6:

            self.pushlog = "radialpush_" + self.network_name + "_" + str(self.scenario) + "_th" + str(self.thread_no) + ".csv"
            with open(self.pushlog, "w") as txt:  #creates empty file with header
                txt.write("Angle,Force\n")
            self.CONFIG["deterministicPushes"]=0 #disables some useless prints
            

        #Lock joints in place
        numJoints = p.getNumJoints(self.ComanId)
        for j in range(numJoints):
            p.setJointMotorControl2( bodyIndex = self.ComanId, jointIndex = j, controlMode = p.POSITION_CONTROL, targetPosition = 0 )


        self.CoP_L = []
        self.CoP_R = []
        # p.enableJointForceTorqueSensor(self.ComanId,42,1)
        # p.enableJointForceTorqueSensor(self.ComanId,56,1)

        self.twalk0 = 0
        self.gtime = 0


        self.posAndori = p.getBasePositionAndOrientation(self.ComanId)
        self.pos = self.posAndori[0]
        self.ori = p.getEulerFromQuaternion(self.posAndori[1])
        self.posp = self.pos
        self.orip = self.ori

        self.idx = 0

        #------------------------------------------ WALK/RUN
        self.walk = True
        self.StepX= 0.0
        self.StepY= 0.0
        self.StepTheta = 0.0
        self.figure8_stepsize = 0.035

        if self.CONFIG["clientMode"]!="GUI":
            self.walk_trajectory_No = 0 #random walk
            
            
        elif self.scenario == 10:
            self.walk_trajectory_No = 4 #input command
            
        elif self.scenario == 9:
            self.walk_trajectory_No = 2 #obstacle course
        elif self.scenario == 10: #set to True to use figure 8
            self.walk_trajectory_No = 1 #figure 8
            self.draw_trajectory()
        else:
            self.walk_trajectory_No = 0 #random walk

        #------------------------------------------ PUSH
        self.ENABLE_PERTURBATIONS = self.CONFIG["perturbations"] #flags

        if self.walk:
            self.push_min_force_norm = 300 #400
            self.push_max_force_norm = 400 #500
        else:
            self.push_min_force_norm = 650
            self.push_max_force_norm = 850
        self.push_min_cooldown = 6.0 / self.SamplingTime #2.5 / self.SamplingTime
        self.push_max_cooldown = 7.0 / self.SamplingTime #3.0 / self.SamplingTime
        self.push_countdown = np.random.randint(self.push_min_cooldown, self.push_max_cooldown )
        self.push_duration = 5
        ang = np.random.rand()*2*math.pi
        
        self.arrow_life = 0
        self.arrow_vec = None
        self.arrow_ID = -1

        self.radial_push_angle = 0
        self.radial_push_force = 0

        self.cinematic_pushNo = -1

        #------------------------------------------ TILT
        self.tilt = None
        self.new_tilt = None
        self.tilt_duration = None

        #------------------------------------------ OBS/AC NOISE
        self.obsNoise = 0.0 * self.thread_no
        self.acsNoise = 0

        #------------------------------------------


        self.UpdateZMP(init=True) # just to get the number of observations
        self.action_size = 20
        self.action_space = spaces.Box(low=np.array([0] * self.action_size), high=np.array([0] * self.action_size), dtype="float32")
        observation_size = len(self.observe())
        self.observation_space = spaces.Box(low=np.zeros(observation_size), high=np.zeros(observation_size), dtype="float32")

        for i in range(p.getNumJoints(self.ComanId)):
            if(p.getDynamicsInfo(self.ComanId, i)[0] == 1):
                #print(p.getJointInfo(self.ComanId, i))
                p.changeDynamics(self.ComanId, i, mass=0, localInertiaDiagonal=[0.0,0.0,0.0])
                #print("Current mass: ", p.getDynamicsInfo(self.ComanId, i)[0])
            #if(p.getJointInfo(self.ComanId, i)[11] == 4):
            #    p.changeDynamics(self.ComanId, i, maxJointVelocity=1)

        p.changeDynamics(self.ComanId, 54, lateralFriction=1) #LAnkSag
        p.changeDynamics(self.ComanId, 40, lateralFriction=1) #RAnkSag

        self.episode_number = 0
        self.penalty_pos = [0,0]
        self.time_since_last_push = 10
        self.action = np.zeros(self.action_size, np.float32)

        if(self.CONFIG["clientMode"]=="GUI"):
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
            p.setDebugObjectColor(self.ComanId,15,objectDebugColorRGB=[1,0,0])
            p.configureDebugVisualizer( p.COV_ENABLE_RENDERING, 1)

        self.update_time_param()
        self.update_move_param()   

    
    
    #######################################################
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

        #Time = (int(time*1000))  % int(2 * self.StepTime * 1000)
        #Time = t % self.StepTime 
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
        
            
        #update_param_balance()

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


        

    def step_cpg(self):
        pass        
    #########################################################

    def load_map(self,mapX):

        p.setGravity(0,0,-9.81)
        self.FloorId = -1

        if(mapX==0):
            z_difference = 0
            stabilization_steps = 100
        if(mapX==1):  
            z_difference = self.create_platform()
            stabilization_steps = 5
        elif(mapX==2):  
            z_difference = self.create_platform_rotating()
            stabilization_steps = 5
        elif(mapX==3):
            z_difference = self.create_uneven_terrain()
            stabilization_steps = 1
        elif(mapX==4):   
            z_difference = self.create_tilting_platform()
            stabilization_steps = 20
        elif(mapX==5):   
            z_difference = self.create_obstacle_course()
            stabilization_steps = 100
            

        if self.FloorId == -1:
            p.setAdditionalSearchPath(pd.getDataPath())
            self.FloorId = p.loadURDF("plane.urdf",[0,0,-z_difference])
            #self.FloorId = p.loadURDF("plane_ice.urdf",[0,0,-z_difference])

        p.setAdditionalSearchPath("")
        # self.ComanId = p.loadURDF(__cwd__ + "/models/coman/model_org_" + platform.node() + ".urdf",[0,0,0.465+0.05])
        self.ComanId = p.loadURDF(__cwd__ + "/models/coman/model_org.urdf",[0,0,0.465+0.05])

        #Stabilize robot for some iterations
        p.setTimeStep(self.SamplingTime)
        for _ in range(stabilization_steps): 
            p.stepSimulation()

        #Save state in RAM for future resets
        self.resetState = p.saveState()
    

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

    def SAT(self, inp, UB,LB):
        if inp>UB:
            return UB
        elif inp<LB:
            return LB
        else:
            return inp

    def PDControl(self, err, derr,PGain, DGain,Saturation):
        return self.SAT((PGain*(err)) + (DGain*derr),Saturation[0],Saturation[1])

    def getFeetCoP(self):

        #Contact points
        lCP = p.getContactPoints(bodyA=self.ComanId, bodyB=self.FloorId, linkIndexA=54) #LAnkle
        rCP = p.getContactPoints(bodyA=self.ComanId, bodyB=self.FloorId, linkIndexA=40) #RAnkle

        #Center of Pressure (default:[0,0,0])
        lCoP = np.zeros(3,  np.float32)
        rCoP = np.zeros(3,  np.float32)

        #Total force per foot
        lCoPforce = 0
        rCoPforce = 0
        
        #Compute left side if foot is touching the ground
        if lCP:
            lCPPos = np.array([i[5] for i in lCP])
            lCPforce = np.array([i[9] for i in lCP])
            lCoPforce = np.sum(lCPforce)

            if lCoPforce:
                #Compute Center of Pressure (world coordinates)
                for i,j in zip(lCPPos, lCPforce): lCoP += i*j
                lCoP /= lCoPforce

                #Transform to local coordinates
                lfwpos, lfwori = p.getLinkState(bodyUniqueId=self.ComanId, linkIndex=54)[0:2]
                inv_lfwpos, inv_lfwori = p.invertTransform(lfwpos, lfwori)
                lCoP, _ = p.multiplyTransforms(inv_lfwpos, inv_lfwori, lCoP, [0,0,1,0])

        #Compute right side if foot is touching the ground
        if rCP:
            rCPPos = np.array([i[5] for i in rCP])
            rCPforce = np.array([i[9] for i in rCP])
            rCoPforce = np.sum(rCPforce)

            if rCoPforce:
                #Compute Center of Pressure (world coordinates)
                for i,j in zip(rCPPos, rCPforce): rCoP += i*j
                rCoP /= rCoPforce

                #Transform to local coordinates
                rfwpos, rfwori = p.getLinkState(bodyUniqueId=self.ComanId, linkIndex=40)[0:2]
                inv_rfwpos, inv_rfwori = p.invertTransform(rfwpos, rfwori)
                rCoP, _ = p.multiplyTransforms(inv_rfwpos, inv_rfwori, rCoP, [0,0,1,0])

        return lCoP, rCoP, lCoPforce, rCoPforce #x: positive=front, y: positive=left, z: positive=up

    def UpdateZMP(self, init=False):

        lCoP, rCoP, _, _ = self.getFeetCoP() #x: positive=front, y: positive=left, z: positive=up

        xCopL = lCoP[1]
        yCopL = lCoP[0]
        xCopR = rCoP[1]
        yCopR = rCoP[0]

        if (init):
            self.CoP_L = [xCopL,yCopL]
            self.CoP_R = [xCopR,yCopR]
        else:
            a = 0.975
            b = 0.025
            self.CoP_L = [(a*self.CoP_L[0]+b*xCopL),(a*self.CoP_L[1]+b*yCopL)]
            self.CoP_R = [(a*self.CoP_R[0]+b*xCopR),(a*self.CoP_R[1]+b*yCopR)]

            #Reset each foot immediately if it's in the air
            if(np.all(lCoP==0)): self.CoP_L = [xCopL, yCopL]
            if(np.all(rCoP==0)): self.CoP_R = [xCopR, yCopR]


    def change_camera(self, dist, distfilter, yaw, yawfilter, pitch, pitchfilter, pos, posfilter):

        camState = p.getDebugVisualizerCamera()

        _po = [posfilter*camState[11][0]+(1-posfilter)*pos[0], 
              posfilter*camState[11][1]+(1-posfilter)*pos[1], 
              posfilter*camState[11][2]+(1-posfilter)*pos[2]]

        _d = distfilter*camState[10]+(1-distfilter)*dist

        while True:
            if yaw > camState[8] + 180: yaw -= 360
            elif yaw < camState[8] - 180: yaw += 360
            else: break

        _y = yawfilter*camState[8]+(1-yawfilter)*yaw

        _p = pitchfilter*camState[9]+(1-pitchfilter)*pitch

        p.resetDebugVisualizerCamera( cameraDistance=_d, cameraYaw=_y, cameraPitch=_p, cameraTargetPosition=_po)


    def step(self, ppo_action):
        """
        Step function

        :param ppo_action: PD[10], zleg, stime, arms[8]
        :return: obs, reward, terminal, info
        """     

        # for i in range(len(ppo_action)):
        #     ppo_action[i] *= (1 + np.random.rand()*(self.acsNoise*2) - self.acsNoise)
        
        
        master_control = 0.2
        if self.scenario == 8 and (self.cinematic_pushNo < 8 or (self.cinematic_pushNo == 8 and self.push_countdown < 750)): #enable in last push until stability is achieved
            master_control = 0.0

        ppo_action *= master_control

        self.action = ppo_action * 0.10 + self.action * 0.90


        self.gtime +=                self.SamplingTime
        self.time_since_last_push += self.SamplingTime

        self.posAndori = p.getBasePositionAndOrientation(self.ComanId)
        self.pos = self.posAndori[0]
        self.ori4 = self.posAndori[1]
        self.ori = p.getEulerFromQuaternion(self.posAndori[1])

        self.commandPenalty = None #penalize robot for deviating from command

        self.UpdateZMP()

        if self.walk_duration > 0:
            self.walk_duration -= 1

        #--------------------------------------- Manual push
        if(self.CONFIG["clientMode"]=="GUI"):
            val = p.readUserDebugParameter(self.param_apply)
            if val!=self.param_apply_val:
                self.param_apply_val = val
                f = p.readUserDebugParameter(self.param_force) * 5 #because only applied for 1 tstep
                ang = p.readUserDebugParameter(self.param_angle) * math.pi / 180.0
                fvec = [math.cos(ang)*f, math.sin(ang)*f, 0]
                fpos = np.add(self.pos, [0,0,0.3])

                print("Manual push: f=", fvec, "d=1")
                p.applyExternalForce(self.ComanId, -1, fvec, fpos, p.WORLD_FRAME)

                self.arrow_life = 0.2 / self.SamplingTime
                self.arrow_vec = np.divide(fvec, 600)

        #--------------------------------------- External perturbations (depends on scenario)
        terminal=False
        
        # if self.ENABLE_PERTURBATIONS:
        #     if self.scenario == 0:
        #         self.auto_push_routine() #apply pushes automatically
        #     elif self.scenario == 3:
        #         self.auto_gravity_routine()
        #     elif self.scenario == 4:  #uneven terrain
        #         self.auto_push_routine() #apply pushes automatically
        #     elif self.scenario == 5 or self.scenario == 6:  #flat / uneven (radial test)
        #         terminal = self.radial_push_routine()
        #     elif self.scenario == 7: #tilting platform
        #         terminal = self.auto_tilt_routine()
        #     elif self.scenario == 8: #cinematic push
        #         self.cinematic_push_routine()
        #     elif self.scenario == 9: #obstacle course
        #         self.auto_push_routine()
        #     elif self.scenario == 10: #figure_8 
        #         self.auto_push_routine()
                

        # #--------------------------------------- Push force arrow
        # if self.arrow_life > 0:
        #     self.arrow_life -= 1
        #     forcepos = np.add(self.pos, [0,0,0.3])
        #     endpos = self.arrow_vec+np.array(forcepos)

        #     self.arrow_ID = -1

        #     if self.arrow_life % 3 == 0:
        #         self.arrow_ID = p.addUserDebugLine(lineFromXYZ=forcepos, lineToXYZ=endpos, 
        #                     lineColorRGB=[1,0,0], lineWidth=2, lifeTime=0.05)#, replaceItemUniqueId=self.arrow_ID)

        #     if self.arrow_life == 0: self.arrow_ID = -1


            #p.addUserDebugText(text=str(self.push_force),textPosition=robotpos+[0,0,1], lifeTime=0.5)


        # if (WriteData==1):
        #     file.write(str(gtime)+","+str(pos[0])+","+str(pos[1])+","+str(pos[2])+",")
        #     file.write(str(ori[0])+","+str(ori[1])+","+str(ori[2])+",") 
        #     file.write(str(CoP_L[0])+","+str(CoP_L[1])+","+str(CoP_R[0])+","+str(CoP_R[1])) 
        #     file.write("\n")

        if (abs(self.gtime-self.twalk0) > (2*self.StepTime)-(self.SamplingTime/2)):
            self.twalk0 = self.gtime
            self.StopForOneStep = 0

            if self.walk_trajectory_No == 0:
                if self.walk_duration == 0:
                    self.generate_random_walk_command()
                    self.walk_duration = self.WALK_DURATION_MAX / self.SamplingTime
                else:
                    self.apply_walk_command()
            elif self.walk_trajectory_No == 1:
                self.generate_8_figure_walk_command()
            elif self.walk_trajectory_No == 2:
                self.generate_obstacle_course_walk_command()
            
            
            

            if (self.NewCommand): 
                self.StepTheta = self.NewStepTheta
                self.StepX = self.NewStepX*math.cos(math.radians(self.StepTheta))-self.NewStepY*math.sin(math.radians(self.StepTheta))
                self.StepY = self.NewStepX*math.sin(math.radians(self.StepTheta))+self.NewStepY*math.cos(math.radians(self.StepTheta))
                self.StepTime = self.NewStepTime
                self.NewCommand = 0
            elif not self.walk:           
                self.StepTheta = 0             
                self.StepX = 0             
                self.StepY = 0             
                self.StepTime = 0.30             
                self.NewCommand = 0
            
            self.StepTime = self.SAT(self.StepTime, 0.5, 0.2)
            self.idx = -1

        else:
           
            twalk = self.gtime - self.twalk0
            self.idx = self.idx+1 #int(round(twalk/SamplingTime))

            
            vL,vA =  p.getBaseVelocity(self.ComanId)

            self.PD_Wx = (self.action[0] * 0.3) + 1*self.PDControl(-self.ori[1], vA[1], 1, -0.05, [0.3,-0.3])
            self.PD_Wy = (self.action[1] * 0.05) + 1*self.PDControl(-self.ori[0], vA[0], 0.5, 0.0, [0.1,-0.1])

            self.PD_HLx = (self.action[2] * 0.25) + -1*self.PDControl(0.0-self.ori[1], vA[1], 2, -0.05, [0.25,-0.25])
            self.PD_HRx = (self.action[3] * 0.25) + -1*self.PDControl(0.0-self.ori[1], vA[1], 2, -0.05, [0.25,-0.25])

            self.PD_HLy = (self.action[4] * 0.25) + -1*self.PDControl(0.0-self.ori[0], vA[0], .7, -0.05, [0.,-0.2])
            self.PD_HRy = (self.action[5] * 0.25) + -1*self.PDControl(0.0-self.ori[0], vA[0], .7, -0.05, [0.2,-0.])

            self.PD_ALx = (self.action[6] * 0.2) + -self.PD_HLx
            self.PD_ARx = (self.action[7] * 0.2) + -self.PD_HRx

            self.PD_ALy = (self.action[8] * 0.2) + -self.PD_HLy
            self.PD_ARy = (self.action[9] * 0.2) + -self.PD_HRy

            self.PD_Wx  = self.SAT(self.PD_Wx , 0.3, -0.3)
            self.PD_Wy  = self.SAT(self.PD_Wy , 0.20, -0.20)
            self.PD_HLx = self.SAT(self.PD_HLx, 0.25, -0.25)
            self.PD_HRx = self.SAT(self.PD_HRx, 0.25, -0.25)
            self.PD_HLy = self.SAT(self.PD_HLy, 0.25, -0.)
            self.PD_HRy = self.SAT(self.PD_HRy, 0., -0.25)
            self.PD_ALx = self.SAT(self.PD_ALx, 0.2, -0.2) #foot front rotation
            self.PD_ARx = self.SAT(self.PD_ARx, 0.2, -0.2) #foot front rotation
            self.PD_ALy = self.SAT(self.PD_ALy, 0.3, -0.3) #foot side rotation
            self.PD_ARy = self.SAT(self.PD_ARy, 0.3, -0.3) #foot side rotation

            #### CHANGE STEPX AND TIME
            
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

            self.PD_Wx  = self.SAT(self.PD_Wx , 0.3, -0.3)
            self.PD_Wy  = self.SAT(self.PD_Wy , 0.20, -0.20)
            self.PD_HLx = self.SAT(self.PD_HLx, 0.25, -0.25)
            self.PD_HRx = self.SAT(self.PD_HRx, 0.25, -0.25)
            self.PD_HLy = self.SAT(self.PD_HLy, 0.25, -0.)
            self.PD_HRy = self.SAT(self.PD_HRy, 0., -0.25)
            self.PD_ALx = self.SAT(self.PD_ALx, 0.2, -0.2) #foot front rotation
            self.PD_ARx = self.SAT(self.PD_ARx, 0.2, -0.2) #foot front rotation
            self.PD_ALy = self.SAT(self.PD_ALy, 0.3, -0.3) #foot side rotation 
            self.PD_ARy = self.SAT(self.PD_ARy, 0.3, -0.3) #foot side rotation


            self.orip = self.ori

            # if (self.StopForOneStep == 1):
            #     self.RfootPosition = [0,0,self.Zleg,0]
            #     self.LfootPosition = [0,0,self.Zleg,0]
            #     self.JointPos = self.UpdateJointPositions(self.RfootPosition,self.LfootPosition)
            #     self.JointPositions = [self.WaistX + self.PD_Wx, self.WaistY + self.PD_Wy, self.WaistZ,
            #                     self.RShX,self.RShY,self.RShZ,self.REB,
            #                     self.LShX,self.LShY,self.LShZ,self.LEB,
            #                     self.JointPos[0] + self.PD_HRx, self.JointPos[1] + self.PD_HRy, self.JointPos[2],self.JointPos[3],self.JointPos[4]  + self.PD_ARx, self.JointPos[5]  + self.PD_ARy,
            #                     self.JointPos[6] + self.PD_HLx, self.JointPos[7] + self.PD_HLy, self.JointPos[8],self.JointPos[9],self.JointPos[10] + self.PD_ALx, self.JointPos[11] + self.PD_ALy]
            # elif (self.idx>=0 and self.idx<len(self.Walking.LeftLegX)):
            if (True):
            
                amp_arm = math.radians(2)
                arm_offset = 20 
                if (self.FirstStepIsRight):
                    self.LShX = math.radians(arm_offset ) + amp_arm*math.sin(twalk*2*math.pi/(2*self.StepTime)) #- EN_ArmX * 2 * PDArmX
                    self.RShX= math.radians(arm_offset ) - amp_arm*math.sin(twalk*2*math.pi/(2*self.StepTime)) #- EN_ArmX * 2 * PDArmX
                    self.Ltheta = math.radians(self.StepTheta)*self.SAT(twalk/self.StepTime,1,0)
                    self.Rtheta = math.radians(self.StepTheta)*self.SAT((twalk-self.StepTime)/self.StepTime,1,0)
                    
                else:
                    self.LShX = math.radians(arm_offset ) - amp_arm*math.sin(twalk*2*math.pi/(2*self.StepTime)) #- EN_ArmX * 2 * PDArmX
                    self.RShX = math.radians(arm_offset ) + amp_arm*math.sin(twalk*2*math.pi/(2*self.StepTime)) #- EN_ArmX * 2 * PDArmX
                    self.Ltheta = math.radians(self.StepTheta)*self.SAT((twalk-self.StepTime)/self.StepTime,1,0)
                    self.Rtheta = math.radians(self.StepTheta)*self.SAT(twalk/self.StepTime,1,0)
                    

                
                lpos, rpos = self.generate_cpg(twalk)
                self.RfootPosition = rpos
                self.LfootPosition = lpos
                self.LShX = math.radians(arm_offset)+lpos[4]
                self.RShX = math.radians(arm_offset)+rpos[4]
                

                print ("t {:2.3f} x {:2.3f} y {:2.3f} z{:2.3f}".format(twalk, self.StepX,self.StepY,self.StepTheta))
                time.sleep(0.001)

                self.JointPos = self.UpdateJointPositions(self.RfootPosition,self.LfootPosition)
                self.JointPositions = [self.WaistX + self.PD_Wx, self.WaistY + self.PD_Wy, self.WaistZ,
                                self.RShX,self.RShY,self.RShZ,self.REB,
                                self.LShX,self.LShY,self.LShZ,self.LEB,
                                self.JointPos[0] + self.PD_HRx, self.JointPos[1] + self.PD_HRy, self.JointPos[2],self.JointPos[3],self.JointPos[4]  + self.PD_ARx, self.JointPos[5]  + self.PD_ARy,
                                self.JointPos[6] + self.PD_HLx, self.JointPos[7] + self.PD_HLy, self.JointPos[8],self.JointPos[9],self.JointPos[10] + self.PD_ALx, self.JointPos[11] + self.PD_ALy]

        pi_2 = math.pi/2
        pi_3 = math.pi/3
        pi_6 = math.pi/6

        #Change arms with PPO
        self.JointPositions[3] += self.action[12] * pi_3 # self.RShX 
        self.JointPositions[4] += self.action[13] * pi_6 # self.RShY 
        self.JointPositions[5] += self.action[14] * pi_3 # self.RShZ 
        self.JointPositions[6] += self.action[15] * pi_2 # self.REB 
        self.JointPositions[7] += self.action[16] * pi_3 # self.LShX 
        self.JointPositions[8] += self.action[17] * pi_6 # self.LShY 
        self.JointPositions[9] += self.action[18] * pi_3 # self.LShZ 
        self.JointPositions[10]+= self.action[19] * pi_2 # self.LEB 


        #Saturation for arms joints
        self.JointPositions[3] =  self.SAT(self.JointPositions[3],  pi_3, -pi_3)    # self.RShX
        self.JointPositions[4] =  self.SAT(self.JointPositions[4],  0,    -pi_3)    # self.RShY
        self.JointPositions[5] =  self.SAT(self.JointPositions[5],  pi_3, -pi_3)    # self.RShZ
        self.JointPositions[6] =  self.SAT(self.JointPositions[6],  0,    -math.pi) # self.REB
        self.JointPositions[7] =  self.SAT(self.JointPositions[7],  pi_3, -pi_3)    # self.LShX
        self.JointPositions[8] =  self.SAT(self.JointPositions[8],  pi_3, 0)        # self.LShY
        self.JointPositions[9] =  self.SAT(self.JointPositions[9],  pi_3, -pi_3)    # self.LShZ
        self.JointPositions[10] = self.SAT(self.JointPositions[10], 0,    -math.pi) # self.LEB


        #Apply action to all joints
        for j in range(len(self.JointIDOrder)):
            if(j in range(3,11)): f=20
            else: f=60
            p.setJointMotorControl2(bodyIndex=self.ComanId, jointIndex=self.JointIDOrder[j], controlMode=p.POSITION_CONTROL,
                                                targetPosition=self.JointPositions[j], force=f)

        p.stepSimulation()

        if(self.CONFIG["clientMode"]=="GUI"):
            camState = p.getDebugVisualizerCamera()
            w = 0.98
            focus = [w*camState[11][0]+(1-w)*self.pos[0], w*camState[11][1]+(1-w)*self.pos[1], 0.5]
            #p.resetDebugVisualizerCamera( cameraDistance=camState[10], cameraYaw=camState[8]+0.1, cameraPitch=camState[9], cameraTargetPosition=focus)
            if p.readUserDebugParameter(self.param_switchcam) % 2 == 1:
                if self.scenario == 7:
                    p.resetDebugVisualizerCamera( cameraDistance=2.0, cameraYaw=camState[8], cameraPitch=-13, cameraTargetPosition=[0,0,0])
                elif self.scenario == 8:
                    pass
                else:
                    robotWorldOri = self.local_to_world_transform([0,0,0],[0,0,0,1])[1]
                    yaw = np.degrees(p.getEulerFromQuaternion(robotWorldOri)[2])+80 #yaw
                    if self.gtime == self.SamplingTime: w=0

                    pitch = 10 if self.scenario == 4 else -15
                    dist = 1.4 if self.walk else 0.9
                    rotSpeed = 2 if self.walk else 10

                    self.change_camera(dist, 1, self.gtime*rotSpeed + yaw, 1, pitch, 1, [self.pos[0],self.pos[1], 0.15 ], w)

                    #p.resetDebugVisualizerCamera( cameraDistance=0.9, cameraYaw=camState[8]+0.07, cameraPitch=-15, cameraTargetPosition=focus)
            

        pos, _ = p.getBasePositionAndOrientation(self.ComanId)


        if self.scenario == 8:
            terminal = terminal or (pos[2] < 0.20 or
                bool(p.getContactPoints(bodyA=self.ComanId, bodyB=self.FloorId, linkIndexA=15)) or
                bool(p.getContactPoints(bodyA=self.ComanId, bodyB=self.FloorId, linkIndexA=30)))
        elif self.scenario != 5 and self.scenario != 6 and self.scenario != 7:
            terminal = terminal or (pos[2] < 0.37 or
                bool(p.getContactPoints(bodyA=self.ComanId, bodyB=self.FloorId, linkIndexA=15)) or
                bool(p.getContactPoints(bodyA=self.ComanId, bodyB=self.FloorId, linkIndexA=30)))



        #------------------------------------- energy penalty

        #PD[10], zleg, stime, arms[8]
        abs_ppo_action = np.abs(ppo_action)

        for i in range(2):      abs_ppo_action[i] *= 0.5    #PD.wx PD.wy
        for i in range(2, 6):   abs_ppo_action[i] *= 0.25   #PD ankle
        for i in range(6, 10):  abs_ppo_action[i] *= 0.25   #PD hip
        for i in range(12, 20): abs_ppo_action[i] *= 0.125  #Arms

        en_penalty = np.sum(abs_ppo_action) / 6

        #------------------------------------- 
        
        reward = 0
        if self.commandPenalty is not None:
            en_pen = 0.5*self.energy_penalty_sum/self.energy_penalty_sum_cnt
            self.energy_penalty_sum = 0.0
            self.energy_penalty_sum_cnt = 0

            reward = 1 - en_pen - (self.commandPenalty*0.4)
            if reward < 0.3: reward = 0.3
            
            if self.CONFIG["clientMode"]=="GUI":
                print("Rew: {:.3f} En: {:.3f} Cmd: {:.3f} ".format(reward,  en_pen, self.commandPenalty))

        else:
            self.energy_penalty_sum += en_penalty
            self.energy_penalty_sum_cnt += 1

            
       
        #print("Rew: {:.3f} PD_hip_penalty: {:.3f}  Zleg_penalty: {:.3f}  stime: {:.3f}".format(
        #    reward,  PD_hip_penalty, Zleg_penalty, Step_time_penalty))
        
        return self.observe(), reward, terminal, "INFO"

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
            posi, vel, _, torque = p.getJointState(self.ComanId,self.JointIDOrder[i]) #pos, vel, reaction force, applied torque
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

        linSpeedWorld,angSpeedWorld = p.getBaseVelocity(bodyUniqueId=self.ComanId)
        linSpeed = self.world_to_local_rotate(linSpeedWorld)[0]
        angSpeed = self.world_to_local_rotate(angSpeedWorld)[0]

        ob.extend(linSpeed)
        symOb.extend([  linSpeed[0],-linSpeed[1],linSpeed[2]  ])
        ob.extend(angSpeed)
        symOb.extend([  -angSpeed[0],angSpeed[1],-angSpeed[2]  ])

        po, _ = p.getBasePositionAndOrientation(self.ComanId)
        ob.append(po[2])    #base height
        symOb.append(po[2]) #base height

        gravity = self.world_to_local_rotate([0,0,-1])[0] #gravity vector in local coordinates
        ob.extend(gravity)   
        symOb.extend([  gravity[0],-gravity[1],gravity[2]  ])

        lcop, rcop, lcopf, rcopf = self.getFeetCoP()
        ob.extend(lcop[0:2]) #Center of pressure L
        ob.extend(rcop[0:2]) #Center of pressure R
        ob.append(lcopf) #LFoot Force
        ob.append(rcopf) #RFoot Force

        symOb.extend([  rcop[0],-rcop[1]  ]) #Center of pressure R
        symOb.extend([  lcop[0],-lcop[1]  ]) #Center of pressure L
        symOb.append(rcopf) #RFoot Force
        symOb.append(lcopf) #LFoot Force

        for i in range(len(ob)):
            ob[i] *= (1 + np.random.rand()*(self.obsNoise*2) - self.obsNoise)

        if self.obsNoise != 0:
            print("Warning: adding noise to obs...")

        #------------------------------------------------------------------ Commands
        #Sensor     Symmetry   Axis positive direction
        #StepX     - same      (front)
        #StepY     - invert    (side)
        #StepTheta - invert    (z)
        ob.append(self.StepX)
        ob.append(self.StepY)
        ob.append(self.StepTheta)

        symOb.append(self.StepX)
        symOb.append(-self.StepY)
        symOb.append(-self.StepTheta)

        #------------------------------------------------------------------ Counter (to learn phase of gait but also value due to sparse reward)
        ob.append(self.idx)
        symOb.append(self.idx)

        #================================== Log observations (NN input)
        ignoreLog = True
        if not ignoreLog:
            import csv
            if not hasattr(self, 'logNN'):
                from datetime import datetime
                self.logNN = "log_NN_input__" + "__scenario_" + str(self.scenario) + datetime.now().strftime("__%d.%m.%Y_%H.%M.%S") + ".csv"
                with open(self.logNN, "w") as f:  # creates empty file with header
                    writer = csv.writer(f)
                    writer.writerow([
                        "waistX_pos",  "waistX_vel",  "waistX_tor",         "waistY_pos",  "waistY_vel",  "waistY_tor",  
                        "waistZ_pos",  "waistZ_vel",  "waistZ_tor",         "RShouX_pos",  "RShouX_vel",  "RShouX_tor",  
                        "RShouY_pos",  "RShouY_vel",  "RShouY_tor",         "RShouZ_pos",  "RShouZ_vel",  "RShouZ_tor",  
                        "RElbow_pos",  "RElbow_vel",  "RElbow_tor",         "LShouX_pos",  "LShouX_vel",  "LShouX_tor",  
                        "LShouY_pos",  "LShouY_vel",  "LShouY_tor",         "LShouZ_pos",  "LShouZ_vel",  "LShouZ_tor",  
                        "LElbow_pos",  "LElbow_vel",  "LElbow_tor",         "RHipX_pos",   "RHipX_vel",   "RHipX_tor",
                        "RHipY_pos",   "RHipY_vel",   "RHipY_tor",          "RHipZ_pos",   "RHipZ_vel",   "RHipZ_tor",
                        "RKneeX_pos",  "RKneeX_vel",  "RKneeX_tor",         "RAnkleX_pos", "RAnkleX_vel", "RAnkleX_tor",    
                        "RAnkleY_pos", "RAnkleY_vel", "RAnkleY_tor",        "LHipX_pos",   "LHipX_vel",   "LHipX_tor",
                        "LHipY_pos",   "LHipY_vel",   "LHipY_tor",          "LHipZ_pos",   "LHipZ_vel",   "LHipZ_tor",            
                        "LKneeX_pos",  "LKneeX_vel",  "LKneeX_tor",         "LAnkleX_pos", "LAnkleX_vel", "LAnkleX_tor",     
                        "LAnkleY_pos", "LAnkleY_vel", "LAnkleY_tor",        
                        "LinSpeedX", "LinSpeedY", "LinSpeedZ", "AngSpeedX", "AngSpeedY", "AngSpeedZ", "Height",
                        "GravityX", "GravityY", "GravityZ", "LCoPX", "LCoPY", "RCoPX", "RCoPY", "LFootF", "RFootF"
                    ])

            with open(self.logNN, "a") as f:  # appends
                writer = csv.writer(f)
                writer.writerow(ob)

        # symAcIdx =  np.array([0,1,3,2,5,4,7,6,9,8,10,11,16,17,18,19,12,13,14,15])
        # symAcSign = np.array([1,-1,1,1,-1,-1,1,1,-1,-1,1,1,1,-1,-1,1,1,-1,-1,1])
        # symObIdx = np.array([0,1,2,3,4,5,6,7,8,21,22,23,24,25,26,27,28,29,30,31,32,9,10,11,12,13,14,15,16,17,18,19,20,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,69,70,71,72,73,74,75,76,77,78,81,82,79,80,84,83,85,86,87,88])
        # symObSign = np.array([1,1,1,-1,-1,-1,-1,-1,-1,1,1,1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,-1,-1,-1,1,1,1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,-1,-1,-1,1,-1,1,-1,1,-1,1,1,-1,1,1,-1,1,-1,1,1,1,-1,-1,1])
        # asdf = np.take(np.asarray(ob, dtype=np.float32), symObIdx)
        # asdf *= symObSign

        # assert len(ob) == len(symOb)
        # for i in range(len(ob)):
        #     print("{:3} {:6.3f} {:6.3f} {:6.3f}".format( i, ob[i], symOb[i], asdf[i]))
        #     assert symOb[i]-0.0001 < asdf[i] and symOb[i]+0.0001 > asdf[i]


        return np.asarray(ob, dtype=np.float32)


    def world_to_local_transform(self,pos,ori=[0,0,0,1], framePos=None):
        """
        World to local coordinates 

        :param pos: world position (vec3)
        :param ori: world orientation quaternion (vec4)
        :return: local coordinates
        """        
        b_pos, b_ori = p.getBasePositionAndOrientation(self.ComanId)
        if framePos is not None: b_pos = framePos
        inv_b_pos, inv_b_ori = p.invertTransform(b_pos, b_ori)
        return p.multiplyTransforms(inv_b_pos, inv_b_ori, pos, ori)

    def world_to_local_rotate(self,pos,ori=[0,0,0,1]):
        """
        World to local coordinates (apply rotation only)

        :param pos: world position (vec3)
        :param ori: world orientation quaternion (vec4)
        :return: local coordinates
        """        
        _, b_ori = p.getBasePositionAndOrientation(self.ComanId)
        inv_b_pos, inv_b_ori = p.invertTransform([0,0,0], b_ori)
        return p.multiplyTransforms(inv_b_pos, inv_b_ori, pos, ori)

    def local_to_world_transform(self,pos,ori=[0,0,0,1]):
        """
        Local to world coordinates

        :param pos: local position (vec3)
        :param ori: local orientation quaternion (vec4)
        :return: world coordinates
        """

        b_pos, b_ori = p.getBasePositionAndOrientation(self.ComanId)
        return p.multiplyTransforms(b_pos, b_ori, pos, ori)

    def local_to_world_rotate(self,pos,ori=[0,0,0,1]):
        """
        Local to world coordinates (apply rotation only)

        :param pos: local position (vec3)
        :param ori: local orientation quaternion (vec4)
        :return: world coordinates
        """

        _, b_ori = p.getBasePositionAndOrientation(self.ComanId)
        return p.multiplyTransforms([0,0,0], b_ori, pos, ori)

    def translate(self,value, IN, OUT):
        # value should be saturated by IN range
        #value = self.SAT(value,IN[1],IN[0])

        # Figure out how 'wide' each range is
        InSpan = IN[1] - IN[0]
        OutSpan = OUT[1] - OUT[0]


        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - IN[0]) / float(InSpan)

        # Convert the 0-1 range into a value in the right range.
        return OUT[0] + (valueScaled * OutSpan)
    
    def close(self):
        p.disconnect()
    
    def apply_walk_command(self):

        # w = [0.75, 0.5, 0.5]
        # self.NewStepX =     self.NewStepX     * w[0] + self.NewStepX_raw     * (1-w[0])
        # self.NewStepY =     self.NewStepY     * w[1] + self.NewStepY_raw     * (1-w[1])
        # self.NewStepTheta = self.NewStepTheta * w[2] + self.NewStepTheta_raw * (1-w[2])

        # if ((self.NewStepY_raw * self.NewStepY < 0 or self.NewStepTheta_raw * self.NewStepTheta < 0) and 
        #     (abs(self.NewStepX)>0.03 or abs(self.NewStepY)>0.03 or abs(self.NewStepTheta)>3)): #stop smoothly before changing direction (switch initial foot)
           
        #     #Converges to a small value to preserve the sign (which is relevant to simplify the control algorithm)
        #     self.NewStepX +=     max( min(math.copysign(1e-7, self.NewStepX) - self.NewStepX, 0.1) , -0.1)
        #     self.NewStepY +=     max( min(math.copysign(1e-7, self.NewStepY) - self.NewStepY, 0.05) , -0.05)
        #     self.NewStepTheta += max( min(math.copysign(1e-7, self.NewStepTheta) - self.NewStepTheta, 5) , -5)
        # else:              
        #     self.NewStepX +=     max( min( self.NewStepX_raw - self.NewStepX, 0.1) , -0.1)
        #     self.NewStepY +=     max( min( self.NewStepY_raw - self.NewStepY, 0.05) , -0.05)
        #     self.NewStepTheta += max( min( self.NewStepTheta_raw - self.NewStepTheta, 5) , -5)
    
        self.NewStepX +=     max( min( self.NewStepX_raw - self.NewStepX, 0.05) , -0.05)
        self.NewStepY +=     max( min( self.NewStepY_raw - self.NewStepY, 0.025) , -0.025)
        self.NewStepTheta += max( min( self.NewStepTheta_raw - self.NewStepTheta, 10) , -10)

        self.NewCommand = 1

        #print("Apply:", self.NewStepX, self.NewStepY, self.NewStepTheta)

        self.predictX = (self.NewStepX * 0.85)
        self.predictY = (self.NewStepY * 0.80)
        self.predictT = (self.NewStepTheta * 1.1 - self.NewStepY * 80)

    def restrict_raw_walk_command(self, isNormalized=False):

        # stepx_lim = [-0.5,0.5]
        # stepy_lim = [-0.3, 0.3]
        # steptheta_lim = [-15, 15]
        
        stepx_lim = [-0.3,0.3]
        stepy_lim = [-0.2, 0.2]
        steptheta_lim = [-15, 15]

        #Normalize if not already
        if not isNormalized:
            x = self.translate( self.NewStepX_raw, stepx_lim, [-0.5,0.5] ) #map to relative values [-0.5, 0.5]
            y = self.translate( self.NewStepY_raw, stepy_lim, [-0.5,0.5] ) #map to relative values [-0.5, 0.5]
            t = self.translate( self.NewStepTheta_raw, steptheta_lim, [-0.5,0.5] ) #map to relative values [-0.5, 0.5]
        else:
            x = self.NewStepX_raw
            y = self.NewStepY_raw
            t = self.NewStepTheta_raw

        #avoid transition stop
        if abs(y) < 0.03: 
            y = 0
        if abs(t) < 0.2: 
            t = 0

        #compensate rotation
        #t += y*1.17

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

        self.NewStepX_raw     = self.translate( np.random.rand(), [0.1,0.9], [-0.5,0.5] ) #creates NO bias and saturation boundaries
        self.NewStepY_raw     = self.translate( np.random.rand(), [0.1,0.9], [-0.5,0.5] ) #creates saturation boundaries
        self.NewStepTheta_raw = self.translate( np.random.rand(), [0.1,0.9], [-0.5,0.5] ) #creates saturation boundaries

        self.restrict_raw_walk_command(True)
        self.apply_walk_command()

    def reset(self):
        p.restoreState(stateId=self.resetState)

        #if self.scenario == 4 or self.scenario == 6:
        #    p.removeBody(self.FloorId)
        #   self.create_uneven_terrain()

        self.episode_number += 1

        if(self.CONFIG["deterministicPushes"]):# and self.episode_number == 1: #changed for tilting platform sync
            np.random.seed(self.episode_number)
            

        self.push_countdown = np.random.randint(self.push_min_cooldown, self.push_max_cooldown )
        if self.scenario == 8: self.push_countdown = 1 / self.SamplingTime
        self.push_duration = 5
        #ang = np.random.rand()*2*math.pi
        #self.push_force = [math.sin(ang)*np.random.randint( self.push_min_force_norm, self.push_max_force_norm ), 
        #                   math.cos(ang)*np.random.randint( self.push_min_force_norm, self.push_max_force_norm ), 0]
        
        #if(self.CONFIG["deterministicPushes"]):
        #    print("Next deterministic push: f=", self.push_force, "d=", self.push_duration)

        p.setGravity(0,0,-9.81)
        self.tilt = np.array([0,0,-9.81])
        self.new_tilt = np.array([0,0,-9.81])
        self.tilt_duration = np.random.randint(50, 150 )

        self.FirstStepIsRight = 0 # np.random.randint(0,2)
        self.Zleg = -0.365
        self.NewCommand = 0
        self.NewStepX = 0
        self.NewStepY = 0
        self.NewStepTheta = 0
        self.NewStepTime = self.StepTime

        if self.walk:
            
            if self.walk_trajectory_No == 0:
                self.generate_random_walk_command()
            elif self.walk_trajectory_No == 1:
                self.figure8_progress = 0
                self.figure8_oldtarget = [0,0]
                self.figure8_target = self.generate_8_figure_target()
                self.generate_8_figure_walk_command()
            elif self.walk_trajectory_No == 2:
                self.obstCourse_progress = None
                self.generate_obstacle_course_walk_command()

            # elif self.walk_trajectory_No == 4: # input command
            #         self.apply_walk_command()
            
                
            # self.predict_FeetCenter, self.predict_Rot = self.get_feet_center_and_rotation()
            self.stats_old_predT = 1
            self.stats_old_predX = 1
            self.stats_old_predY = 1
            self.energy_penalty_sum = 0
            self.energy_penalty_sum_cnt = 0
            self.WALK_DURATION_MAX = 7
            self.walk_duration = self.WALK_DURATION_MAX / self.SamplingTime


        self.twalk0 = 0
        self.gtime = 0

        self.idx = 0
        self.time_since_last_push = 10

        self.penalty_pos = [0,0]
        self.action = np.zeros(self.action_size, np.float32)

        self.UpdateZMP(init=True)

        #Generate walking trajectory
        self.StepX= 0.0
        self.StepY= 0.0
        self.StepTheta = 0.0
        self.SwingStepZ = 0.03

        if (self.StepY<0.000 or self.StepTheta>0.01):
            self.FirstStepIsRight = 1

        if (self.FirstStepIsRight):
            FR0X = self.StepX/2
            FR0Y = -self.DistanceBetweenFeet/2
            FL0X = 0
            FL0Y = self.DistanceBetweenFeet/2
        else:
            FL0X = self.StepX/2    
            FL0Y = self.DistanceBetweenFeet/2
            FR0X = 0
            FR0Y = -self.DistanceBetweenFeet/2    
                    
        self.Walking = GenrateWalkingTrajectories(FR0X,FR0Y,FL0X,FL0Y,
            self.StepX,self.StepY,self.SwingStepZ,self.StepTime,self.SamplingTime,self.FirstStepIsRight,-self.Zleg,0)
        self.Walking.Generate()

        self.tiltplat_joints = np.array([0,0],np.float32)

        #for i in range(p.getNumJoints(self.ComanId)):
        #    print(p.getJointInfo(self.ComanId, i))

        return self.observe()
       
