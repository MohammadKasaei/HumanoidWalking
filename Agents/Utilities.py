import pybullet as p
import time
import math
import numpy as np
import os, json
import random
import platform

import threading
from getkey import getkey, keys

import matplotlib.pyplot as plt

__cwd__ = os.path.realpath( os.path.join(os.getcwd(), os.path.dirname(__file__)))



def world_to_local_transform(wPos,wOri,framePos=[0,0,0],frameOri=[0,0,0,1] ):
    """
    World to local coordinates 

    :param wPos: world position (vec3)
    :param wOri: world orientation quaternion (vec4)
    :param framePos: frame position, set to [0,0,0] to apply only rotation
    :param frameOri: frame orientation, set to [0,0,0,1] to apply only translation
    :return: local coordinates
    """        
    inv_f_pos, inv_f_ori = p.invertTransform(framePos, frameOri)
    return p.multiplyTransforms(inv_f_pos, inv_f_ori, wPos, wOri)


def local_to_world_transform(lPos,lOri,framePos=[0,0,0],frameOri=[0,0,0,1] ):
    """
    Local to world coordinates

    :param lPos: local position (vec3)
    :param lOri: local orientation quaternion (vec4)
    :param framePos: frame position, set to [0,0,0] to apply only rotation
    :param frameOri: frame orientation, set to [0,0,0,1] to apply only translation
    :return: world coordinates
    """

    return p.multiplyTransforms(framePos, frameOri, lPos, lOri)


def translate(value, IN, OUT):
    """
    Maps from range IN=[low,high] to OUT=[low,high] without saturating output
    e.g. map x=0 from [-1,1] to [2,3] returns 2.5
    e.g. map x=3 from [ 0,1] to [0,2] returns 6

    :param IN: input range (list[2])
    :param OUT: output range (list[2])
    :return: mapped value
    """
    # Figure out how 'wide' each range is
    InSpan = IN[1] - IN[0]
    OutSpan = OUT[1] - OUT[0]

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - IN[0]) / float(InSpan)

    # Convert the 0-1 range into a value in the right range.
    return OUT[0] + (valueScaled * OutSpan)

def normalize_deg(x):
    return (x+180.0)%(360.0)-180.0

def normalize_rad(x):
    return (x+math.pi)%(math.pi*2)-math.pi


def change_bullet_camera(dist, distfilter, yaw, yawfilter, pitch, pitchfilter, pos, posfilter):

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


class bullet_agent():

    def __init__(self, id, floorid):
        self.ID = id #pybullet body ID
        self.floorID = floorid

    def update(self):
        '''
        Update agent variables (datatype: float32 numpy array)
        '''
        global p
        po, o = p.getBasePositionAndOrientation(self.ID)
        po, o = np.array(po, np.float32), np.array(o, np.float32)

        #agent position (in world coordinates)
        self.pos_3d_torso = po
        self.pos_2d_torso = po[0:2]
        self.pos_3d_torso_proj = np.append(po[0:2],0) #projection of position on ground plane
        self.pos_z_torso = po[2]

        #agent orientation (in world coordinates)
        self.ori_4d_torso = o
        self.ori_3d_torso_rad = np.array(p.getEulerFromQuaternion(o), np.float32)
        self.ori_z_torso_rad = self.ori_3d_torso_rad[2]
        self.ori_z_torso_deg = np.degrees(self.ori_3d_torso_rad[2])
        self.ori_4d_torso_z_only = p.getQuaternionFromEuler([0,0,self.ori_z_torso_rad])

        #agent velocity (both coordiante systems)
        self.wLinVel_3d_torso, self.wAngVel_3d_torso = p.getBaseVelocity(bodyUniqueId=self.ID) #linear velocity and angular velocity (in world coordinates)
        self.rLinVel_3d_torso = world_to_local_transform(self.wLinVel_3d_torso,[0,0,0,1],[0,0,0],self.ori_4d_torso)[0] #lin. vel. (in rel. coords. [front,left,up])
        self.rAngVel_3d_torso = world_to_local_transform(self.wAngVel_3d_torso,[0,0,0,1],[0,0,0],self.ori_4d_torso)[0] #ang. vel. (in rel. coords. [tilt r, tilt fwd, rot l])

        #position/orientation of body parts (in world coordinates)
        self.pos_3d_lfoot, self.ori_4d_lfoot = p.getLinkState(bodyUniqueId=self.ID, linkIndex=54)[0:2] #position of each foot in world coordinates
        self.pos_3d_rfoot, self.ori_4d_rfoot = p.getLinkState(bodyUniqueId=self.ID, linkIndex=40)[0:2] 
        self.pos_3d_lfoot, self.ori_4d_lfoot = np.array(self.pos_3d_lfoot, np.float32), np.array(self.ori_4d_lfoot, np.float32)
        self.pos_3d_rfoot, self.ori_4d_rfoot = np.array(self.pos_3d_rfoot, np.float32), np.array(self.ori_4d_rfoot, np.float32)

        self.pos_3d_feet = (self.pos_3d_lfoot + self.pos_3d_rfoot)/2.0 # position of feet center

        self.pos_3d_tf = (self.pos_3d_feet + self.pos_3d_torso)/2.0 # average position of {torso, feet center}

        #contact points with ground (and force vector) (in local coordinates  ->  x: positive=front, y: positive=left, z: positive=up)
        self.pos_3d_lfoot_CoP, self.pos_3d_rfoot_CoP, self.force_3d_lfoot_CoP, self.force_3d_rfoot_CoP = self._getFeetCoP()

        #flag raised when hands are touching the ground
        self.flag_hands_touch_ground =  (bool(p.getContactPoints(bodyA=self.ID, bodyB=self.floorID, linkIndexA=15)) or
                                         bool(p.getContactPoints(bodyA=self.ID, bodyB=self.floorID, linkIndexA=30)))

    def _getFeetCoP(self): #private function

        #Contact points
        lCP = p.getContactPoints(bodyA=self.ID, bodyB=self.floorID, linkIndexA=54) #LAnkle
        rCP = p.getContactPoints(bodyA=self.ID, bodyB=self.floorID, linkIndexA=40) #RAnkle

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
                lfwpos, lfwori = p.getLinkState(bodyUniqueId=self.ID, linkIndex=54)[0:2]
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
                rfwpos, rfwori = p.getLinkState(bodyUniqueId=self.ID, linkIndex=40)[0:2]
                inv_rfwpos, inv_rfwori = p.invertTransform(rfwpos, rfwori)
                rCoP, _ = p.multiplyTransforms(inv_rfwpos, inv_rfwori, rCoP, [0,0,1,0])

        return lCoP, rCoP, lCoPforce, rCoPforce #x: positive=front, y: positive=left, z: positive=up


