#!/bin/python3

import numpy as np
from numpy import arccos, arcsin, pi, cos, shape, sin
from mgi_legs import *
from robot_state_reader import *

class Moves_12dof:
    # Robot configurations 
    HAA_2_UPPER_LEG = 19.5 * 0.001 
    HALF_LEG_LENGTH = 160 * 0.001  
    TARGET_HEIGHT = 270 * 0.001
    topic = '/perrobot_12dof_controller/joint_states'
    
    
    @staticmethod
    def move_foot(foot, xyz=[-HAA_2_UPPER_LEG, 0.0, -TARGET_HEIGHT]):
        position = listener(Moves_12dof.topic, foot)
        qinit = [position[foot+'_HAA'], position[foot+'_HFE'], position[foot+'_KFE']]
        
        return mgi(xyz, qinit)
    
    @staticmethod
    def generate_sequences(a1, a2, n):
        a1 = np.array(a1)
        a2 = np.array(a2)

        return [(1 - t) * a1 + t * a2 for t in np.linspace(0, 1, n + 1)]
    
    @staticmethod
    def hight_to_angles(h=TARGET_HEIGHT, L=HALF_LEG_LENGTH):
        arg = 0.5 * (h / L)

        q2 = arccos(arg)
        gamma = arcsin(arg)
        q3 = gamma - q2 - pi/2
        
        return q2, q3

    @staticmethod
    def pose(q2, q3, Rpose='stand_x'):
        
        rpose = {
            'stand_x':[q2, q3, q2, q3, -q2, -q3, -q2, -q3, 0.0, 0.0, 0.0, 0.0],
            'stand_nx':[-q2, -q3, -q2, -q3, q2, q3, q2, q3, 0.0, 0.0, 0.0, 0.0],
            'stand_ncc':[-q2, -q3, -q2, -q3, -q2, -q3, -q2, -q3, 0.0, 0.0, 0.0, 0.0],
            'stand_cc':[q2, q3, q2, q3, q2, q3, q2, q3, 0.0, 0.0, 0.0, 0.0],
            'on_flor':[pi/2, pi, pi/2, pi, -pi/2, -pi, -pi/2, -pi, 0.0, 0.0, 0.0, 0.0],
        }
        
        values = np.array(rpose['stand_x']) + np.array([pi/2, -pi, pi/2, -pi, -pi/2, pi, -pi/2, pi, 0.0, 0.0, 0.0, 0.0])
        rpose['floor_recovery'] = list(values)
        
        return rpose[Rpose] if Rpose in rpose.keys() else None
    
    @staticmethod
    def get_to_pose(current_pose, target_pose, h=TARGET_HEIGHT, L=HALF_LEG_LENGTH, cut=30):
        q2, q3 = Moves_12dof.hight_to_angles(h, L)
        current_pose = Moves_12dof.pose(q2, q3, current_pose)
        target_pose = Moves_12dof.pose(q2, q3, target_pose)
        
        return Moves_12dof.generate_sequences(current_pose, target_pose, cut)
    
    @staticmethod
    def angles_for_height(h, L, cut=10, Rpose='stand_x'):
        path = np.linspace(0, h, cut)
        Values = []

        for height in path:
            s = Moves_12dof.hight_to_angles(height, L)
            Values.append(s)
        
        return Values
        