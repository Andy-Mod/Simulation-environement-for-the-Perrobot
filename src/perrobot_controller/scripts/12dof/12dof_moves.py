#!/bin/python3

import numpy as np
from numpy import arccos, arcsin, pi, cos, shape, sin

class Moves_12dof:
    HALF_LEG_LENGTH = 0.16
    TARGET_HEIGHT = 0.27
    HAA_2_UPPER_LEG = 0.014
    UNIT_VALUE_FOR_A_STEP_F = pi/28
    UNIT_VALUE_FOR_A_STEP_B = pi/10
    SET_1 = [0, 1, 4, 5]
    SET_2 = [2, 3, 6, 7]
    FAST = 1
    SLOW = 3
    NUMBER_OF_JOINTS = 12 
    
    @staticmethod
    def generate_sequences(a1, a2, n):
        a1 = np.array(a1)
        a2 = np.array(a2)

        return [(1 - t) * a1 + t * a2 for t in np.linspace(0, 1, n + 1)]
    
    @staticmethod
    def hight_to_angles(h=TARGET_HEIGHT, L=HALF_LEG_LENGTH):
        arg = 0.5 * (h / L)

        alpha = arccos(arg)
        gamma = arcsin(arg)
        beta = gamma - alpha - pi/2
        
        return alpha, beta

    @staticmethod
    def pose(alpha, beta, Rpose='stand_x'):
        
        rpose = {
            'stand_x':[alpha, beta, alpha, beta, -alpha, -beta, -alpha, -beta, 0.0, 0.0, 0.0, 0.0],
            'stand_nx':[-alpha, -beta, -alpha, -beta, alpha, beta, alpha, beta, 0.0, 0.0, 0.0, 0.0],
            'stand_ncc':[-alpha, -beta, -alpha, -beta, -alpha, -beta, -alpha, -beta, 0.0, 0.0, 0.0, 0.0],
            'stand_cc':[alpha, beta, alpha, beta, alpha, beta, alpha, beta, 0.0, 0.0, 0.0, 0.0],
            'on_flor':[pi/2, pi, pi/2, pi, -pi/2, -pi, -pi/2, -pi, 0.0, 0.0, 0.0, 0.0],
        }
        
        values = np.array(rpose['stand_x']) + np.array([pi/2, -pi, pi/2, -pi, -pi/2, pi, -pi/2, pi, 0.0, 0.0, 0.0, 0.0])
        rpose['floor_recovery'] = list(values)
        
        return rpose[Rpose] if Rpose in rpose.keys() else None
    
    @staticmethod
    def get_to_pose(current_pose, target_pose, h=TARGET_HEIGHT, L=HALF_LEG_LENGTH, cut=30):
        alpha, beta = Moves_12dof.hight_to_angles(h, L)
        current_pose = Moves_12dof.pose(alpha, beta, current_pose)
        target_pose = Moves_12dof.pose(alpha, beta, target_pose)
        
        return Moves_12dof.generate_sequences(current_pose, target_pose, cut)
    
    
    @staticmethod
    def step_from_pose(current_pose, cut=30, h=TARGET_HEIGHT, L=HALF_LEG_LENGTH, f=True):
        alpha, beta = Moves_12dof.hight_to_angles(h, L)
        current_pose = Moves_12dof.pose(alpha, beta, current_pose)
        sens = 'foward' if f else 'backward'
        
        set_1 = np.array(current_pose).copy()
        set_2 = np.array(current_pose).copy()
        
        
        offsets = {
            'foward':np.array([-Moves_12dof.UNIT_VALUE_FOR_A_STEP_F,
                     -Moves_12dof.UNIT_VALUE_FOR_A_STEP_F, 
                     -Moves_12dof.UNIT_VALUE_FOR_A_STEP_F, 
                     -Moves_12dof.UNIT_VALUE_FOR_A_STEP_F
                ]),
            
            'backward':np.array([Moves_12dof.UNIT_VALUE_FOR_A_STEP_B,
                     Moves_12dof.UNIT_VALUE_FOR_A_STEP_B, 
                     Moves_12dof.UNIT_VALUE_FOR_A_STEP_B, 
                     Moves_12dof.UNIT_VALUE_FOR_A_STEP_B
                ])
        }
        
        set_1[Moves_12dof.SET_1] += offsets[sens]
        set_2[Moves_12dof.SET_2] += offsets[sens]
          
        first = Moves_12dof.generate_sequences(current_pose, set_1, cut)
        second = Moves_12dof.generate_sequences(first[-1], current_pose, cut)
        third = Moves_12dof.generate_sequences(second[-1], set_2, cut)
        last = Moves_12dof.generate_sequences(third[-1], current_pose, cut)
        
        return first + second + third + last
    
    @staticmethod
    def shift_left(current_pose, cut=30, h=TARGET_HEIGHT, L=HALF_LEG_LENGTH, f=True):
        pass