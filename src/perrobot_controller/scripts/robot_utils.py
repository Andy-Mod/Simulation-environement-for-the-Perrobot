#!/bin/python3

import numpy as np
from numpy import arccos, arcsin, pi, cos, sin

class RobotUtils:
    HALF_LEG_LENGTH = 0.16
    TARGET_HEIGHT = 0.23
    UNIT_VALUE_FOR_A_STEP = pi/12

    @staticmethod
    def init_pose():
        alpha, beta = pi/2, pi
        return [alpha, beta, alpha, beta, -alpha, -beta, -alpha, -beta]

    @staticmethod
    def angles_for_height(h, L, cut=30, Rpose='x'):
        path = np.linspace(0, h, cut)
        Values = [RobotUtils.init_pose()]

        Values.extend(RobotUtils.hight_to_angles(height, L, Rpose) for height in path)
        
        return Values

    @staticmethod
    def hight_to_angles(h, L, Rpose='x', s=False):
        arg = 0.5 * (h / L)

        alpha = arccos(arg)
        gamma = arcsin(arg)
        beta = gamma - alpha - pi / 2

        a, b = round(alpha, 2), round(beta, 2)

        if Rpose == 'x':
            values = [a, b, a, b, -a, -b, -a, -b]
        elif Rpose == 'nx':
            values = [-a, -b, -a, -b, a, b, a, b]
        elif Rpose == 'cc':
            values = [-a, -b, -a, -b, -a, -b, -a, -b]
        elif Rpose == 'ncc':
            values = [a, b, a, b, a, b, a, b]
        else:
            values = None

        return (a, b) if s else values

    @staticmethod
    def angles_to_height(alpha, L):
        return round(2 * L * cos(alpha), 2)

    @staticmethod
    def generate_sequences(a1, a2, n):
        a1 = np.array(a1)
        a2 = np.array(a2)

        return [(1 - t) * a1 + t * a2 for t in np.linspace(0, 1, n + 1)]
    
    @staticmethod
    def from_init_to_pose(h, L, target_pose, cut=30):
        init = np.array(RobotUtils.init_pose())
        target = np.array(RobotUtils.hight_to_angles(h, L, target_pose))

        return RobotUtils.generate_sequences(init, target, cut)

    @staticmethod
    def from_pose_to_init(h, L, source_pose, cut=30):
        source = np.array(RobotUtils.hight_to_angles(h, L, source_pose))
        init = np.array(RobotUtils.init_pose())

        return RobotUtils.generate_sequences(source, init, cut)
    
    @staticmethod
    def from_X_to_NX(h, L, cut=30):
        source_pose = 'x'
        target_pose = 'nx'
        source = np.array(RobotUtils.hight_to_angles(h, L, source_pose))
        target = np.array(RobotUtils.hight_to_angles(h, L, target_pose))

        return RobotUtils.generate_sequences(source, target, cut)
    
    @staticmethod
    def from_NX_to_X(h, L, cut=30):
        source_pose = 'nx'
        target_pose = 'x'
        source = np.array(RobotUtils.hight_to_angles(h, L, source_pose))
        target = np.array(RobotUtils.hight_to_angles(h, L, target_pose))
        
        return RobotUtils.generate_sequences(source, target, cut)
    
    @staticmethod
    def from_floor_to_X(h, L, cut=30):
        X = np.array(RobotUtils.hight_to_angles(h, L, 'x'))
        target_pose = X.copy() 
        target_pose[0] = target_pose[0] - 3*pi/4
        target_pose[2] = target_pose[2] - 3*pi/4
        target_pose[4] = target_pose[4] + 3*pi/4
        target_pose[6] = target_pose[6] + 3*pi/4
        
        return RobotUtils.generate_sequences(X, target_pose, cut)
    
    @staticmethod
    def FL_HR_up_step(current_pose, cut=30):
        current_pose_array = np.array(current_pose)
        set_1_mid_step_target = current_pose_array.copy()
        
        set_1_mid_step_target[0] = set_1_mid_step_target[0] - RobotUtils.UNIT_VALUE_FOR_A_STEP
        set_1_mid_step_target[1] = set_1_mid_step_target[1] - RobotUtils.UNIT_VALUE_FOR_A_STEP
        set_1_mid_step_target[4] = set_1_mid_step_target[4] - RobotUtils.UNIT_VALUE_FOR_A_STEP   
        set_1_mid_step_target[5] = set_1_mid_step_target[5] + RobotUtils.UNIT_VALUE_FOR_A_STEP   
            
        return list(RobotUtils.generate_sequences(current_pose_array, set_1_mid_step_target, cut))
        
    @staticmethod
    def FL_HR_down_step_1(current_pose, cut=30):
        current_pose_array = np.array(current_pose)
        set_1_mid_step_target = current_pose_array.copy()
        
        set_1_mid_step_target[4] = set_1_mid_step_target[4] + RobotUtils.UNIT_VALUE_FOR_A_STEP   
        set_1_mid_step_target[5] = set_1_mid_step_target[5] - RobotUtils.UNIT_VALUE_FOR_A_STEP   
            
        return list(RobotUtils.generate_sequences(current_pose_array, set_1_mid_step_target, cut))
    
    @staticmethod
    def FL_HR_down_step_2(current_pose, cut=30):
        current_pose_array = np.array(current_pose)
        set_1_mid_step_target = current_pose_array.copy()
        
        set_1_mid_step_target[0] = set_1_mid_step_target[0] + RobotUtils.UNIT_VALUE_FOR_A_STEP
        set_1_mid_step_target[1] = set_1_mid_step_target[1] + RobotUtils.UNIT_VALUE_FOR_A_STEP
            
        return list(RobotUtils.generate_sequences(current_pose_array, set_1_mid_step_target, cut))
        
    
    @staticmethod
    def rotx(alpha):
        c, s = cos(alpha), sin(alpha)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    @staticmethod
    def roty(beta):
        c, s = cos(beta), sin(beta)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    @staticmethod
    def rotz(gamma):
        c, s = cos(gamma), sin(gamma)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    @staticmethod
    def rotxyz(alpha, beta, gamma):
        return RobotUtils.rotx(alpha).dot(RobotUtils.roty(beta)).dot(RobotUtils.rotz(gamma))

    @staticmethod
    def homog_transxyz(dx, dy, dz):
        return np.array([[1, 0, 0, dx], [0, 1, 0, dy], [0, 0, 1, dz], [0, 0, 0, 1]])

    @staticmethod
    def homog_transform(dx, dy, dz, alpha, beta, gamma):
        rot4x4 = np.eye(4)
        rot4x4[:3, :3] = RobotUtils.rotxyz(alpha, beta, gamma)
        return np.dot(RobotUtils.homog_transxyz(dx, dy, dz), rot4x4)

    @staticmethod
    def homog_transform_inverse(matrix):
        inv = np.eye(4)
        inv[:3, :3] = matrix[:3, :3].T
        inv[:3, 3] = -np.dot(inv[:3, :3], matrix[:3, 3])
        return inv
