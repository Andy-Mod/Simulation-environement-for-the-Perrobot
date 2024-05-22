#!/bin/python3

import numpy as np
from numpy import arccos, arcsin, pi, cos, sin

class RobotUtils:
    HALF_LEG_LENGTH = 0.16
    TARGET_HEIGHT = 0.23
    UNIT_VALUE_FOR_A_STEP_F = pi/24
    UNIT_VALUE_FOR_A_STEP_B = pi/10
    SET_1 = [0, 1, 4, 5]
    SET_2 = [2, 3, 6, 7]
    FAST = 1
    SLOW = 2

    @staticmethod
    def init_pose(cut=10):
        current_pose = RobotUtils.hight_to_angles(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='x')
        alpha, beta = pi/2, pi
        goal = [alpha, beta, alpha, beta, -alpha, -beta, -alpha, -beta]
        return RobotUtils.generate_sequences(current_pose, goal, cut)

    @staticmethod
    def angles_for_height(h, L, cut=10, Rpose='x'):
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
    def from_init_to_pose(h, L, target_pose, cut=10):
        init = np.array(RobotUtils.init_pose())
        target = np.array(RobotUtils.hight_to_angles(h, L, target_pose))

        return RobotUtils.generate_sequences(init, target, cut)

    @staticmethod
    def from_pose_to_init(h, L, source_pose, cut=10):
        source = np.array(RobotUtils.hight_to_angles(h, L, source_pose))
        init = np.array(RobotUtils.init_pose())

        return RobotUtils.generate_sequences(source, init, cut)
    
    @staticmethod
    def from_X_to_NX(h, L, cut=10):
        source_pose = 'x'
        target_pose = 'nx'
        source = np.array(RobotUtils.hight_to_angles(h, L, source_pose))
        target = np.array(RobotUtils.hight_to_angles(h, L, target_pose))

        return RobotUtils.generate_sequences(source, target, cut)
    
    @staticmethod
    def from_NX_to_X(h, L, cut=10):
        source_pose = 'nx'
        target_pose = 'x'
        source = np.array(RobotUtils.hight_to_angles(h, L, source_pose))
        target = np.array(RobotUtils.hight_to_angles(h, L, target_pose))
        
        return RobotUtils.generate_sequences(source, target, cut)
    
    @staticmethod
    def one_step(current_pose, cut=10, foward=True):
        current_pose_array = np.array(current_pose).copy()
        set_1 = np.array(current_pose).copy()
        set_2 = np.array(current_pose).copy()
        
        
        offsets = np.array([- RobotUtils.UNIT_VALUE_FOR_A_STEP_F,
                   - RobotUtils.UNIT_VALUE_FOR_A_STEP_F, 
                   - RobotUtils.UNIT_VALUE_FOR_A_STEP_F, 
                     RobotUtils.UNIT_VALUE_FOR_A_STEP_F
                ]) if foward else np.array([RobotUtils.UNIT_VALUE_FOR_A_STEP_B,
                     RobotUtils.UNIT_VALUE_FOR_A_STEP_B, 
                     RobotUtils.UNIT_VALUE_FOR_A_STEP_B, 
                     RobotUtils.UNIT_VALUE_FOR_A_STEP_B
                ])
        
        set_1[RobotUtils.SET_1] += offsets
        set_2[RobotUtils.SET_2] += offsets
          
        first = RobotUtils.generate_sequences(current_pose, set_1, cut)
        second = RobotUtils.generate_sequences(first[-1], current_pose, cut)
        third = RobotUtils.generate_sequences(second[-1], set_2, cut)
        last = RobotUtils.generate_sequences(third[-1], current_pose, cut)
        
        return first + second + third + last
    
    @staticmethod
    def walk_from_x(current_pose, number_of_steps=5, fast=False, foward=True):
        steps = []
        cut = RobotUtils.FAST if fast else RobotUtils.SLOW
        cut = cut + 1 if foward and not fast else cut
        
        RobotUtils.UNIT_VALUE_FOR_A_STEP_B = pi/12 if not fast and not foward else RobotUtils.UNIT_VALUE_FOR_A_STEP_B
        
        for _ in range(number_of_steps):
            steps += RobotUtils.one_step(current_pose, cut, foward)
        
        return steps
    
    @staticmethod
    def from_x_to_cc(current_pose, cut=10):
        goal = np.array(current_pose).copy()
        goal[4::] += -np.array(RobotUtils.init_pose())[4::]

        return RobotUtils.generate_sequences(current_pose, goal, cut)
    
    @staticmethod
    def sit_from_x(current_pose, cut=10):
        goal = np.array(current_pose).copy()
        goal[4::] += (-np.array(RobotUtils.init_pose())[4::]/4)

        return RobotUtils.generate_sequences(current_pose, goal, cut)
    
    @staticmethod
    def stand_from_sit(current_pose, cut=10):
        goal = np.array(current_pose).copy()
        goal[4::] += (np.array(RobotUtils.init_pose())[4::]/4)

        return RobotUtils.generate_sequences(current_pose, goal, cut)
    
    @staticmethod
    def four_pose_sequence(cut=10):
        steps = []
        x = RobotUtils.hight_to_angles(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='x')
        nx = RobotUtils.hight_to_angles(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='nx')
        cc = RobotUtils.hight_to_angles(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='cc')
        ncc = RobotUtils.hight_to_angles(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='ncc')
        
        first = RobotUtils.generate_sequences(x, ncc, cut)
        second = RobotUtils.generate_sequences(first[-1], nx, cut)
        third = RobotUtils.generate_sequences(second[-1], cc, cut)
        quad = RobotUtils.generate_sequences(third[-1], x, cut)
        
        
        return first + second + third + quad 
    
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
