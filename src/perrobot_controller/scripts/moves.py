#!/bin/python3

from numpy import shape
from robot_controller import RobotController
from robot_utils import RobotUtils

class Moves:
    @staticmethod
    def fall_recovery(cut=20):
        values = RobotUtils.fall_recovery(cut)
        controller = RobotController(values)
        
        controller.publishing_init_joint_poses()
    
    @staticmethod
    def four_legs_sequence(cut=20):
        values = RobotUtils.four_pose_sequence(cut=20)
        controller = RobotController(values)
        
        controller.publishing_init_joint_poses()
        
    @staticmethod
    def stand_x():
        values = RobotUtils.angles_for_height(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='x')
        controller = RobotController(values)
        
        controller.publishing_init_joint_poses()
    
    @staticmethod
    def stand_cc():
        values = RobotUtils.angles_for_height(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='cc')
        
        controller = RobotController(values)
        
        controller.publishing_init_joint_poses()
    
    @staticmethod
    def stand_nx():
        values = RobotUtils.angles_for_height(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='nx')
        controller = RobotController(values)
        
        controller.publishing_init_joint_poses()
    
    @staticmethod
    def stand_ncc():
        values = RobotUtils.angles_for_height(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='ncc')
        controller = RobotController(values)
        
        controller.publishing_init_joint_poses()
    
    @staticmethod
    def one_step_foward(cut=1):
        current_pose = RobotUtils.hight_to_angles(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='x')
        final = RobotUtils.one_step(current_pose, cut)
    
        controller = RobotController(final)
        controller.publishing_init_joint_poses()
    
    @staticmethod
    def one_step_backward(cut=1):
        current_pose = RobotUtils.hight_to_angles(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='x')
        final = RobotUtils.one_step(current_pose, cut=cut, foward=False)
    
        controller = RobotController(final)
        controller.publishing_init_joint_poses()
        
    @staticmethod
    def from_x_to_nx_and_back(cut=1):
        values = RobotUtils.from_X_to_NX_and_back(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH)
        controller = RobotController(values)
        
        controller.publishing_init_joint_poses()
        
    @staticmethod
    def sit_and_return_to_x(cut=30):
        x = RobotUtils.hight_to_angles(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='x')
        current_pose = RobotUtils.sit_from_x(x, cut=cut)
        values = RobotUtils.stand_from_sit(current_pose[-1], cut=cut)
        controller = RobotController(values)
        
        controller.publishing_init_joint_poses()