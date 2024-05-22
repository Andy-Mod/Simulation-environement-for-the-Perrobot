#!/bin/python3

from numpy import shape
from robot_controller import RobotController
from robot_utils import RobotUtils
import rospy



if __name__ == '__main__':
    current_pose = RobotUtils.hight_to_angles(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='x')
    final = RobotUtils.one_step(current_pose, 2)
    
    
    controller = RobotController(final)
    controller.publishing_init_joint_poses()
    
    rospy.signal_shutdown("Done publishing")