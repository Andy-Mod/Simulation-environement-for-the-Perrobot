#!/bin/python3

from robot_controller import RobotController
from robot_utils import RobotUtils
import rospy

init_height = 0.22

if __name__ == '__main__':
    values = RobotUtils.angles_for_height(init_height, RobotUtils.HALF_LEG_LENGTH, Rpose='x')
    controller = RobotController(values)
    
    controller.publishing_init_joint_poses()
    rospy.signal_shutdown("Done publishing")