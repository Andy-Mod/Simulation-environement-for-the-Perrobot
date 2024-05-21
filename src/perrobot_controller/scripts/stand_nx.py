#!/bin/python3

from robot_controller import RobotController
from robot_utils import RobotUtils
import rospy


if __name__ == '__main__':
    values = RobotUtils.angles_for_height(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='nx')
    controller = RobotController(values)
    
    controller.publishing_init_joint_poses()
    rospy.signal_shutdown("Done publishing")