#!/bin/python3

from robot_controller import RobotController
from robot_utils import RobotUtils
import rospy


if __name__ == '__main__':
    values = RobotUtils.from_floor_to_X(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH)
    controller = RobotController(values)
    
    controller.publishing_init_joint_poses()
    rospy.signal_shutdown("Done publishing")