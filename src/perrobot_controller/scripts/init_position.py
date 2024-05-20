#!/bin/python3

from robot_controller import RobotController
from robot_utils import RobotUtils
import rospy

REPEAT = 10

if __name__ == '__main__':
    values = [RobotUtils.init_pose()]
    controller = RobotController(values)
    for _ in range(REPEAT):
        controller.publishing_init_joint_poses()
    rospy.signal_shutdown("Done publishing")