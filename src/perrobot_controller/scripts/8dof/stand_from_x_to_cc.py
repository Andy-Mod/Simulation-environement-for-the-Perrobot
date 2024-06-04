#!/bin/python3

from robot_controller import RobotController
from robot_utils import RobotUtils
import rospy



if __name__ == '__main__':
    current_pose = RobotUtils.hight_to_angles(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='x')
    values = RobotUtils.from_x_to_cc(current_pose, cut=30)
    controller = RobotController(values)
    
    controller.publishing_init_joint_poses()
    rospy.signal_shutdown("Done publishing")