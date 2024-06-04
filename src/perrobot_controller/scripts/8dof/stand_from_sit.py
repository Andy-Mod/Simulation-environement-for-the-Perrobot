#!/bin/python3

from robot_controller import RobotController
from robot_utils import RobotUtils
import rospy



if __name__ == '__main__':
    x = RobotUtils.hight_to_angles(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='x')
    current_pose = RobotUtils.sit_from_x(x, cut=5)
    values = RobotUtils.stand_from_sit(current_pose[-1], cut=5)
    controller = RobotController(values)
    
    controller.publishing_init_joint_poses()
    rospy.signal_shutdown("Done publishing")