#!/bin/python3

from robot_controller import RobotController
from robot_utils import RobotUtils
import rospy



if __name__ == '__main__':
    values = RobotUtils.four_pose_sequence(cut=20)
    controller = RobotController(values)
    
    controller.publishing_init_joint_poses()
    rospy.signal_shutdown("Done publishing")