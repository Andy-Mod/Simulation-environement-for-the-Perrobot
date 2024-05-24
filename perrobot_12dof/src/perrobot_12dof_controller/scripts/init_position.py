#!/bin/python3

from robot_controller import RobotController
from robot_utils import RobotUtils
import rospy

if __name__ == '__main__':
    rospy.init_node('stand_x', anonymous=True)
    values = RobotUtils.init_pose()
    controller = RobotController(values)
    
    controller.publishing_init_joint_poses()
    rospy.signal_shutdown("Done publishing")