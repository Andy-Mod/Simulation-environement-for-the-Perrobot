#!/bin/python3

from robot_controller import RobotController
import rospy

if __name__ == '__main__':
    controller = RobotController(init_height=0.22, half_leg_length=0.16, rpose='x')
    controller.publishing_init_joint_poses()
    rospy.signal_shutdown("Done publishing")