#!/bin/python3

from robot_controller import RobotController
import rospy

REPEAT = 10

if __name__ == '__main__':
    controller = RobotController(init_height=0.22, half_leg_length=0.16, rpose='init')
    for _ in range(REPEAT):
        controller.publishing_init_joint_poses()
    rospy.signal_shutdown("Done publishing")