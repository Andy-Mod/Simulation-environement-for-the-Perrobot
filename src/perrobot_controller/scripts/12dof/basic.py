#!/bin/python3

from robot_publisher_12dof import RobotPublisher
from moves12dof import Moves_12dof
import numpy as np
import rospy
from read_csv import read_csv_and_plot


if __name__ == '__main__':
    rospy.init_node('move_basic', anonymous=True)
    values = Moves_12dof.basic_MLCU(50)
    rate = 10
    controller = RobotPublisher(rate)
    controller.publishing_init_joint_poses(values)
    rospy.signal_shutdown("Done publishing")