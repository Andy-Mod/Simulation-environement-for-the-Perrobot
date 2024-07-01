#!/bin/python3

from robot_publisher_12dof import RobotPublisher
from moves12dof import Moves_12dof
import numpy as np
import rospy



if __name__ == '__main__':
    # rospy.init_node('move_FR', anonymous=True)
    values = []
    set = ['FR', 'HL']
    
    rate = 10
    controller = RobotPublisher(rate)
    
    # controller.publish_on_leg_set(values, set)
    # controller.publisher_one_leg(values, 'HR')
    controller.publishing_init_joint_poses(values)
    