#!/bin/python3

from robot_publisher_12dof import RobotPublisher
from moves12dof import Moves_12dof
import numpy as np
import rospy



if __name__ == '__main__':
    rospy.init_node('move_FR', anonymous=True)
    
    q2, q3 = Moves_12dof.hight_to_angles(h=Moves_12dof.TARGET_HEIGHT, L=Moves_12dof.HALF_LEG_LENGTH)
    qinit = [0.0, q2, q3]
    
    
    rate = 10
    controller = RobotPublisher(rate)
    values = Moves_12dof.move_foot(qinit, 0.16, 0.05, 10)
    
    
    
    controller.publishing_init_joint_poses(values, leg='FR')
    controller.publishing_init_joint_poses(values, leg='HL')
    rospy.signal_shutdown("Done publishing")