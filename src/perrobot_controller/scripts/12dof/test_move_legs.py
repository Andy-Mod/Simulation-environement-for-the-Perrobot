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
    values = Moves_12dof.move_foot(qinit, 0.075, 0.075, on_x=True, dt=0.01, sub=5) # safe sub=15
    value = Moves_12dof.move_foot(qinit, 0.075, 0.075, on_x=True, dt=0.01, sub=5, front=False)
    while 1 :
        controller.publishing_init_joint_poses(values, leg='FR')
        # rospy.sleep(0.1)
        controller.publishing_init_joint_poses(value, leg='HL')
        # rospy.sleep(0.1)
        controller.publishing_init_joint_poses(values, leg='FL')
        # rospy.sleep(0.1)
        controller.publishing_init_joint_poses(value, leg='HR')
    
    rospy.signal_shutdown("Done publishing")