#!/bin/python3

from robot_publisher_12dof import RobotPublisher
from moves12dof import Moves_12dof
import numpy as np
import rospy



if __name__ == '__main__':
    rospy.init_node('move_FR', anonymous=True)
    
    q2, q3 = Moves_12dof.hight_to_angles(h=Moves_12dof.TARGET_HEIGHT, L=Moves_12dof.HALF_LEG_LENGTH)
    qinit = [0.0, q2, q3]
    
    values = Moves_12dof.move_foot(qinit, 0.075, 0.035, on_x=True, num_points=20, sub=8, front=True)
    set = ['FR', 'HL']
    
    rate = 10
    controller = RobotPublisher(rate)
    
    # controller.publish_on_leg_set(values, set)
    controller.publisher_one_leg(values, 'FL')
    