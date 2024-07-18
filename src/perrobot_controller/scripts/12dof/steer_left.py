#!/bin/python3

from robot_publisher_12dof import RobotPublisher
from moves12dof import Moves_12dof
import numpy as np
import rospy
from read_csv import read_csv_and_plot


if __name__ == '__main__':
    rospy.init_node('move_FR', anonymous=True)
    
    q2, q3 = Moves_12dof.hight_to_angles(h=Moves_12dof.TARGET_HEIGHT, L=Moves_12dof.HALF_LEG_LENGTH)
    qinit = np.array([0.0, q2, q3])
    
   
    FR = Moves_12dof.move_foot(qinit, period=-0.09, amplitude=0.01, on_x=False, num_points=6, front=True)
    FR_2 = Moves_12dof.move_foot(FR[-1], period=0.001, amplitude=0.00, on_x=False, num_points=6, front=True)
    first = list(FR)
    second = list(FR_2)
    
    
    first.append(qinit)
    print(first)
    rate = 10
    controller = RobotPublisher(rate)
    
    controller.publisher_one_leg(FR, 'FR')
    controller.publisher_one_leg(FR, 'FL')
    controller.publish_on_leg_set(first, ['FR', 'FL'])

    