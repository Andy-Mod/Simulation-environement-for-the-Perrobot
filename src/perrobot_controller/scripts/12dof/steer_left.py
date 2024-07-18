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
    
    FL_steer = Moves_12dof.move_foot(qinit, period=0.009, amplitude=0.005, on_x=False, num_points=6, front=True)
    # FL_back = Moves_12dof.move_foot(FL_steer[-1], period=-0.01, amplitude=0.025, on_x=False, num_points=6, front=True)
   
    # values = np.concatenate((FL_steer, FL_back[1:]))
    rate = 10
    controller = RobotPublisher(rate)
    
    # print(values)
    
    
    controller.publisher_one_leg(FL_steer, 'FL')
    