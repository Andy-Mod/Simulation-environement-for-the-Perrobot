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
    
    FL = Moves_12dof.move_foot(qinit, period=0.065, amplitude=0.035, on_x=True, num_points=6, front=True)
    FR = Moves_12dof.move_foot(qinit, period=0.065, amplitude=0.035, on_x=True, num_points=6, front=True)
    HL = Moves_12dof.move_foot(qinit, period=0.065, amplitude=0.035, on_x=True, num_points=6, front=False)
    HR = Moves_12dof.move_foot(qinit, period=0.065, amplitude=0.035, on_x=True, num_points=6, front=False)
    
    values = np.column_stack((FR, HL))
    values2 = np.column_stack((FL, HR))
    
    sets = ['FR', 'HL']
    sets2 = ['FL', 'HR']
    rate = 10
    controller = RobotPublisher(rate)
    
    while True:
        controller.publisher_one_leg(FL, 'FL')
        controller.publisher_one_leg(FR, 'FR')
        controller.publisher_one_leg(HL, 'HL')
        controller.publisher_one_leg(HR, 'HR')
        
        
    rospy.signal_shutdown("Done publishing")