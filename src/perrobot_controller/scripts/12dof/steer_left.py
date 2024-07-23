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
    
    initial_vector = Moves_12dof.MGD(qinit)
    angle_in_radians = np.pi / 2
    
    ik = Moves_12dof.steer(initial_vector, angle_in_radians)
    ik = np.concatenate((ik, [qinit]))

    rate = 10
    controller = RobotPublisher(rate)
    
    
    controller.publisher_one_leg(ik, 'FL')
    rospy.signal_shutdown("Done publishing")