#!/bin/python3

from robot_publisher_12dof import RobotPublisher
from moves12dof import Moves_12dof
import numpy as np
import rospy



if __name__ == '__main__':
    rospy.init_node('move_FR', anonymous=True)
    
    sets = [
        {
        'FR':True,
        'HL':False
        },
        {
        'FL':True,
        'HR':False
        } 
    ]
    q2, q3 = Moves_12dof.hight_to_angles(h=Moves_12dof.TARGET_HEIGHT, L=Moves_12dof.HALF_LEG_LENGTH)
    qinit = np.array([0.0, q2, q3])
    
    FL = Moves_12dof.move_foot(qinit, period=0.035, amplitude=0.02, on_x=True, num_points=10, sub=3, front=True)
    FR = Moves_12dof.move_foot(qinit, period=0.035, amplitude=0.02, on_x=True, num_points=10, sub=3, front=True)
    HL = Moves_12dof.move_foot(-qinit, period=0.099, amplitude=0.02, on_x=True, num_points=10, sub=3, front=True)
    HR = Moves_12dof.move_foot(-qinit, period=0.099, amplitude=0.02, on_x=True, num_points=10, sub=3, front=True)
    
    values = np.column_stack((FR, HL))
    values2 = np.column_stack((FL, HR))
    
    sets = ['FR', 'HL']
    sets2 = ['FL', 'HR']
    rate = 10
    controller = RobotPublisher(rate)
    
    while True:
        controller.publish_on_leg_set(values, sets)
        rospy.sleep(0.01)
        controller.publish_on_leg_set(values2, sets2)
        
    rospy.signal_shutdown("Done publishing")