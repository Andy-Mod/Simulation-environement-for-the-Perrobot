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
    
    set1, values = Moves_12dof.move_set_of_legs(sets[0], 0.075, 0.025, on_x=True, sub=8)
    set2, value = Moves_12dof.move_set_of_legs(sets[1], 0.075, 0.025, on_x=True, sub=8)
    
    rate = 10
    controller = RobotPublisher(rate)
    
    controller.publish_on_leg_set(values, set1)
    controller.publish_on_leg_set(value, set2)
    
    rospy.signal_shutdown("Done publishing")