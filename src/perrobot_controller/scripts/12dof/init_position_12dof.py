#!/bin/python3

from robot_publisher_12dof import RobotPublisher
from moves12dof import Moves_12dof
import rospy



if __name__ == '__main__':
    rospy.init_node('stand_x', anonymous=True)
    values = Moves_12dof.angles_for_height(Moves_12dof.TARGET_HEIGHT, Moves_12dof.HALF_LEG_LENGTH, cut=20, Rpose='stand_x')
    rate = 8
    controller = RobotPublisher(rate)
    
    
    controller.publishing_init_joint_poses(values)
    rospy.signal_shutdown("Done publishing")