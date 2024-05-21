#!/bin/python3

from robot_controller import RobotController
from robot_utils import RobotUtils
import rospy



if __name__ == '__main__':
    current_pose = RobotUtils.hight_to_angles(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='x')
    first = RobotUtils.FL_HR_up_step(current_pose, 20)
    controller = RobotController(first)
    controller.publishing_init_joint_poses()
    
    # second = RobotUtils.FL_HR_down_step_1(first, 20)
    # controller = RobotController(second)
    # controller.publishing_init_joint_poses()
    
    # third = RobotUtils.FL_HR_down_step_1(second, 20)
    # controller = RobotController(third)
    # controller.publishing_init_joint_poses()
    
    
    
    rospy.signal_shutdown("Done publishing")