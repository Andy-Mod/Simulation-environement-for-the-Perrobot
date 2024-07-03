from robot_publisher_12dof import RobotPublisher
from moves12dof import Moves_12dof
import numpy as np
import rospy


amp = 0.01  
length = 0.075
stance_coef = 1/3
start = [-0.0195, 0., -0.25]
gait_name = 'walk'
num_point = 20

Moves_12dof.gait(gait_name, amp, length, stance_coef, num_point, sub=30)