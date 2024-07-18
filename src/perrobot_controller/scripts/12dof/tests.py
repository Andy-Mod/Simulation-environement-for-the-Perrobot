from robot_publisher_12dof import RobotPublisher
from moves12dof import Moves_12dof
import numpy as np
import rospy


if __name__ == '__main__':
    rospy.init_node('move_FR', anonymous=True) 
    
    amp = 0.025
    length = -0.055
    stance_coef = 1/3
    q2, q3 = Moves_12dof.hight_to_angles(h=Moves_12dof.TARGET_HEIGHT, L=Moves_12dof.HALF_LEG_LENGTH)
    qinit = np.array([0.0, q2, q3])
    start = Moves_12dof.MGD(qinit)
    gait_name = 'walk'
    num_point = 6

    rate = 10
    controller = RobotPublisher(rate)
    legs, values = Moves_12dof.gait(gait_name, amp, length, stance_coef, num_point)
    
    FL, FR, HL, HR = values[0], values[1], values[2], values[3]
    
    
    
    values = np.column_stack((FR, HL))
    values2 = np.column_stack((FL, HR))
    
    sets = ['FR', 'HL']
    sets2 = ['FL', 'HR']
    
    while True:
        controller.publish_on_leg_set(values, sets)
        controller.publish_on_leg_set(values2, sets2) 