from robot_publisher_12dof import RobotPublisher
from moves12dof import Moves_12dof
import numpy as np
import rospy


if __name__ == '__main__':
    rospy.init_node('move_FR', anonymous=True) 
    
    amp = 0.05
    length = -0.075
    stance_coef = 1/3
    q2, q3 = Moves_12dof.hight_to_angles(h=Moves_12dof.TARGET_HEIGHT, L=Moves_12dof.HALF_LEG_LENGTH)
    qinit = np.array([0.0, q2, q3])
    start = Moves_12dof.MGD(qinit)
    gait_name = 'trot'
    rate = 1000 * 0.055

    Moves_12dof.gait2(gait_name, amp, length, stance_coef, rate)

   
    