#!/bin/python3

import rospy
from std_msgs.msg import Float64
from robot_utils import hight_to_angles

RATE = 10
INIT_HEIGHT = 0.20
HALF_LEG_LENGTH = 0.16
REPEAT = 20

alpha, beta = hight_to_angles(INIT_HEIGHT, HALF_LEG_LENGTH)

topics = [
    "/perrobot_controller/FL_HFE_joint/command",
    "/perrobot_controller/FL_KFE_joint/command",
    "/perrobot_controller/FR_HFE_joint/command",
    "/perrobot_controller/FR_KFE_joint/command",
    "/perrobot_controller/HR_HFE_joint/command",
    "/perrobot_controller/HR_KFE_joint/command",
    "/perrobot_controller/HL_HFE_joint/command",
    "/perrobot_controller/HL_KFE_joint/command"
]

values = [
    alpha,
    beta, 
    alpha,
    beta,
    -alpha,
    -beta,
    -alpha,
    -beta
]

def publishing_init_joint_poses():
    rospy.init_node('init_joint_command_publisher')

    publishers = []
    for topic in topics:
        publishers.append(rospy.Publisher(topic, Float64, queue_size=10))

    rate = rospy.Rate(RATE)
    for _ in range(REPEAT):  
        for i in range(len(topics)):
            message = Float64()
            message.data = values[i]
            publishers[i].publish(message)
            print(f"Publishing on {topics[i]}: {values[i]}")
        
        rate.sleep()

if __name__ == '__main__':
    publishing_init_joint_poses()
    rospy.signal_shutdown("Done publishing")