#!/bin/python3

import rospy
from std_msgs.msg import Float64
from robot_utils import angles_for_hight

RATE = 10
INIT_HEIGHT = 0.22
HALF_LEG_LENGTH = 0.16

Values = angles_for_hight(INIT_HEIGHT, HALF_LEG_LENGTH, Rpose='ncc')
print(Values)

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

def publishing_init_joint_poses():
    rospy.init_node('init_joint_command_publisher')

    publishers = []
    for topic in topics:
        publishers.append(rospy.Publisher(topic, Float64, queue_size=10))

    rate = rospy.Rate(RATE)
    for values in Values:
        for i in range(len(topics)):
            message = Float64()
            message.data = values[i]
            publishers[i].publish(message)
            print(f"Publishing on {topics[i]}: {values[i]}")
        
        rate.sleep()

if __name__ == '__main__':
    publishing_init_joint_poses()
    rospy.signal_shutdown("Done publishing")