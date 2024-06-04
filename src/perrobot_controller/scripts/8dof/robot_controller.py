#!/bin/python3

import rospy
from std_msgs.msg import Float64

class RobotController:
    RATE = 10

    def __init__(self, values):

        self.topics = [
            "/perrobot_controller/FL_HFE_joint/command",
            "/perrobot_controller/FL_KFE_joint/command",
            "/perrobot_controller/FR_HFE_joint/command",
            "/perrobot_controller/FR_KFE_joint/command",
            "/perrobot_controller/HR_HFE_joint/command",
            "/perrobot_controller/HR_KFE_joint/command",
            "/perrobot_controller/HL_HFE_joint/command",
            "/perrobot_controller/HL_KFE_joint/command"
        ]
        
        self.values = values
        
    def publishing_init_joint_poses(self):

        publishers = [rospy.Publisher(topic, Float64, queue_size=10) for topic in self.topics]

        rate = rospy.Rate(self.RATE)
        for values in self.values:
            for i in range(len(self.topics)):
                message = Float64()
                message.data = values[i]
                publishers[i].publish(message)
                print(f"Publishing on {self.topics[i]}: {values[i]}")
            rate.sleep()