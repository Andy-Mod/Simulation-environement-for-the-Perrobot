#!/bin/python3

import rospy
from std_msgs.msg import Float64

class RobotPublisher:
    def __init__(self, rate):
        self.RATE = rate
        self.topics = {
            "all": [
                "/perrobot_12dof_controller/FL_HFE_joint/command",
                "/perrobot_12dof_controller/FL_KFE_joint/command",
                "/perrobot_12dof_controller/FR_HFE_joint/command",
                "/perrobot_12dof_controller/FR_KFE_joint/command",
                "/perrobot_12dof_controller/HR_HFE_joint/command",
                "/perrobot_12dof_controller/HR_KFE_joint/command",
                "/perrobot_12dof_controller/HL_HFE_joint/command",
                "/perrobot_12dof_controller/HL_KFE_joint/command",
                "/perrobot_12dof_controller/HR_HAA_joint/command",
                "/perrobot_12dof_controller/FR_HAA_joint/command",
                "/perrobot_12dof_controller/HL_HAA_joint/command",
                "/perrobot_12dof_controller/FL_HAA_joint/command"
            ],
            "FR": [
                "/perrobot_12dof_controller/FR_HAA_joint/command",
                "/perrobot_12dof_controller/FR_HFE_joint/command",
                "/perrobot_12dof_controller/FR_KFE_joint/command"
            ],
            "FL": [
                "/perrobot_12dof_controller/FL_HAA_joint/command",
                "/perrobot_12dof_controller/FL_HFE_joint/command",
                "/perrobot_12dof_controller/FL_KFE_joint/command"
            ],
            "HR": [
                "/perrobot_12dof_controller/HR_HAA_joint/command",
                "/perrobot_12dof_controller/HR_HFE_joint/command",
                "/perrobot_12dof_controller/HR_KFE_joint/command"
            ],
            "HL": [
                "/perrobot_12dof_controller/HL_HAA_joint/command",
                "/perrobot_12dof_controller/HL_HFE_joint/command",
                "/perrobot_12dof_controller/HL_KFE_joint/command"
            ]
        }
        
    def publisher_one_leg(self, values, leg):
        if leg not in self.topics:
            raise ValueError("Invalid leg identifier")

        topics = self.topics[leg]
        publishers = [rospy.Publisher(topic, Float64, queue_size=10) for topic in topics]

        rate = rospy.Rate(self.RATE)
        for joint_values in values:
            for i, pub in enumerate(publishers):
                message = Float64()
                message.data = joint_values[i]
                pub.publish(message)
                print(f"Publishing on {topics[i]}: {joint_values[i]}")
            rate.sleep()
    
    def publish_on_leg_set(self, values, leg_set):
        topics = []
        for leg in leg_set:
            if leg not in self.topics:
                raise ValueError("Invalid leg identifier")
            topics += self.topics[leg]

        publishers = [rospy.Publisher(topic, Float64, queue_size=10) for topic in topics]

        rate = rospy.Rate(self.RATE)
        for joint_values in values:
            for i, pub in enumerate(publishers):
                message = Float64()
                message.data = joint_values[i]
                pub.publish(message)
                print(f"Publishing on {topics[i]}: {joint_values[i]}")
            rate.sleep()
    
    def publishing_init_joint_poses(self, values):
        topics = self.topics["all"]
        publishers = [rospy.Publisher(topic, Float64, queue_size=10) for topic in topics]

        rate = rospy.Rate(self.RATE)
        for joint_values in values:
            for i, pub in enumerate(publishers):
                message = Float64()
                message.data = joint_values[i]
                pub.publish(message)
                print(f"Publishing on {topics[i]}: {joint_values[i]}")
            rate.sleep()
