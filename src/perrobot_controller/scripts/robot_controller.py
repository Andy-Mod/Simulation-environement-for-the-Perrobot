#!/bin/python3

import rospy
from std_msgs.msg import Float64
from robot_utils import RobotUtils

class RobotController:
    RATE = 10

    def __init__(self, init_height, half_leg_length, rpose='x', cut=20):
        self.init_height = init_height
        self.half_leg_length = half_leg_length
        self.rpose = rpose
        self.cut = cut

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
        
        if self.rpose == 'init':
            self.values = [RobotUtils.init_pose()]
        else:
            self.values = RobotUtils.angles_for_height(self.init_height, self.half_leg_length, self.cut, self.rpose)

    def publishing_init_joint_poses(self):
        rospy.init_node('init_joint_command_publisher')

        publishers = [rospy.Publisher(topic, Float64, queue_size=10) for topic in self.topics]

        rate = rospy.Rate(self.RATE)
        for values in self.values:
            for i in range(len(self.topics)):
                message = Float64()
                message.data = values[i]
                publishers[i].publish(message)
                print(f"Publishing on {self.topics[i]}: {values[i]}")
            rate.sleep()