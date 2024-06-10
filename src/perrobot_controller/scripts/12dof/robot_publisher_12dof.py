#!/bin/python3

import rospy
from std_msgs.msg import Float64

class RobotPublisher:
    

    def __init__(self, rate):
        self.RATE = rate
        self.all = [
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
        ]
        
        self.FR = [
            "/perrobot_12dof_controller/FR_HAA_joint/command",
            "/perrobot_12dof_controller/FR_HFE_joint/command",
            "/perrobot_12dof_controller/FR_KFE_joint/command"
        ]
        
        self.FL = [
            "/perrobot_12dof_controller/FL_HAA_joint/command",
            "/perrobot_12dof_controller/FL_HFE_joint/command",
            "/perrobot_12dof_controller/FL_KFE_joint/command"
        ]
        
        self.HR = [
            "/perrobot_12dof_controller/HR_HAA_joint/command",
            "/perrobot_12dof_controller/HR_HFE_joint/command",
            "/perrobot_12dof_controller/HR_KFE_joint/command"
        ]
        
        self.HL = [
            "/perrobot_12dof_controller/HL_HAA_joint/command",
            "/perrobot_12dof_controller/HL_HFE_joint/command",
            "/perrobot_12dof_controller/HL_KFE_joint/command"
        ]
        
        
        self.topics = []
        
    def publishing_init_joint_poses(self, values, leg=''):
        
        if leg == 'FR':
            self.topics = self.FR
        elif leg == 'FL':
            self.topics = self.FL
        elif leg == 'HL':
            self.topics = self.HL
        elif leg == 'HR':
            self.topics = self.HR
        else :
            self.topics = self.all
            
        publishers = [rospy.Publisher(topic, Float64, queue_size=10) for topic in self.topics]

        rate = rospy.Rate(self.RATE)
        for values in values:
            for i in range(len(self.topics)):
                message = Float64()
                message.data = values[i]
                publishers[i].publish(message)
                print(f"Publishing on {self.topics[i]}: {values[i]}")
                
            rate.sleep()