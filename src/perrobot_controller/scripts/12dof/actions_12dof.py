#!/bin/python3

import rospy
from std_msgs.msg import String
from moves12dof import Moves_12dof
import numpy as np
from robot_publisher_12dof import RobotPublisher
from robot_utils import RobotUtils

def action_sender(data):
    key = data.data
    if key == 'Up':
        print("walk Foward")
        q2, q3 = Moves_12dof.hight_to_angles(h=Moves_12dof.TARGET_HEIGHT, L=Moves_12dof.HALF_LEG_LENGTH)
        qinit = np.array([0.0, q2, q3])
        num = 10
        
        FL = Moves_12dof.move_foot(qinit, period=0.055, amplitude=0.015, on_x=True, num_points=6, front=True)
        FR = Moves_12dof.move_foot(qinit, period=0.055, amplitude=0.015, on_x=True, num_points=6, front=True)
        HL = Moves_12dof.move_foot(-qinit, period=0.055, amplitude=0.015, on_x=True, num_points=6, front=False)
        HR = Moves_12dof.move_foot(-qinit, period=0.055, amplitude=0.015, on_x=True, num_points=6, front=False)
        
        values = np.column_stack((FR, HL))
        values2 = np.column_stack((FL, HR))
        
        sets = ['FR', 'HL']
        sets2 = ['FL', 'HR']
        rate = num
        controller = RobotPublisher(rate)
        
        controller.publish_on_leg_set(values, sets)
        controller.publish_on_leg_set(values2, sets2)
        
    elif key == 'Down':
        print("walk backward")
        q2, q3 = Moves_12dof.hight_to_angles(h=Moves_12dof.TARGET_HEIGHT, L=Moves_12dof.HALF_LEG_LENGTH)
        qinit = np.array([0.0, q2, q3])
        num = 10
        
        FL = Moves_12dof.move_foot(qinit, period=-0.055, amplitude=0.015, on_x=True, num_points=6, front=True)
        FR = Moves_12dof.move_foot(qinit, period=-0.055, amplitude=0.015, on_x=True, num_points=6, front=True)
        HL = Moves_12dof.move_foot(-qinit, period=-0.055, amplitude=0.015, on_x=True, num_points=6, front=False)
        HR = Moves_12dof.move_foot(-qinit, period=-0.055, amplitude=0.015, on_x=True, num_points=6, front=False)
        
        values = np.column_stack((FR, HL))
        values2 = np.column_stack((FL, HR))
        
        sets = ['FR', 'HL']
        sets2 = ['FL', 'HR']
        rate = num
        controller = RobotPublisher(rate)
        
        controller.publish_on_leg_set(values, sets)
        controller.publish_on_leg_set(values2, sets2)
        
    elif key == 'l':
        print("Four legs sequences")
        x = RobotUtils.hight_to_angles(Moves_12dof.TARGET_HEIGHT, Moves_12dof.HALF_LEG_LENGTH, Rpose='x')
        nx = RobotUtils.hight_to_angles(Moves_12dof.TARGET_HEIGHT, Moves_12dof.HALF_LEG_LENGTH, Rpose='nx')
        cc = RobotUtils.hight_to_angles(Moves_12dof.TARGET_HEIGHT, Moves_12dof.HALF_LEG_LENGTH, Rpose='cc')
        ncc = RobotUtils.hight_to_angles(Moves_12dof.TARGET_HEIGHT, Moves_12dof.HALF_LEG_LENGTH, Rpose='ncc')
        
        first = Moves_12dof.generate_sequences(x, ncc, 15)
        second = Moves_12dof.generate_sequences(first[-1], nx, 15)
        third = Moves_12dof.generate_sequences(second[-1], cc, 15)
        quad = Moves_12dof.generate_sequences(third[-1], x, 15)
        
        
        s = first + second + third + quad 
        q1 = np.zeros((len(s), 4))
        
        out = np.column_stack((s, q1))
        print(out)
        rate = 10
        controller = RobotPublisher(rate)
        
        controller.publishing_init_joint_poses(out)
        
    elif key == 'x':
        print("Go to position x")
        path = np.linspace(0, Moves_12dof.TARGET_HEIGHT, 15)
        Values = []

        for height in path:
            s = RobotUtils.hight_to_angles(height, Moves_12dof.HALF_LEG_LENGTH, Rpose='x')
            Values.append(s)

        q1 = np.zeros((len(Values), 4))
        out = np.column_stack((Values, q1))
        
        rate = 10
        controller = RobotPublisher(rate)
        controller.publishing_init_joint_poses(out)
        
        
    elif key == '<':
        print("Go to position cc")
        path = np.linspace(0, Moves_12dof.TARGET_HEIGHT, 15)
        Values = []

        for height in path:
            s = RobotUtils.hight_to_angles(height, Moves_12dof.HALF_LEG_LENGTH, Rpose='cc')
            Values.append(s)

        q1 = np.zeros((len(Values), 4))
        out = np.column_stack((Values, q1))
        
        rate = 10
        controller = RobotPublisher(rate)
        controller.publishing_init_joint_poses(out)
        
    elif key == '>':
        print("Go to position ↃↃ")
        path = np.linspace(0, Moves_12dof.TARGET_HEIGHT, 15)
        Values = []

        for height in path:
            s = RobotUtils.hight_to_angles(height, Moves_12dof.HALF_LEG_LENGTH, Rpose='ncc')
            Values.append(s)

        q1 = np.zeros((len(Values), 4))
        out = np.column_stack((Values, q1))
        
        rate = 10
        controller = RobotPublisher(rate)
        controller.publishing_init_joint_poses(out)
        
    elif key == 'n':
        print("Go to position ><")
        path = np.linspace(0, Moves_12dof.TARGET_HEIGHT, 15)
        Values = []

        for height in path:
            s = RobotUtils.hight_to_angles(height, Moves_12dof.HALF_LEG_LENGTH, Rpose='nx')
            Values.append(s)

        q1 = np.zeros((len(Values), 4))
        out = np.column_stack((Values, q1))
        
        rate = 10
        controller = RobotPublisher(rate)
        controller.publishing_init_joint_poses(out)
        
    elif key == 'v':
        print("from x to nx and back")
        source_pose = 'x'
        target_pose = 'nx'
        source = np.array(RobotUtils.hight_to_angles(Moves_12dof.TARGET_HEIGHT, Moves_12dof.HALF_LEG_LENGTH, source_pose))
        target = np.array(RobotUtils.hight_to_angles(Moves_12dof.TARGET_HEIGHT, Moves_12dof.HALF_LEG_LENGTH, target_pose))

        first = RobotUtils.generate_sequences(source, target, 15)
        last = RobotUtils.generate_sequences(first[-1], source, 15)
        
        s = first + last
        
        q1 = np.zeros((len(s), 4))
        out = np.column_stack((s, q1))
        rate = 10
        controller = RobotPublisher(rate)
        controller.publishing_init_joint_poses(out)
        
    elif key == 'f':
        print("fall recovery")
        q2, q3 =  Moves_12dof.hight_to_angles(Moves_12dof.TARGET_HEIGHT, Moves_12dof.HALF_LEG_LENGTH)
        goal = Moves_12dof.pose(q2, q3, Rpose='fall_recovery')
        start = Moves_12dof.pose(q2, q3, Rpose='stand_x')
        
        out = RobotUtils.generate_sequences(start, goal, 15)
        
        rate = 10
        controller = RobotPublisher(rate)
        controller.publishing_init_joint_poses(out)
        
    elif key == 's':
        print("sit")
        x = RobotUtils.hight_to_angles(RobotUtils.TARGET_HEIGHT, RobotUtils.HALF_LEG_LENGTH, Rpose='x')
        current_pose = RobotUtils.sit_from_x(x, cut=15)
        values = RobotUtils.stand_from_sit(current_pose[-1], cut=15)
        
        q1 = np.zeros((len(values), 4))
        out = np.column_stack((values, q1))
        rate = 10
        controller = RobotPublisher(rate)
        controller.publishing_init_joint_poses(out)
    

def listener():
    rospy.init_node("teleop_key_listener")
    rospy.Subscriber("key_pressed", String, action_sender)
    rospy.spin()


if __name__ == '__main__':
    listener()
