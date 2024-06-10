#!/bin/python3

import rospy
from sensor_msgs.msg import JointState

# Function to handle received messages
def reader(data, set='all'):
    joint_names = data.name
    joint_positions = data.position
    other = ['FR', 'FL','HR', 'HL']
    positions = None 
    
    if set == 'all':
        # Print all joint positions
        positions = {name: pos for name, pos in zip(joint_names, joint_positions)}
        
    
    elif set in other:
        positions = {name: pos for name, pos in zip(joint_names, joint_positions) if set in name }
        
    print(positions)
    return positions
    
    

# Listener function that subscribes to the joint states topic
def listener(topic, set='all'):
    # rospy.init_node("joint_state_position_reader")
    rospy.Subscriber(topic, JointState, reader, callback_args=set)
    rospy.spin()

if __name__ == "__main__":
    # Start the listener with the desired set
    listener('/perrobot_12dof_controller/joint_states', 'FR')  # or 'FL', 'HR', 'HL', or 'all'