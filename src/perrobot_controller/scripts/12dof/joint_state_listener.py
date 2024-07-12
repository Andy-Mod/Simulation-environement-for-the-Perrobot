#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import csv
import atexit
import matplotlib.pyplot as plt
from robot_publisher_12dof import RobotPublisher
from moves12dof import Moves_12dof

# Dictionary to store joint names and their positions
joint_positions_data = {}

def joint_states_callback(msg):
    """
    Callback function to process data from the /joint_states topic.
    """
    # Extract joint names and positions
    joint_names = msg.name
    joint_positions = msg.position

    # Append the positions to the respective joint names in the dictionary
    for name, position in zip(joint_names, joint_positions):
        if name not in joint_positions_data:
            joint_positions_data[name] = []
        joint_positions_data[name].append(position)
    
    

def save_to_csv():
    """
    Save the collected joint positions data to a CSV file.
    """
    with open('/home/perrobot/perrobot/src/perrobot_controller/scripts/12dof/joint_positions.csv', 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)

        # Write header (joint names)
        csv_writer.writerow(joint_positions_data.keys())

        # Write positions
        # Get the maximum number of recorded positions
        max_length = max(len(positions) for positions in joint_positions_data.values())

        # Write each position value, filling with empty strings if necessary
        for i in range(max_length):
            row = [joint_positions_data[name][i] if i < len(joint_positions_data[name]) else '' for name in joint_positions_data.keys()]
            csv_writer.writerow(row)

def listener():
    """
    Initializes the node and subscribes to the /joint_states topic.
    """
    # Initialize the node
    rospy.init_node('joint_state_listener', anonymous=True)

    # Subscribe to the /joint_states topic
    rospy.Subscriber("/perrobot_12dof_controller/joint_states", JointState, joint_states_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        # Register the save_to_csv function to be called upon script exit
        
        
        
        atexit.register(save_to_csv)
        listener()
    except rospy.ROSInterruptException:
        pass
