#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
import csv
import atexit

# File to save the data
save_file = '/home/perrobot/perrobot/src/perrobot_controller/scripts/12dof/z.csv'

# Dictionary to store z position values
values = {'position_z': []}

def save_to_csv():
    """
    Save the collected z position data to a CSV file.
    """
    with open(save_file, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)

        # Write header
        csv_writer.writerow(values.keys())

        # Write positions
        # Get the maximum number of recorded positions
        max_length = max(len(positions) for positions in values.values())

        # Write each position value, filling with empty strings if necessary
        for i in range(max_length):
            row = [values[name][i] if i < len(values[name]) else '' for name in values.keys()]
            csv_writer.writerow(row)

def model_states_callback(msg):

    robot_model_name = 'perrobot_12dof'

    if robot_model_name in msg.name:
        index = msg.name.index(robot_model_name)
        position_z = msg.pose[index].position.z
        values['position_z'].append(position_z - 5 * 0.001)

def listener():
    rospy.init_node('model_states_listener', anonymous=True)
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        # Register the save_to_csv function to be called upon script exit
        atexit.register(save_to_csv)
        listener()
    except rospy.ROSInterruptException:
        pass
