#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
import csv
import atexit
import tf

# File to save the data
save_file = '/home/perrobot/perrobot/src/perrobot_controller/scripts/12dof/yaw.csv'

# Dictionary to store yaw values
values = {'yaw': []}

def save_to_csv():
    """
    Save the collected yaw values data to a CSV file.
    """
    with open(save_file, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)

        # Write header
        csv_writer.writerow(values.keys())

        # Write yaw values
        max_length = max(len(yaws) for yaws in values.values())
        for i in range(max_length):
            row = [values[name][i] if i < len(values[name]) else '' for name in values.keys()]
            csv_writer.writerow(row)

def model_states_callback(msg):
    robot_model_name = 'perrobot_12dof'

    if robot_model_name in msg.name:
        index = msg.name.index(robot_model_name)
        orientation = msg.pose[index].orientation

        # Convert quaternion to Euler angles to get the yaw
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        values['yaw'].append(yaw)
        rospy.loginfo(f"Robot model {robot_model_name} yaw: {yaw}")

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
