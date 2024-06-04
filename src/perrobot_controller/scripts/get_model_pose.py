#!/bin/python3 

import rospy
from gazebo_msgs.msg import ModelStates

def model_states_callback(data):
    rospy.loginfo("Model States Received")
    for i, name in enumerate(data.name):
        if name == 'perrobot':
            position = data.pose[i].position
            orientation = data.pose[i].orientation
            rospy.loginfo("Model: %s", name)
            rospy.loginfo("  Position - x: %f, y: %f, z: %f", position.x, position.y, position.z)
            rospy.loginfo("  Orientation - x: %f, y: %f, z: %f, w: %f", orientation.x, orientation.y, orientation.z, orientation.w)
            break

def listener():
    rospy.init_node('gazebo_model_states_listener', anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
