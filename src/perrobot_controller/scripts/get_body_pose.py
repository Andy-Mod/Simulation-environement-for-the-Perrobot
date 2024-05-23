#!/bin/python3

import rospy
from sensor_msgs.msg import Imu
import numpy as np
import tf.transformations


def imu_callback(data):
    
    orientation = data.orientation
    euler = tf.transformations.euler_from_quaternion([
        orientation.x, orientation.y, orientation.z, orientation.w
    ])
    rospy.loginfo('Euler angles: roll=%f, pitch=%f, yaw=%f', euler[0], euler[1], euler[2])

if __name__ == "__main__":

    rospy.init_node('imu_data_retriever')
    rospy.Subscriber('/perrobot/base_link_orientation', Imu, imu_callback)
    rospy.spin()
