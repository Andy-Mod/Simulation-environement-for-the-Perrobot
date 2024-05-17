import rospy
from std_msgs.msg import Float64

# Define the joint names
joint_names = [
    "FL_HFE_joint",
    "FR_HFE_joint",
    "HL_HFE_joint",
    "HR_HFE_joint",
    "FL_KFE_joint",
    "FR_KFE_joint",
    "HL_KFE_joint",
    "HR_KFE_joint"
]

# Define the ROS topics for commanding each joint
joint_topics = ["/perrobot_controller/" + name + "/command" for name in joint_names]

def command_joint(joint_topic, angle):
    """
    Publishes a command to a joint.
    """
    pub = rospy.Publisher(joint_topic, Float64, queue_size=10)
    rospy.loginfo("Publishing command to {}: {}".format(joint_topic, angle))
    pub.publish(angle)

def command_all_joints(angles):
    """
    Publishes commands to all joints.
    """
    for i, angle in enumerate(angles):
        command_joint(joint_topics[i], angle)

if __name__ == '__main__':
    rospy.init_node('joint_commander', anonymous=True)
    
    # Example angles for each joint (in radians)
    # Adjust these values according to your robot's requirements
    angles = [0.78, 0.78, 0.78, 0.78, -0.78, -0.78, -0.78, -0.78]
    
    command_all_joints(angles)
    
    rospy.spin()
