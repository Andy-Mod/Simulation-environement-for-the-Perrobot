import rospy
from std_msgs.msg import Float64

RATE = 0.5  # Messages per second

topics = [
    "/perrobot_controller/FL_HFE_joint/command",
    "/perrobot_controller/FL_KFE_joint/command",
    "/perrobot_controller/FR_HFE_joint/command",
    "/perrobot_controller/FR_KFE_joint/command",
    "/perrobot_controller/HR_HFE_joint/command",
    "/perrobot_controller/HR_KFE_joint/command",
    "/perrobot_controller/HL_HFE_joint/command",
    "/perrobot_controller/HL_KFE_joint/command"
]

joints = [
    "FL_HFE",
    "FL_KFE",
    "FR_HFE",
    "FR_KFE",
    "HR_HFE",
    "HR_KFE",
    "HL_HFE",
    "HL_KFE"
]

values = [
    0.68,
    -1.35,
    0.68,
    -1.35,
    -0.68,
    1.35,
    -0.68,
    1.35
]

# Initialize ROS Node (single node)
rospy.init_node('joint_command_publisher')

# Create publishers for each topic
publisers = []
for topic in topics:
  publisers.append(rospy.Publisher(topic, Float64, queue_size=10))

# Publish messages (once per topic)
rate = rospy.Rate(RATE)
for i in range(len(topics)):
  # Create message
  message = Float64()
  message.data = values[i]

  # Publish message
  publisers[i].publish(message)
  print("Publishing on "+topics[i])

  # Sleep to maintain publishing rate
  rate.sleep()

# Shutdown the node (optional)
rospy.signal_shutdown("Done publishing")
