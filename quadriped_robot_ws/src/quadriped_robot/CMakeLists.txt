cmake_minimum_required(VERSION 3.0.2)
project(quadriped_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES quadriped_robot
  # CATKIN_DEPENDS roscpp rospy std_msgs
  # DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME}_node src/quadriped_robot_node.cpp)

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
