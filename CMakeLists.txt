cmake_minimum_required(VERSION 3.8)
project(serial_link_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################################################################################################
#                                         DEPENDENCIES                                             #
####################################################################################################
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

####################################################################################################
#                                      GENERATE INTERFACES                                         #
####################################################################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowTransform.action"
  "action/FollowTwist.action"
  "action/TrackCartesianTrajectory.action"
  "action/TrackJointTrajectory.action"
  "msg/CartesianState.msg"
  "msg/CartesianTrajectoryPoint.msg"
  "msg/JointCommand.msg"
  "msg/JointState.msg"
  "msg/JointTrajectoryPoint.msg"
  "msg/Statistics.msg"
  DEPENDENCIES std_msgs geometry_msgs
)


ament_package()
