cmake_minimum_required(VERSION 3.8)
project(serial_link_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################################################################################################
#                                         DEPENDENCIES                                             #
####################################################################################################

# ROS2 packages:
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Header files:
include_directories(include)

####################################################################################################
#                                      GENERATE INTERFACES                                         #
####################################################################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowTransform.action"
  "action/FollowTwist.action"
  "action/HoldConfiguration.action"
  "action/HoldPose.action"
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

####################################################################################################
#                                             EXECUTABLES                                          #
####################################################################################################

# Interactive marker
add_executable(interactive_marker src/nodes/interactive_marker_node.cpp
                                  src/interactive_marker.cpp)
                                         
ament_target_dependencies(interactive_marker "ament_cmake"
                                             "geometry_msgs"
                                             "interactive_markers"
                                             "rclcpp"
                                             "tf2_ros"
                                             "visualization_msgs")

# Joy/Twist mapper
add_executable(joy_twist_mapper src/nodes/joy_twist_mapper_node.cpp
                                src/joy_twist_mapper.cpp)
                                         
ament_target_dependencies(joy_twist_mapper "ament_cmake"
                                           "geometry_msgs"
                                           "rclcpp"
                                           "sensor_msgs")
 
####################################################################################################
#                                       INSTALLATION INSTRUCTIONS                                  #
####################################################################################################      
                                       
install(TARGETS interactive_marker
                joy_twist_mapper
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
