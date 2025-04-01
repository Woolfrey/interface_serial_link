/**
 * @file    joy_twist_mapper_exec.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    March 2025
 * @version 1.0
 * @brief   Creates & spins a ROS2 node for the JoyTwistMapper class.
 * 
 * @details This file contains the source code for running the JoyTwistMapper class.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <serial_link_interfaces/joy_twist_mapper.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                                                       // Start up ROS2

    auto node = std::make_shared<serial_link_interfaces::JoyTwistMapper>();                         // Create node
    
    rclcpp::spin(node);                                                                             // Run the node indefinitely

    rclcpp::shutdown();                                                                             // Stop ROS2
    
    return 0;                                                                                       // No problems with main
}
