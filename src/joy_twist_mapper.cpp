/**
 * @file    joy_twist_map.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    March 2025
 * @version 1.0
 * @brief   Source code for the JoyTwistMapper class
 * 
 * @details This node subscribes to a sensor_msgs/msg/Joy topic, and converts the signals to a
 *          geometry_msgs/msg/TwistStamped topic. This can then be used as a command input for a robot.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */

#include <serial_link_interfaces/joy_twist_mapper.hpp>

namespace serial_link_interfaces {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
JoyTwistMapper::JoyTwistMapper(const std::string &nodeName)
: Node(nodeName)
{
    std::array<std::string, 6> fieldNames = {"linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"};
    
    for (int i = 0; i < 6; ++i)
    {
        _defaultMap[i].axis            = this->declare_parameter("joy_twist_map.default.axis." + fieldNames[i], -1);
        _defaultMap[i].button_positive = this->declare_parameter("joy_twist_map.default.button.positive." + fieldNames[i], -1);
        _defaultMap[i].button_negative = this->declare_parameter("joy_twist_map.default.button.negative." + fieldNames[i], -1);
    }
    
    // Internal parameters
    _frameName              = this->declare_parameter<std::string>("joy_twist_map.frame_name", "world");
    _maxAngularAcceleration = this->declare_parameter<double>("joy_twist_map.max_angular_acceleration", 0.25);
    _maxAngularVelocity     = this->declare_parameter<double>("joy_twist_map.max_angular_velocity", 0.1);
    _maxLinearAcceleration  = this->declare_parameter<double>("joy_twist_map.max_linear_acceleration", 1.0);
    _maxLinearVelocity      = this->declare_parameter<double>("joy_twist_map.max_linear_velocity", 0.2);
    _timeout                = this->declare_parameter<double>("joy_twist_map.timeout", 0.5);

    // Create the subscriber
    std::string subscriptionName = this->declare_parameter<std::string>("joy_twist_map.subscription_name", "joy");
   
    _subscriber = this->create_subscription<sensor_msgs::msg::Joy>(subscriptionName, 1, std::bind(&JoyTwistMapper::joy_callback, this, std::placeholders::_1));

    // Create the publisher
    std::string publisherName = this->declare_parameter<std::string>("joy_twist_map.publisher_name", "twist_command");
   
    _publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>(publisherName, 1);
    
    // Timer for continuous publishing
    int frequency = this->declare_parameter("joy_twist_map.publisher_frequency", 20);
    
    _timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000/frequency)), std::bind(&JoyTwistMapper::publish_twist, this));
    
    // Set initial values
    _lastInputTime = this->now();                                                                   // We need this to start things
     
    _oldTwist.linear.x  = 0.0;
    _oldTwist.linear.y  = 0.0;
    _oldTwist.linear.z  = 0.0;
    _oldTwist.angular.x = 0.0;
    _oldTwist.angular.y = 0.0;
    _oldTwist.angular.z = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "Started the '%s' node. Subscribing to the '%s' topic. Publishing to the '%s' topic at %u Hz in the `%s` frame.",
                nodeName.c_str(), subscriptionName.c_str(), publisherName.c_str(), (unsigned int)(1000 / frequency) , _frameName.c_str());
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Callback method for sensor_msgs/msg/Joy topic                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
JoyTwistMapper::joy_callback(const std::shared_ptr<const sensor_msgs::msg::Joy> &input)

{
    _lastInput = *input;
    _lastInputTime = this->now();
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Continually publish twist commands                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
JoyTwistMapper::publish_twist()
{
    geometry_msgs::msg::TwistStamped command;                                                       // We want to compute this
    command.header.frame_id = _frameName;                                                           // Specify the reference frame for the twist vector
    command.header.stamp    = this->now();                                                          // Add the time info
    
    geometry_msgs::msg::Twist newTwist;                                                             // We need to populate this
    
    // Create pointer arrays so we can iterate over the twist fields:
    std::array<double*, 6> twist = {&newTwist.linear.x,
                                    &newTwist.linear.y,
                                    &newTwist.linear.z,
                                    &newTwist.angular.x,
                                    &newTwist.angular.y,
                                    &newTwist.angular.z};
                                    
    std::array<double*, 6> old = {&_oldTwist.linear.x,
                                  &_oldTwist.linear.y,
                                  &_oldTwist.linear.z,
                                  &_oldTwist.angular.x,
                                  &_oldTwist.angular.y,
                                  &_oldTwist.angular.z};
    
    if((this->now() - _lastInputTime).seconds() < _timeout and not _lastInput.axes.empty() and not _lastInput.buttons.empty())
    {                         
        for (int i = 0; i < 6; ++i)
        {
            double scalar = (i < 3) ? _maxLinearVelocity : _maxAngularVelocity;
           
            *twist[i]  = (_defaultMap[i].axis >= 0)            ? _lastInput.axes[_defaultMap[i].axis] * scalar : 0.0;
            *twist[i]  = (_defaultMap[i].button_positive >= 0) ? _lastInput.buttons[_defaultMap[i].button_positive] * scalar : *twist[i];
            *twist[i] += (_defaultMap[i].button_negative >= 0) ? _lastInput.buttons[_defaultMap[i].button_negative] * -scalar : 0.0;
        
            double max = (i < 3) ? _maxLinearAcceleration : _maxAngularAcceleration;
            
            *twist[i] = std::min(*twist[i], max / _frequency + *old[i]);                            // 
            *twist[i] = std::max(*twist[i],-max / _frequency + *old[i]);
        }
    }
    else
    {
        for( int i = 0; i < 6; ++i) *twist[i] = 0.0;
    }
    
    command.twist = newTwist;
    
    _oldTwist = newTwist;
    
    _publisher->publish(command);
}

} // namespace
