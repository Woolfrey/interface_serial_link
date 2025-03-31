/**
 * @file    joy_twist_mapper.cpp
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
    // Internal parameters
    _maxAngularAcceleration = this->declare_parameter<double>("joy_twist_mapper.max_angular_acceleration", 0.5);
    _maxAngularVelocity     = this->declare_parameter<double>("joy_twist_mapper.max_angular_velocity", 0.1);
    _maxLinearAcceleration  = this->declare_parameter<double>("joy_twist_mapper.max_linear_acceleration", 1.0);
    _maxLinearVelocity      = this->declare_parameter<double>("joy_twist_mapper.max_linear_velocity", 0.2);
    _timeout                = this->declare_parameter<double>("joy_twist_mapper.timeout", 0.1);
    
    const std::array<std::string, 6> paramNames = {"linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"};

    for (int i = 0; i < 6; ++i)
    {
        _defaultMap[i].axes    = this->declare_parameter<std::vector<long int>>("joy_twist_mapper.default." + paramNames[i] + ".axes", std::vector<long int>{0, 0});
        _defaultMap[i].buttons = this->declare_parameter<std::vector<long int>>("joy_twist_mapperdefault." + paramNames[i] + ".buttons", std::vector<long int>{0, 0});
        
        _alternateMap[i].axes    = this->declare_parameter<std::vector<long int>>("joy_twist_mapper.alternate." + paramNames[i] + ".axes", std::vector<long int>{0, 0});
        _alternateMap[i].buttons = this->declare_parameter<std::vector<long int>>("joy_twist_mapper.alternate." + paramNames[i] + ".buttons", std::vector<long int>{0, 0});
    }
   
    // Create the subscriber
    std::string subscriptionName = this->declare_parameter<std::string>("joy_twist_mapper.subscription_name", "joy");
    _subscriber = this->create_subscription<sensor_msgs::msg::Joy>
        (subscriptionName,
                        1,
         std::bind(&JoyTwistMapper::joy_callback, this, std::placeholders::_1)
        );

    // Create the publisher
    std::string publisherName = this->declare_parameter<std::string>("joy_twist_mapper.publisher_name", "twist_command");
    _publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>(publisherName, 1);
    
    // Timer for continuous publishing
    int frequency = this->declare_parameter("joy_twist_mapper.publisher_frequency", 20);
    
    _timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000/frequency)),
                                     std::bind(&JoyTwistMapper::publish_twist, this));
                                     
    _lastInputTime = this->now();                                                                   //We need this to start things
    
    RCLCPP_INFO(this->get_logger(),
        "Started the '%s' node. "
        "Subscribing to the '%s' topic. "
        "Publishing to the '%s' topic at %u Hz.",
        nodeName.c_str(), subscriptionName.c_str(), publisherName.c_str(), (unsigned int)(1000 / frequency));

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
    command.header.stamp = this->now();                                                             // Add the time info
    
    // Create a pointer lookup table for twist fields so we can iterate over them
    double *twistFields[6] = {&command.twist.linear.x,
                              &command.twist.linear.y,
                              &command.twist.linear.z,
                              &command.twist.angular.x,
                              &command.twist.angular.y,
                              &command.twist.angular.z};
                                  
    if((this->now() - _lastInputTime).seconds() < _timeout)
    {
        // Check if switch condition is met
        bool switchActivated = true;
        for (const int &number : _switchButtons)
        {
            if (_lastInput.buttons[number] == 0)                                                    // If any required button is not pressed, don't switch
            {
                switchActivated = false;
                break;
            }
        }

        const std::array<JoyMapping, 6> &activeMap = switchActivated ? _alternateMap : _defaultMap; // Get the map based on switch condition

        for (int i = 0; i < 6; ++i)
        {
            *twistFields[i] = 0.0;

            try
            {                                                                  
                for (size_t j = 0; j < activeMap[i].axes.size() && j < _lastInput.axes.size(); ++j) 
                {
                    *twistFields[i] += activeMap[i].axes[j] * _lastInput.axes[j];
                }
                
                for (size_t j = 0; j < activeMap[i].buttons.size() && j < _lastInput.buttons.size(); ++j) 
                {
                    *twistFields[i] += activeMap[i].buttons[j] * _lastInput.buttons[j];
                }
            }
            catch(const std::exception &exception)
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Exception: %s", exception.what());
            }
        }
    }
    else for(auto &field : twistFields) *field = 0.0;                                               // Assign zero

    _publisher->publish(command);
}

} // namespace
