/**
 * @file    joy_twist_mapper.hpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Converts signals from a joystick to a twist command.
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

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace serial_link_interfaces {

/**
 * @brief A data structure that specifies how to convert sensor_msgs::msg::Joy fields to a twist vector.
 */
struct JoyMapping
{
    std::vector<long int> axes    = {0, 0};                                                         ///< Which axes map to a twist field (-1, 0, or 1)
    std::vector<long int> buttons = {0, 0};                                                         ///< Which buttons map to a twist field (-1, 0, or 1)
};                                                                                                  // Semicolon needed after struct declaration

/**
 * @brief A class the subscribes to a sensor_msgs::msg::Joy topic, and convert to a geometry_msgs::msg::TwistStamped.
 */
class JoyTwistMapper : public rclcpp::Node
{
    public:

        /**
         * @brief Constructor
         * @param subscriptionTopicName The topic name for subscribing to Joy messages.
         * @param publisherTopicName The topic name for publishing TwistStamped messages.
         */ 
        JoyTwistMapper(const std::string &nodeName = "joy_twist_mapper");
        
    private:
    
        double _timeout = 0.1;
        
        double _maxLinearVelocity = 0.1;                                                            ///< Maximum linear speed (m/s)
        
        double _maxLinearAcceleration = 2.0;                                                        ///< Use to ramp velocity up/down (m/s/s)
        
        double _maxAngularVelocity = 0.05;                                                          ///< Maximum angular speed (rad/s)
        
        double _maxAngularAcceleration = 1.0;                                                       ///< Ramps angular velocity up/down (rad/s/s)

        rclcpp::TimerBase::SharedPtr _timer;                                                        ///< Used to regulate the publisher
        
        rclcpp::Time _lastInputTime;                                                                ///< Records when last sensor_msgs::msg::Joy was received
        
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subscriber;                         ///< Listens for sensor_msgs/msg/Joy topic
        
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _publisher;                  ///< Advertises geometry_msgs/msg/TwistStamped topic
   
        sensor_msgs::msg::Joy _lastInput;                                                           ///< Records the last received sensor_msgs::msg::Joy topic
        
        std::array<JoyMapping, 6> _defaultMap;                                                      ///< Defines conversion from joystick input to twist vector
        
        std::array<JoyMapping, 6> _alternateMap;                                                    ///< Alternate conversion from joystick to twist
        
        std::string _frameName = "world";                                                           ///< Reference frame for the twist vector
        
        std::vector<int> _switchButtons;                                                            ///< Defines which combination of buttons 
        
        unsigned int _frequency = 50;                                                               ///< Default frequency for sending twist commands
                
        /**
         * @brief Listens for joystick inputs and saves them.
         * @param input Pointer to the joystick signal.
         */
        void
        joy_callback(const std::shared_ptr<const sensor_msgs::msg::Joy> &input);
        
        /**
         * @brief Continually publishes twist command. Converts recent Joy messages to twist.
         */
        void
        publish_twist();
};                                                                                                  // Semicolon needed after a class declaration

}
