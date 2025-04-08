/**
 * @file    interactive_marker.hpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Broadcasts an interactive marker (transform) that can be manipulated in RViz.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */
 
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace serial_link_interfaces {

class InteractiveMarker : public rclcpp::Node
{
    public:
    
        /**
         * @brief Constructor
         * @param nodeName The name that will be seen over the ROS2 network
         */
        InteractiveMarker(const std::string &nodeName   = "interactive_marker",
                          const std::string &serverName = "interactive_marker_server");

    private:
    
        rclcpp::TimerBase::SharedPtr _timer;                                                        ///< Regulates publication of transform
    
        geometry_msgs::msg::TransformStamped _transform;                                            ///< Store changes from RViz                   
        
        std::shared_ptr<interactive_markers::InteractiveMarkerServer> _server;                      ///< Gets latest marker pose
        
        std::shared_ptr<tf2_ros::TransformBroadcaster> _transformBroadcaster;                       ///< Responsible for publishing the latest transform

        /**
         * @brief Callback method for the interactive marker server. Updates transform when RViz changes.
         * @param feedback Object containing latest pose of marker from RViz.
         */
        void
        process_feedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
        
        /**
         * @brief Method bound to the timer. Publishes the desired transform to ROS2.
         */
        void
        broadcast_transform();
};

} // namespace
