/**
 * @file    interactive_marker.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    April 2025
 * @version 1.0
 * @brief   Broadcasts an interactive marker (transform) that can be manipulated in RViz.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation.
 */
 
#include <serial_link_interfaces/interactive_marker.hpp>

namespace serial_link_interfaces {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
InteractiveMarker::InteractiveMarker(const std::string &nodeName,
                                     const std::string &serverName)
: Node(nodeName)
{
    _server = std::make_shared<interactive_markers::InteractiveMarkerServer>(serverName, this, rclcpp::QoS(10), rclcpp::QoS(10));
    
    _transformBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    _transform.header.frame_id = this->declare_parameter("interactive_marker.frame_id", "world");
    
    _transform.child_frame_id = this->declare_parameter("interactive_marker.name", "desired");
    
    // Create
    visualization_msgs::msg::InteractiveMarker interactiveMarker;
    interactiveMarker.header.frame_id = _transform.header.frame_id;
    interactiveMarker.name            = _transform.child_frame_id;
    interactiveMarker.description     = this->declare_parameter("interactive_marker.description", "Drag to move.");
    interactiveMarker.scale           = this->declare_parameter<double>("interactive_marker.scale", 0.2);

    // Create a marker (cube)
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    
    std::vector<double> scale = this->declare_parameter<std::vector<double>>("interactive_marker.marker.scale", std::vector<double>{0.1, 0.1, 0.1});
    marker.scale.x = scale[0];
    marker.scale.y = scale[1];
    marker.scale.z = scale[2];
    
    std::vector<double> rgba =this->declare_parameter<std::vector<double>>("interactive_marker.marker.color", std::vector<double>{0.0, 1.0, 0.0, 1.0});
    marker.color.r = rgba[0];
    marker.color.g = rgba[1];
    marker.color.b = rgba[2];
    marker.color.a = rgba[3];

    // Attach marker to interactive marker
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(marker);
    interactiveMarker.controls.push_back(control);

    // Add 6DOF controls
    std::vector<std::string> axes = {"x", "y", "z"};
    for (const auto &axis : axes)
    {
        for (const auto &mode : {visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS, 
                                 visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS})
        {
            visualization_msgs::msg::InteractiveMarkerControl control;
            if (axis == "x") control.orientation.x = 1.0;
            if (axis == "y") control.orientation.y = 1.0;
            if (axis == "z") control.orientation.z = 1.0;
            control.orientation.w = 1.0;
            control.name = (mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS ? "move_" : "rotate_") + axis;
            control.interaction_mode = mode;
            interactiveMarker.controls.push_back(control);
        }
    }
    
    // Insert into server with feedback callback
    _server->insert(interactiveMarker, std::bind(&InteractiveMarker::process_feedback, this, std::placeholders::_1));
    _server->applyChanges();
    
    RCLCPP_INFO(this->get_logger(), "Started the '%s' node. Adveritising the '%s' transform.",
                nodeName.c_str(), _transform.child_frame_id.c_str());
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Callback method for when marker changes                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
InteractiveMarker::process_feedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
{

    _transform.header.stamp = this->get_clock()->now();
    _transform.transform.translation.x = feedback->pose.position.x;
    _transform.transform.translation.y = feedback->pose.position.y;
    _transform.transform.translation.z = feedback->pose.position.z;

    _transform.transform.rotation = feedback->pose.orientation;

    _transformBroadcaster->sendTransform(_transform);
}

} // namespace
