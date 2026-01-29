// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ATRI_TRACKER__TRACKER_NODE_HPP_
#define ATRI_TRACKER__TRACKER_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// interfaces
#include <atri_interfaces/msg/color_block.hpp>
#include <atri_interfaces/msg/color_block_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

namespace atri_tracker
{
using tf2_filter = tf2_ros::MessageFilter<atri_interfaces::msg::ColorBlockArray>;

class TrackerNode : public rclcpp::Node
{
public:
  explicit TrackerNode(const rclcpp::NodeOptions & options);

private:
  void colorBlockCallback(const atri_interfaces::msg::ColorBlockArray::SharedPtr color_block_msg);

  // tf2 relevant
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<atri_interfaces::msg::ColorBlockArray> color_block_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // TF tree
};

}  // namespace atri_tracker

#endif