// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "atri_tracker/tracker_node.hpp"

namespace atri_tracker
{

TrackerNode::TrackerNode(const rclcpp::NodeOptions & options) : Node("atri_tracker", options)
{
  // create TF tree

  // Parameters
  target_frame_ = this->declare_parameter("target_frame", "color_block_circle");

  // Subscriber with tf2 message_filter
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  color_block_sub_.subscribe(this, "detector/color_blocks", rmw_qos_profile_sensor_data);
  tf2_filter_ = std::make_shared<tf2_filter>(
    color_block_sub_, *tf2_buffer_, target_frame_, 20, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  tf2_filter_->registerCallback(&TrackerNode::colorBlockCallback, this);
}

void TrackerNode::colorBlockCallback(
  const atri_interfaces::msg::ColorBlockArray::SharedPtr color_block_msg)
{
  for (auto & color_block : color_block_msg->color_blocks) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf2_buffer_->lookupTransform(
        target_frame_, color_block_msg->header.frame_id, color_block_msg->header.stamp,
        rclcpp::Duration::from_seconds(0.0));
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }
    tf2::doTransform(color_block.pose, color_block.pose, transform_stamped);
  }

  // Init message
  /* rclcpp::Time time = color_block_msg->header.stamp;
  atri_interfaces::msg::RunnerInfo rune_info_msg;
  rune_info_msg.header.stamp = time;
  rune_info_msg.header.frame_id = target_frame_;
  atri_interfaces::msg::Rune rune_msg;
  rune_msg.header.stamp = time;
  rune_msg.header.frame_id = target_frame_;
  geometry_msgs::msg::PoseArray pnp_result_msg;
  pnp_result_msg.header = rune_msg.header; */
}
}  // namespace atri_tracker

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(atri_tracker::TrackerNode)