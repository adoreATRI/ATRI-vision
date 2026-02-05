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
#include <visualization_msgs/msg/marker.hpp>

// interfaces
#include <atri_interfaces/msg/color_block.hpp>
#include <atri_interfaces/msg/color_block_array.hpp>
#include <atri_interfaces/msg/rune.hpp>
#include <atri_interfaces/msg/rune_info.hpp>

// tracker
#include "atri_tracker/extended_kalman_filter.hpp"
#include "atri_tracker/tracker.hpp"

// STD
#include <memory>
#include <string>
#include <vector>

// Eigen
#include <Eigen/Dense>

namespace atri_tracker
{
using tf2_filter = tf2_ros::MessageFilter<atri_interfaces::msg::ColorBlockArray>;

class TrackerNode : public rclcpp::Node
{
public:
  explicit TrackerNode(const rclcpp::NodeOptions & options);

private:
  void colorBlockCallback(const atri_interfaces::msg::ColorBlockArray::SharedPtr color_block_msg);
  double dt_;

  rclcpp::Time last_time_;

  // Threshold
  double lost_time_threshold_;

  std::unique_ptr<Tracker> tracker_;

  // Param
  double s2qxyz_;
  double s2qtheta_;
  double s2qr_;
  double r_block_;
  double r_center_;

  // GNS Params
  double min_a_;
  double max_a_;
  double min_w_;
  double max_w_;
  int max_iter_;
  double min_step_;
  int obs_max_size_;
  double s2q_a_;
  double s2q_w_;
  double s2q_c_;
  double r_a_;
  double r_w_;
  double r_c_;

  // tf2 relevant
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<atri_interfaces::msg::ColorBlockArray> color_block_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // TF tree

  // Publishers
  rclcpp::Publisher<atri_interfaces::msg::Rune>::SharedPtr rune_publisher_;
  rclcpp::Publisher<atri_interfaces::msg::RuneInfo>::SharedPtr rune_info_publisher_;

  // Visualization
  void initVisualization();
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr block_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr center_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr measure_marker_pub_;
  visualization_msgs::msg::Marker block_marker_;
  visualization_msgs::msg::Marker center_marker_;
  visualization_msgs::msg::Marker measure_marker_;

  // EKF
  void initEKF();

  // GNS
  void initGNS();

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pnp_result_pub_;
};

}  // namespace atri_tracker

#endif