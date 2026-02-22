#ifndef ATRI_DETECTOR__DETECTOR_NODE_HPP_
#define ATRI_DETECTOR__DETECTOR_NODE_HPP_

// atri_interfaces
#include <atri_interfaces/msg/color_block.hpp>
#include <atri_interfaces/msg/color_block_array.hpp>

// tf2_ros
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

// ros2

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Eigen
#include <Eigen/Dense>

// STD
#include <chrono>
#include <vector>

// atri_detector
#include "atri_detector/color_block.hpp"
#include "atri_detector/detector.hpp"
#include "atri_detector/pnp_solver.hpp"

// YAML
#include <yaml-cpp/yaml.h>

namespace atri_detector
{
class DetectorNode : public rclcpp::Node
{
public:
  explicit DetectorNode(const rclcpp::NodeOptions & options);

private:
  // Publishers
  rclcpp::Publisher<atri_interfaces::msg::ColorBlockArray>::SharedPtr color_blocks_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr latency_pub_;

  // Subscribers
  // Camera info subscriber
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info_;

  // Image subscriber
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  void imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg);

  // Detect
  std::vector<ColorBlock> DetectColorBlocks(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg);
  std::unique_ptr<Detector> detector_;

  // PnP Solver
  std::unique_ptr<PnPSolver> pnp_solver_;
};
}  // namespace atri_detector

#endif  // ATRI_DETECTOR__DETECTOR_NODE_HPP_