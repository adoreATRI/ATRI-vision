#ifndef ATRI_DETECTOR__DETECTOR_NODE_HPP_
#define ATRI_DETECTOR__DETECTOR_NODE_HPP_

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>

// STL
#include <vector>

// atri_interfaces
#include <atri_interfaces/msg/color_block.hpp>
#include <atri_interfaces/msg/color_block_array.hpp>

#include "atri_detector/detector.hpp"
#include "atri_detector/pnp_solver.hpp"

namespace atri_detector
{
class DetectorNode : public rclcpp::Node
{
public:
  explicit DetectorNode(const rclcpp::NodeOptions & options);

private:
  // Publishers
  rclcpp::Publisher<atri_interfaces::msg::ColorBlockArray>::SharedPtr color_blocks_pub_;

  // Subscribers
  // Camera info subscriber
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  // Image subscriber
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  void imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg);
  // Keyboard control subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_control_sub_;

  // Detect
  std::vector<ColorBlock> DetectColorBlocks(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg);
  std::unique_ptr<Detector> detector_;

  // PnP Solver
  std::unique_ptr<PnPSolver> pnp_solver_;
};
}  // namespace atri_detector

#endif  // ATRI_DETECTOR__DETECTOR_NODE_HPP_