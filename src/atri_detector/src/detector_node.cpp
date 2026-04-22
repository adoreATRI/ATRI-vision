#include "atri_detector/detector_node.hpp"

// C++
#include <chrono>

// TF2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace atri_detector
{
DetectorNode::DetectorNode(const rclcpp::NodeOptions & options) : Node("detector_node", options)
{
  detector_ = std::make_unique<Detector>();

  // Create Publishers
  color_blocks_pub_ =
    this->create_publisher<atri_interfaces::msg::ColorBlockArray>("detector/color_blocks", 10);

  // Create Subscribers
  keyboard_control_sub_ = this->create_subscription<std_msgs::msg::String>(
    "keyboard_node/key", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
      if (msg->data == "r") {
        RCLCPP_INFO(this->get_logger(), "Reset detector");
        detector_->resetDetector();
      }
    });
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "usb_camera/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      camera_info_sub_.reset();
    });
  image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    "usb_camera/image_compressed", rclcpp::SensorDataQoS(),
    std::bind(&DetectorNode::imageCallback, this, std::placeholders::_1));
}

void DetectorNode::imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg)
{
  // Detect ColorBlocks
  atri_interfaces::msg::ColorBlockArray color_block_array;
  color_block_array.header = msg->header;
  color_block_array.header.frame_id = "camera_optical_frame";

  std::vector<ColorBlock> color_blocks = DetectColorBlocks(msg);
  if (color_blocks.size() < 1 || !detector_->locked) {
    return;
  }

  for (size_t i = 0; i < color_blocks.size(); ++i) {
    atri_interfaces::msg::ColorBlock color_block_msg;
    if (!pnp_solver_) return;

    cv::Mat rvec, tvec;
    if (pnp_solver_->solvePnP(color_blocks[i], rvec, tvec)) {
      color_block_msg.diff = color_blocks[i].diff;
      color_block_msg.pose.position.x = tvec.at<double>(0);
      color_block_msg.pose.position.y = tvec.at<double>(1);
      color_block_msg.pose.position.z = tvec.at<double>(2);

      cv::Mat rotation_matrix;
      cv::Rodrigues(rvec, rotation_matrix);

      tf2::Matrix3x3 tf_rotation_matrix(
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
        rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
        rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
        rotation_matrix.at<double>(2, 2));
      tf2::Quaternion tf_quaternion;
      tf_rotation_matrix.getRotation(tf_quaternion);
      color_block_msg.pose.orientation = tf2::toMsg(tf_quaternion);
      color_block_array.color_blocks.emplace_back(color_block_msg);
    }
  }
  color_blocks_pub_->publish(color_block_array);
}

std::vector<ColorBlock> DetectorNode::DetectColorBlocks(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg)
{
  std::vector<ColorBlock> result;

  cv::Mat image_bgr = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
  if (image_bgr.empty()) {
    return result;
  }

  auto start_time = std::chrono::steady_clock::now();

  result = detector_->Detect(image_bgr);

  auto end_time = std::chrono::steady_clock::now();
  auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  return result;
}

}  // namespace atri_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(atri_detector::DetectorNode)
