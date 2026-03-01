#ifndef USB_CAMERA__USB_CAMERA_NODE_HPP_
#define USB_CAMERA__USB_CAMERA_NODE_HPP_

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

// ROS2
#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

// YAML
#include "yaml-cpp/yaml.h"

namespace usb_camera
{

class USBCameraNode : public rclcpp::Node
{
public:
  explicit USBCameraNode(const rclcpp::NodeOptions & options);
  ~USBCameraNode();

private:
  struct V4L2Buffer
  {
    void * start{nullptr};
    size_t length{0};
  };

  // Paramters
  YAML::Node cfg_;
  void declareParameters();
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // CaptureLoop
  void captureLoop();
  bool openCameraV4L2();
  void closeCameraV4L2();

  // V4L2 controls
  void applyV4L2Controls();
  bool setV4L2Control(int id, int value);

  // V4L2
  std::mutex mutex_;
  int v4l2_fd_{-1};
  std::vector<V4L2Buffer> v4l2_buffers_;

  // parameters
  std::string camera_device_url_;
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  int img_width_;
  int img_height_;
  std::atomic<int> frame_rate_;
  std::atomic<int> exposure_time_;
  std::atomic<int> exposure_time_mode_;
  std::atomic<int> brightness_;
  std::atomic<int> contrast_;
  std::atomic<int> saturation_;

  // Status
  std::atomic<bool> running_{false};
  std::atomic<bool> camera_connected_{false};
  std::thread capture_thread_;

  // Publisher
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
};

}  // namespace usb_camera

#endif  // USB_CAMERA__USB_CAMERA_NODE_HPP_