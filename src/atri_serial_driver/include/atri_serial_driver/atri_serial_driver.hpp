// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#ifndef ATRI_SERIAL_DRIVER__ATRI_SERIAL_DRIVER_HPP_
#define ATRI_SERIAL_DRIVER__ATRI_SERIAL_DRIVER_HPP_

// TF2
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// ROS2
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

// C++
#include <thread>
#include <vector>

// interfaces
#include "atri_interfaces/msg/rune.hpp"
#include "atri_interfaces/msg/time_info.hpp"
#include "atri_serial_driver/packet.hpp"

namespace atri_serial_driver
{
class ATRISerialDriver : public rclcpp::Node
{
public:
  explicit ATRISerialDriver(const rclcpp::NodeOptions & options);

  ~ATRISerialDriver() override;

private:
  // Get parameters
  void getParams();

  // Send data
  void sendData(
    const atri_interfaces::msg::Rune::ConstSharedPtr & rune_msg,
    const atri_interfaces::msg::TimeInfo::ConstSharedPtr & time_info_msg);
  bool started_send_{false};

  // Solve attitude
  void solveAttitude(
    const atri_interfaces::msg::Rune::ConstSharedPtr & rune_msg, SendPacket & packet);

  // Receive data
  void receiveData();
  bool port_connected_{false};

  // Reopen port
  void reopenPort();

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;

  // Thread
  std::thread receive_thread_;

  // TF
  double timestamp_offset_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  // Publishers
  rclcpp::Publisher<atri_interfaces::msg::TimeInfo>::SharedPtr time_info_pub_;

  // Subscribers
  message_filters::Subscriber<atri_interfaces::msg::Rune> rune_sub_;
  message_filters::Subscriber<atri_interfaces::msg::TimeInfo> time_info_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_control_sub_;

  // keyboard control
  void keyboardControlCallback(const std_msgs::msg::String::ConstSharedPtr msg);

  // Synchronizer
  typedef message_filters::sync_policies::ApproximateTime<
    atri_interfaces::msg::Rune, atri_interfaces::msg::TimeInfo>
    syncpolicy;
  typedef message_filters::Synchronizer<syncpolicy> Sync;
  std::unique_ptr<Sync> sync_;
};
}  // namespace atri_serial_driver

#endif  // ATRI_SERIAL_DRIVER__ATRI_SERIAL_DRIVER_HPP_