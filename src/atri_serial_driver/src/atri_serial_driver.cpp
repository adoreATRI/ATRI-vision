// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

// TF2
#include <tf2/LinearMath/Quaternion.h>

// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++
#include <future>
#include <memory>
#include <string>

// Eigen
#include <Eigen/Dense>

// serial_driver
#include "atri_serial_driver/atri_serial_driver.hpp"
#include "atri_serial_driver/crc.hpp"
#include "atri_serial_driver/packet.hpp"

namespace atri_serial_driver
{
ATRISerialDriver::ATRISerialDriver(const rclcpp::NodeOptions & options)
: Node("atri_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  // Parameters
  // Serial
  getParams();
  // offset
  yaw_z_ = this->declare_parameter("tf_offset.gimbal_yaw.z", 0.0855);
  pitch_y_ = this->declare_parameter("tf_offset.gimbal_pitch.y", -0.00919);
  pitch_z_ = this->declare_parameter("tf_offset.gimbal_pitch.z", 0.039);
  camera_link_y_ = this->declare_parameter("tf_offset.camera_link.y", -0.0364);
  laser_link_y_ = this->declare_parameter("tf_offset.laser_link.y", -0.03009);

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create publishers
  time_info_pub_ =
    this->create_publisher<atri_interfaces::msg::TimeInfo>("serial_driver/time_info", 10);

  // Receive thread
  receive_thread_ = std::thread(&ATRISerialDriver::receiveData, this);

  // Subscribers
  rune_sub_.subscribe(this, "tracker/rune");
  time_info_sub_.subscribe(this, "serial_driver/time_info");
  keyboard_control_sub_ = this->create_subscription<std_msgs::msg::String>(
    "keyboard_node/key", 10,
    std::bind(&ATRISerialDriver::keyboardControlCallback, this, std::placeholders::_1));

  // Synchronizer
  sync_ = std::make_unique<Sync>(syncpolicy(1500), rune_sub_, time_info_sub_);
  sync_->registerCallback(
    std::bind(&ATRISerialDriver::sendData, this, std::placeholders::_1, std::placeholders::_2));
}

ATRISerialDriver::~ATRISerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void ATRISerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  // Wait for the port to be connected
  while (rclcpp::ok()) {
    if (!port_connected_) {
      try {
        serial_driver_->init_port(device_name_, *device_config_);
        if (!serial_driver_->port()->is_open()) {
          serial_driver_->port()->open();
        }
        port_connected_ = true;

      } catch (const std::exception & ex) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }
    }

    try {
      serial_driver_->port()->receive(header);

      if (header[0] == 0x5A) {
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        ReceivePacket packet = fromVector(data);

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {
          geometry_msgs::msg::TransformStamped yaw_tf_msg;
          geometry_msgs::msg::TransformStamped pitch_tf_msg;

          RCLCPP_INFO(
            get_logger(), "Received packet: yaw=%.3f, pitch=%.3f, timestamp=%u, CRC=0x%04X",
            packet.yaw, packet.pitch, packet.timestamp, packet.checksum);

          // yaw
          yaw_tf_msg.header.stamp = this->now();
          yaw_tf_msg.header.frame_id = "base_link";
          yaw_tf_msg.child_frame_id = "gimbal_yaw";
          // rotation
          tf2::Quaternion q_yaw;
          q_yaw.setRPY(0.0, 0.0, packet.yaw);
          yaw_tf_msg.transform.rotation = tf2::toMsg(q_yaw);
          // translation
          yaw_tf_msg.transform.translation.x = 0.0;
          yaw_tf_msg.transform.translation.y = 0.0;
          yaw_tf_msg.transform.translation.z = yaw_z_;

          // pitch
          pitch_tf_msg.header.stamp = yaw_tf_msg.header.stamp;
          pitch_tf_msg.header.frame_id = "gimbal_yaw";
          pitch_tf_msg.child_frame_id = "gimbal_pitch";
          // rotation
          tf2::Quaternion q_pitch;
          q_pitch.setRPY(0.0, (M_PI / 2) - packet.pitch, 0.0);
          pitch_tf_msg.transform.rotation = tf2::toMsg(q_pitch);
          // translation
          pitch_tf_msg.transform.translation.x = 0.0;
          pitch_tf_msg.transform.translation.y = pitch_y_;
          pitch_tf_msg.transform.translation.z = pitch_z_;

          tf_broadcaster_->sendTransform(yaw_tf_msg);
          tf_broadcaster_->sendTransform(pitch_tf_msg);

          // Publish time
          atri_interfaces::msg::TimeInfo time_info_msg;
          time_info_msg.header = pitch_tf_msg.header;
          time_info_msg.time = packet.timestamp;
          time_info_pub_->publish(time_info_msg);
        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!, CRC= %04X", packet.checksum);
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void ATRISerialDriver::sendData(
  const atri_interfaces::msg::Rune::ConstSharedPtr & rune,
  const atri_interfaces::msg::TimeInfo::ConstSharedPtr & time_info)
{
  if (started_send_) {
    try {
      // Prepare packet
      SendPacket packet;
      packet.state = rune->tracking ? 1 : 0;
      packet.cap_timestamp = time_info->time;
      packet.t_offset = timestamp_offset_;

      // Solve attitude
      solveAttitude(rune, packet);

      crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

      std::vector<uint8_t> data = toVector(packet);
      serial_driver_->port()->send(data);
      RCLCPP_INFO(
        get_logger(), "Sent packet: state=%d, yaw=%.3f, pitch=%.3f, cap_timestamp=%u, t_offset=%u",
        packet.state, packet.yaw, packet.pitch, packet.cap_timestamp, packet.t_offset);

    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
      reopenPort();
    }
  }
}

void ATRISerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 115200);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "none");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or "
        "hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "none");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "1.0");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void ATRISerialDriver::reopenPort()
{
  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }
  port_connected_ = false;
}

void ATRISerialDriver::keyboardControlCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  std::string command = msg->data;
  if (command == "r") {
    started_send_ = false;
    RCLCPP_INFO(get_logger(), "Wait send the data");
  } else if (command == "s") {
    started_send_ = true;
    RCLCPP_INFO(get_logger(), "Start sending data");
  } else if (command == "f") {
    started_send_ = false;
    sendFalseCommand();
    RCLCPP_INFO(get_logger(), "False tracking");
  }
}

void ATRISerialDriver::solveAttitude(
  const atri_interfaces::msg::Rune::ConstSharedPtr & rune_msg, SendPacket & packet)
{
  // Calculate the offset theta
  double latency_time = packet.t_offset / 1000.0;
  // ignore the center_position offset
  double cx = rune_msg->position.x;
  double cy = rune_msg->position.y;
  double cz = rune_msg->position.z;
  double r = rune_msg->r;

  double t_now = rune_msg->t_offset / 1000.0;
  double phase_now = rune_msg->w * t_now;  // original phase
  double phase_future = rune_msg->w * (t_now + latency_time);

  double delta_theta;
  if (rune_msg->w < 1e-6) {
    delta_theta = rune_msg->b * latency_time;
  } else {
    delta_theta = -rune_msg->a / rune_msg->w * (cos(phase_future) - cos(phase_now)) +
                  rune_msg->b * latency_time;
  }
  double theta_future = rune_msg->theta + delta_theta;

  // Predict the future position
  Eigen::Vector3d axis_u(rune_msg->axis_u.x, rune_msg->axis_u.y, rune_msg->axis_u.z);
  Eigen::Vector3d axis_v(rune_msg->axis_v.x, rune_msg->axis_v.y, rune_msg->axis_v.z);

  Eigen::Vector3d predicted_pos;
  predicted_pos.x() = cx + r * (cos(theta_future) * axis_u.x() + sin(theta_future) * axis_v.x());
  predicted_pos.y() = cy + r * (cos(theta_future) * axis_u.y() + sin(theta_future) * axis_v.y());
  predicted_pos.z() = cz + r * (cos(theta_future) * axis_u.z() + sin(theta_future) * axis_v.z());

  // Calculate the attitude using inverse kinematics
  double dx = predicted_pos.x();
  double dy = predicted_pos.y();
  double dz = predicted_pos.z();

  double y_offset = pitch_y_ + camera_link_y_ + laser_link_y_;
  double z_offset = yaw_z_ + pitch_z_;

  double distance_xy = sqrt(dx * dx + dy * dy);

  if (distance_xy > fabs(y_offset)) {
    packet.yaw = atan2(dy, dx) - asin(y_offset / distance_xy);

    double distance_shot = sqrt(distance_xy * distance_xy - y_offset * y_offset);
    double elevation = atan2(dz - z_offset, distance_shot);

    packet.pitch = elevation + M_PI / 2.0;
  } else {
    packet.yaw = atan2(dy, dx);
    packet.pitch = atan2(dz - z_offset, distance_xy) + M_PI / 2.0;
  }
}

void ATRISerialDriver::sendFalseCommand()
{
  SendPacket packet;
  packet.state = 2;
  packet.cap_timestamp = 0;
  packet.t_offset = 0;
  packet.yaw = 0.0;
  packet.pitch = 0.0;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  std::vector<uint8_t> data = toVector(packet);
  serial_driver_->port()->send(data);
}

}  // namespace atri_serial_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(atri_serial_driver::ATRISerialDriver)