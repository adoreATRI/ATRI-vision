#include "atri_detector/detector_node.hpp"

namespace atri_detector
{
DetectorNode::DetectorNode(const rclcpp::NodeOptions & options) : Node("detector_node", options)
{
  detector_ = std::make_unique<Detector>();

  // TF tree
  dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // 调试
  /*  detector_->InitHsvTuner(); */

  color_blocks_pub_ =
    this->create_publisher<atri_interfaces::msg::ColorBlockArray>("detector/color_blocks", 10);

  latency_pub_ = this->create_publisher<std_msgs::msg::String>("inference/latency", 10);

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      camera_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      camera_info_sub_.reset();
    });
  image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    "image_compressed", rclcpp::SensorDataQoS(),
    std::bind(&DetectorNode::imageCallback, this, std::placeholders::_1));

  /* params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&DetectorNode::paramsCallback, this, std::placeholders::_1)); */
}

/* rcl_interfaces::msg::SetParametersResult DetectorNode::paramsCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
  }
  return result;
} */

void DetectorNode::imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg)
{
  std::vector<ColorBlock> color_blocks = DetectColorBlocks(msg);

  atri_interfaces::msg::ColorBlockArray color_block_array;
  color_block_array.header = msg->header;
  color_block_array.header.frame_id = "camera_optical_frame";

  for (size_t i = 0; i < color_blocks.size(); ++i) {
    atri_interfaces::msg::ColorBlock color_block_msg;
    color_block_msg.diff = color_blocks[i].diff;

    // 将圆和矩形分开进行PnP解算
    cv::Mat rvec, tvec;
    if (i == 0) {
      if (pnp_solver_->solvePnP_circle(color_blocks[i], rvec, tvec)) {
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

        // broadcast TF
        /* geometry_msgs::msg::TransformStamped dynamic_camera_optical_to_circle_color_block;
        dynamic_camera_optical_to_circle_color_block.header.stamp = msg->header.stamp;
        dynamic_camera_optical_to_circle_color_block.header.frame_id = "camera_optical_frame";
        dynamic_camera_optical_to_circle_color_block.child_frame_id = "color_block_circle";
        dynamic_camera_optical_to_circle_color_block.transform.translation.x = tvec.at<double>(0);
        dynamic_camera_optical_to_circle_color_block.transform.translation.y = tvec.at<double>(1);
        dynamic_camera_optical_to_circle_color_block.transform.translation.z = tvec.at<double>(2);
        dynamic_camera_optical_to_circle_color_block.transform.rotation = tf2::toMsg(tf_quaternion);
        dynamic_broadcaster_->sendTransform(dynamic_camera_optical_to_circle_color_block); */

      } else {
        RCLCPP_WARN(this->get_logger(), "PnP circle failed");
      }
    } else {
      if (pnp_solver_->solvePnP(color_blocks[i], rvec, tvec)) {
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
      } else {
        RCLCPP_WARN(this->get_logger(), "PnP failed");
      }
    }
  }
  color_blocks_pub_->publish(color_block_array);

  // target
  targetColorBlocks(color_block_array);
}

std::vector<ColorBlock> DetectorNode::DetectColorBlocks(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg)
{
  cv::Mat image_bgr = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

  cv::Mat image_copy = image_bgr.clone();

  auto start_time = std::chrono::steady_clock::now();

  std::vector<ColorBlock> result = detector_->Detect(image_copy);

  auto end_time = std::chrono::steady_clock::now();
  auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std_msgs::msg::String latency_msg;
  latency_msg.data = std::to_string(time.count());
  latency_pub_->publish(latency_msg);

  return result;
}

void DetectorNode::targetColorBlocks(
  const atri_interfaces::msg::ColorBlockArray & color_block_array)
{
  float MIN_DIFF = 100.0f;
  int index = -1;
  for (size_t i = 1; i < color_block_array.color_blocks.size(); ++i) {
    if (color_block_array.color_blocks[i].diff < MIN_DIFF) {
      MIN_DIFF = color_block_array.color_blocks[i].diff;
      index = static_cast<int>(i);
    }
  }
  if (index > 0) {
    geometry_msgs::msg::TransformStamped dynamic_camera_optical_to_target_color_block;
    dynamic_camera_optical_to_target_color_block.header.stamp = color_block_array.header.stamp;
    dynamic_camera_optical_to_target_color_block.header.frame_id = "camera_optical_frame";
    dynamic_camera_optical_to_target_color_block.child_frame_id = "color_block_target";
    dynamic_camera_optical_to_target_color_block.transform.translation.x =
      color_block_array.color_blocks[index].pose.position.x;
    dynamic_camera_optical_to_target_color_block.transform.translation.y =
      color_block_array.color_blocks[index].pose.position.y;
    dynamic_camera_optical_to_target_color_block.transform.translation.z =
      color_block_array.color_blocks[index].pose.position.z;
    dynamic_camera_optical_to_target_color_block.transform.rotation =
      color_block_array.color_blocks[index].pose.orientation;
    dynamic_broadcaster_->sendTransform(dynamic_camera_optical_to_target_color_block);
  }
}
}  // namespace atri_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(atri_detector::DetectorNode)