#include "atri_detector/detector_node.hpp"

namespace atri_detector
{
DetectorNode::DetectorNode(const rclcpp::NodeOptions & options) : Node("detector_node", options)
{
  detector_ = std::make_unique<Detector>();

  // TF tree
  /* dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this); */

  // 调试
  /*   detector_->InitHsvTuner(); */

  color_blocks_pub_ =
    this->create_publisher<atri_interfaces::msg::ColorBlockArray>("detector/color_blocks", 10);

  latency_pub_ = this->create_publisher<std_msgs::msg::String>("inference/latency", 10);

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      camera_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      camera_matrix = cv::Mat(3, 3, CV_64F, (void *)camera_info->k.data()).clone();
      dist_coeffs = cv::Mat(1, 5, CV_64F, (void *)camera_info->d.data()).clone();
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

  if (color_blocks.size() < 2) {
    return;
  }
  for (size_t i = 1; i < color_blocks.size(); ++i) {
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

      // 使用目标色块约束圆心色块的解算
      if (i == 1) {
        Eigen::Vector3d block_pos(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        Eigen::Vector3d z_axis(
          rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 2),
          rotation_matrix.at<double>(2, 2));

        double buff_r = 160.0 / 1000.0;
        Eigen::Vector3d center_from_rect = block_pos - z_axis * buff_r;

        std::vector<cv::Point3f> center_point_3d = {
          cv::Point3f(center_from_rect.x(), center_from_rect.y(), center_from_rect.z())};
        std::vector<cv::Point2f> center_2d_projected;
        cv::projectPoints(
          center_point_3d, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F),
          camera_matrix, dist_coeffs, center_2d_projected);

        // 与检测到的圆心比较
        double center_error = cv::norm(center_2d_projected[0] - color_blocks[0].kpt[4]);

        cv::Mat rvec_circle, tvec_circle;
        if (pnp_solver_->solvePnP_circle(color_blocks[0], rvec_circle, tvec_circle)) {
          Eigen::Vector3d center_from_2d = Eigen::Vector3d::Zero();
          if (center_error < 3.0) {
            center_from_2d = Eigen::Vector3d(
              tvec_circle.at<double>(0), tvec_circle.at<double>(1), tvec_circle.at<double>(2));
          } else {
            center_from_2d = center_from_rect;
          }

          // 加权融合
          double weight_2d = 0.3;
          double weight_rect = 0.7;
          Eigen::Vector3d center_fused =
            weight_2d * center_from_2d + weight_rect * center_from_rect;

          atri_interfaces::msg::ColorBlock circle_block_msg;
          circle_block_msg.pose.position.x = center_fused.x();
          circle_block_msg.pose.position.y = center_fused.y();
          circle_block_msg.pose.position.z = center_fused.z();
          circle_block_msg.pose.orientation = tf2::toMsg(tf_quaternion);
          color_block_array.color_blocks.emplace_back(circle_block_msg);

          // Debug
          /* double yaw, pitch, roll;
          tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
          RCLCPP_INFO(
            rclcpp::get_logger("atri_detector"), "yaw: %.3f, pitch: %.3f, roll: %.3f", yaw, pitch,
            roll); */
        }
      }
        color_block_array.color_blocks.emplace_back(color_block_msg);

        // Debug
        /* if (color_blocks[i].kpt.size() == 5) {
          std::vector<cv::Point2f> projected;
          std::vector<cv::Point2f> image_point;
          for (size_t t = 0; t < color_blocks[i].kpt.size() - 1; t++) {
            image_point.emplace_back(color_blocks[i].kpt[t]);
          }
          cv::projectPoints(
            pnp_solver_->block_points, rvec, tvec, camera_matrix, dist_coeffs, projected);
          double total_error = 0.0;
          for (size_t j = 0; j < image_point.size(); j++) {
            double error = cv::norm(image_point[j] - projected[j]);
            total_error += error;
          }
          double mean_error = total_error / image_point.size();
          RCLCPP_INFO(
            this->get_logger(), "PnP reprojection error: %.3f pixels, distance: %.3f m", mean_error,
            tvec.at<double>(2));
        } */
    }
  }
  color_blocks_pub_->publish(color_block_array);
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

}  // namespace atri_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(atri_detector::DetectorNode)