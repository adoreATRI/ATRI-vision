// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "atri_tracker/tracker_node.hpp"

// C++
#include <memory>
#include <string>
#include <thread>

// ROS2
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Eigen
#include <Eigen/Dense>

namespace atri_tracker
{

TrackerNode::TrackerNode(const rclcpp::NodeOptions & options) : Node("atri_tracker", options)
{
  // Parameters
  target_frame_ = this->declare_parameter("target_frame", "odom");
  double max_match_theta = this->declare_parameter("tracker.max_match_theta", 0.628);
  double max_match_center_xoy = this->declare_parameter("tracker.max_match_center_xoy", 1.5);
  lost_time_threshold_ = this->declare_parameter("tracker.lost_time_threshold", 0.5);
  tracker_ = std::make_unique<Tracker>(max_match_theta, max_match_center_xoy);
  tracker_->tracking_threshold = this->declare_parameter("tracker.tracking_threshold", 10);

  // Visualization initialize
  initVisualization();

  // EKF
  initEKF();

  // Gauss Newton Solver
  initGNS();

  // Create Publishers
  rune_publisher_ = this->create_publisher<atri_interfaces::msg::Rune>("tracker/rune", 10);
  block_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("tracker/block_marker", 10);
  center_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("tracker/center_marker", 10);
  measure_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("tracker/measure_marker", 10);
  debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("tracker/debug_data", 10);
  tf2_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Subscriber with tf2 message_filter
  keyboard_control_sub_ = this->create_subscription<std_msgs::msg::String>(
    "keyboard_node/key", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg->data == "r") {
        task_mode_ = (task_mode_ == "small_buff") ? "large_buff" : "small_buff";
        tracker_->tracker_state = Tracker::State::LOST;
        RCLCPP_INFO(
          rclcpp::get_logger("TrackerNode"), "Reset tracker, Task mode switched to: %s",
          task_mode_.c_str());
      } else if (msg->data == "f") {
        tracker_->tracker_state = Tracker::State::LOST;
      }
    });

  // tf2_filter
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
  if (color_block_msg->color_blocks.size() < 1) {
    return;
  }

  for (auto & color_block : color_block_msg->color_blocks) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      rclcpp::Time msg_time(color_block_msg->header.stamp, this->get_clock()->get_clock_type());
      transform_stamped = tf2_buffer_->lookupTransform(
        target_frame_, color_block_msg->header.frame_id, msg_time,
        rclcpp::Duration::from_seconds(0.0));
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }
    tf2::doTransform(color_block.pose, color_block.pose, transform_stamped);
  }

  // Init message
  rclcpp::Time time(color_block_msg->header.stamp, this->get_clock()->get_clock_type());
  atri_interfaces::msg::Rune rune_msg;
  rune_msg.header.stamp = time;
  rune_msg.header.frame_id = target_frame_;

  measure_marker_.header.stamp = time;

  measure_marker_.pose.position.x = color_block_msg->color_blocks[0].pose.position.x;
  measure_marker_.pose.position.y = color_block_msg->color_blocks[0].pose.position.y;
  measure_marker_.pose.position.z = color_block_msg->color_blocks[0].pose.position.z;
  measure_marker_pub_->publish(measure_marker_);

  if (tracker_->tracker_state == Tracker::State::LOST) {
    tracker_->init(color_block_msg);
    rune_msg.tracking = false;
    last_time_ = rclcpp::Time(color_block_msg->header.stamp, this->get_clock()->get_clock_type());
  } else {
    dt_ = (time - last_time_).seconds();
    tracker_->lost_threshold = static_cast<int>(lost_time_threshold_ / dt_);
    tracker_->update(color_block_msg);
    if (task_mode_ == "large_buff") {
      tracker_->solve(time);
    }

    if (
      tracker_->tracker_state == Tracker::State::DETECTING ||
      tracker_->tracker_state == Tracker::State::TRACKING ||
      tracker_->tracker_state == Tracker::State::TEMP_LOST) {
      bool tracking_active = tracker_->tracker_state == Tracker::State::TRACKING ||
                             tracker_->tracker_state == Tracker::State::TEMP_LOST;

      rune_msg.tracking = tracking_active;
      const auto & state = tracker_->target_state;

      // Calculate from prediction
      Tracker::block_target block_predict;
      tracker_->getTrackerPosition(block_predict);

      rune_msg.position = block_predict.center_position;
      rune_msg.velocity.x = state(3);
      rune_msg.velocity.y = state(4);
      rune_msg.velocity.z = state(5);
      // Update rotation basis
      rune_msg.axis_u.x = tracker_->rotation_basis.u.x();
      rune_msg.axis_u.y = tracker_->rotation_basis.u.y();
      rune_msg.axis_u.z = tracker_->rotation_basis.u.z();
      rune_msg.axis_v.x = tracker_->rotation_basis.v.x();
      rune_msg.axis_v.y = tracker_->rotation_basis.v.y();
      rune_msg.axis_v.z = tracker_->rotation_basis.v.z();
      // Visual the rotation axis
      geometry_msgs::msg::TransformStamped rotation_axis;
      rotation_axis.header.stamp = time;
      rotation_axis.header.frame_id = target_frame_;
      rotation_axis.child_frame_id = "rune_center_frame";
      rotation_axis.transform.translation.x = block_predict.center_position.x;
      rotation_axis.transform.translation.y = block_predict.center_position.y;
      rotation_axis.transform.translation.z = block_predict.center_position.z;
      Eigen::Matrix3d rotation_matrix;
      rotation_matrix.col(0) = tracker_->rotation_basis.v;
      rotation_matrix.col(1) = tracker_->rotation_basis.u;
      rotation_matrix.col(2) = tracker_->rotation_basis.rotation_axis;
      Eigen::Quaterniond center_q(rotation_matrix);
      rotation_axis.transform.rotation.x = center_q.x();
      rotation_axis.transform.rotation.y = center_q.y();
      rotation_axis.transform.rotation.z = center_q.z();
      rotation_axis.transform.rotation.w = center_q.w();
      tf2_broadcaster_->sendTransform(rotation_axis);

      rune_msg.r = state(6);
      rune_msg.theta = block_predict.theta;
      rune_msg.a = 0.0;
      rune_msg.w = 0.0;
      rune_msg.c = 0.0;
      rune_msg.b = state(8);

      auto now_sec = time.seconds();
      auto obs_time = tracker_->obs_start_time.seconds();

      if (tracking_active && task_mode_ == "large_buff") {
        const auto & gns_state = tracker_->spd_state;
        int sign = state(8) >= 0 ? 1 : -1;
        if (tracker_->solver_status == Tracker::SolverStatus::VALID) {
          rune_msg.a = gns_state(0) * sign;
          rune_msg.w = gns_state(1);
          rune_msg.c = gns_state(2);
          rune_msg.b = (2.09 - gns_state(0)) * sign;
          int T = 2 * M_PI / rune_msg.w * 1000;
          rune_msg.t_offset = int((now_sec - obs_time + rune_msg.c / rune_msg.w) * 1000) % T;

        } else {
          rune_msg.tracking = false;
        }
      }

      center_marker_.header.stamp = time;
      center_marker_.pose.position.x = block_predict.center_position.x;
      center_marker_.pose.position.y = block_predict.center_position.y;
      center_marker_.pose.position.z = block_predict.center_position.z;
      center_marker_pub_->publish(center_marker_);

      block_marker_.header.stamp = time;
      block_marker_.pose.position.x = block_predict.block_position.x;
      block_marker_.pose.position.y = block_predict.block_position.y;
      block_marker_.pose.position.z = block_predict.block_position.z;
      auto q = tf2::Quaternion();
      q.setRPY(
        atan2(block_predict.center_position.y, block_predict.center_position.x), -M_PI / 2, 0);
      block_marker_.pose.orientation = tf2::toMsg(q);
      block_marker_pub_->publish(block_marker_);

      // Publish debug data for PlotJuggler
      std_msgs::msg::Float64MultiArray debug_msg;
      debug_msg.data.push_back(tracker_->measurement(3));  // [0] raw measurement theta
      debug_msg.data.push_back(state(7));                  // [1] filter state theta
      debug_msg.data.push_back(rune_msg.b);                // [2] filter state omega (estimated)
      debug_pub_->publish(debug_msg);
    }
  }
  last_time_ = time;
  rune_publisher_->publish(rune_msg);
}

void TrackerNode::initVisualization()
{
  block_marker_ = visualization_msgs::msg::Marker();
  block_marker_.header.frame_id = target_frame_;
  block_marker_.ns = "color_block";
  block_marker_.id = 0;
  block_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  block_marker_.action = visualization_msgs::msg::Marker::ADD;
  block_marker_.scale.x = 0.03;
  block_marker_.scale.y = 0.03;
  block_marker_.scale.z = 0.03;
  block_marker_.color.r = 1.0;
  block_marker_.color.g = 0.0;
  block_marker_.color.b = 0.0;
  block_marker_.color.a = 1.0;

  center_marker_ = visualization_msgs::msg::Marker();
  center_marker_.header.frame_id = target_frame_;
  center_marker_.ns = "center";
  center_marker_.id = 0;
  center_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  center_marker_.action = visualization_msgs::msg::Marker::ADD;
  center_marker_.scale.x = 0.03;
  center_marker_.scale.y = 0.03;
  center_marker_.scale.z = 0.03;
  center_marker_.color.r = 0.0;
  center_marker_.color.g = 1.0;
  center_marker_.color.b = 0.0;
  center_marker_.color.a = 1.0;

  measure_marker_ = visualization_msgs::msg::Marker();
  measure_marker_.header.frame_id = target_frame_;
  measure_marker_.ns = "measure";
  measure_marker_.id = 0;
  measure_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  measure_marker_.action = visualization_msgs::msg::Marker::ADD;
  measure_marker_.scale.x = 0.03;
  measure_marker_.scale.y = 0.03;
  measure_marker_.scale.z = 0.03;
  measure_marker_.color.r = 0.0;
  measure_marker_.color.g = 0.0;
  measure_marker_.color.b = 1.0;
  measure_marker_.color.a = 1.0;
}

void TrackerNode::initEKF()
{  // state: x, y, z, vx, vy, vz, r, theta
  // f - Process function
  auto f = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(3) * dt_;
    x_new(1) += x(4) * dt_;
    x_new(2) += x(5) * dt_;
    x_new(7) += x(8) * dt_;
    return x_new;
  };

  // J_f - Jacobian of process function
  auto j_f = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd f(9, 9);
    // clang-format off
    f <<  1,   0,   0,   dt_, 0,   0,   0,   0,   0,
          0,   1,   0,   0,   dt_, 0,   0,   0,   0,
          0,   0,   1,   0,   0,   dt_, 0,   0,   0, 
          0,   0,   0,   1,   0,   0,   0,   0,   0,
          0,   0,   0,   0,   1,   0,   0,   0,   0,
          0,   0,   0,   0,   0,   1,   0,   0,   0,
          0,   0,   0,   0,   0,   0,   1,   0,   0,
          0,   0,   0,   0,   0,   0,   0,   1,   dt_,
          0,   0,   0,   0,   0,   0,   0,   0,   1;
    // clang-format on
    return f;
  };

  auto h = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    double xc = x(0), yc = x(1), zc = x(2), r = x(6), theta = x(7);
    double ct = cos(theta), st = sin(theta);
    const auto & u = tracker_->rotation_basis.u;
    const auto & v = tracker_->rotation_basis.v;

    z(0) = xc + r * (ct * u.x() + st * v.x());
    z(1) = yc + r * (ct * u.y() + st * v.y());
    z(2) = zc + r * (ct * u.z() + st * v.z());
    z(3) = theta;
    return z;
  };

  auto j_h = [this](const Eigen::VectorXd & x) {
    Eigen::MatrixXd H(4, 9);
    double r = x(6), theta = x(7);
    double ct = cos(theta), st = sin(theta);
    const auto & u = tracker_->rotation_basis.u;
    const auto & v = tracker_->rotation_basis.v;

    // clang-format off
    H <<  1, 0, 0, 0, 0, 0,  ct*u.x()+st*v.x(),  r*(-st*u.x()+ct*v.x()), 0,
          0, 1, 0, 0, 0, 0,  ct*u.y()+st*v.y(),  r*(-st*u.y()+ct*v.y()), 0,
          0, 0, 1, 0, 0, 0,  ct*u.z()+st*v.z(),  r*(-st*u.z()+ct*v.z()), 0,
          0, 0, 0, 0, 0, 0,  0,                   1,                      0;
    // clang-format on
    return H;
  };

  // update_Q - process noise covariance matrix
  s2qxyz_ = declare_parameter("ekf.sigma2_q_xyz", 5e-4);
  s2qtheta_ = declare_parameter("ekf.sigma2_q_theta", 1e-2);
  s2qr_ = declare_parameter("ekf.sigma2_q_r", 1e-6);
  auto u_q = [this]() {
    Eigen::MatrixXd q(9, 9);
    
    double t = dt_;
    double x = s2qxyz_, y = s2qxyz_, z = s2qxyz_, theta = s2qtheta_, r = s2qr_;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
    double q_z_z = pow(t, 4) / 4 * z, q_z_vz = pow(t, 3) / 2 * z, q_vz_vz = pow(t, 2) * z;
    double q_r = pow(t, 4) / 4 * r;
    double q_theta = pow(t, 4) / 4 * theta, q_theta_omega = pow(t, 3) / 2 * theta,
           q_omega_omega = pow(t, 2) * theta;
    // clang-format off
    //    x       y       z       v_x     v_y     v_z     r       theta          omega
    q <<  q_x_x,  0,      0,      q_x_vx, 0,      0,      0,      0,             0,
          0,      q_y_y,  0,      0,      q_y_vy, 0,      0,      0,             0,
          0,      0,      q_z_z,  0,      0,      q_z_vz, 0,      0,             0,
          q_x_vx, 0,      0,      q_vx_vx,0,      0,      0,      0,             0,
          0,      q_y_vy, 0,      0,      q_vy_vy,0,      0,      0,             0,
          0,      0,      q_z_vz, 0,      0,      q_vz_vz,0,      0,             0,
          0,      0,      0,      0,      0,      0,      q_r,    0,             0,
          0,      0,      0,      0,      0,      0,      0,      q_theta,       q_theta_omega,
          0,      0,      0,      0,      0,      0,      0,      q_theta_omega, q_omega_omega;
    // clang-format on
    return q;
  };
  // update_R - measurement noise covariance matrix
  r_block_ = declare_parameter("ekf.r_block", 5e-6);
  r_center_ = declare_parameter("ekf.r_center", 5e-5);
  r_block_min_ = declare_parameter("ekf.r_block_min", 1e-7);
  r_center_min_ = declare_parameter("ekf.r_center_min", 1e-7);
  auto u_r = [this](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 4> r;
    double xb = r_block_;
    double xc = r_center_;
    r.diagonal() << std::max(abs(xb * z(0)), r_block_min_),
                     std::max(abs(xb * z(1)), r_block_min_),
                     std::max(abs(xb * z(2)), r_block_min_),
                     std::max(abs(xc * z(3)), r_center_min_);
    return r;
  };
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 9> p0;
  p0.setIdentity();
  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};
}

void TrackerNode::initGNS()
{
  // GaussNewtonSolver
  auto u_fx = [](const Eigen::VectorXd & x, const std::vector<double> & ob) {
    double t = ob.at(0);
    double y = ob.at(1);
    double a = x(0), w = x(1), c = x(2);
    Eigen::MatrixXd fx(1, 1);

    fx << y - (a * sin(w * t + c) + (2.09 - a));
    return fx;
  };

  auto u_J = [](const Eigen::VectorXd & x, const std::vector<double> & ob) {
    double t = ob.at(0);
    double a = x(0), w = x(1), c = x(2);
    Eigen::MatrixXd J(1, 3);

    // clang-format off
    //   a                     w                         c
    J << -sin(w * t + c) + 1,  -t * a * cos(w * t + c),  -a * cos(w * t + c);
    // clang-format on
    return J;
  };

  min_a_ = declare_parameter("gns.min_a", 0.4);
  max_a_ = declare_parameter("gns.max_a", 1.3);
  min_w_ = declare_parameter("gns.min_w", 1.5);
  max_w_ = declare_parameter("gns.max_w", 2.3);
  auto constraint = [this](const Eigen::VectorXd & x) {
    double a = x(0), w = x(1);
    bool a_con = a > min_a_ && a < max_a_;
    bool w_con = w > min_w_ && w < max_w_;
    return a_con && w_con;
  };

  max_iter_ = declare_parameter("gns.max_iter", 50);
  min_step_ = declare_parameter("gns.min_step", 1e-10);
  obs_max_size_ = declare_parameter("gns.obs_max_size", 150);
  tracker_->gns = GaussNewtonSolver{u_fx, u_J, constraint, max_iter_, min_step_, obs_max_size_};

  tracker_->a_start = declare_parameter("gns.a_start", 0.9125);
  tracker_->w_start = declare_parameter("gns.w_start", 1.942);
  tracker_->c_start = declare_parameter("gns.c_start", 0.0);
  tracker_->min_first_solve_time = declare_parameter("gns.min_first_solve_time", 1.5);

  // GNS EKF
  // state: a, w, c
  // measurement: a, w, c
  // f - Process function
  auto f_gns = [this](const Eigen::VectorXd & x) { return x; };
  // J_f - Jacobian of process function
  auto j_f_gns = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd f(3, 3);
    f.setIdentity();
    return f;
  };
  // h - Observation function
  auto h_gns = [](const Eigen::VectorXd & x) { return x; };
  // J_h - Jacobian of observation function
  auto j_h_gns = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd h(3, 3);
    h.setIdentity();
    return h;
  };
  // update_Q - process noise covariance matrix
  s2q_a_ = declare_parameter("ekf_gns.sigma2_q_a", 0.1);
  s2q_w_ = declare_parameter("ekf_gns.sigma2_q_w", 0.1);
  s2q_c_ = declare_parameter("ekf_gns.sigma2_q_c", 100.0);
  auto u_q_gns = [this]() {
    Eigen::MatrixXd q(3, 3);
    // clang-format off
    q <<  s2q_a_, 0,       0,
          0,       s2q_w_, 0,
          0,       0,       s2q_c_;
    // clang-format on
    return q;
  };
  // update_R - measurement noise covariance matrix
  r_a_ = declare_parameter("ekf_gns.r_a", 1e-8);
  r_w_ = declare_parameter("ekf_gns.r_w", 5e-4);
  r_c_ = declare_parameter("ekf_gns.r_c", 1e-8);
  auto u_r_gns = [this](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 3> r;
    r.diagonal() << std::max(abs(r_a_ * z(0)), 1e-4),
                     std::max(abs(r_w_ * z(1)), 1e-4),
                     std::max(abs(r_c_ * z(2)), 1e-4);
    return r;
  };
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 3> p0_gns;
  p0_gns.setIdentity();
  tracker_->ekf_gns =
    ExtendedKalmanFilter{f_gns, h_gns, j_f_gns, j_h_gns, u_q_gns, u_r_gns, p0_gns};
}

}  // namespace atri_tracker

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(atri_tracker::TrackerNode)