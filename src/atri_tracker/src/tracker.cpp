// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "atri_tracker/tracker.hpp"

namespace atri_tracker
{
Tracker::Tracker(double max_match_theta, double max_match_center_xoy)
: tracker_state(LOST),
  target_state(Eigen::VectorXd::Zero(9)),
  measurement(Eigen::VectorXd::Zero(4)),
  spd_state(Eigen::VectorXd::Zero(3)),
  max_match_theta_(max_match_theta),
  max_match_center_xoy_(max_match_center_xoy),
  lost_count_(0)
{
}

void Tracker::init(const atri_interfaces::msg::ColorBlockArray::SharedPtr & color_block_msg)
{
  // Init state
  last_theta_ = 0.0;

  // Reset rotation basis
  rotation_basis.init = false;
  rotation_basis.locked = false;

  // Get block target
  block_tracked = getTargetBlock(color_block_msg);

  // init EKF
  initEKF(block_tracked);
  RCLCPP_DEBUG(rclcpp::get_logger("atri_tracker"), "Init EKF");

  tracker_state = DETECTING;
  detect_count_ = 0;

  gns.obs.clear();
  solver_status = WAITING;
}

void Tracker::initEKF(const block_target & block_tracked)
{
  double xc = block_tracked.center_position.x;
  double yc = block_tracked.center_position.y;
  double zc = block_tracked.center_position.z;
  double r = BUFF_R / 1000;
  double theta = block_tracked.theta;
  double omega = OMEGA;
  target_state = Eigen::VectorXd::Zero(9);
  target_state << xc, yc, zc, 0, 0, 0, r, theta, omega;
  ekf.setInitState(target_state);
}

void Tracker::update(const atri_interfaces::msg::ColorBlockArray::SharedPtr & color_block_msg)
{
  Eigen::VectorXd ekf_prediction = ekf.predict();

  bool is_detected = false;
  target_state = ekf_prediction;
  block_target block_predict;
  calculateMeasurementFromPrediction(block_predict, ekf_prediction);

  double theta_diff = 0.0;
  center_xoy_diff = 0.0;

  block_tracked = getTargetBlock(color_block_msg);

  theta_diff = angles::shortest_angular_distance(block_predict.theta, block_tracked.theta);
  center_xoy_diff = sqrt(
    pow(block_tracked.center_position.x - block_predict.center_position.x, 2) +
    pow(block_tracked.center_position.y - block_predict.center_position.y, 2));

  if (center_xoy_diff < max_match_center_xoy_ || tracker_state == DETECTING) {
    is_detected = true;
    measurement = Eigen::VectorXd::Zero(4);
    bool need_update = true;
    if (abs(theta_diff) > max_match_theta_) {
      // If theta diff is too large, ignore this measurement
      measurement << block_predict.block_position.x, block_predict.block_position.y,
        block_predict.block_position.z, block_predict.theta;
      is_detected = false;
      need_update = false;
    } else {
      measurement << block_tracked.block_position.x, block_tracked.block_position.y,
        block_tracked.block_position.z, block_tracked.theta;
    }
    // transfer theta from [-pi, pi] to [-inf, inf]
    measurement(3) = last_theta_ + angles::shortest_angular_distance(last_theta_, measurement(3));
    if (need_update) {
      last_theta_ = measurement(3);
      target_state = ekf.update(measurement);
      RCLCPP_DEBUG(rclcpp::get_logger("atri_tracker"), "EKF update");
    }
  } else {
    detect_count_ = detect_count_ ? detect_count_ - 1 : 0;
  }

  target_state(6) = BUFF_R / 1000;
  ekf.setState(target_state);

  if (tracker_state == DETECTING) {
    if (is_detected) {
      detect_count_++;
      if (detect_count_ > tracking_threshold) {
        detect_count_ = 0;

        rotation_basis.locked = true;

        Eigen::Vector3d delta(
          block_tracked.block_position.x - block_tracked.center_position.x,
          block_tracked.block_position.y - block_tracked.center_position.y,
          block_tracked.block_position.z - block_tracked.center_position.z);
        double new_theta = atan2(delta.dot(rotation_basis.v), delta.dot(rotation_basis.u));

        new_theta = last_theta_ + angles::shortest_angular_distance(last_theta_, new_theta);
        last_theta_ = new_theta;
        target_state(7) = new_theta;
        ekf.setState(target_state);

        auto P = ekf.getP();
        P(7, 7) = std::max(P(7, 7), 0.1);
        P(8, 8) = std::max(P(8, 8), 0.1);
        ekf.setP(P);

        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!is_detected) {
      lost_count_++;
      tracker_state = TEMP_LOST;
    } else {
      lost_count_ = 0;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!is_detected) {
      lost_count_++;
      if (lost_count_ > lost_threshold) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      lost_count_ = 0;
      tracker_state = TRACKING;
    }
  }
}

void Tracker::solve(const rclcpp::Time & time)
{
  if (tracker_state == DETECTING) {
    solver_status = WAITING;
    return;
  } else if (tracker_state == TRACKING && solver_status == WAITING) {
    solver_status = NOT_ENOUGH_OBS;
    obs_start_time = time;
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
    x0 << a_start, w_start, c_start;
    gns.setStartValue(x0);
    spd_state = Eigen::VectorXd::Zero(3);

    ekf_gns.setInitState(x0);
  } else if (tracker_state == TRACKING || tracker_state == TEMP_LOST) {
    double dt = (time - obs_start_time).seconds();
    double spd = abs(target_state(8));

    if (tracker_state == TRACKING) {
      gns.addObservation(dt, spd);
    }

    if (dt > min_first_solve_time) {
      int fail_count = 0;
      while (gns.solve() != GaussNewtonSolver::SUCCESS) {
        Eigen::VectorXd xn = gns.getState();
        xn(2) += 0.5;
        if (xn(2) > 8) {
          xn(2) -= 8;
        }
        gns.setStartValue(xn);
        fail_count++;
        if (fail_count > 20) {
          if (solver_status == VALID) {
            spd_state = ekf_gns.predict();
            if (!gns.obs.empty()) {
              gns.obs.pop_back();
            }
          } else {
            solver_status = INVALID;
            if (!gns.obs.empty()) {
              gns.obs.erase(gns.obs.begin());
            }
            lost_count_++;
            tracker_state = TEMP_LOST;
          }
          return;
        }
      }
      ekf_gns.predict();
      solver_status = VALID;
      auto gns_state = gns.getState();

      spd_state = ekf_gns.update(gns_state);
      spd_state(2) = angles::normalize_angle_positive(spd_state(2));
    } else {
      solver_status = NOT_ENOUGH_OBS;
    }
  }
  if (lost_count_ > lost_threshold) {
    lost_count_ = 0;
    tracker_state = LOST;
  }
}

void Tracker::calculateMeasurementFromPrediction(
  block_target & block_predict, const Eigen::VectorXd & ekf_prediction)
{
  double xc = ekf_prediction(0), yc = ekf_prediction(1), zc = ekf_prediction(2);
  double r = ekf_prediction(6), theta = ekf_prediction(7);
  double ct = cos(theta), st = sin(theta);

  const auto & u = rotation_basis.u;
  const auto & v = rotation_basis.v;
  block_predict.block_position.x = xc + r * (ct * u.x() + st * v.x());
  block_predict.block_position.y = yc + r * (ct * u.y() + st * v.y());
  block_predict.block_position.z = zc + r * (ct * u.z() + st * v.z());
  block_predict.center_position.x = xc;
  block_predict.center_position.y = yc;
  block_predict.center_position.z = zc;
  block_predict.theta = angles::normalize_angle(theta);
}

void Tracker::getTrackerPosition(block_target & block)
{
  calculateMeasurementFromPrediction(block, target_state);
  block.theta = angles::normalize_angle(block.theta);
  block.block_position = block.block_position;
}

Tracker::block_target Tracker::getTargetBlock(
  const atri_interfaces::msg::ColorBlockArray::SharedPtr & color_block_msg)
{
  block_target block_targeted;
  geometry_msgs::msg::Point block_pos = color_block_msg->color_blocks[0].pose.position;
  Eigen::Vector3d block_pos_eigen(block_pos.x, block_pos.y, block_pos.z);
  tf2::Quaternion q;
  tf2::convert(color_block_msg->color_blocks[0].pose.orientation, q);
  tf2::Quaternion q_m;
  tf2::fromMsg(color_block_msg->color_blocks[0].pose.orientation, q_m);
  tf2::Matrix3x3 rotation_matrix(q_m);

  tf2::Vector3 z_axis = rotation_matrix.getColumn(2);
  Eigen::Vector3d z(z_axis.x(), z_axis.y(), z_axis.z());

  double buff_r = BUFF_R / 1000.0;
  Eigen::Vector3d center_from_rect = block_pos_eigen - z * buff_r;

  block_targeted.center_position = geometry_msgs::msg::Point();
  block_targeted.center_position.x = center_from_rect.x();
  block_targeted.center_position.y = center_from_rect.y();
  block_targeted.center_position.z = center_from_rect.z();
  block_targeted.block_position = block_pos;

  // Get rune_axis
  if (!rotation_basis.locked) {
    tf2::Vector3 rune_x(1, 0, 0);
    tf2::Vector3 rune_axis = tf2::quatRotate(q, rune_x);
    Eigen::Vector3d axis(rune_axis.x(), rune_axis.y(), rune_axis.z());
    updateRotationAxis(axis);
  }

  Eigen::Vector3d delta(
    block_pos.x - block_targeted.center_position.x, block_pos.y - block_targeted.center_position.y,
    block_pos.z - block_targeted.center_position.z);
  double proj_u = delta.dot(rotation_basis.u);
  double proj_v = delta.dot(rotation_basis.v);
  block_targeted.theta = atan2(proj_v, proj_u);

  return block_targeted;
}

void Tracker::updateRotationAxis(const Eigen::Vector3d & measured_axis)
{
  Eigen::Vector3d axis = measured_axis.normalized();
  if (!rotation_basis.init) {
    rotation_basis.rotation_axis = axis;
    rotation_basis.init = true;
  } else {
    double alpha = 0.8;
    rotation_basis.rotation_axis =
      (alpha * axis + (1.0 - alpha) * rotation_basis.rotation_axis).normalized();
  }

  double n_h = sqrt(
    rotation_basis.rotation_axis.x() * rotation_basis.rotation_axis.x() +
    rotation_basis.rotation_axis.y() * rotation_basis.rotation_axis.y());
  if (n_h < 1e-6) {
    Eigen::Vector3d x_axis(1, 0, 0);
    rotation_basis.v = rotation_basis.rotation_axis.cross(x_axis).normalized();
    rotation_basis.u = rotation_basis.v.cross(rotation_basis.rotation_axis).normalized();
  } else {
    double nz = rotation_basis.rotation_axis.z();
    double nx = rotation_basis.rotation_axis.x();
    double ny = rotation_basis.rotation_axis.y();
    rotation_basis.u = Eigen::Vector3d(-nz * nx / n_h, -nz * ny / n_h, n_h);
    rotation_basis.v = Eigen::Vector3d(ny / n_h, -nx / n_h, 0);
  }
}

}  // namespace atri_tracker