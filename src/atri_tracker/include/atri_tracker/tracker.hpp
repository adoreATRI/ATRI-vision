// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef BUFF_TRACKER__TRACKER_HPP_
#define BUFF_TRACKER__TRACKER_HPP_

// interfaces
#include <atri_interfaces/msg/color_block.hpp>
#include <atri_interfaces/msg/color_block_array.hpp>

// ros2
#include <angles/angles.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Eigen
#include <Eigen/Dense>

// tracker
#include "atri_tracker/extended_kalman_filter.hpp"
#include "atri_tracker/gauss_newton_solver.hpp"

// STD

#include <memory>

#define BUFF_R 160.0  // mm
#define PI 3.1415926
#define OMEGA 0.0

namespace atri_tracker
{
class Tracker
{
public:
  Tracker(double max_match_theta, double max_match_center_xoy);

  struct block_target
  {
    geometry_msgs::msg::Point block_position;
    geometry_msgs::msg::Point center_position;
    double theta;
  };
  block_target block_tracked;
  block_target getTargetBlock(
    const atri_interfaces::msg::ColorBlockArray::SharedPtr & color_block_msg);

  struct RotationBasis
  {
    Eigen::Vector3d rotation_axis;
    Eigen::Vector3d u;
    Eigen::Vector3d v;
    bool init = false;
    bool locked = false;
  };
  RotationBasis rotation_basis;
  void updateRotationAxis(const Eigen::Vector3d & measured_axis);

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;
  std::unique_ptr<Tracker> tracker_;

  enum SolverStatus {
    WAITING,
    NOT_ENOUGH_OBS,
    VALID,
    INVALID,
  } solver_status;

  void init(const atri_interfaces::msg::ColorBlockArray::SharedPtr & color_block_msg);

  // Threshold
  int lost_threshold;
  int tracking_threshold;

  double center_xoy_diff;

  // EKF
  ExtendedKalmanFilter ekf;
  Eigen::VectorXd target_state;
  Eigen::VectorXd measurement;

  // Init
  void initEKF(const block_target & tracked_block);

  // Update
  // Predict
  void update(const atri_interfaces::msg::ColorBlockArray::SharedPtr & color_block_msg);
  void calculateMeasurementFromPrediction(
    block_target & block_predict, const Eigen::VectorXd & ekf_prediction);

  // GNS
  double a_start, w_start, c_start;
  double min_first_solve_time;
  GaussNewtonSolver gns;
  rclcpp::Time obs_start_time;
  Eigen::VectorXd spd_state;
  ExtendedKalmanFilter ekf_gns;

  void solve(const rclcpp::Time & time);

  void getTrackerPosition(block_target & block);

private:
  // Param
  double max_match_theta_;
  double max_match_center_xoy_;

  int detect_count_;
  int lost_count_;

  double last_theta_;
};
}  // namespace atri_tracker

#endif  // BUFF_TRACKER__TRACKER_HPP_