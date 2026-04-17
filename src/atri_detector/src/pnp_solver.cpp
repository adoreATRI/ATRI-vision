// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "atri_detector/pnp_solver.hpp"

// OpenCV
#include <opencv2/calib3d.hpp>

namespace atri_detector
{
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(
    cv::Mat(1, dist_coeffs.size(), CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  // Unit: m
  constexpr double half_diagonal_length = DIAGONAL_LENGTH / 2 / 1000;

  // Model coordinate: x forward, y left, z up
  // Start from the point closest to the circle in clockwise order
  block_points.emplace_back(cv::Point3f(0, 0, -half_diagonal_length));
  block_points.emplace_back(cv::Point3f(0, half_diagonal_length, 0));
  block_points.emplace_back(cv::Point3f(0, 0, half_diagonal_length));
  block_points.emplace_back(cv::Point3f(0, -half_diagonal_length, 0));
}

bool PnPSolver::solvePnP(const ColorBlock & block, cv::Mat & rvec, cv::Mat & tvec)
{
  std::vector<cv::Point2f> image_block_points;

  // Fill in image points
  if (block.kpt.size() < 5) {
    return false;
  }
  image_block_points.emplace_back(cv::Point2f(block.kpt[0].x, block.kpt[0].y));
  image_block_points.emplace_back(cv::Point2f(block.kpt[1].x, block.kpt[1].y));
  image_block_points.emplace_back(cv::Point2f(block.kpt[2].x, block.kpt[2].y));
  image_block_points.emplace_back(cv::Point2f(block.kpt[3].x, block.kpt[3].y));

  return cv::solvePnP(
    block_points, image_block_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
}
}  // namespace atri_detector