// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ATRI_DETECTOR__PNP_SOLVER_HPP_
#define ATRI_DETECTOR__PNP_SOLVER_HPP_

#include <array>
#include <opencv2/core.hpp>
#include <vector>

#include "atri_detector/color_block.hpp"

namespace atri_detector
{
class PnPSolver
{
public:
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    // 畸变系数这个相机暂时用不上
    const std::vector<double> & distortion_coefficients);

  bool solvePnP(const ColorBlock & block, cv::Mat & rvec, cv::Mat & tvec);
  std::vector<cv::Point3f> block_points;
  bool solvePnP_circle(const ColorBlock & block, cv::Mat & rvec, cv::Mat & tvec);

private:
  // Unit: mm
  static constexpr float DIAGONAL_LENGTH = 133.13;
  static constexpr float CIRCLE_RADIUS = 40.0;

  std::vector<cv::Point3f> circle_points_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
};
}  // namespace atri_detector

#endif