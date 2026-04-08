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

    const std::vector<double> & distortion_coefficients);

  bool solvePnP(const ColorBlock & block, cv::Mat & rvec, cv::Mat & tvec);
  std::vector<cv::Point3f> block_points;

private:
  // Unit: mm
  // static constexpr float DIAGONAL_LENGTH = 133.13;
  // static constexpr float DIAGONAL_LENGTH = 39.59;  // 28mm debug
  static constexpr float DIAGONAL_LENGTH = 55.15;  // 39mm

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
};
}  // namespace atri_detector

#endif