// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ATRI_DETECTOR__DETECTOR_HPP_
#define ATRI_DETECTOR__DETECTOR_HPP_

#include <float.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "atri_detector/color_block.hpp"

namespace atri_detector
{
class Detector
{
public:
  Detector();
  // 调试
  /* void InitHsvTuner(); */

  // Detector ColorBlocks
  std::vector<ColorBlock> Detect(cv::Mat & image);

  // Process image
  std::vector<std::vector<cv::Point>> processImage(cv::Mat image);

  bool calculateCircularity(const std::vector<cv::Point> & contour);
  bool isPointNearLine(
    const cv::Point2f & point, const cv::Point2f & line_start, const cv::Point2f & line_end,
    double threshold);
  void sortCorners(const cv::Point2f & center, std::vector<cv::Point2f> & corners);
  void getCircleABMean(
    const cv::Mat & image_Lab, const cv::Point2f & center, float side_length,
    std::vector<float> & ab_channels);
  void getABMean(
    const cv::Mat & image_Lab, const cv::Point2f & center, std::vector<float> & ab_channels);
  void ABDistance(const std::vector<float> & ab1, const std::vector<float> & ab2, float & distance);

private:
  // 调试
  int h_min_;
  int h_max_;
  int s_min_;
  int s_max_;
  int v_min_;
  int v_max_;
};

}  // namespace atri_detector

#endif  // ATRI_DETECTOR__DETECTOR_HPP_