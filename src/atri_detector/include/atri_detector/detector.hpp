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
  void InitHsvTuner();
  void drawDetectedBlocks(cv::Mat & image, const std::vector<ColorBlock> & blocks);

  // Detector ColorBlocks
  std::vector<ColorBlock> Detect(cv::Mat & image);

  // Process image
  std::vector<std::vector<cv::Point>> processImage(cv::Mat image);

  // Find circle colorblock
  void findCircleColorBlock(
    const std::vector<std::vector<cv::Point>> & contours, ColorBlock & circle_block);
  // Find rectangle colorblocks
  void findRectangleColorBlocks(
    const std::vector<std::vector<cv::Point>> & contours, std::vector<ColorBlock> & blocks);

  // Get Color Features
  void getColorFeatures(const cv::Mat & image, std::vector<ColorBlock> & blocks);

  // Calculate
  bool calculateCircularity(const std::vector<cv::Point> & contour);

  bool isPointNearLine(
    const cv::Point2f & point, const cv::Point2f & line_start, const cv::Point2f & line_end,
    double threshold);
  void sortCorners(const cv::Point2f & center, std::vector<cv::Point2f> & corners);

  void getCircleColorFeatures(
    const cv::Mat & image, const ColorBlock & circle_block, std::vector<int> & ab_channels_circle,
    int & h_circle, int & s_circle, int & gray_circle);
  void getColorFeatures(
    const cv::Mat & image, ColorBlock & block, std::vector<int> & ab_channels, int & h_value,
    int & s_value, int & gray_value);
  void abDistance(const std::vector<int> & ab1, const std::vector<int> & ab2, float & distance);

private:
  // 调试
  int h_min_;
  int h_max_;
  int s_min_;
  int s_max_;
  int v_min_;
  int v_max_;
  int gray_min_ = 100;
  int gray_max_ = 255;
  int ab_diff_;
  int h_diff_;
  int s_diff_;
  int gray_diff_;
};

}  // namespace atri_detector

#endif  // ATRI_DETECTOR__DETECTOR_HPP_