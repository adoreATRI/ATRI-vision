#ifndef ATRI_DETECTOR__DETECTOR_HPP_
#define ATRI_DETECTOR__DETECTOR_HPP_

#include <float.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include "atri_detector/color_block.hpp"
#include "atri_detector/onnx_inference.hpp"

// YAML
#include <yaml-cpp/yaml.h>

namespace atri_detector
{
struct YoloDetection
{
  int class_id;
  float confidence;
  cv::Rect bbox;
  cv::Point2f kpt;
};

class Detector
{
public:
  Detector(const YAML::Node & cfg);
  int rect_count = 0;

  // Debug
  void drawDetectedBlocks(
    cv::Mat & image, const std::vector<ColorBlock> & blocks,
    const std::vector<YoloDetection> & yolo_result);

  // Detector ColorBlocks
  std::vector<ColorBlock> Detect(cv::Mat & image);

  // YOLO & ONNX Runtime
  std::unique_ptr<OnnxInference> onnx;
  std::vector<YoloDetection> getYoloResult(const cv::Mat & image);
  cv::Mat yoloPreprocess(const cv::Mat & image, float & scale, int & pad_x, int & pad_y);
  void nms(std::vector<YoloDetection> & results);

  cv::dnn::Net yolo_net;
  int input_size;
  float confidence_threshold;
  float iou_thresh;

  // Process image
  std::vector<std::vector<cv::Point>> processImage(cv::Mat image);

  // Find circle colorblock
  void findCircleColorBlock(
    const std::vector<std::vector<cv::Point>> & contours, ColorBlock & circle_block);
  // Find rectangle colorblocks
  void findRectangleColorBlocks(
    const std::vector<std::vector<cv::Point>> & contours, ColorBlock & block, const cv::Rect roi,
    const cv::Point2f yolo_kpt);

  // Optimize detection
  void optimizeDetection(cv::Mat & image, std::vector<ColorBlock> & blocks);

  // Get Color Features
  void getColorFeatures(const cv::Mat & image, std::vector<ColorBlock> & blocks);
  void getCircleColorFeatures(
    const cv::Mat & image, const ColorBlock & circle_block, std::vector<int> & ab_channels_circle);
  void getRectColorFeatures(
    const cv::Mat & image, ColorBlock & block, std::vector<int> & ab_channels);
  void computeDiff(const cv::Mat & image, std::vector<ColorBlock> & blocks);
  cv::Mat computeABHistogram(const cv::Mat & image, const ColorBlock & block);
  cv::Mat computeHSHistogram(const cv::Mat & image, const ColorBlock & block);

  // Calculate
  bool calculateCircularity(const std::vector<cv::Point> & contour);
  void sortCorners(const cv::Point2f & yolo_kpt, std::vector<cv::Point2f> & kpts);
  void abDistance(const std::vector<int> & ab1, const std::vector<int> & ab2, float & distance);

private:
  // Get target block
  struct Vote
  {
    int vote_count = 0;
    cv::Mat hist = cv::Mat::zeros(32, 32, CV_32F);
    double distance;
  };
  std::vector<Vote> votes_;
  bool locked_ = false;
  bool is_vote_started_ = false;
  cv::Mat locked_hist_;
  int lock_votes_threshold_;
};

}  // namespace atri_detector

#endif  // ATRI_DETECTOR__DETECTOR_HPP_