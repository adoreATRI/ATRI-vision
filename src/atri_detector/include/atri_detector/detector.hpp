#ifndef ATRI_DETECTOR__DETECTOR_HPP_
#define ATRI_DETECTOR__DETECTOR_HPP_

#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// detector
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
  Detector();
  int rect_count = 0;

  // Detector ColorBlocks
  std::vector<ColorBlock> Detect(cv::Mat & image);
  void resetDetector();
  bool locked = false;

private:
  // Debug
  void drawDetectedBlocks(
    cv::Mat & image, const std::vector<ColorBlock> & blocks,
    const std::vector<YoloDetection> & yolo_result);

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
  void findTargetBlock(const cv::Mat & image, std::vector<ColorBlock> & blocks);
  void findBestBlock(
    const cv::Mat & image, std::vector<ColorBlock> & blocks, cv::Mat & best_block_hist);

  // YOLO & ONNX Runtime
  std::unique_ptr<OnnxInference> onnx_;
  std::vector<YoloDetection> getYoloResult(const cv::Mat & image);
  cv::Mat yoloPreprocess(const cv::Mat & image, float & scale, int & pad_x, int & pad_y);
  void nms(std::vector<YoloDetection> & results);

  cv::dnn::Net yolo_net_;
  int input_size_;
  float confidence_threshold_;
  float iou_thresh_;

  // Get target block
  struct Vote
  {
    int vote_count = 0;
    cv::Mat hist = cv::Mat::zeros(32, 32, CV_32F);
    double distance;
  };
  std::vector<Vote> votes_;
  bool is_vote_started_ = false;
  cv::Mat locked_hist_;
  int lock_votes_threshold_;

  // Config
  YAML::Node cfg_;
};

cv::Mat computeCircleHistogram(const cv::Mat & image, const ColorBlock & circle_block);
cv::Mat computeHSHistogram(const cv::Mat & image, const ColorBlock & block);

// Calculate
bool calculateCircularity(const std::vector<cv::Point> & contour);
void sortCorners(const cv::Point2f & yolo_kpt, std::vector<cv::Point2f> & kpts);

}  // namespace atri_detector

#endif  // ATRI_DETECTOR__DETECTOR_HPP_