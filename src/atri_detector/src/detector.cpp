#include "atri_detector/detector.hpp"

// C++
#include <float.h>

#include <algorithm>
#include <chrono>
#include <cmath>

// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>

namespace atri_detector
{
Detector::Detector()
{
  cfg_ = YAML::LoadFile(
    ament_index_cpp::get_package_share_directory("atri_detector") + "/config/config.yaml");
  const std::string onnxpath =
    ament_index_cpp::get_package_share_directory("atri_detector") + "/model/block_detector.onnx";
  onnx_ = std::make_unique<OnnxInference>();

  // Load parameters
  input_size_ = cfg_["YOLO"]["input_size"].as<int>();
  confidence_threshold_ = cfg_["YOLO"]["confidence_threshold"].as<float>();
  iou_thresh_ = cfg_["YOLO"]["iou_thresh"].as<float>();
  lock_votes_threshold_ = cfg_["ColorFeature"]["lock_votes_threshold"].as<int>();

  // Load ONNX model
  if (!onnx_->loadOnnx(onnxpath)) {
    throw std::runtime_error("ONNX model load failed");
  }
}

std::vector<ColorBlock> Detector::Detect(cv::Mat & image)
{
  std::vector<ColorBlock> blocks;
  blocks.resize(6);  // block[5] is to store the circle block for locking

  // YOLO detection
  std::vector<YoloDetection> yolo_result;
  yolo_result = getYoloResult(image);

  // Find colorblocks
  // Find rectangle colorblocks
  rect_count = 0;
  for (const auto & det : yolo_result) {
    if (det.class_id == 1) {
      cv::Rect roi = det.bbox & cv::Rect(0, 0, image.cols, image.rows);
      cv::Point2f yolo_kpt = det.kpt;
      cv::Mat roi_img = image(roi).clone();

      // Process roi image lonely to avoid interference from other blocks
      auto contours = processImage(roi_img);
      findRectangleColorBlocks(contours, blocks[rect_count], roi, yolo_kpt);
      if (blocks[rect_count].kpt.size() == 5) {
        rect_count++;
      }
      if (rect_count >= 5) {
        break;
      }
    }
  }

  // Find circle colorblock in the study period to lock the target block hist
  if (!locked) {
    ColorBlock circle_block;
    for (const auto & det : yolo_result) {
      if (det.class_id == 0) {
        cv::Rect roi = det.bbox & cv::Rect(0, 0, image.cols, image.rows);
        cv::Mat roi_img = image(roi).clone();

        auto contours = processImage(roi_img);
        findCircleColorBlock(contours, circle_block);

        // Map roi coordinates back to original image
        if (circle_block.kpt.size() == 5) {
          for (auto & pt : circle_block.kpt) {
            pt.x += roi.x;
            pt.y += roi.y;
          }

          blocks[5] = circle_block;
        }
        break;
      }
    }
  }

  // Get target block
  if ((blocks[5].kpt.size() == 5 || locked) && blocks[0].kpt.size() == 5) {
    findTargetBlock(image, blocks);

    // Igonore unavailable blocks and circle block
    blocks.resize(rect_count);

    // Optimize
    optimizeDetection(image, blocks);

    // Debug
    drawDetectedBlocks(image, blocks, yolo_result);
  }

  cv::imshow("debug", image);
  cv::waitKey(1);

  return blocks;
}

std::vector<std::vector<cv::Point>> Detector::processImage(cv::Mat image)
{
  cv::Mat image_blurred;
  cv::medianBlur(image, image_blurred, 5);

  cv::Mat image_gray;
  cv::cvtColor(image_blurred, image_gray, cv::COLOR_BGR2GRAY);
  cv::Mat image_hsv;
  cv::cvtColor(image_blurred, image_hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::adaptiveThreshold(
    image_gray, mask, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 19, 2);

  cv::Mat mask_other1;
  cv::Mat mask_other2;
  cv::inRange(image_hsv, cv::Scalar(20, 50, 60), cv::Scalar(40, 200, 255), mask_other1);
  cv::inRange(image_hsv, cv::Scalar(0, 60, 50), cv::Scalar(25, 255, 255), mask_other2);
  cv::bitwise_or(mask, mask_other1, mask);
  cv::bitwise_or(mask, mask_other2, mask);

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 1);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  return contours;
}

void Detector::findCircleColorBlock(
  const std::vector<std::vector<cv::Point>> & contours, ColorBlock & circle_block)
{
  cv::Point2f circle_center;
  double max_circle_area = 0.0;
  int index_of_circle = -1;

  for (size_t i = 0; i < contours.size(); ++i) {
    double area = cv::contourArea(contours[i]);
    if (area < 500) {
      continue;
    }
    if (calculateCircularity(contours[i])) {
      if (area > max_circle_area) {
        max_circle_area = area;
        index_of_circle = i;
      }
    } else {
      continue;
    }
  }

  if (index_of_circle >= 0 && contours[index_of_circle].size() >= 5) {
    cv::RotatedRect ellipse = cv::fitEllipse(contours[index_of_circle]);
    cv::Point2f center = ellipse.center;
    float a = ellipse.size.width / 2;
    float b = ellipse.size.height / 2;
    float angle_rad = ellipse.angle * CV_PI / 180.0;

    cv::Point2f left = center + cv::Point2f(a * cos(angle_rad), a * sin(angle_rad));
    cv::Point2f right = center - cv::Point2f(a * cos(angle_rad), a * sin(angle_rad));
    cv::Point2f top = center + cv::Point2f(-b * sin(angle_rad), b * cos(angle_rad));
    cv::Point2f bottom = center - cv::Point2f(-b * sin(angle_rad), b * cos(angle_rad));

    circle_block.kpt = {left, top, right, bottom, center};
  }
}

void Detector::findRectangleColorBlocks(
  const std::vector<std::vector<cv::Point>> & contours, ColorBlock & block, const cv::Rect roi,
  const cv::Point2f yolo_kpt)
{
  for (const auto & contour : contours) {
    double area = cv::contourArea(contour);
    if (area < 100) {
      continue;
    }

    // Find corners
    std::vector<cv::Point2f> corners;
    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true), true);
    if (approx.size() == 4) {
      for (const auto & pt : approx) {
        corners.push_back(cv::Point2f(pt.x, pt.y));
      }
    } else {
      cv::RotatedRect rect = cv::minAreaRect(contour);
      cv::Point2f pts[4];
      rect.points(pts);
      for (int i = 0; i < 4; i++) {
        corners.push_back(pts[i]);
      }
    }

    // Map corners back to original image coordinates
    for (auto & pt : corners) {
      pt.x += roi.x;
      pt.y += roi.y;
    }

    // Ratio
    float aspect_ratio =
      std::min(cv::norm(corners[0] - corners[1]), cv::norm(corners[1] - corners[2])) /
      std::max(cv::norm(corners[0] - corners[1]), cv::norm(corners[1] - corners[2]));

    if (aspect_ratio < 0.5f) {
      continue;
    }

    // Calculate the center point of the block
    cv::Point2f center = cv::Point2f(0, 0);
    cv::Moments m = cv::moments(contour);
    if (m.m00 > 1e-6) {
      center = cv::Point2f(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
      center.x += roi.x;
      center.y += roi.y;
    }

    // Package block
    std::vector<cv::Point2f> kpts;
    kpts.reserve(5);
    kpts = corners;
    kpts.push_back(center);
    sortCorners(yolo_kpt, kpts);
    block.kpt = kpts;
    break;
  }
}

void Detector::optimizeDetection(cv::Mat & image, std::vector<ColorBlock> & blocks)
{
  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

  for (size_t i = 0; i < blocks.size(); ++i) {
    std::vector<cv::Point2f> corners;
    for (size_t j = 0; j < 4; ++j) {
      corners.push_back(blocks[i].kpt[j]);
    }
    cv::cornerSubPix(
      image_gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01));
    for (size_t j = 0; j < 4; ++j) {
      blocks[i].kpt[j] = corners[j];
    }
  }
}

void Detector::findTargetBlock(const cv::Mat & image, std::vector<ColorBlock> & blocks)
{
  // Study period
  if (!locked) {
    // Initialize votes
    if (!is_vote_started_) {
      for (int i = 0; i < rect_count; ++i) {
        Vote vote;
        vote.hist = computeHSHistogram(image, blocks[i]);
        votes_.push_back(vote);
      }
      is_vote_started_ = true;
      RCLCPP_INFO(rclcpp::get_logger("Detector"), "Vote init");
    } else {
      // Find the best block
      cv::Mat best_block_hist;
      findBestBlock(image, blocks, best_block_hist);

      // Find the valent vote according to the best block
      for (size_t i = 0; i < votes_.size(); ++i) {
        votes_[i].distance =
          cv::compareHist(votes_[i].hist, best_block_hist, cv::HISTCMP_BHATTACHARYYA);
      }
      std::sort(votes_.begin(), votes_.end(), [](const Vote & a, const Vote & b) {
        return a.distance < b.distance;
      });
      votes_[0].vote_count++;

      // EMA update histogram
      float alpha = 0.9f;
      votes_[0].hist = alpha * votes_[0].hist + (1 - alpha) * best_block_hist;
      if (votes_[0].vote_count >= lock_votes_threshold_) {
        locked = true;
        locked_hist_ = votes_[0].hist.clone();
        RCLCPP_INFO(rclcpp::get_logger("Detector"), "Locked the target block");
      }
    }
  } else {  // Locked period
    for (int i = 0; i < rect_count; ++i) {
      cv::Mat hist = computeHSHistogram(image, blocks[i]);
      blocks[i].diff = cv::compareHist(hist, locked_hist_, cv::HISTCMP_BHATTACHARYYA);
    }

    std::sort(
      blocks.begin(), blocks.begin() + rect_count,
      [](const ColorBlock & a, const ColorBlock & b) { return a.diff < b.diff; });

    // Maintain the locked histogram with EMA update
    if (blocks[0].diff < 0.6f) {
      cv::Mat best_hist = computeHSHistogram(image, blocks[0]);
      float alpha = 0.8f;
      locked_hist_ = alpha * locked_hist_ + (1 - alpha) * best_hist;
    }
  }
}

void Detector::findBestBlock(
  const cv::Mat & image, std::vector<ColorBlock> & blocks, cv::Mat & best_block_hist)
{
  cv::Mat circle_hist = computeCircleHistogram(image, blocks[5]);

  float min_distance = FLT_MAX;
  for (int i = 0; i < rect_count; ++i) {
    cv::Mat hist = computeHSHistogram(image, blocks[i]);
    float distance = cv::compareHist(circle_hist, hist, cv::HISTCMP_BHATTACHARYYA);
    if (distance < min_distance) {
      min_distance = distance;
      best_block_hist = hist.clone();
    }
  }
}

void Detector::drawDetectedBlocks(
  cv::Mat & image, const std::vector<ColorBlock> & blocks,
  const std::vector<YoloDetection> & yolo_result)
{
  // yolo
  for (const auto & det : yolo_result) {
    cv::rectangle(image, det.bbox, cv::Scalar(255, 0, 255), 2);
  }

  // rect blocks
  for (int i = 0; i < rect_count; ++i) {
    for (int j = 0; j < 4; ++j) {
      cv::line(image, blocks[i].kpt[j], blocks[i].kpt[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
    }

    cv::putText(
      image, std::to_string(blocks[i].diff), blocks[i].kpt[4], cv::FONT_HERSHEY_SIMPLEX, 0.6,
      cv::Scalar(0, 0, 255), 3);
    cv::putText(
      image, std::to_string(i), blocks[i].kpt[4] + cv::Point2f(0, 20), cv::FONT_HERSHEY_SIMPLEX,
      0.6, cv::Scalar(255, 0, 0), 2);

    for (int k = 0; k < 4; ++k) {
      cv::putText(
        image, std::to_string(k), blocks[i].kpt[k], cv::FONT_HERSHEY_SIMPLEX, 0.6,
        cv::Scalar(255, 0, 0), 2);
    }
  }

  // target block
  for (int k = 0; k < 4; ++k) {
    cv::circle(image, blocks[0].kpt[k], 5, cv::Scalar(255, 255, 0), -1);
  }
}

bool calculateCircularity(const std::vector<cv::Point> & contour)
{
  double area = cv::contourArea(contour);
  double perimeter = cv::arcLength(contour, true);

  if (perimeter < 1e-6) {
    return false;
  }

  double circularity = (4.0 * CV_PI * area) / (perimeter * perimeter);
  return circularity > 0.8;
}

void sortCorners(const cv::Point2f & yolo_kpt, std::vector<cv::Point2f> & kpts)
{
  if (kpts.size() != 5) return;

  struct corners_info
  {
    cv::Point2f pt;
    double angle;
  };

  std::vector<corners_info> corners;
  corners.reserve(3);

  // Find the corner closest to the YOLO keypoint as the starting point
  int start_index = 0;
  double min_dist = cv::norm(kpts[0] - yolo_kpt);

  for (size_t i = 1; i < kpts.size() - 1; ++i) {
    double dist = cv::norm(kpts[i] - yolo_kpt);
    if (dist < min_dist) {
      min_dist = dist;
      start_index = static_cast<int>(i);
    }
  }

  // Compute the absolute angle of the start corner relative to center
  double start_angle = std::atan2(kpts[start_index].y - kpts[4].y, kpts[start_index].x - kpts[4].x);

  // Find the corners info
  for (size_t i = 0; i < kpts.size() - 1; ++i) {
    if (i == static_cast<size_t>(start_index)) continue;
    double angle = std::atan2(kpts[i].y - kpts[4].y, kpts[i].x - kpts[4].x);
    double relative_angle = angle - start_angle;
    if (relative_angle <= 0) relative_angle += 2.0 * CV_PI;
    corners.push_back({kpts[i], relative_angle});
  }

  std::sort(corners.begin(), corners.end(), [](const corners_info & a, const corners_info & b) {
    return a.angle < b.angle;
  });

  std::vector<cv::Point2f> sorted;
  sorted.reserve(kpts.size());

  // Sort kpts
  sorted.push_back(kpts[start_index]);
  for (auto & kpt : corners) {
    sorted.push_back(kpt.pt);
  }
  sorted.push_back(kpts[4]);
  kpts = sorted;
};

cv::Mat computeCircleHistogram(const cv::Mat & image, const ColorBlock & circle_block)
{
  cv::Mat image_hsv;
  cv::cvtColor(image, image_hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
  float radius = cv::norm(circle_block.kpt[4] - circle_block.kpt[1]);
  cv::circle(mask, circle_block.kpt[4], static_cast<int>(0.8 * radius), cv::Scalar(255), -1);
  cv::circle(mask, circle_block.kpt[4], static_cast<int>(0.5 * radius), cv::Scalar(0), -1);

  int channels[] = {0, 1};
  int histSize[] = {32, 32};

  float h_range[] = {0, 180};
  float s_range[] = {0, 256};
  const float * ranges[] = {h_range, s_range};

  cv::Mat hist;
  cv::calcHist(&image_hsv, 1, channels, mask, hist, 2, histSize, ranges);

  cv::normalize(hist, hist, 1.0, 0.0, cv::NORM_L1);

  return hist;
}

std::vector<YoloDetection> Detector::getYoloResult(const cv::Mat & image)
{
  std::vector<YoloDetection> results;

  float scale;
  int pad_x, pad_y;
  int orig_w = image.cols;
  int orig_h = image.rows;

  cv::Mat blob = yoloPreprocess(image, scale, pad_x, pad_y);

  int blob_total = blob.total();
  std::vector<float> input_data(blob_total);
  std::memcpy(input_data.data(), blob.ptr<float>(), blob_total * sizeof(float));

  std::vector<float> output_data;
  if (!onnx_->infer(input_data, output_data)) {
    return results;
  }

  auto output_dims = onnx_->getOutputDims();

  int num_anchors = output_dims[2];
  int num_classes = 2;

  for (int i = 0; i < num_anchors; i++) {
    float cx = output_data[0 * num_anchors + i];
    float cy = output_data[1 * num_anchors + i];
    float w = output_data[2 * num_anchors + i];
    float h = output_data[3 * num_anchors + i];

    float kpt_x = output_data[(4 + num_classes + 0) * num_anchors + i];
    float kpt_y = output_data[(4 + num_classes + 1) * num_anchors + i];

    float max_score = -1.0f;
    int class_id = -1;
    for (int c = 0; c < num_classes; c++) {
      float score = output_data[(4 + c) * num_anchors + i];
      if (score > max_score) {
        max_score = score;
        class_id = c;
      }
    }

    if (max_score < confidence_threshold_) {
      continue;
    }

    float x1 = (cx - w / 2 - pad_x) / scale;
    float y1 = (cy - h / 2 - pad_y) / scale;
    float bw = w / scale;
    float bh = h / scale;

    float kpt_x_orig = (kpt_x - pad_x) / scale;
    float kpt_y_orig = (kpt_y - pad_y) / scale;

    x1 = std::max(0.0f, std::min(x1, static_cast<float>(orig_w)));
    y1 = std::max(0.0f, std::min(y1, static_cast<float>(orig_h)));

    YoloDetection det;
    det.class_id = class_id;
    det.confidence = max_score;
    det.bbox = cv::Rect2f(x1, y1, bw, bh);
    det.kpt = cv::Point2f(kpt_x_orig, kpt_y_orig);
    results.push_back(det);
  }
  nms(results);
  return results;
}

cv::Mat Detector::yoloPreprocess(const cv::Mat & image, float & scale, int & pad_x, int & pad_y)
{
  int orig_w = image.cols;
  int orig_h = image.rows;
  scale =
    std::min(static_cast<float>(input_size_) / orig_w, static_cast<float>(input_size_) / orig_h);

  int new_w = static_cast<int>(orig_w * scale);
  int new_h = static_cast<int>(orig_h * scale);
  pad_x = (input_size_ - new_w) / 2;
  pad_y = (input_size_ - new_h) / 2;

  cv::Mat resized;
  cv::resize(image, resized, cv::Size(new_w, new_h));

  cv::Mat padded(input_size_, input_size_, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(padded(cv::Rect(pad_x, pad_y, new_w, new_h)));

  cv::Mat blob = cv::dnn::blobFromImage(padded, 1.0 / 255.0, cv::Size(), cv::Scalar(), true);
  return blob;
}

void Detector::nms(std::vector<YoloDetection> & result)
{
  std::sort(result.begin(), result.end(), [](const YoloDetection & a, const YoloDetection & b) {
    return a.confidence > b.confidence;
  });

  std::vector<bool> suppressed(result.size(), false);
  for (size_t i = 0; i < result.size(); i++) {
    if (suppressed[i]) continue;
    for (size_t j = i + 1; j < result.size(); j++) {
      if (suppressed[j]) continue;
      if (result[i].class_id != result[j].class_id) continue;

      float inter_x1 = std::max(result[i].bbox.x, result[j].bbox.x);
      float inter_y1 = std::max(result[i].bbox.y, result[j].bbox.y);
      float inter_x2 =
        std::min(result[i].bbox.x + result[i].bbox.width, result[j].bbox.x + result[j].bbox.width);
      float inter_y2 = std::min(
        result[i].bbox.y + result[i].bbox.height, result[j].bbox.y + result[j].bbox.height);

      float inter_area = std::max(0.0f, inter_x2 - inter_x1) * std::max(0.0f, inter_y2 - inter_y1);
      float union_area = result[i].bbox.area() + result[j].bbox.area() - inter_area;

      if (inter_area / union_area > iou_thresh_) {
        suppressed[j] = true;
      }
    }
  }

  std::vector<YoloDetection> result_filtered;
  for (size_t i = 0; i < result.size(); i++) {
    if (!suppressed[i]) result_filtered.push_back(result[i]);
  }
  result = result_filtered;
}

cv::Mat computeHSHistogram(const cv::Mat & image, const ColorBlock & block)
{
  cv::Mat image_hsv;
  cv::cvtColor(image, image_hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
  std::vector<cv::Point> poly = {
    cv::Point(block.kpt[0]), cv::Point(block.kpt[1]), cv::Point(block.kpt[2]),
    cv::Point(block.kpt[3])};
  cv::fillConvexPoly(mask, poly, cv::Scalar(255));
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::erode(mask, mask, kernel, cv::Point(-1, -1), 2);

  int channels[] = {0, 1};
  int histSize[] = {32, 32};

  float h_range[] = {0, 180};
  float s_range[] = {0, 256};
  const float * ranges[] = {h_range, s_range};

  cv::Mat hist;
  cv::calcHist(&image_hsv, 1, channels, mask, hist, 2, histSize, ranges);

  cv::normalize(hist, hist, 1.0, 0.0, cv::NORM_L1);

  return hist;
}

void Detector::resetDetector()
{
  locked = false;
  is_vote_started_ = false;
  votes_.clear();
  locked_hist_ = cv::Mat();
}

}  // namespace atri_detector
