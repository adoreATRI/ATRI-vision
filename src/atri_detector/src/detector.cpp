#include "atri_detector/detector.hpp"

namespace atri_detector
{
Detector::Detector()
{
  // 调试
  h_min_ = 20;
  h_max_ = 40;
  s_min_ = 30;
  s_max_ = 200;
  v_min_ = 60;
  v_max_ = 255;
}

// 调试
void Detector::InitHsvTuner()
{
  cv::namedWindow("debug_tuner", cv::WINDOW_NORMAL);
  /* cv::createTrackbar("H Min", "debug_tuner", &h_min_, 179);
  cv::createTrackbar("H Max", "debug_tuner", &h_max_, 179);
  cv::createTrackbar("S Min", "debug_tuner", &s_min_, 255);
  cv::createTrackbar("S Max", "debug_tuner", &s_max_, 255);
  cv::createTrackbar("V Min", "debug_tuner", &v_min_, 255);
  cv::createTrackbar("V Max", "debug_tuner", &v_max_, 255);
  cv::createTrackbar("Gray Min", "debug_tuner", &gray_min_, 1);
  cv::createTrackbar("Gray Max", "debug_tuner", &gray_max_, 255); */
  cv::createTrackbar("AB Diff", "debug_tuner", &ab_diff_, 10);
  cv::createTrackbar("H Diff", "debug_tuner", &h_diff_, 10);
  cv::createTrackbar("S Diff", "debug_tuner", &s_diff_, 10);
  cv::createTrackbar("Gray Diff", "debug_tuner", &gray_diff_, 10);
}

std::vector<ColorBlock> Detector::Detect(cv::Mat & image)
{
  std::vector<ColorBlock> blocks;

  // Process image
  std::vector<std::vector<cv::Point>> contours = processImage(image);

  // Find colorblocks
  // Find circle colorblock
  ColorBlock circle_block;
  findCircleColorBlock(contours, circle_block);

  if (circle_block.kpt.size() == 5) {
    blocks.push_back(circle_block);

    // Find rectangle colorblocks
    findRectangleColorBlocks(contours, blocks);

    // Optimize
    optimizeDetection(image, blocks);

    // Get Color Features
    if (blocks.size() > 1) {
      getColorFeatures(image, blocks);

      // Debug
      drawDetectedBlocks(image, blocks);
    }
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
    image_gray, mask, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 19, 3.5);

  cv::Mat mask_other1;
  cv::Mat mask_other2;
  cv::inRange(image_hsv, cv::Scalar(20, 50, 60), cv::Scalar(40, 200, 255), mask_other1);
  cv::inRange(image_hsv, cv::Scalar(0, 60, 50), cv::Scalar(25, 255, 255), mask_other2);
  cv::bitwise_or(mask, mask_other1, mask);
  cv::bitwise_or(mask, mask_other2, mask);

  // 调试
  /* cv::Mat mask_other; */
  /* cv::inRange(
    image_hsv, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), mask_other); */
  /*   cv::inRange(image_gray, gray_min_, gray_max_, mask_other);
  cv::Vec3b hsv = image_hsv.at<cv::Vec3b>(image_hsv.rows / 2, image_hsv.cols / 2);
  std::cout << "[Detector] HSV at center: (" << (int)hsv[0] << ", " << (int)hsv[1] << ", "
            << (int)hsv[2] << ")" << std::endl; */

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 1);

  cv::medianBlur(mask, mask, 5);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  cv::imshow("mask", mask);
  cv::waitKey(1);

  return contours;
}

void Detector::findCircleColorBlock(
  const std::vector<std::vector<cv::Point>> & contours, ColorBlock & circle_block)
{
  cv::Point2f circle_center;
  double max_circle_area = 0.0;
  int index_of_circle = -1;

  // 找到最大的圆形轮廓
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

  // 确定圆形色块存在后，找到关键点
  /* if (index_of_circle >= 0) {
    cv::Moments m = cv::moments(contours[index_of_circle]);
    if (m.m00 > 1e-6) {
      circle_center =
        cv::Point2f(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
    }

    cv::Point2f left, right, top, bottom;
    float min_dx = +FLT_MAX;
    float max_dx = -FLT_MAX;
    float min_dy = +FLT_MAX;
    float max_dy = -FLT_MAX;

    for (auto & pt : contours[index_of_circle]) {
      float dx = pt.x - circle_center.x;
      float dy = pt.y - circle_center.y;

      if (dx < min_dx) {
        min_dx = dx;
        left = pt;
      }
      if (dx > max_dx) {
        max_dx = dx;
        right = pt;
      }
      if (dy < min_dy) {
        min_dy = dy;
        top = pt;
      }
      if (dy > max_dy) {
        max_dy = dy;
        bottom = pt;
      }
    }

    std::vector<cv::Point2f> circle_kpt = {left, top, right, bottom};
    circle_block.kpt = circle_kpt;
    circle_block.kpt.push_back(circle_center);
  */

  // 用椭圆拟合找关键点
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
  const std::vector<std::vector<cv::Point>> & contours, std::vector<ColorBlock> & blocks)
{
  for (const auto & contour : contours) {
    if (calculateCircularity(contour)) {
      continue;
    }

    double area = cv::contourArea(contour);
    if (
      area < 100 || area > (cv::norm(blocks[0].kpt[4] - blocks[0].kpt[1]) *
                            cv::norm(blocks[0].kpt[4] - blocks[0].kpt[1]) * 10)) {
      continue;
    }

    // 获取矩形轮廓的四个角点
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

    // 筛选矩形色块
    // 长宽比
    float aspect_ratio =
      std::min(cv::norm(corners[0] - corners[1]), cv::norm(corners[1] - corners[2])) /
      std::max(cv::norm(corners[0] - corners[1]), cv::norm(corners[1] - corners[2]));

    if (aspect_ratio < 0.5f) {
      continue;
    }

    // 矩形对角线是否通过圆心
    if (
      !isPointNearLine(blocks[0].kpt[4], corners[0], corners[2], 15.0) &&
      !isPointNearLine(blocks[0].kpt[4], corners[1], corners[3], 15.0)) {
      continue;
    }

    cv::Point2f center = cv::Point2f(0, 0);
    cv::Moments m = cv::moments(contour);
    if (m.m00 > 1e-6) {
      center = cv::Point2f(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
    }

    // 矩形到圆心距离
    if (cv::norm(blocks[0].kpt[4] - center) > 6 * cv::norm(blocks[0].kpt[4] - blocks[0].kpt[1])) {
      continue;
    }

    // 包装block
    ColorBlock block;
    std::vector<cv::Point2f> kpt;
    sortCorners(blocks[0].kpt[4], corners);
    block.kpt = corners;
    block.kpt.push_back(center);
    blocks.push_back(block);
  }
}

void Detector::optimizeDetection(cv::Mat & image, std::vector<ColorBlock> & blocks)
{
  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

  for (size_t i = 1; i < blocks.size(); ++i) {
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

void Detector::getColorFeatures(const cv::Mat & image, std::vector<ColorBlock> & blocks)
{
  std::vector<int> ab_channels_circle;
  int h_circle;
  int s_circle;
  int gray_circle;

  getCircleColorFeatures(image, blocks[0], ab_channels_circle, h_circle, s_circle, gray_circle);
  for (size_t i = 1; i < blocks.size(); ++i) {
    std::vector<int> ab_channels;
    int h_value;
    int s_value;
    int gray_value;
    getColorFeatures(image, blocks[i], ab_channels, h_value, s_value, gray_value);

    float ab_distance = 0.0f;
    float h_distance = 0.0f;
    float s_distance = 0.0f;
    float gray_distance = 0.0f;
    abDistance(ab_channels_circle, ab_channels, ab_distance);
    h_distance = std::abs(h_circle - h_value);
    s_distance = std::abs(s_circle - s_value);
    gray_distance = std::abs(gray_circle - gray_value);

    blocks[i].diff = 2.0 * (2.0f * 0.4f * ab_distance + 0.1f * h_distance + 0.4f * s_distance +
                            0.1f * gray_distance);

    // Debug
    /*     blocks[i].diff = 2.0 * (2.0f * static_cast<float>(ab_diff_) / 10 * ab_distance +
                            static_cast<float>(h_diff_) / 10 * h_distance +
                            static_cast<float>(s_diff_) / 10 * s_distance +
                            static_cast<float>(gray_diff_) / 10 * gray_distance); */
  }

  // 按颜色特征差值从小到大排序
  std::sort(blocks.begin() + 1, blocks.end(), [](const ColorBlock & a, const ColorBlock & b) {
    return a.diff < b.diff;
  });
}

void Detector::drawDetectedBlocks(cv::Mat & image, const std::vector<ColorBlock> & blocks)
{
  for (size_t i = 1; i < blocks.size(); ++i) {
    for (size_t j = 0; j < 4; ++j) {
      cv::line(image, blocks[i].kpt[j], blocks[i].kpt[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
    }

    cv::putText(
      image, std::to_string(blocks[i].diff), blocks[i].kpt[4], cv::FONT_HERSHEY_SIMPLEX, 0.6,
      cv::Scalar(0, 0, 255), 3);
    for (size_t k = 0; k < 4; ++k) {
      cv::putText(
        image, std::to_string(k), blocks[i].kpt[k], cv::FONT_HERSHEY_SIMPLEX, 0.6,
        cv::Scalar(255, 0, 0), 2);
    }
  }

  for (size_t k = 0; k < 4; ++k) {
    cv::circle(image, blocks[1].kpt[k], 5, cv::Scalar(255, 255, 0), -1);
  }
}

bool Detector::calculateCircularity(const std::vector<cv::Point> & contour)
{
  double area = cv::contourArea(contour);
  double perimeter = cv::arcLength(contour, true);

  if (perimeter < 1e-6) {
    return false;
  }

  double circularity = (4.0 * CV_PI * area) / (perimeter * perimeter);
  return circularity > 0.8;
}

bool Detector::isPointNearLine(
  const cv::Point2f & point, const cv::Point2f & line_start, const cv::Point2f & line_end,
  double threshold)
{
  double distance =
    std::abs(
      (line_end.y - line_start.y) * point.x - (line_end.x - line_start.x) * point.y +
      line_end.x * line_start.y - line_end.y * line_start.x) /
    std::sqrt(std::pow(line_end.y - line_start.y, 2) + std::pow(line_end.x - line_start.x, 2));

  return distance < threshold;
}

void Detector::sortCorners(const cv::Point2f & circle_center, std::vector<cv::Point2f> & corners)
{
  if (corners.size() < 2) {
    return;
  }

  cv::Point2f rect_center(0, 0);
  for (const auto & p : corners) {
    rect_center.x += p.x;
    rect_center.y += p.y;
  }
  rect_center.x /= static_cast<float>(corners.size());
  rect_center.y /= static_cast<float>(corners.size());

  struct CornerInfo
  {
    cv::Point2f pt;
    double angle;
  };

  std::vector<CornerInfo> infos;
  infos.reserve(corners.size());
  for (const auto & p : corners) {
    double dx = static_cast<double>(p.x - rect_center.x);
    double dy = static_cast<double>(p.y - rect_center.y);
    double angle = std::atan2(dy, dx);
    if (angle < 0) {
      angle += 2.0 * CV_PI;
    }
    infos.push_back(CornerInfo{p, angle});
  }

  std::sort(infos.begin(), infos.end(), [](const CornerInfo & a, const CornerInfo & b) {
    return a.angle < b.angle;
  });

  int nearest_pos = 0;
  double min_dist = cv::norm(infos[0].pt - circle_center);
  for (size_t i = 1; i < infos.size(); ++i) {
    double dist = cv::norm(infos[i].pt - circle_center);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_pos = static_cast<int>(i);
    }
  }

  std::vector<cv::Point2f> sorted;
  sorted.reserve(infos.size());
  for (size_t k = 0; k < infos.size(); ++k) {
    sorted.push_back(infos[(nearest_pos + k) % infos.size()].pt);
  }

  corners.swap(sorted);
}

void Detector::getCircleColorFeatures(
  const cv::Mat & image, const ColorBlock & circle_block, std::vector<int> & ab_channels_circle,
  int & h_circle, int & s_circle, int & gray_circle)
{
  cv::Mat image_lab;
  cv::Mat image_hsv;
  cv::Mat image_gray;
  cv::cvtColor(image, image_hsv, cv::COLOR_BGR2HSV);
  cv::cvtColor(image, image_lab, cv::COLOR_BGR2Lab);
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
  cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
  float radius = cv::norm(circle_block.kpt[4] - circle_block.kpt[1]);
  cv::circle(mask, circle_block.kpt[4], static_cast<int>(0.8 * radius), cv::Scalar(255), -1);
  cv::circle(mask, circle_block.kpt[4], static_cast<int>(0.5 * radius), cv::Scalar(0), -1);

  cv::Scalar mean_lab = cv::mean(image_lab, mask);
  cv::Scalar mean_hsv = cv::mean(image_hsv, mask);
  cv::Scalar mean_gray = cv::mean(image_gray, mask);
  ab_channels_circle.resize(2);
  ab_channels_circle[0] = static_cast<int>(mean_lab[1]);
  ab_channels_circle[1] = static_cast<int>(mean_lab[2]);
  h_circle = static_cast<int>(mean_hsv[0]);
  s_circle = static_cast<int>(mean_hsv[1]);
  gray_circle = static_cast<int>(mean_gray[0]);
}

void Detector::getColorFeatures(
  const cv::Mat & image, ColorBlock & block, std::vector<int> & ab_channels, int & h_value,
  int & s_value, int & gray_value)
{
  cv::Mat image_lab;
  cv::Mat image_hsv;
  cv::Mat image_gray;
  cv::cvtColor(image, image_hsv, cv::COLOR_BGR2HSV);
  cv::cvtColor(image, image_lab, cv::COLOR_BGR2Lab);
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

  cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
  cv::circle(mask, block.kpt[4], 5, cv::Scalar(255), -1);

  cv::Scalar mean_lab = cv::mean(image_lab, mask);
  cv::Scalar mean_hsv = cv::mean(image_hsv, mask);
  cv::Scalar mean_gray = cv::mean(image_gray, mask);
  ab_channels.resize(2);
  ab_channels[0] = static_cast<int>(mean_lab[1]);
  ab_channels[1] = static_cast<int>(mean_lab[2]);
  h_value = static_cast<int>(mean_hsv[0]);
  s_value = static_cast<int>(mean_hsv[1]);
  gray_value = static_cast<int>(mean_gray[0]);
}

void Detector::abDistance(
  const std::vector<int> & ab1, const std::vector<int> & ab2, float & distance)
{
  distance = std::sqrt(std::pow(ab1[0] - ab2[0], 2) + std::pow(ab1[1] - ab2[1], 2));
}
}  // namespace atri_detector
