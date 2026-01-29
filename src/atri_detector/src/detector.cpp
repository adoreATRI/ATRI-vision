// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "atri_detector/detector.hpp"

namespace atri_detector
{
Detector::Detector()
{
  // 调试
  /* h_min_ = 20;
  h_max_ = 40;
  s_min_ = 30;
  s_max_ = 200;
  v_min_ = 60;
  v_max_ = 255; */
}

// 调试
/* void Detector::InitHsvTuner()
{
  cv::namedWindow("hsv_tuner", cv::WINDOW_NORMAL);
  cv::createTrackbar("H Min", "hsv_tuner", &h_min_, 179);
  cv::createTrackbar("H Max", "hsv_tuner", &h_max_, 179);
  cv::createTrackbar("S Min", "hsv_tuner", &s_min_, 255);
  cv::createTrackbar("S Max", "hsv_tuner", &s_max_, 255);
  cv::createTrackbar("V Min", "hsv_tuner", &v_min_, 255);
  cv::createTrackbar("V Max", "hsv_tuner", &v_max_, 255);
} */

std::vector<ColorBlock> Detector::Detect(cv::Mat & image)
{
  std::vector<ColorBlock> blocks;

  cv::Mat image_Lab;
  cv::cvtColor(image, image_Lab, cv::COLOR_BGR2Lab);

  // Process image
  std::vector<std::vector<cv::Point>> contours = processImage(image);

  cv::Point2f circle_center;
  double max_circle_area = 0.0;
  int index_of_circle = -1;

  // 先确定中心圆色块
  for (size_t i = 0; i < contours.size(); ++i) {
    double area = cv::contourArea(contours[i]);
    if (area < 1000) {
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

  if (index_of_circle >= 0) {
    ColorBlock block;
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
    block.kpt = circle_kpt;
    block.kpt.push_back(circle_center);
    blocks.push_back(block);
    // 调试
    for (size_t i = 0; i < block.kpt.size(); ++i) {
      cv::putText(
        image, std::to_string(i), block.kpt[i], cv::FONT_HERSHEY_SIMPLEX, 0.8,
        cv::Scalar(255, 0, 0), 2);
    }
    cv::drawContours(image, contours, index_of_circle, cv::Scalar(255, 0, 0), 2);
  }

  // 再检测矩形色块
  float side_length = 0.0f;
  for (const auto & contour : contours) {
    if (calculateCircularity(contour)) {
      continue;
    }
    double area = cv::contourArea(contour);
    if (area < 1000) {
      continue;
    }

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

    // 剔除其他矩形
    float aspect_ratio =
      std::min(cv::norm(corners[0] - corners[1]), cv::norm(corners[1] - corners[2])) /
      std::max(cv::norm(corners[0] - corners[1]), cv::norm(corners[1] - corners[2]));

    if (aspect_ratio < 0.7) {
      continue;
    }

    if (
      !isPointNearLine(circle_center, corners[0], corners[2], 10.0) &&
      !isPointNearLine(circle_center, corners[1], corners[3], 10.0)) {
      continue;
    }

    cv::Point2f center = cv::Point2f(0, 0);
    cv::Moments m = cv::moments(contour);
    if (m.m00 > 1e-6) {
      center = cv::Point2f(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
    }

    if (
      cv::norm(circle_center - center) >
      3 * std::max(cv::norm(corners[0] - corners[1]), cv::norm(corners[1] - corners[2]))) {
      continue;
    }

    side_length = std::min(cv::norm(corners[0] - corners[1]), cv::norm(corners[1] - corners[2]));

    // 包装ColorBlock
    ColorBlock block;
    std::vector<cv::Point2f> kpt;
    std::vector<float> ab_channels = {0.0f, 0.0f};

    sortCorners(circle_center, corners);
    getABMean(image_Lab, center, ab_channels);

    block.ab_channels = ab_channels;
    block.kpt = corners;
    block.kpt.push_back(center);
    blocks.push_back(block);

    // 调试
    for (int j = 0; j < 4; j++) {
      cv::line(image, corners[j], corners[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
    }
    for (size_t k = 0; k < block.kpt.size() - 1; ++k) {
      cv::putText(
        image, std::to_string(k), block.kpt[k], cv::FONT_HERSHEY_SIMPLEX, 0.8,
        cv::Scalar(255, 0, 0), 2);
    }
  }

  // 获取圆的ab均值，计算色差
  if (index_of_circle >= 0) {
    std::vector<float> circle_ab_channels{0.0f, 0.0f};
    getCircleABMean(image_Lab, circle_center, side_length, circle_ab_channels);
    blocks[0].ab_channels = circle_ab_channels;

    for (size_t i = 1; i < blocks.size(); ++i) {
      float distance = 0.0f;
      ABDistance(blocks[0].ab_channels, blocks[i].ab_channels, distance);
      blocks[i].diff = distance;

      cv::putText(
        image, "D:" + std::to_string(static_cast<int>(distance)), blocks[i].kpt.back(),
        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
    }
  }

  cv::imshow("detector", image);

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

  /*   std::vector<cv::Mat> lab_planes;
  cv::split(image_Lab, lab_planes);
   double clipLimit = 2.0;
  cv::Size tileGridSize(8, 8);
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clipLimit, tileGridSize);
  cv::Mat L_clahe;
  clahe->apply(lab_planes[0], L_clahe);
  lab_planes[0] = L_clahe;
  cv::merge(lab_planes, image_Lab); */

  cv::Mat mask;
  cv::adaptiveThreshold(
    image_gray, mask, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 11, 2.0);
  cv::Mat mask_other1;
  cv::Mat mask_other2;
  cv::inRange(image_hsv, cv::Scalar(25, 30, 60), cv::Scalar(40, 130, 200), mask_other1);
  cv::inRange(image_hsv, cv::Scalar(10, 40, 0), cv::Scalar(20, 110, 255), mask_other2);
  cv::bitwise_or(mask, mask_other1, mask);
  cv::bitwise_or(mask, mask_other2, mask);

  // 调试
  /* cv::Mat mask_other;
  cv::inRange(
    image_hsv, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), mask_other);
  cv::Vec3b hsv = image_hsv.at<cv::Vec3b>(image_hsv.rows / 2, image_hsv.cols / 2);
  std::cout << "[Detector] HSV at center: (" << (int)hsv[0] << ", " << (int)hsv[1] << ", "
            << (int)hsv[2] << ")" << std::endl; */

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat kernel_big = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_big, cv::Point(-1, -1), 1);

  cv::Canny(mask, mask, 50, 150);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // debug
  cv::imshow("canny", mask);
  cv::waitKey(1);

  return contours;
}

bool Detector::calculateCircularity(const std::vector<cv::Point> & contour)
{
  double area = cv::contourArea(contour);
  double perimeter = cv::arcLength(contour, true);

  if (perimeter < 1e-6) {
    return false;
  }

  double circularity = (4.0 * CV_PI * area) / (perimeter * perimeter);
  return circularity > 0.85;
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

void Detector::getCircleABMean(
  const cv::Mat & image_Lab, const cv::Point2f & center, float side_length,
  std::vector<float> & ab_channels)
{
  cv::Mat mask = cv::Mat::zeros(image_Lab.size(), CV_8UC1);
  cv::circle(mask, center, static_cast<int>(side_length * 3 / 8), cv::Scalar(255), -1);
  cv::circle(mask, center, static_cast<int>(side_length * 5 / 16), cv::Scalar(0), -1);

  cv::Scalar mean_Lab = cv::mean(image_Lab, mask);
  ab_channels[0] = static_cast<float>(mean_Lab[1]);
  ab_channels[1] = static_cast<float>(mean_Lab[2]);
}

void Detector::getABMean(
  const cv::Mat & image_Lab, const cv::Point2f & center, std::vector<float> & ab_channels)
{
  cv::Mat mask = cv::Mat::zeros(image_Lab.size(), CV_8UC1);
  cv::circle(mask, center, 5, cv::Scalar(255), -1);

  cv::Scalar mean_Lab = cv::mean(image_Lab, mask);
  ab_channels[0] = static_cast<float>(mean_Lab[1]);
  ab_channels[1] = static_cast<float>(mean_Lab[2]);
}

void Detector::ABDistance(
  const std::vector<float> & ab1, const std::vector<float> & ab2, float & distance)
{
  distance = std::sqrt(std::pow(ab1[0] - ab2[0], 2) + std::pow(ab1[1] - ab2[1], 2));
}
}  // namespace atri_detector
