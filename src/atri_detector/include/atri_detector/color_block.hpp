#ifndef ATRI_DETECTOR__COLOR_BLOCK_HPP_
#define ATRI_DETECTOR__COLOR_BLOCK_HPP_

#include <opencv2/core.hpp>
#include <vector>

namespace atri_detector
{
struct ColorBlock
{
  ColorBlock() = default;
  std::vector<cv::Point2f> kpt;
  std::vector<float> ab_channels;
  float diff;
};

}  // namespace atri_detector

#endif  // ATRI_DETECTOR__COLOR_BLOCK_HPP_
