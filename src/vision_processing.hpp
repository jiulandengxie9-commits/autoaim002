#pragma once

#include <opencv2/core.hpp>

#include <vector>

namespace vision {

struct MorphologyParams {
  int kernel_size = 5;
  int rb_threshold = 30;
  int erode_iterations = 1;
  int dilate_iterations = 1;
  double max_contour_area = 0.0;
};

struct MorphologyResult {
  cv::Mat mask;
  cv::Mat eroded;
  cv::Mat dilated;
  cv::Mat opened;
  cv::Mat closed;
  cv::Mat final_mask;
  std::vector<std::vector<cv::Point>> contours;
  std::size_t contours_filtered = 0;
};

struct LightDescriptor {
  float width = 0.0F;
  float length = 0.0F;
  cv::Point2f center;
  float angle = 0.0F;
  float area = 0.0F;

  cv::RotatedRect rect() const {
    return cv::RotatedRect(center, cv::Size2f(width, length), angle);
  }
};

struct ArmorDescriptor {
  LightDescriptor left;
  LightDescriptor right;
  cv::Point2f vertex[4];
};

struct DetectionParams {
  double min_light_area = 100.0;
  double min_contour_solidity = 0.3;
  double max_light_ratio = 0.8;
  double max_angle_diff = 15.0;
  double max_height_diff_ratio = 0.2;
  double max_y_diff_ratio = 0.5;
  double min_x_diff_ratio = 0.5;
  double max_armor_ratio = 2.5;
  double min_armor_ratio = 1.0;
};

struct DetectionResult {
  std::vector<LightDescriptor> lights;
  std::vector<ArmorDescriptor> armors;
};

cv::Mat splitRedMask(const cv::Mat& bgr, int rb_threshold);

MorphologyResult applyMorphology(const cv::Mat& mask,
                                 const MorphologyParams& params = {});

DetectionResult detect(const std::vector<std::vector<cv::Point>>& contours,
                       const DetectionParams& params = {});

cv::Mat drawResult(const cv::Mat& bgr, const MorphologyResult& m,
                   const DetectionResult& det);

cv::Mat makeGrid(const cv::Mat& bgr, const MorphologyResult& m,
                 const DetectionResult& det, int cell_width = 480);

}  // namespace vision
