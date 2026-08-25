#include "vision_processing.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace vision {

cv::Mat splitRedMask(const cv::Mat& bgr, int rb_threshold) {
  std::vector<cv::Mat> bgr_channels;
  cv::split(bgr, bgr_channels);

  cv::Mat red_minus_blue;
  cv::subtract(bgr_channels[0], bgr_channels[2], red_minus_blue);

  cv::Mat mask;
  cv::threshold(red_minus_blue, mask, rb_threshold, 255, cv::THRESH_BINARY);
  return mask;
}

MorphologyResult applyMorphology(const cv::Mat& mask,
                                 const MorphologyParams& params) {
  MorphologyResult r;
  r.mask = mask.clone();

  const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(params.kernel_size, params.kernel_size));

  cv::erode(r.mask, r.eroded, kernel, cv::Point(-1, -1), params.erode_iterations);
  cv::dilate(r.mask, r.dilated, kernel, cv::Point(-1, -1), params.dilate_iterations);

  cv::morphologyEx(r.mask, r.opened, cv::MORPH_OPEN, kernel,
                   cv::Point(-1, -1), params.erode_iterations);
  cv::morphologyEx(r.mask, r.closed, cv::MORPH_CLOSE, kernel,
                   cv::Point(-1, -1), params.dilate_iterations);

  cv::morphologyEx(r.opened, r.final_mask, cv::MORPH_CLOSE, kernel,
                   cv::Point(-1, -1), params.dilate_iterations);

  std::vector<std::vector<cv::Point>> all_contours;
  cv::findContours(r.final_mask, all_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  r.contours.clear();
  r.contours_filtered = 0;
  for (auto& contour : all_contours) {
    if (params.max_contour_area > 0.0 &&
        cv::contourArea(contour) > params.max_contour_area) {
      ++r.contours_filtered;
      continue;
    }
    r.contours.push_back(std::move(contour));
  }
  return r;
}

namespace {

float lightDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
  const float dx = p1.x - p2.x;
  const float dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

cv::RotatedRect adjustLightRect(const cv::RotatedRect& rec) {
  float width = rec.size.width;
  float height = rec.size.height;
  float angle = rec.angle;

  if (width > height) {
    std::swap(width, height);
    angle += 90.0f;
  }

  if (angle >= 90.0f) angle -= 180.0f;
  if (angle < -90.0f) angle += 180.0f;

  if (std::abs(angle) > 45.0f) {
    std::swap(width, height);
    angle = angle >= 0 ? angle - 90.0f : angle + 90.0f;
  }

  while (angle >= 90.0f) angle -= 180.0f;
  while (angle < -90.0f) angle += 180.0f;

  return cv::RotatedRect(rec.center, cv::Size2f(width, height), angle);
}

void adjustVertexOrder(cv::Point2f* pts) {
  int idx = 0;
  for (int i = 1; i < 4; ++i) {
    if (pts[i].x + pts[i].y < pts[idx].x + pts[idx].y) idx = i;
  }
  cv::Point2f temp[4];
  for (int i = 0; i < 4; ++i) temp[i] = pts[(idx + i) % 4];
  std::copy(temp, temp + 4, pts);
}

}  // namespace

DetectionResult detect(const std::vector<std::vector<cv::Point>>& contours,
                       const DetectionParams& params) {
  DetectionResult result;

  std::vector<LightDescriptor> lights;
  for (const auto& contour : contours) {
    const double light_contour_area = cv::contourArea(contour);
    if (light_contour_area <= params.min_light_area) continue;
    if (contour.size() < 5) continue;

    const cv::RotatedRect adjusted = adjustLightRect(cv::minAreaRect(contour));

    LightDescriptor light;
    light.width = adjusted.size.width;
    light.length = adjusted.size.height;
    light.center = adjusted.center;
    light.angle = adjusted.angle;
    light.area = light.width * light.length;

    cv::Mat hull;
    cv::convexHull(contour, hull);
    const double hull_area = cv::contourArea(hull);
    const double solidity = light_contour_area / hull_area;
    if (solidity < params.min_contour_solidity) continue;

    if (light.width / light.length > params.max_light_ratio) continue;

    lights.push_back(light);
  }

  std::sort(lights.begin(), lights.end(),
            [](const LightDescriptor& a, const LightDescriptor& b) {
              return a.center.x < b.center.x;
            });

  for (std::size_t i = 0; i < lights.size(); ++i) {
    for (std::size_t j = i + 1; j < lights.size(); ++j) {
      const LightDescriptor& left = lights[i];
      const LightDescriptor& right = lights[j];

      const float angle_diff = std::abs(left.angle - right.angle);
      const float len_diff_ratio =
          std::abs(left.length - right.length) / std::max(left.length, right.length);
      if (angle_diff > params.max_angle_diff ||
          len_diff_ratio > params.max_height_diff_ratio) {
        continue;
      }

      const float distance = lightDistance(left.center, right.center);
      const float mean_len = (left.length + right.length) / 2.0F;
      const float y_diff = std::abs(left.center.y - right.center.y);
      const float y_diff_ratio = y_diff / mean_len;
      const float x_diff = std::abs(left.center.x - right.center.x);
      const float x_diff_ratio = x_diff / mean_len;
      const float ratio = distance / mean_len;

      if (y_diff_ratio > params.max_y_diff_ratio ||
          x_diff_ratio < params.min_x_diff_ratio ||
          ratio > params.max_armor_ratio ||
          ratio < params.min_armor_ratio) {
        continue;
      }

      const float angle = (left.angle + right.angle) / 2.0F;
      const cv::Point2f center =
          (left.center + right.center) * 0.5F;
      const float width = std::sqrt((right.center - left.center).ddot(right.center - left.center));
      const float height = mean_len;

      cv::RotatedRect armor_rect(center, cv::Size2f(width, height), angle);

      ArmorDescriptor armor;
      armor.left = left;
      armor.right = right;
      armor_rect.points(armor.vertex);
      adjustVertexOrder(armor.vertex);

      result.armors.push_back(armor);
    }
  }

  result.lights = std::move(lights);
  return result;
}

cv::Mat drawResult(const cv::Mat& bgr, const MorphologyResult& m,
                   const DetectionResult& det) {
  cv::Mat result = bgr.clone();

  cv::drawContours(result, m.contours, -1, cv::Scalar(0, 255, 0), 1);

  for (const auto& light : det.lights) {
    const cv::RotatedRect rect = light.rect();
    cv::Point2f vertices[4];
    rect.points(vertices);
    for (int i = 0; i < 4; ++i) {
      cv::line(result, vertices[i], vertices[(i + 1) % 4],
               cv::Scalar(0, 255, 255), 2);
    }
  }

  for (const auto& armor : det.armors) {
    const std::vector<cv::Point> box(armor.vertex, armor.vertex + 4);
    cv::polylines(result, box, true, cv::Scalar(255, 0, 0), 3);
  }

  return result;
}

namespace {

cv::Mat makeCell(const cv::Mat& image, const std::string& label, int width) {
  cv::Mat cell;
  if (image.channels() == 1) {
    cv::cvtColor(image, cell, cv::COLOR_GRAY2BGR);
  } else {
    cell = image.clone();
  }
  const double font_scale = std::max(0.5, width / 450.0);
  const int thickness = width >= 480 ? 2 : 1;
  cv::putText(cell, label, cv::Point(8, 26), cv::FONT_HERSHEY_SIMPLEX,
              font_scale, cv::Scalar(0, 255, 255), thickness, cv::LINE_AA);
  const double scale = static_cast<double>(width) / cell.cols;
  cv::resize(cell, cell, cv::Size(width, static_cast<int>(cell.rows * scale)));
  return cell;
}

}  // namespace

cv::Mat makeGrid(const cv::Mat& bgr, const MorphologyResult& m,
                 const DetectionResult& det, int cell_width) {
  const cv::Mat cells[2][4] = {
      {makeCell(bgr, "Original", cell_width),
       makeCell(m.mask, "Threshold (R-B)", cell_width),
       makeCell(m.eroded, "Erode", cell_width),
       makeCell(m.dilated, "Dilate", cell_width)},
      {makeCell(m.opened, "Open (erode+dilate)", cell_width),
       makeCell(m.closed, "Close (dilate+erode)", cell_width),
       makeCell(m.final_mask, "Open then Close", cell_width),
       makeCell(drawResult(bgr, m, det), "Result (lights+armors)", cell_width)}};

  cv::Mat row0, row1, grid;
  cv::hconcat(std::vector<cv::Mat>(std::begin(cells[0]), std::end(cells[0])), row0);
  cv::hconcat(std::vector<cv::Mat>(std::begin(cells[1]), std::end(cells[1])), row1);
  cv::vconcat(std::vector<cv::Mat>{row0, row1}, grid);
  return grid;
}

}  // namespace vision
