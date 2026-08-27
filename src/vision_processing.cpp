#include "vision_processing.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace vision {

cv::Mat splitRedMask(const cv::Mat& bgr, int rb_threshold) {
  return splitColorMask(bgr, LightColor::Red, rb_threshold);
}

cv::Mat splitColorMask(const cv::Mat& bgr, LightColor color,
                       int rb_threshold) {
  std::vector<cv::Mat> bgr_channels;
  cv::split(bgr, bgr_channels);

  cv::Mat channel_diff;
  if (color == LightColor::Blue) {
    cv::subtract(bgr_channels[2], bgr_channels[0], channel_diff);  // B - R
  } else {
    cv::subtract(bgr_channels[0], bgr_channels[2], channel_diff);  // R - B
  }

  cv::Mat mask;
  cv::threshold(channel_diff, mask, rb_threshold, 255, cv::THRESH_BINARY);
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

  if (params.enable_open) {
    cv::morphologyEx(r.mask, r.opened, cv::MORPH_OPEN, kernel,
                     cv::Point(-1, -1), params.erode_iterations);
    cv::morphologyEx(r.mask, r.closed, cv::MORPH_CLOSE, kernel,
                     cv::Point(-1, -1), params.dilate_iterations);
    cv::morphologyEx(r.opened, r.final_mask, cv::MORPH_CLOSE, kernel,
                     cv::Point(-1, -1), params.dilate_iterations);
  } else {
    cv::morphologyEx(r.mask, r.closed, cv::MORPH_CLOSE, kernel,
                     cv::Point(-1, -1), params.dilate_iterations);
    r.final_mask = r.closed.clone();
  }

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
  cv::Point2f center(0.0F, 0.0F);
  for (int i = 0; i < 4; ++i) center += pts[i];
  center *= 0.25F;

  // Image coordinates have +x right and +y down, so increasing atan2 angle
  // around the center is clockwise.
  std::array<int, 4> order{0, 1, 2, 3};
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    return std::atan2(pts[a].y - center.y, pts[a].x - center.x) <
           std::atan2(pts[b].y - center.y, pts[b].x - center.x);
  });

  // For a rotated rectangle, the top-most point is not necessarily the
  // image-space top-left point. The minimum x+y corner is the stable TL
  // choice, including for normal perspective/rotation angles.
  int start = 0;
  double best_score = static_cast<double>(pts[order[0]].x) +
                      static_cast<double>(pts[order[0]].y);
  for (int i = 1; i < 4; ++i) {
    const cv::Point2f& p = pts[order[i]];
    const double score = static_cast<double>(p.x) + static_cast<double>(p.y);
    if (score < best_score) {
      best_score = score;
      start = i;
    }
  }

  std::array<cv::Point2f, 4> sorted{};
  for (int i = 0; i < 4; ++i) sorted[i] = pts[order[(start + i) % 4]];

  // Positive shoelace area is clockwise in image coordinates. Reverse the
  // winding while preserving the upper-left first corner when necessary.
  double area2 = 0.0;
  for (int i = 0; i < 4; ++i) {
    const auto& a = sorted[i];
    const auto& b = sorted[(i + 1) % 4];
    area2 += static_cast<double>(a.x) * b.y -
             static_cast<double>(b.x) * a.y;
  }
  if (area2 < 0.0) std::swap(sorted[1], sorted[3]);
  std::copy(sorted.begin(), sorted.end(), pts);
}

}  // namespace

ArmorDescriptor orderedArmor(const ArmorDescriptor& armor) {
  ArmorDescriptor ordered = armor;
  adjustVertexOrder(ordered.vertex);
  ordered.left.center = (ordered.vertex[0] + ordered.vertex[3]) * 0.5F;
  ordered.right.center = (ordered.vertex[1] + ordered.vertex[2]) * 0.5F;
  return ordered;
}

void normalizeArmorVertices(DetectionResult& result) {
  for (auto& armor : result.armors) {
    armor = orderedArmor(armor);
  }
}

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

      ArmorCandidate cand;
      cand.left_idx = static_cast<int>(i);
      cand.right_idx = static_cast<int>(j);
      cand.angle_diff = std::abs(left.angle - right.angle);
      cand.len_diff_ratio =
          std::abs(left.length - right.length) / std::max(left.length, right.length);
      cand.distance = lightDistance(left.center, right.center);
      cand.mean_len = (left.length + right.length) / 2.0F;
      cand.y_diff_ratio =
          std::abs(left.center.y - right.center.y) / cand.mean_len;
      cand.x_diff_ratio =
          std::abs(left.center.x - right.center.x) / cand.mean_len;
      cand.ratio = cand.distance / cand.mean_len;

      if (cand.angle_diff > params.max_angle_diff ||
          cand.len_diff_ratio > params.max_height_diff_ratio) {
        result.candidates.push_back(cand);
        continue;
      }

      if (cand.y_diff_ratio > params.max_y_diff_ratio ||
          cand.x_diff_ratio < params.min_x_diff_ratio ||
          cand.ratio > params.max_armor_ratio ||
          cand.ratio < params.min_armor_ratio) {
        result.candidates.push_back(cand);
        continue;
      }

      cand.accepted = true;
      result.candidates.push_back(cand);

      const float angle = (left.angle + right.angle) / 2.0F;
      const cv::Point2f center =
          (left.center + right.center) * 0.5F;
      const float width = std::sqrt((right.center - left.center).ddot(right.center - left.center));
      const float height = cand.mean_len;

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
    const ArmorDescriptor ordered = orderedArmor(armor);
    const std::vector<cv::Point> box(ordered.vertex, ordered.vertex + 4);
    cv::polylines(result, box, true, cv::Scalar(255, 0, 0), 3);
    for (int i = 0; i < 4; ++i) {
      cv::putText(result, std::to_string(i + 1), ordered.vertex[i],
                  cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 0, 255),
                  2, cv::LINE_AA);
    }
  }

  return result;
}

namespace {

cv::Mat makeCell(const cv::Mat& image, const std::string& label, int width) {
  cv::Mat cell;
  if (image.empty()) {
    // Placeholder for disabled stages (e.g. open with --no-open). 4:3 to
    // match the 1440x1080 source so hconcat keeps a uniform cell height.
    cell = cv::Mat::zeros(120, 160, CV_8UC3);
  } else if (image.channels() == 1) {
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

namespace {

cv::Matx33d quatToRot(double qx, double qy, double qz, double qw) {
  const double w = qw, x = qx, y = qy, z = qz;
  return cv::Matx33d(
      1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
      2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
      2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y));
}

// Convert an SDK wxyz quaternion to a rotation matrix.
cv::Matx33d quatWxyzToRot(double qw, double qx, double qy, double qz) {
  return quatToRot(qx, qy, qz, qw);
}

}  // namespace

cv::Vec3d cameraToGimbal(const cv::Vec3d& p_cam,
                         const CameraExtrinsics& extrinsics) {
  // Extrinsics map gimbal -> camera_optical: p_cam = R * p_gimbal + t.
  // Inverse: p_gimbal = R^T * (p_cam - t).
  const cv::Matx33d R = quatToRot(extrinsics.quaternion_xyzw[0],
                                  extrinsics.quaternion_xyzw[1],
                                  extrinsics.quaternion_xyzw[2],
                                  extrinsics.quaternion_xyzw[3]);
  const cv::Vec3d t(extrinsics.translation_m[0], extrinsics.translation_m[1],
                    extrinsics.translation_m[2]);
  return R.t() * (p_cam - t);
}

cv::Vec3d gimbalToCamera(const cv::Vec3d& p_gimbal,
                         const CameraExtrinsics& extrinsics) {
  // p_cam = R * p_gimbal + t.
  const cv::Matx33d R = quatToRot(extrinsics.quaternion_xyzw[0],
                                  extrinsics.quaternion_xyzw[1],
                                  extrinsics.quaternion_xyzw[2],
                                  extrinsics.quaternion_xyzw[3]);
  const cv::Vec3d t(extrinsics.translation_m[0], extrinsics.translation_m[1],
                    extrinsics.translation_m[2]);
  return R * p_gimbal + t;
}

cv::Point2f projectPoint(const cv::Vec3d& p_cam,
                         const CameraIntrinsics& intrinsics) {
  const double z = p_cam[2];
  if (z <= 1e-6) return cv::Point2f(-1.0F, -1.0F);
  const double x = p_cam[0] / z;
  const double y = p_cam[1] / z;
  // Plumb-bob distortion: k1, k2, p1, p2, k3.
  const double k1 = intrinsics.distortion[0];
  const double k2 = intrinsics.distortion[1];
  const double p1 = intrinsics.distortion[2];
  const double p2 = intrinsics.distortion[3];
  const double k3 = intrinsics.distortion[4];
  const double r2 = x * x + y * y;
  const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
  const double xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
  const double yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
  return cv::Point2f(static_cast<float>(intrinsics.fx * xd + intrinsics.cx),
                     static_cast<float>(intrinsics.fy * yd + intrinsics.cy));
}

cv::Vec3d pixelToCamera(const cv::Point2f& uv, double depth,
                        const CameraIntrinsics& intrinsics) {
  // Remove radial/tangential distortion to get normalized coordinates.
  const double x_n = (uv.x - intrinsics.cx) / intrinsics.fx;
  const double y_n = (uv.y - intrinsics.cy) / intrinsics.fy;
  const double k1 = intrinsics.distortion[0];
  const double k2 = intrinsics.distortion[1];
  const double p1 = intrinsics.distortion[2];
  const double p2 = intrinsics.distortion[3];
  const double k3 = intrinsics.distortion[4];
  if (k1 != 0.0 || k2 != 0.0 || k3 != 0.0 || p1 != 0.0 || p2 != 0.0) {
    // One-iteration iterative undistortion (plumb-bob, small distortion).
    double x = x_n, y = y_n;
    for (int it = 0; it < 5; ++it) {
      const double r2 = x * x + y * y;
      const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
      const double ddx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
      const double ddy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
      x = (x_n - ddx) / radial;
      y = (y_n - ddy) / radial;
    }
    return cv::Vec3d(x * depth, y * depth, depth);
  }
  return cv::Vec3d(x_n * depth, y_n * depth, depth);
}

cv::Vec3d pixelToWorld(const cv::Point2f& uv, double depth,
                       const CameraIntrinsics& intrinsics,
                       const CameraExtrinsics& extrinsics) {
  return cameraToGimbal(pixelToCamera(uv, depth, intrinsics), extrinsics);
}

cv::Vec2d gimbalRelativeAimAngles(const cv::Vec3d& p_gimbal) {
  const double horizontal = std::hypot(p_gimbal[0], p_gimbal[2]);
  if (horizontal <= 1e-9 && std::abs(p_gimbal[1]) <= 1e-9) {
    return cv::Vec2d(0.0, 0.0);
  }

  const double yaw = std::atan2(p_gimbal[0], p_gimbal[2]);
  const double pitch = std::atan2(p_gimbal[1], horizontal);
  return cv::Vec2d(yaw * 180.0 / CV_PI, pitch * 180.0 / CV_PI);
}

cv::Vec2d absoluteWorldAimAngles(const cv::Vec3d& target_world,
                                 const GimbalWorldPose& gimbal_pose) {
  const cv::Vec3d direction(
      target_world[0] - gimbal_pose.position_m[0],
      target_world[1] - gimbal_pose.position_m[1],
      target_world[2] - gimbal_pose.position_m[2]);
  const double horizontal = std::hypot(direction[0], direction[2]);
  const double yaw = std::atan2(direction[0], direction[2]);
  const double pitch = std::atan2(direction[1], horizontal);
  return cv::Vec2d(yaw * 180.0 / CV_PI, 90.0 + pitch * 180.0 / CV_PI);
}

cv::Vec2d worldToAimAngles(const cv::Vec3d& p_cam) {
  const cv::Vec2d relative = gimbalRelativeAimAngles(p_cam);
  return cv::Vec2d(relative[0], 90.0 + relative[1]);
}

cv::Vec2d absoluteAimAngles(const cv::Vec3d& p_gimbal, double gimbal_yaw_deg,
                            double gimbal_pitch_deg, double camera_tilt_deg) {
  (void)camera_tilt_deg;
  const cv::Vec2d relative = gimbalRelativeAimAngles(p_gimbal);
  return cv::Vec2d(gimbal_yaw_deg + relative[0],
                   gimbal_pitch_deg + relative[1]);
}

cv::Vec2d aimWithGravity(const cv::Vec3d& p_cam, double gimbal_yaw_deg,
                         double gimbal_pitch_deg, double camera_tilt_deg,
                         double projectile_speed_mps) {
  if (projectile_speed_mps <= 0.0) {
    return absoluteAimAngles(p_cam, gimbal_yaw_deg, gimbal_pitch_deg,
                             camera_tilt_deg);
  }

  const double distance = cv::norm(p_cam);
  if (distance <= 1e-6) {
    return absoluteAimAngles(p_cam, gimbal_yaw_deg, gimbal_pitch_deg,
                             camera_tilt_deg);
  }

  // Ballistic drop: p(t) = p0 + v0 t + 0.5 g t^2, g = [0,-9.81,0] (world +Y
  // up). Air drag disabled. Flight time t = distance/v0; drop = 0.5 g t^2.
  // The gun must pitch up by atan2(drop, distance). Because pitch_deg
  // increases when aiming up (90 = level), we add the drop angle.
  const double t = distance / projectile_speed_mps;
  const double drop = 0.5 * 9.81 * t * t;
  const double drop_deg = std::atan2(drop, distance) * 180.0 / CV_PI;

  const cv::Vec2d base =
      absoluteAimAngles(p_cam, gimbal_yaw_deg, gimbal_pitch_deg, camera_tilt_deg);
  return cv::Vec2d(base[0], base[1] + drop_deg);
}

cv::Vec3d gimbalToWorld(const cv::Vec3d& p_gimbal,
                        const GimbalWorldPose& pose) {
  const cv::Matx33d R = quatWxyzToRot(pose.quaternion_wxyz[0],
                                      pose.quaternion_wxyz[1],
                                      pose.quaternion_wxyz[2],
                                      pose.quaternion_wxyz[3]);
  const cv::Vec3d t(pose.position_m[0], pose.position_m[1],
                    pose.position_m[2]);
  return R * p_gimbal + t;
}

cv::Vec3d cameraToWorld(const cv::Vec3d& p_camera,
                        const GimbalWorldPose& pose) {
  const cv::Matx33d R = quatWxyzToRot(pose.camera_quaternion_wxyz[0],
                                      pose.camera_quaternion_wxyz[1],
                                      pose.camera_quaternion_wxyz[2],
                                      pose.camera_quaternion_wxyz[3]);
  const cv::Vec3d t(pose.camera_position_m[0], pose.camera_position_m[1],
                    pose.camera_position_m[2]);
  return R * p_camera + t;
}

cv::Vec3d worldToGimbal(const cv::Vec3d& p_world,
                        const GimbalWorldPose& pose) {
  const cv::Matx33d R = quatWxyzToRot(pose.quaternion_wxyz[0],
                                      pose.quaternion_wxyz[1],
                                      pose.quaternion_wxyz[2],
                                      pose.quaternion_wxyz[3]);
  const cv::Vec3d t(pose.position_m[0], pose.position_m[1],
                    pose.position_m[2]);
  return R.t() * (p_world - t);
}

cv::Vec3d rotationCameraToGimbal(const cv::Vec3d& rvec_cam,
                                 const CameraExtrinsics& extrinsics) {
  // R_armor2gimbal = R_ext^T * R_armor2camera.
  cv::Mat R_armor2cam;
  cv::Rodrigues(rvec_cam, R_armor2cam);
  const cv::Matx33d R_ext = quatToRot(extrinsics.quaternion_xyzw[0],
                                      extrinsics.quaternion_xyzw[1],
                                      extrinsics.quaternion_xyzw[2],
                                      extrinsics.quaternion_xyzw[3]);
  cv::Mat R_armor2gimbal = R_ext.t() * R_armor2cam;
  cv::Vec3d rvec_gimbal;
  cv::Rodrigues(R_armor2gimbal, rvec_gimbal);
  return rvec_gimbal;
}

cv::Vec3d rotationGimbalToWorld(const cv::Vec3d& rvec_gimbal,
                                const GimbalWorldPose& world_pose) {
  // R_armor2world = R_gimbal2world * R_armor2gimbal.
  cv::Mat R_armor2gimbal;
  cv::Rodrigues(rvec_gimbal, R_armor2gimbal);
  const cv::Matx33d R_gimbal2world =
      quatWxyzToRot(world_pose.quaternion_wxyz[0],
                    world_pose.quaternion_wxyz[1],
                    world_pose.quaternion_wxyz[2],
                    world_pose.quaternion_wxyz[3]);
  cv::Mat R_armor2world = R_gimbal2world * R_armor2gimbal;
  cv::Vec3d rvec_world;
  cv::Rodrigues(R_armor2world, rvec_world);
  return rvec_world;
}

cv::Vec3d rotationMatrixToYpr(const cv::Matx33d& R) {
  // ZYX intrinsic (yaw about z, then pitch about y, then roll about x),
  // matching tongji tools::eulers(R, 2, 1, 0).
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  const double pitch = std::atan2(-R(2, 0), std::hypot(R(2, 1), R(2, 2)));
  const double roll = std::atan2(R(2, 1), R(2, 2));
  return cv::Vec3d(yaw, pitch, roll);
}

cv::Vec3d xyzToYpd(const cv::Vec3d& xyz) {
  const double yaw = std::atan2(xyz[1], xyz[0]);
  const double pitch =
      std::atan2(xyz[2], std::hypot(xyz[0], xyz[1]));
  const double distance = cv::norm(xyz);
  return cv::Vec3d(yaw, pitch, distance);
}

void drawAimHud(cv::Mat& bgr, double gimbal_yaw_deg, double gimbal_pitch_deg,
                bool has_target, const cv::Vec2d& target_aim,
                double target_distance_m, bool has_world_point,
                const cv::Vec3d& target_world, bool send_enabled,
                bool has_send_data, const cv::Vec2d& send_angles,
                double send_distance_m) {
  const int cx = bgr.cols / 2;
  const int cy = bgr.rows / 2;

  // Center crosshair (current aim center of the image).
  cv::line(bgr, cv::Point(cx - 25, cy), cv::Point(cx + 25, cy),
           cv::Scalar(0, 165, 255), 1);
  cv::line(bgr, cv::Point(cx, cy - 25), cv::Point(cx, cy + 25),
           cv::Scalar(0, 165, 255), 1);

  // Horizontal "level" reference line.
  cv::line(bgr, cv::Point(40, cy), cv::Point(bgr.cols - 40, cy),
           cv::Scalar(255, 0, 0), 1);

  // HUD text panel (top-left).
  const cv::Scalar color(0, 255, 255);
  const cv::Scalar target_color(0, 255, 0);
  const cv::Scalar dim(160, 160, 160);
  char line[128];
  std::snprintf(line, sizeof(line), "gimbal yaw=%7.2f  pitch=%7.2f deg",
                gimbal_yaw_deg, gimbal_pitch_deg);
  cv::putText(bgr, line, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.55,
              color, 1, cv::LINE_AA);

  if (has_target) {
    // target_aim is the absolute gimbal angle needed to aim at the target.
    std::snprintf(line, sizeof(line),
                  "target yaw=%7.2f  pitch=%7.2f deg  dist=%6.2f m",
                  target_aim[0], target_aim[1], target_distance_m);
    cv::putText(bgr, line, cv::Point(12, 52), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                target_color, 1, cv::LINE_AA);

    if (has_world_point) {
      std::snprintf(line, sizeof(line),
                    "target world=(%7.3f, %7.3f, %7.3f) m",
                    target_world[0], target_world[1], target_world[2]);
    cv::putText(bgr, line, cv::Point(12, 76), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                  target_color, 1, cv::LINE_AA);
    }

    // Error bars: offset between the absolute target aim and the current
    // gimbal angle, drawn around the center crosshair.
    const double yaw_offset = target_aim[0] - gimbal_yaw_deg;   // degrees
    const double pitch_offset = target_aim[1] - gimbal_pitch_deg;
    const double yaw_px = yaw_offset * 8.0;       // px per degree
    const double pitch_px = pitch_offset * 8.0;
    const cv::Point target_px(static_cast<int>(cx + yaw_px),
                              static_cast<int>(cy - pitch_px));
    cv::circle(bgr, target_px, 6, target_color, 2);
    cv::line(bgr, cv::Point(cx, cy), target_px, target_color, 1);
  } else {
    cv::putText(bgr, "target: none", cv::Point(12, 52),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, dim, 1, cv::LINE_AA);
  }

  // Show exactly the values that the caller passes to the simulator command.
  // Keep this line visible even when no target is available.
  const char* send_state = send_enabled ? "on" : "off";
  if (has_send_data) {
    std::snprintf(line, sizeof(line),
                  "send[%s] yaw=%7.2f pitch=%7.2f dist=%6.2f m",
                  send_state, send_angles[0], send_angles[1],
                  send_distance_m);
  } else {
    std::snprintf(line, sizeof(line), "send[%s] no target data", send_state);
  }
  cv::putText(bgr, line, cv::Point(12, 100), cv::FONT_HERSHEY_SIMPLEX, 0.55,
              send_enabled ? target_color : dim, 1, cv::LINE_AA);
}

void drawCoordinateHud(cv::Mat& bgr, bool valid,
                       const cv::Vec3d& target_camera,
                       const cv::Vec3d& target_gimbal,
                       const cv::Vec3d& target_world_fixed,
                       const cv::Vec3d& target_world_sdk) {
  if (!valid) return;
  char line[192];
  const int x = 12;
  const int y = 124;
  std::snprintf(line, sizeof(line), "cam=(%.3f, %.3f, %.3f) m",
                target_camera[0], target_camera[1], target_camera[2]);
  cv::putText(bgr, line, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.50,
              cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
  std::snprintf(line, sizeof(line), "gimbal=(%.3f, %.3f, %.3f) m",
                target_gimbal[0], target_gimbal[1], target_gimbal[2]);
  cv::putText(bgr, line, cv::Point(x, y + 22), cv::FONT_HERSHEY_SIMPLEX, 0.50,
              cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
  std::snprintf(line, sizeof(line), "world_fixed=(%.3f, %.3f, %.3f) m",
                target_world_fixed[0], target_world_fixed[1],
                target_world_fixed[2]);
  cv::putText(bgr, line, cv::Point(x, y + 44), cv::FONT_HERSHEY_SIMPLEX, 0.50,
              cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
  std::snprintf(line, sizeof(line), "world_sdk=(%.3f, %.3f, %.3f) err=%.3f m",
                target_world_sdk[0], target_world_sdk[1], target_world_sdk[2],
                cv::norm(target_world_fixed - target_world_sdk));
  cv::putText(bgr, line, cv::Point(x, y + 66), cv::FONT_HERSHEY_SIMPLEX, 0.50,
              cv::Scalar(0, 165, 255), 1, cv::LINE_AA);
}

KalmanFilter3D::KalmanFilter3D() {
  Q_ = cv::Mat::zeros(9, 9, CV_64F);
  R_ = cv::Mat::zeros(3, 3, CV_64F);
}

void KalmanFilter3D::init(const cv::Vec3d& pos, double dt) {
  x_ = cv::Mat::zeros(9, 1, CV_64F);
  x_.at<double>(0) = pos[0];
  x_.at<double>(1) = pos[1];
  x_.at<double>(2) = pos[2];

  // Diagonal process-noise covariance in the (x,y,z) / (vx,vy,vz) /
  // (ax,ay,az) blocks. Each block is sigma^2.
  Q_ = cv::Mat::zeros(9, 9, CV_64F);
  const double q_pos = process_pos_sigma * process_pos_sigma;
  const double q_vel = process_vel_sigma * process_vel_sigma;
  const double q_acc = process_acc_sigma * process_acc_sigma;
  for (int i = 0; i < 3; ++i) {
    Q_.at<double>(i, i) = q_pos;
    Q_.at<double>(3 + i, 3 + i) = q_vel;
    Q_.at<double>(6 + i, 6 + i) = q_acc;
  }

  // Measurement noise: 3x3 diagonal.
  R_ = cv::Mat::zeros(3, 3, CV_64F);
  const double r = measurement_sigma * measurement_sigma;
  for (int i = 0; i < 3; ++i) R_.at<double>(i, i) = r;

  // Initial covariance: position confidence from measurement noise, larger
  // for velocity/acceleration (unknown).
  P_ = cv::Mat::eye(9, 9, CV_64F);
  for (int i = 0; i < 3; ++i) {
    P_.at<double>(i, i) = 10.0 * r;
    P_.at<double>(3 + i, 3 + i) = 4.0;
    P_.at<double>(6 + i, 6 + i) = 16.0;
  }

  predicted_pos_ = pos;
  initialized_ = true;
}

void KalmanFilter3D::predict(double dt) {
  if (!initialized_) return;
  if (dt <= 0.0) dt = 1.0 / 60.0;

  // Constant-acceleration transition matrix.
  cv::Mat A = cv::Mat::eye(9, 9, CV_64F);
  const double dt2 = 0.5 * dt * dt;
  for (int i = 0; i < 3; ++i) {
    A.at<double>(i, 3 + i) = dt;
    A.at<double>(i, 6 + i) = dt2;
    A.at<double>(3 + i, 6 + i) = dt;
  }

  // x_pred = A * x
  cv::Mat x_pred = A * x_;
  // P_pred = A * P * A^T + Q
  cv::Mat P_pred = A * P_ * A.t() + Q_;

  x_ = x_pred;
  P_ = P_pred;

  predicted_pos_ = cv::Vec3d(x_.at<double>(0), x_.at<double>(1),
                             x_.at<double>(2));
}

bool KalmanFilter3D::update(const cv::Vec3d& z) {
  if (!initialized_) return false;

  // Measurement matrix H: extract position.
  cv::Mat H = cv::Mat::zeros(3, 9, CV_64F);
  H.at<double>(0, 0) = 1.0;
  H.at<double>(1, 1) = 1.0;
  H.at<double>(2, 2) = 1.0;

  cv::Mat z_mat = (cv::Mat_<double>(3, 1) << z[0], z[1], z[2]);

  // Innovation y = z - H*x_pred
  cv::Mat y = z_mat - H * x_;
  // S = H*P*H^T + R
  cv::Mat S = H * P_ * H.t() + R_;
  // K = P*H^T*S^-1
  cv::Mat K = P_ * H.t() * S.inv(cv::DECOMP_SVD);

  // x = x_pred + K*y
  x_ = x_ + K * y;
  // P = (I - K*H)*P
  cv::Mat I = cv::Mat::eye(9, 9, CV_64F);
  P_ = (I - K * H) * P_;

  return true;
}

cv::Vec3d KalmanFilter3D::position() const {
  return cv::Vec3d(x_.at<double>(0), x_.at<double>(1), x_.at<double>(2));
}

cv::Vec3d KalmanFilter3D::velocity() const {
  return cv::Vec3d(x_.at<double>(3), x_.at<double>(4), x_.at<double>(5));
}

cv::Vec3d KalmanFilter3D::predictedPosition() const { return predicted_pos_; }

TargetPose solveArmorPose(const ArmorDescriptor& armor,
                          const CameraIntrinsics& intrinsics,
                          const CameraExtrinsics& extrinsics,
                          double armor_width, double armor_height) {
  TargetPose pose;

  const double hw = armor_width * 0.5;
  const double hh = armor_height * 0.5;

  // Armor local frame: +X right, +Y down, +Z outward (toward the camera).
  // Order must match adjustVertexOrder output (TL, TR, BR, BL).
  const std::vector<cv::Point3f> object_points = {
      cv::Point3f(-hw, -hh, 0.0F),   // TL
      cv::Point3f(+hw, -hh, 0.0F),   // TR
      cv::Point3f(+hw, +hh, 0.0F),   // BR
      cv::Point3f(-hw, +hh, 0.0F),   // BL
  };
  const ArmorDescriptor ordered = orderedArmor(armor);
  const std::vector<cv::Point2f> image_points = {
      ordered.vertex[0], ordered.vertex[1], ordered.vertex[2], ordered.vertex[3]};

  const cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << intrinsics.fx, 0.0, intrinsics.cx, 0.0,
       intrinsics.fy, intrinsics.cy, 0.0, 0.0, 1.0);
  const cv::Mat distortion =
      (cv::Mat_<double>(1, 5) << intrinsics.distortion[0],
       intrinsics.distortion[1], intrinsics.distortion[2],
       intrinsics.distortion[3], intrinsics.distortion[4]);

  cv::Mat rvec, tvec;
  try {
    if (!cv::solvePnP(object_points, image_points, camera_matrix, distortion,
                      rvec, tvec, false, cv::SOLVEPNP_IPPE)) {
      return pose;
    }
  } catch (const cv::Exception&) {
    return pose;
  }

  // IPPE may return success with non-finite values for degenerate layouts.
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(tvec.at<double>(i)) || !std::isfinite(rvec.at<double>(i))) {
      return pose;
    }
  }

  pose.valid = true;
  pose.t_cam = cv::Vec3d(tvec.at<double>(0), tvec.at<double>(1),
                         tvec.at<double>(2));
  pose.r_cam = cv::Vec3d(rvec.at<double>(0), rvec.at<double>(1),
                         rvec.at<double>(2));
  pose.r_gimbal = rotationCameraToGimbal(pose.r_cam, extrinsics);
  pose.distance_m = cv::norm(pose.t_cam);
  pose.center_gimbal = cameraToGimbal(pose.t_cam, extrinsics);
  pose.t_gimbal = pose.center_gimbal;

  cv::Mat R_cam;
  cv::Rodrigues(rvec, R_cam);

  // Armor corners in the gimbal frame.
  for (int i = 0; i < 4; ++i) {
    const cv::Mat local = (cv::Mat_<double>(3, 1) << object_points[i].x,
                           object_points[i].y, object_points[i].z);
    const cv::Mat p_cam = R_cam * local + tvec;
    const cv::Vec3d p_cam_v(p_cam.at<double>(0), p_cam.at<double>(1),
                            p_cam.at<double>(2));
    const cv::Vec3d p_gimbal = cameraToGimbal(p_cam_v, extrinsics);
    pose.corners_gimbal[i] =
        cv::Point3f(static_cast<float>(p_gimbal[0]),
                    static_cast<float>(p_gimbal[1]),
                    static_cast<float>(p_gimbal[2]));
  }

  // Light-bar centers: intersect the ray through each light center pixel with
  // the armor plane (z=0 in armor local frame).
  const cv::Mat n_cam =
      R_cam * (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
  const double nx = n_cam.at<double>(0);
  const double ny = n_cam.at<double>(1);
  const double nz = n_cam.at<double>(2);
  const double px = pose.t_cam[0];
  const double py = pose.t_cam[1];
  const double pz = pose.t_cam[2];
  const double denom = px * nx + py * ny + pz * nz;

  const cv::Point2f light_centers[2] = {armor.left.center,
                                        armor.right.center};
  for (int i = 0; i < 2; ++i) {
    const cv::Point2f& uv = light_centers[i];
    const double dx = (uv.x - intrinsics.cx) / intrinsics.fx;
    const double dy = (uv.y - intrinsics.cy) / intrinsics.fy;
    // Ray direction through pixel; intersect with the armor plane (through
    // t_cam with normal n_cam).
    const double dir_dot_n = dx * nx + dy * ny + nz;
    if (std::abs(dir_dot_n) < 1e-12 || std::abs(denom) < 1e-12) continue;
    const double lambda = denom / dir_dot_n;
    const cv::Vec3d p_cam(lambda * dx, lambda * dy, lambda);
    pose.light_center_gimbal[i] =
        cv::Point3f(static_cast<float>(cameraToGimbal(p_cam, extrinsics)[0]),
                    static_cast<float>(cameraToGimbal(p_cam, extrinsics)[1]),
                    static_cast<float>(cameraToGimbal(p_cam, extrinsics)[2]));
  }

  return pose;
}

}  // namespace vision
