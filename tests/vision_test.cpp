// vision 功能包单元测试（无需模拟器，直接运行即可）：
//   ./build/vision_test
// 覆盖：颜色掩膜分割、形态学滤波、灯条/装甲板识别、坐标变换、投影、
// PnP 位姿解算与卡尔曼滤波。

#include "tasks/vision/vision_processing.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

namespace {

int g_failures = 0;

#define CHECK(cond)                                                    \
  do {                                                                 \
    if (!(cond)) {                                                     \
      std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << ": "      \
                << #cond << "\n";                                      \
      ++g_failures;                                                    \
    }                                                                  \
  } while (0)

void testSplitColorMask() {
  // 语义（与实现一致）：red 分支 = B−R（检测 B 高、R 低的区域），
  // blue 分支 = R−B（检测 R 高、B 低的区域）。
  cv::Mat img = cv::Mat::zeros(100, 100, CV_8UC3);
  cv::rectangle(img, cv::Rect(10, 10, 30, 30), cv::Scalar(50, 50, 200), cv::FILLED);   // R 高
  cv::rectangle(img, cv::Rect(60, 60, 30, 30), cv::Scalar(200, 50, 50), cv::FILLED);   // B 高

  const cv::Mat red = vision::splitColorMask(img, vision::LightColor::Red, 30);   // B − R
  CHECK(cv::countNonZero(red) == 30 * 30);
  if (cv::countNonZero(red) > 0) {
    cv::Mat nz;
    cv::findNonZero(red, nz);
    CHECK(cv::boundingRect(nz) == cv::Rect(60, 60, 30, 30));  // 命中 B 高那块
  }

  const cv::Mat blue = vision::splitColorMask(img, vision::LightColor::Blue, 30);  // R − B
  CHECK(cv::countNonZero(blue) == 30 * 30);
  if (cv::countNonZero(blue) > 0) {
    cv::Mat nz;
    cv::findNonZero(blue, nz);
    CHECK(cv::boundingRect(nz) == cv::Rect(10, 10, 30, 30));  // 命中 R 高那块
  }
}

void testApplyMorphology() {
  cv::Mat mask = cv::Mat::zeros(100, 100, CV_8UC1);
  cv::rectangle(mask, cv::Rect(20, 20, 40, 40), 255, cv::FILLED);

  vision::MorphologyParams params;
  params.kernel_size = 3;
  params.rb_threshold = 30;
  params.enable_open = false;
  const vision::MorphologyResult r = vision::applyMorphology(mask, params);

  CHECK(r.mask.rows == mask.rows && r.mask.cols == mask.cols);
  CHECK(r.final_mask.rows == mask.rows && r.final_mask.cols == mask.cols);
  CHECK(r.contours.size() == 1);
  CHECK(cv::countNonZero(r.final_mask) > 0);
}

void testDetect() {
  // 两条竖灯条（按 red 分支语义 B−R，构造 B 高、R 低），应被识别为一个装甲板。
  cv::Mat img = cv::Mat::zeros(200, 400, CV_8UC3);
  cv::ellipse(img, cv::Point(110, 110), cv::Size(10, 50), 0, 0, 360,
              cv::Scalar(200, 50, 50), cv::FILLED);
  cv::ellipse(img, cv::Point(290, 110), cv::Size(10, 50), 0, 0, 360,
              cv::Scalar(200, 50, 50), cv::FILLED);

  vision::MorphologyParams params;
  params.kernel_size = 1;
  params.rb_threshold = 30;
  params.enable_open = false;
  const vision::MorphologyResult morph =
      vision::applyMorphology(
          vision::splitColorMask(img, vision::LightColor::Red, 30), params);

  vision::DetectionParams det_params;
  det_params.min_light_area = 10.0;
  const vision::DetectionResult det = vision::detect(morph.contours, det_params);

  CHECK(det.lights.size() == 2);
  CHECK(det.armors.size() >= 1);
}

void testCoordinateTransforms() {
  // 单位外参（identity）：相机系 == 云台系。
  vision::CameraExtrinsics e;
  const cv::Vec3d p(0.1, -0.2, 3.0);
  const cv::Vec3d back = vision::gimbalToCamera(vision::cameraToGimbal(p, e), e);
  CHECK(cv::norm(back - p) < 1e-9);

  // 非零平移 + 90° 旋转（绕 Z 轴 wxyz）下的往返一致性。
  vision::CameraExtrinsics e2;
  e2.translation_m[0] = 0.05;
  e2.translation_m[1] = -0.02;
  e2.translation_m[2] = 0.1;
  const double qw = std::cos(0.785398163);  // 45°
  const double qz = std::sin(0.785398163);
  e2.quaternion_xyzw[0] = 0.0;
  e2.quaternion_xyzw[1] = 0.0;
  e2.quaternion_xyzw[2] = qz;
  e2.quaternion_xyzw[3] = qw;
  const cv::Vec3d q(0.3, 0.4, 2.5);
  const cv::Vec3d back2 = vision::gimbalToCamera(vision::cameraToGimbal(q, e2), e2);
  CHECK(cv::norm(back2 - q) < 1e-9);
}

void testProjectPoint() {
  vision::CameraIntrinsics k;
  k.fx = 1000.0;
  k.fy = 1000.0;
  k.cx = 500.0;
  k.cy = 500.0;

  const cv::Point2f uv = vision::projectPoint(cv::Vec3d(0.0, 0.0, 1.0), k);
  CHECK(std::fabs(uv.x - 500.0) < 1e-3);
  CHECK(std::fabs(uv.y - 500.0) < 1e-3);

  const cv::Point2f off = vision::projectPoint(cv::Vec3d(0.1, 0.2, 1.0), k);
  CHECK(std::fabs(off.x - 600.0) < 1e-3);
  CHECK(std::fabs(off.y - 700.0) < 1e-3);
}

void testSolveArmorPose() {
  // 仿真：装甲板在云台系 (0,0,3)，单位外参，fx=fy=1000，中心 (500,500)。
  // 图像四角 = 中心 ± (fx*w/(2z), fy*h/(2z))。
  vision::CameraIntrinsics k;
  k.fx = 1000.0;
  k.fy = 1000.0;
  k.cx = 500.0;
  k.cy = 500.0;

  const double z = 3.0;
  const double hw = 0.135 / 2.0;  // 半宽 0.0675 m
  const double hh = 0.056 / 2.0;  // 半高 0.028 m
  const double px = k.fx * hw / z;
  const double py = k.fy * hh / z;

  vision::ArmorDescriptor armor;
  armor.vertex[0] = cv::Point2f(500.0f - static_cast<float>(px),
                                500.0f - static_cast<float>(py));  // TL
  armor.vertex[1] = cv::Point2f(500.0f + static_cast<float>(px),
                                500.0f - static_cast<float>(py));  // TR
  armor.vertex[2] = cv::Point2f(500.0f + static_cast<float>(px),
                                500.0f + static_cast<float>(py));  // BR
  armor.vertex[3] = cv::Point2f(500.0f - static_cast<float>(px),
                                500.0f + static_cast<float>(py));  // BL
  armor.left.center = (armor.vertex[0] + armor.vertex[3]) * 0.5f;
  armor.right.center = (armor.vertex[1] + armor.vertex[2]) * 0.5f;

  vision::CameraExtrinsics e;
  const vision::TargetPose pose =
      vision::solveArmorPose(armor, k, e, 0.135, 0.056);

  CHECK(pose.valid);
  if (pose.valid) {
    CHECK(pose.distance_m > 2.0 && pose.distance_m < 4.0);
    CHECK(std::fabs(pose.t_gimbal[2] - 3.0) < 0.2);
  }
}

void testKalman() {
  vision::KalmanFilter3D kf;
  kf.measurement_sigma = 0.01;
  kf.process_pos_sigma = 0.001;
  kf.process_vel_sigma = 0.01;
  kf.process_acc_sigma = 0.01;

  kf.init(cv::Vec3d(1.0, 2.0, 3.0), 0.05);
  CHECK(kf.valid());

  kf.predict(0.05);
  kf.update(cv::Vec3d(1.05, 2.0, 3.0));
  const cv::Vec3d pos = kf.position();
  CHECK(std::fabs(pos[0] - 1.05) < 0.05);

  // 连续跟踪 10 帧的匀速目标，速度估计应收敛到 (1,0,0)。
  for (int i = 0; i < 10; ++i) {
    kf.predict(0.05);
    kf.update(cv::Vec3d(1.05 + 1.0 * 0.05 * (i + 1), 2.0, 3.0));
  }
  const cv::Vec3d vel = kf.velocity();
  CHECK(vel[0] > 0.5 && vel[0] < 1.5);
  CHECK(std::fabs(vel[1]) < 0.2 && std::fabs(vel[2]) < 0.2);
}

}  // namespace

int main() {
  testSplitColorMask();
  testApplyMorphology();
  testDetect();
  testCoordinateTransforms();
  testProjectPoint();
  testSolveArmorPose();
  testKalman();

  if (g_failures == 0) {
    std::cout << "vision_test: all tests passed\n";
    return 0;
  }
  std::cerr << "vision_test: " << g_failures << " check(s) failed\n";
  return 1;
}
