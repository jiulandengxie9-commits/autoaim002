#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace vision {

// Target light-bar color to segment.
enum class LightColor { Red, Blue };

struct MorphologyParams {
  int kernel_size = 5;
  int rb_threshold = 30;
  int erode_iterations = 1;
  int dilate_iterations = 1;
  double max_contour_area = 0.0;
  bool enable_open = true;
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
  double min_light_area = 40.0;
  double min_contour_solidity = 0.3;
  double max_light_ratio = 0.8;
  double max_angle_diff = 15.0;
  double max_height_diff_ratio = 0.2;
  double max_y_diff_ratio = 0.5;
  double min_x_diff_ratio = 0.5;
  double max_armor_ratio = 3.0;
  double min_armor_ratio = 1.0;
};

// One candidate armor pair (two lights) and the constraint values used to
// decide whether it is accepted. Filled by detect() for every light pair so
// the thresholds can be tuned from the terminal (see --det-debug).
struct ArmorCandidate {
  int left_idx = -1;
  int right_idx = -1;
  bool accepted = false;
  float angle_diff = 0.0F;       // |a1 - a2|
  float len_diff_ratio = 0.0F;   // |len1-len2| / max
  float y_diff_ratio = 0.0F;     // |y1-y2| / meanLen
  float x_diff_ratio = 0.0F;     // |x1-x2| / meanLen
  float ratio = 0.0F;            // distance / meanLen
  float distance = 0.0F;         // light center distance (px)
  float mean_len = 0.0F;         // mean light length (px)
};

struct DetectionResult {
  std::vector<LightDescriptor> lights;
  std::vector<ArmorDescriptor> armors;
  std::vector<ArmorCandidate> candidates;
};

// Camera intrinsic parameters (from camera-calibration.json / readCameraInfo).
struct CameraIntrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double distortion[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
};

// Extrinsic transform of the gimbal -> camera_optical frame. The SDK exposes a
// camera fixed to the gimbal; its pose in the robot's gimbal frame is the same
// set of static extrinsics as in camera-calibration.json.
struct CameraExtrinsics {
  double translation_m[3] = {0.0, 0.0, 0.0};  // gimbal -> camera translation
  double quaternion_xyzw[4] = {0.0, 0.0, 0.0, 1.0};  // gimbal -> camera rotation
};

// Pose of the gimbal in the absolute world (odom) frame, as provided by
// TalosMetadataReader::readExposureStateForFrame(). quaternion_wxyz uses the
// SDK's wxyz ordering.
struct GimbalWorldPose {
  double position_m[3] = {0.0, 0.0, 0.0};
  double quaternion_wxyz[4] = {1.0, 0.0, 0.0, 0.0};
  bool valid = false;
};

// A full 3D pose (world coordinates) of a detected target, expressed in the
// camera optical frame and in the robot gimbal frame (the "world" for aiming).
struct TargetPose {
  bool valid = false;
  double distance_m = 0.0;          // camera -> target center, meters
  cv::Vec3d t_cam;                  // translation in camera optical frame
  cv::Vec3d r_cam;                  // rotation vector in camera optical frame
  cv::Vec3d t_gimbal;               // translation in gimbal (world) frame
  cv::Vec3d r_gimbal;               // rotation vector in gimbal frame
  cv::Vec3d center_gimbal;          // target center in gimbal frame
  cv::Point3f corners_gimbal[4];    // armor 4 corners in gimbal frame
  cv::Point3f light_center_gimbal[2];  // left/right light bar centers in gimbal frame
};

// Solve the pose of an armor plate with solvePnP using its 4 image corners,
// then re-project each light-bar center onto the armor plane to obtain the
// light bar 3D position in the gimbal (world) frame. Also transforms the
// armor rotation into the gimbal frame (R_armor2gimbal = R_ext^T * R_armor2cam).
TargetPose solveArmorPose(const ArmorDescriptor& armor,
                          const CameraIntrinsics& intrinsics,
                          const CameraExtrinsics& extrinsics,
                          double armor_width = 0.135,
                          double armor_height = 0.056);

// Transform an armor rotation vector from the camera frame into the gimbal
// frame: R_armor2gimbal = R_ext^T * R_armor2camera.
cv::Vec3d rotationCameraToGimbal(const cv::Vec3d& rvec_cam,
                                 const CameraExtrinsics& extrinsics);

// Transform a gimbal-frame rotation into the world (odom) frame:
// R_armor2world = R_gimbal2world * R_armor2gimbal.
cv::Vec3d rotationGimbalToWorld(const cv::Vec3d& rvec_gimbal,
                                const GimbalWorldPose& world_pose);

// Extract ZYX Euler angles (yaw, pitch, roll) in radians from a rotation
// matrix (same convention as tongji tools::eulers(R, 2, 1, 0)).
cv::Vec3d rotationMatrixToYpr(const cv::Matx33d& R);

// Cartesian (x, y, z) -> spherical (yaw, pitch, distance), radians/meters.
// yaw = atan2(y, x); pitch = atan2(z, hypot(x, y)) (world frame convention).
cv::Vec3d xyzToYpd(const cv::Vec3d& xyz);

// Transform a camera-optical-frame point into the gimbal (world) frame using
// the static gimbal->camera extrinsics.
cv::Vec3d cameraToGimbal(const cv::Vec3d& p_cam,
                         const CameraExtrinsics& extrinsics);

// Inverse of cameraToGimbal: gimbal (world) frame -> camera optical frame.
cv::Vec3d gimbalToCamera(const cv::Vec3d& p_gimbal,
                         const CameraExtrinsics& extrinsics);

// Project a camera-optical-frame 3D point to pixel coordinates using the
// pinhole model (distortion applied when nonzero).
cv::Point2f projectPoint(const cv::Vec3d& p_cam,
                         const CameraIntrinsics& intrinsics);

// Unproject a pixel at a known depth (meters along the optical axis) into the
// camera optical frame. Distortion is removed when nonzero.
cv::Vec3d pixelToCamera(const cv::Point2f& uv, double depth,
                        const CameraIntrinsics& intrinsics);

// Pixel -> world (gimbal) frame: unproject at depth, then apply the static
// gimbal->camera extrinsics (cameraToGimbal).
cv::Vec3d pixelToWorld(const cv::Point2f& uv, double depth,
                       const CameraIntrinsics& intrinsics,
                       const CameraExtrinsics& extrinsics);

// Yaw/pitch angles (degrees) relative to the current gimbal orientation for a
// target expressed in the gimbal frame. The gimbal frame uses +x right, +y up,
// +z forward. Pitch is 90 degrees at level and increases when aiming up.
cv::Vec2d gimbalRelativeAimAngles(const cv::Vec3d& p_gimbal);

// Yaw/pitch angles (degrees) to aim the gimbal at a camera-frame point.
// This legacy helper assumes the camera optical frame is already aligned with
// the gimbal axes and should not be used when the fixed camera extrinsic is
// non-identity. Prefer gimbalRelativeAimAngles() after cameraToGimbal().
cv::Vec2d worldToAimAngles(const cv::Vec3d& p_cam);

// Absolute gimbal yaw/pitch (degrees) the gimbal must be at to aim at a
// gimbal-frame target point. The relative angles are added to the actual
// exposure-synchronized gimbal angles, so the result is an absolute command.
cv::Vec2d absoluteAimAngles(const cv::Vec3d& p_gimbal, double gimbal_yaw_deg,
                            double gimbal_pitch_deg, double camera_tilt_deg);

// Gravity-compensated absolute aim angles. Projectile follows ballistic
// motion: p(t) = p0 + v0 t + 0.5 g t^2 with g = [0,-9.81,0] (world +Y up).
// Flight time t = distance/v0; the gun must pitch up by atan2(0.5 g t^2,
// distance) to compensate the drop (air drag is disabled in the contest).
cv::Vec2d aimWithGravity(const cv::Vec3d& p_cam, double gimbal_yaw_deg,
                         double gimbal_pitch_deg, double camera_tilt_deg,
                         double projectile_speed_mps);

// Transform a gimbal-frame point into the absolute world (odom) frame using
// the gimbal world pose. p_world = R(q_world) * p_gimbal + t_world.
cv::Vec3d gimbalToWorld(const cv::Vec3d& p_gimbal,
                        const GimbalWorldPose& pose);

// Inverse of gimbalToWorld: world (odom) frame -> gimbal frame.
cv::Vec3d worldToGimbal(const cv::Vec3d& p_world,
                        const GimbalWorldPose& pose);

// Draw an aiming HUD on a BGR image: current gimbal angles, the absolute
// gimbal yaw/pitch needed to aim at the target (target_aim, from
// absoluteAimAngles) and distance, plus a center crosshair and a horizontal
// "level" reference line. The error circle is drawn at the offset between
// target_aim and the current gimbal angle.
void drawAimHud(cv::Mat& bgr, double gimbal_yaw_deg, double gimbal_pitch_deg,
                bool has_target, const cv::Vec2d& target_aim,
                double target_distance_m);

// Linear Kalman filter for a 3D target position. State vector is
// [x, y, z, vx, vy, vz, ax, ay, az] with a constant-acceleration model
// (same A/H structure as big_homework.cpp's per-axis Kalman, but in 3D and
// with configurable process/measurement noise). Measurements are the raw PnP
// gimbal-frame coordinates; predict() advances the motion model with dt,
// update() fuses a new observation.
class KalmanFilter3D {
 public:
  KalmanFilter3D();

  // Noise standard deviations (meters / meters-per-second / m/s^2) and
  // measurement std-dev (meters). Configure before first use.
  double process_pos_sigma = 0.01;
  double process_vel_sigma = 0.1;
  double process_acc_sigma = 0.2;
  double measurement_sigma = 0.03;

  void init(const cv::Vec3d& pos, double dt = 1.0 / 60.0);

  // Advance the motion model by dt seconds (no measurement).
  void predict(double dt);

  // Fuse a position measurement. Returns true on success.
  bool update(const cv::Vec3d& z);

  bool valid() const { return initialized_; }

  cv::Vec3d position() const;    // filtered position (posterior)
  cv::Vec3d velocity() const;    // filtered velocity
  cv::Vec3d predictedPosition() const;  // last prediction (prior)

 private:
  cv::Mat x_;  // 9x1 state
  cv::Mat P_;  // 9x9 covariance
  cv::Mat Q_;  // 9x9 process noise
  cv::Mat R_;  // 3x3 measurement noise
  bool initialized_ = false;
  cv::Vec3d predicted_pos_;
};

cv::Mat splitRedMask(const cv::Mat& bgr, int rb_threshold);

// Split the mask of a colored light bar: red uses (R-B), blue uses (B-R).
cv::Mat splitColorMask(const cv::Mat& bgr, LightColor color,
                       int rb_threshold);

MorphologyResult applyMorphology(const cv::Mat& mask,
                                 const MorphologyParams& params = {});

DetectionResult detect(const std::vector<std::vector<cv::Point>>& contours,
                       const DetectionParams& params = {});

cv::Mat drawResult(const cv::Mat& bgr, const MorphologyResult& m,
                   const DetectionResult& det);

cv::Mat makeGrid(const cv::Mat& bgr, const MorphologyResult& m,
                 const DetectionResult& det, int cell_width = 480);

}  // namespace vision
