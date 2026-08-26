#pragma once

#include <opencv2/core.hpp>

namespace planning {

// Ballistic trajectory solution (no air drag, flat ground model), ported from
// tongji tools/trajectory.cpp. Given muzzle speed v0 (m/s), horizontal distance
// d (m) and height h (m, positive up), solves the quadratic for the two
// possible launch pitches and keeps the flat/short-time one.
struct BallisticSolution {
  bool solvable = false;
  double pitch_rad = 0.0;   // launch elevation above horizontal
  double fly_time_s = 0.0;  // time of flight
};

BallisticSolution solveBallistic(double v0, double d, double h,
                                 double g = 9.81);

// Predict target position t seconds ahead given position/velocity/acceleration
// (constant-acceleration model, same as KalmanFilter3D state).
cv::Vec3d predictAhead(const cv::Vec3d& pos, const cv::Vec3d& vel,
                       const cv::Vec3d& acc, double t);

// Aim-point used by the planner: predicted target + ballistic lead.
struct AimSolution {
  bool valid = false;
  cv::Vec3d aim_point;      // predicted aim point in gimbal frame (m)
  double fly_time_s = 0.0;  // projectile time of flight
  double pitch_rad = 0.0;   // launch elevation
  double distance_m = 0.0;  // straight-line distance to aim point
};

// Iterate prediction <-> ballistic flight time (like tongji Aimer::aim's
// 10-iteration loop). Returns the converged aim solution.
AimSolution planAimPoint(const cv::Vec3d& pos, const cv::Vec3d& vel,
                         const cv::Vec3d& acc, double bullet_speed,
                         int max_iter = 10, double tol_s = 0.001);

}  // namespace planning
