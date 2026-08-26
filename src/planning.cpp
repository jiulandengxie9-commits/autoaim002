#include "planning.hpp"

#include <cmath>

namespace planning {

BallisticSolution solveBallistic(double v0, double d, double h, double g) {
  BallisticSolution s;
  if (v0 <= 0.0 || d <= 0.0) return s;

  // p = tan(pitch): h = d*tan - g*d^2/(2 v0^2) * (1+tan^2)
  // => (g d^2 / (2 v0^2)) * tan^2 - d * tan + (h + g d^2/(2 v0^2)) = 0
  const double a = g * d * d / (2.0 * v0 * v0);
  const double b = -d;
  const double c = a + h;
  const double delta = b * b - 4.0 * a * c;
  if (delta < 0.0) return s;  // unreachable

  const double tan_p1 = (-b + std::sqrt(delta)) / (2.0 * a);
  const double tan_p2 = (-b - std::sqrt(delta)) / (2.0 * a);
  const double p1 = std::atan(tan_p1);
  const double p2 = std::atan(tan_p2);
  const double t1 = d / (v0 * std::cos(p1));
  const double t2 = d / (v0 * std::cos(p2));

  s.solvable = true;
  if (t1 < t2) {
    s.pitch_rad = p1;
    s.fly_time_s = t1;
  } else {
    s.pitch_rad = p2;
    s.fly_time_s = t2;
  }
  return s;
}

cv::Vec3d predictAhead(const cv::Vec3d& pos, const cv::Vec3d& vel,
                       const cv::Vec3d& acc, double t) {
  return pos + vel * t + 0.5 * acc * (t * t);
}

AimSolution planAimPoint(const cv::Vec3d& pos, const cv::Vec3d& vel,
                         const cv::Vec3d& acc, double bullet_speed,
                         int max_iter, double tol_s) {
  AimSolution out;
  if (bullet_speed <= 0.0) return out;

  // Initial guess: aim at current position, flight time = distance/v0.
  double fly_time = cv::norm(pos) / bullet_speed;

  for (int i = 0; i < max_iter; ++i) {
    const cv::Vec3d predicted = predictAhead(pos, vel, acc, fly_time);
    const double d = std::hypot(predicted[0], predicted[2]);  // horizontal
    const double h = predicted[1];                           // height (world +Y up)
    const BallisticSolution sol = solveBallistic(bullet_speed, d, h);

    if (!sol.solvable) return out;
    if (std::abs(sol.fly_time_s - fly_time) < tol_s) {
      out.valid = true;
      out.aim_point = predicted;
      out.fly_time_s = sol.fly_time_s;
      out.pitch_rad = sol.pitch_rad;
      out.distance_m = cv::norm(predicted);
      return out;
    }
    fly_time = sol.fly_time_s;
  }

  // Accept the last iterate even if not fully converged.
  const cv::Vec3d predicted = predictAhead(pos, vel, acc, fly_time);
  const double d = std::hypot(predicted[0], predicted[2]);
  const double h = predicted[1];
  const BallisticSolution sol = solveBallistic(bullet_speed, d, h);
  if (!sol.solvable) return out;
  out.valid = true;
  out.aim_point = predicted;
  out.fly_time_s = sol.fly_time_s;
  out.pitch_rad = sol.pitch_rad;
  out.distance_m = cv::norm(predicted);
  return out;
}

}  // namespace planning
