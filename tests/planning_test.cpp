// planning 功能包单元测试（无需模拟器，直接运行即可）：
//   ./build/planning_test
// 覆盖：弹道解算、位置预测、瞄准点规划。

#include "tasks/planning/planning.hpp"

#include <opencv2/core.hpp>

#include <cmath>
#include <cstdio>
#include <iostream>

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

void testSolveBallistic() {
  // v0=25 m/s, 水平 3 m, 高 0.5 m：应可解、飞行时间正、俯仰角合理。
  const planning::BallisticSolution s =
      planning::solveBallistic(25.0, 3.0, 0.5);
  CHECK(s.solvable);
  if (s.solvable) {
    CHECK(s.fly_time_s > 0.0 && s.fly_time_s < 2.0);
    CHECK(s.pitch_rad > -0.5 && s.pitch_rad < 1.0);
  }

  // 水平发射（h=0）：小角度高抛解，飞行时间约 d/v0 = 0.12 s。
  const planning::BallisticSolution flat =
      planning::solveBallistic(25.0, 3.0, 0.0);
  CHECK(flat.solvable);
  if (flat.solvable) {
    CHECK(std::fabs(flat.fly_time_s - 3.0 / 25.0) < 0.05);
  }

  // 不可达目标（弹速不足以越过）：应不可解。
  const planning::BallisticSolution unreachable =
      planning::solveBallistic(5.0, 50.0, 20.0);
  CHECK(!unreachable.solvable);

  // 非法输入。
  CHECK(!planning::solveBallistic(0.0, 3.0, 0.0).solvable);
  CHECK(!planning::solveBallistic(25.0, 0.0, 0.0).solvable);
}

void testPredictAhead() {
  const cv::Vec3d p = planning::predictAhead(
      cv::Vec3d(1.0, 2.0, 3.0), cv::Vec3d(1.0, 0.0, 0.0),
      cv::Vec3d(0.0, 0.0, 0.0), 2.0);
  CHECK(std::fabs(p[0] - 3.0) < 1e-9);
  CHECK(std::fabs(p[1] - 2.0) < 1e-9);
  CHECK(std::fabs(p[2] - 3.0) < 1e-9);

  // 匀加速：p = p0 + v t + 0.5 a t^2。
  const cv::Vec3d q = planning::predictAhead(
      cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(0.0, 0.0, 0.0),
      cv::Vec3d(2.0, 0.0, 0.0), 3.0);
  CHECK(std::fabs(q[0] - 9.0) < 1e-9);
}

void testPlanAimPoint() {
  // 静止目标在 (0,0,3)，弹速 25 m/s：瞄准点应接近目标本身。
  const planning::AimSolution s = planning::planAimPoint(
      cv::Vec3d(0.0, 0.0, 3.0), cv::Vec3d(0.0, 0.0, 0.0),
      cv::Vec3d(0.0, 0.0, 0.0), 25.0, 0.0);
  CHECK(s.valid);
  if (s.valid) {
    CHECK(std::fabs(s.distance_m - 3.0) < 0.1);
    CHECK(std::fabs(cv::norm(s.aim_point - cv::Vec3d(0.0, 0.0, 3.0))) < 0.1);
  }

  // 横向匀速运动目标：瞄准点应落在目标正前方（x 前移），说明打了提前量。
  const planning::AimSolution moving = planning::planAimPoint(
      cv::Vec3d(0.0, 0.0, 3.0), cv::Vec3d(2.0, 0.0, 0.0),
      cv::Vec3d(0.0, 0.0, 0.0), 25.0, 0.0);
  CHECK(moving.valid);
  if (moving.valid) {
    CHECK(moving.aim_point[0] > 0.2);  // 目标沿 +x 运动，应预判到前方
  }

  // 非法弹速：不可解。
  CHECK(!planning::planAimPoint(cv::Vec3d(0.0, 0.0, 3.0),
                                cv::Vec3d(0.0, 0.0, 0.0),
                                cv::Vec3d(0.0, 0.0, 0.0), 0.0)
             .valid);
}

}  // namespace

int main() {
  testSolveBallistic();
  testPredictAhead();
  testPlanAimPoint();

  if (g_failures == 0) {
    std::cout << "planning_test: all tests passed\n";
    return 0;
  }
  std::cerr << "planning_test: " << g_failures << " check(s) failed\n";
  return 1;
}
