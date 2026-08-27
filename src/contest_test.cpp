// 校内赛自瞄测试工具
//
// 按《RoboMaster 机甲大师校内赛规则手册》装甲板自瞄任务（60 分）的 6 个工况
// 驱动比赛模拟器自动跑测试：
//   1. 原地旋转 低速档 (|ω|=3~5 rad/s)   满分 6
//   2. 原地旋转 高速档 (|ω|=8~10 rad/s)  满分 8
//   3. 横向平移 低速档 (v=0.4~0.8 m/s)   满分 8
//   4. 横向平移 高速档 (v=1.2~1.8 m/s)   满分 10
//   5. 组合运动 低速档                   满分 12
//   6. 组合运动 高速档                   满分 16
//
// 每个工况：设置靶车运动 → 稳定 3 s → 进入计分窗口（最多 30 s）自动读帧、
// 检测装甲板、PnP 解算、云台瞄准并开火，最多发射 50 发。命中数用 SDK
// getLatestArmorHit() 读取（权威物理判定，按 event_id 去重），得分按规则公式
// 满分 × hits/50 计算。
//
// 用法：
//   ./build/autoaim002_test [--ipc-dir DIR] [--calibration PATH] [--rounds 50]
//                           [--only N] [--no-fire]
// 需先启动模拟器：
//   /home/xqy/autoaim/linux-x86_64/daedalus-contest.sh start --scene shooting-range

#include <daedalus_sim_sdk/contest_client.hpp>
#include <daedalus_sim_sdk/scene_control_client.hpp>
#include <daedalus_sim_sdk/talos_metadata_reader.hpp>

#include "detector.hpp"
#include "planning.hpp"
#include "vision_processing.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <optional>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace daedalus::sim::sdk::v1;

namespace {

struct Options {
  std::string ipc_directory;
  std::string calibration_path =
      "/home/xqy/autoaim/linux-x86_64/camera-calibration.json";
  std::uint32_t rounds = 50;
  int only = -1;      // 只跑某个工况（0..5），-1 跑全部
  bool fire = true;
  bool show_window = true;
  bool set_motion = true;  // 跳过靶车运动设置（用于静止靶车验证）
  bool use_kalman = false;
  std::string window_title = "AutoAim Test";
  double bullet_speed = 25.0;  // 比赛版弹丸初速 25 m/s
  double fire_delay = 0.0;     // 开火延迟（命令到实际发射），秒，可调
  detect::Backend detector_backend = detect::Backend::NeuralNetwork;  // 默认神经网络
  std::string nn_model_path = "/home/xqy/桌面/tongji/assets/yolo11.xml";
  vision::LightColor color = vision::LightColor::Red;  // 模拟器敌方装甲板为红色灯条
  double armor_width = 0.135;
  double armor_height = 0.056;
  vision::DetectionParams det;
};

struct Condition {
  const char* name;
  RangeMotionMode mode;
  double speed;      // m/s 或 rad/s，视模式
  double span;       // m，平移峰峰行程
  double spin_rps;   // rad/s（旋转）
  double full_marks; // 该工况满分
};

// 6 个工况（速度取档位中值；行程取 1.5~2.0 m 中值 1.75）。
const std::vector<Condition> kConditions = {
    {"spin-low  原地旋转 低速", RangeMotionMode::Spin, 0.0, 0.0, 4.0, 6.0},
    {"spin-high 原地旋转 高速", RangeMotionMode::Spin, 0.0, 0.0, 9.0, 8.0},
    {"linear-low 横向平移 低速", RangeMotionMode::Linear, 0.6, 1.75, 0.0, 8.0},
    {"linear-high 横向平移 高速", RangeMotionMode::Linear, 1.5, 1.75, 0.0, 10.0},
    {"combo-low 组合 低速", RangeMotionMode::LinearAndSpin, 0.6, 1.75, 4.0, 12.0},
    {"combo-high 组合 高速", RangeMotionMode::LinearAndSpin, 1.5, 1.75, 9.0, 16.0},
};

const char* motionName(RangeMotionMode m) {
  switch (m) {
    case RangeMotionMode::Stationary: return "Stationary";
    case RangeMotionMode::Linear: return "Linear";
    case RangeMotionMode::Spin: return "Spin";
    case RangeMotionMode::LinearAndSpin: return "LinearAndSpin";
  }
  return "?";
}

const char* statusName(SceneControlStatus s) {
  switch (s) {
    case SceneControlStatus::Ok: return "Ok";
    case SceneControlStatus::InvalidRequest: return "InvalidRequest";
    case SceneControlStatus::Unsupported: return "Unsupported";
    case SceneControlStatus::NotReady: return "NotReady";
    case SceneControlStatus::InternalError: return "InternalError";
  }
  return "Unknown";
}

void printUsage(const char* argv0) {
  std::cout
      << "Usage: " << argv0 << " [options]\n"
      << "Run the 6-condition armor self-aim test on the Daedalus contest\n"
      << "simulator per the RM school-competition rulebook (60 pts armor task).\n\n"
      << "Options:\n"
      << "  --ipc-dir PATH      simulator IPC directory\n"
      << "                      (default: $XDG_RUNTIME_DIR or /tmp +\n"
      << "                       /daedalus-contest-<uid>)\n"
      << "  --calibration PATH  camera-calibration.json path (default: the\n"
      << "                      release's camera-calibration.json)\n"
      << "  --rounds N          rounds per condition (default 50)\n"
      << "  --only N            run only condition N (0..5), default all\n"
      << "  --no-fire           aim only, do not fire\n"
      << "  --no-motion         do not set target motion (stationary target)\n"
      << "  --kalman            enable Kalman tracking (off by default)\n"
      << "  --no-kalman         use raw PnP positions instead of Kalman tracking\n"
      << "  --no-window         disable the OpenCV visualization window\n"
      << "  --window NAME       visualization window title (default AutoAim Test)\n"
      << "  --armor-width M     armor width in meters (default 0.135)\n"
      << "  --armor-height M    armor height in meters (default 0.056)\n"
      << "  --color NAME        light bar color: red | blue (default red,\n"
      << "                      the simulator's enemy armor light bars)\n"
      << "  --bullet-speed M/S  projectile muzzle speed in m/s (default 25.0;\n"
      << "                      gravity drop is compensated with this)\n"
      << "  --fire-delay SEC    fixed command-to-shot latency in seconds\n"
      << "                      (gimbal response + firing delay) used to lead\n"
      << "                      the target; default 0 (default 0.0)\n"
      << "  --detector NAME     detector backend: traditional | nn (default nn,\n"
      << "                      the YOLO neural network via OpenVINO)\n"
      << "  --nn-model PATH     OpenVINO/ONNX model for --detector nn (default\n"
      << "                      tongji's yolo11.xml)\n"
      << "  -h, --help          show this help and exit\n";
}

double readJsonDouble(const std::string& text, const std::string& key) {
  const std::string needle = "\"" + key + "\"";
  std::size_t pos = text.find(needle);
  if (pos == std::string::npos) return 0.0;
  pos = text.find(':', pos + needle.size());
  if (pos == std::string::npos) return 0.0;
  return std::strtod(text.c_str() + pos + 1, nullptr);
}

bool readJsonArray(const std::string& text, const std::string& key, double* out,
                   std::size_t count) {
  const std::string needle = "\"" + key + "\"";
  std::size_t pos = text.find(needle);
  if (pos == std::string::npos) return false;
  pos = text.find('[', pos + needle.size());
  if (pos == std::string::npos) return false;
  ++pos;
  for (std::size_t i = 0; i < count; ++i) {
    while (pos < text.size() &&
           (text[pos] == ' ' || text[pos] == '\t' || text[pos] == '\n' ||
            text[pos] == ',' || text[pos] == '\r')) {
      ++pos;
    }
    out[i] = std::strtod(text.c_str() + pos, nullptr);
    while (pos < text.size() && text[pos] != ',' && text[pos] != ']') ++pos;
    if (pos >= text.size()) return false;
  }
  return true;
}

bool loadCalibration(const std::string& path, vision::CameraIntrinsics& k,
                     vision::CameraExtrinsics& e) {
  std::ifstream in(path);
  if (!in) return false;
  const std::string text((std::istreambuf_iterator<char>(in)),
                         std::istreambuf_iterator<char>());
  k.fx = readJsonDouble(text, "fx");
  k.fy = readJsonDouble(text, "fy");
  k.cx = readJsonDouble(text, "cx");
  k.cy = readJsonDouble(text, "cy");
  if (k.fx <= 0.0 || k.fy <= 0.0) return false;
  readJsonArray(text, "distortion", k.distortion, 5);
  readJsonArray(text, "translation_m", e.translation_m, 3);
  readJsonArray(text, "quaternion_xyzw", e.quaternion_xyzw, 4);
  return true;
}

cv::Mat frameToBgr(const TcpImageFrame& frame) {
  const tcp_image::FrameHeader& h = frame.header;
  if (h.format == tcp_image::PixelFormat::Rgba32) {
    cv::Mat rgba(static_cast<int>(h.height), static_cast<int>(h.width), CV_8UC4,
                 const_cast<std::uint8_t*>(frame.payload.data()));
    cv::Mat bgr;
    cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
    return bgr;
  }
  if (h.format == tcp_image::PixelFormat::Rgb24) {
    cv::Mat rgb(static_cast<int>(h.height), static_cast<int>(h.width), CV_8UC3,
                const_cast<std::uint8_t*>(frame.payload.data()));
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    return bgr;
  }
  return cv::Mat();
}

Options parseArgs(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next = [&](const char* f) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << f << "\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "--ipc-dir") {
      o.ipc_directory = next("--ipc-dir");
    } else if (arg == "--calibration") {
      o.calibration_path = next("--calibration");
    } else if (arg == "--rounds") {
      o.rounds = static_cast<std::uint32_t>(std::strtoul(next("--rounds").c_str(), nullptr, 10));
      if (o.rounds == 0) {
        std::cerr << "--rounds must be > 0\n";
        std::exit(2);
      }
    } else if (arg == "--only") {
      o.only = std::atoi(next("--only").c_str());
    } else if (arg == "--no-fire") {
      o.fire = false;
    } else if (arg == "--no-motion") {
      o.set_motion = false;
    } else if (arg == "--kalman") {
      o.use_kalman = true;
    } else if (arg == "--no-kalman") {
      o.use_kalman = false;
    } else if (arg == "--no-window") {
      o.show_window = false;
    } else if (arg == "--window") {
      o.window_title = next("--window");
    } else if (arg == "--armor-width") {
      o.armor_width = std::strtod(next("--armor-width").c_str(), nullptr);
    } else if (arg == "--armor-height") {
      o.armor_height = std::strtod(next("--armor-height").c_str(), nullptr);
    } else if (arg == "--color") {
      const std::string name = next("--color");
      if (name == "red") {
        o.color = vision::LightColor::Red;
      } else if (name == "blue") {
        o.color = vision::LightColor::Blue;
      } else {
        std::cerr << "--color must be red or blue\n";
        std::exit(2);
      }
    } else if (arg == "--bullet-speed") {
      o.bullet_speed = std::strtod(next("--bullet-speed").c_str(), nullptr);
      if (o.bullet_speed <= 0.0) {
        std::cerr << "--bullet-speed must be positive\n";
        std::exit(2);
      }
    } else if (arg == "--fire-delay") {
      o.fire_delay = std::strtod(next("--fire-delay").c_str(), nullptr);
      if (o.fire_delay < 0.0) {
        std::cerr << "--fire-delay must be >= 0\n";
        std::exit(2);
      }
    } else if (arg == "--detector") {
      const std::string name = next("--detector");
      if (name == "traditional") {
        o.detector_backend = detect::Backend::Traditional;
      } else if (name == "nn") {
        o.detector_backend = detect::Backend::NeuralNetwork;
      } else {
        std::cerr << "--detector must be traditional or nn\n";
        std::exit(2);
      }
    } else if (arg == "--nn-model") {
      o.nn_model_path = next("--nn-model");
    } else if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      std::exit(0);
    } else {
      std::cerr << "unknown option: " << arg << "\n";
      printUsage(argv[0]);
      std::exit(2);
    }
  }
  if (o.ipc_directory.empty()) {
    const char* runtime = std::getenv("XDG_RUNTIME_DIR");
    std::ostringstream ss;
    if (runtime != nullptr && *runtime != '\0') {
      ss << runtime;
    } else {
      ss << "/tmp";
    }
    ss << "/daedalus-contest-" << ::getuid();
    o.ipc_directory = ss.str();
  }
  return o;
}

struct ConditionStats {
  std::uint32_t rounds_fired = 0;
  std::uint64_t locked_frames = 0;  // 检测到装甲板且发出瞄准命令的帧数
  std::uint64_t total_frames = 0;
  std::uint64_t hits = 0;  // 真实命中数（来自 getLatestArmorHit，按 event_id 去重）
  double elapsed_s = 0.0;
};

}  // namespace

// Read the exposure-synced gimbal world (odom) pose for a frame sequence.
// Needed to filter/plan in the world frame so gimbal rotation does not corrupt
// the target velocity estimate (the cause of gimbal oscillation).
vision::GimbalWorldPose readGimbalWorldPose(
    const TalosMetadataMapping& mapping, std::uint64_t frame_seq) {
  vision::GimbalWorldPose pose;
  auto reader_res = mapping.reader();
  if (!reader_res.ok()) return pose;
  const TalosMetadataReader& reader = *reader_res.value;
  auto exp = reader.readExposureStateForFrame(frame_seq);
  if (!exp.ok() || !exp.value) return pose;
  const ExposureState& es = *exp.value;
  pose.valid = true;
  for (int i = 0; i < 3; ++i) {
    pose.position_m[i] = es.gimbal_position_world[i];
    pose.camera_position_m[i] = es.camera_position_world[i];
    pose.quaternion_wxyz[i] = es.gimbal_quaternion_world_wxyz[i];
    pose.camera_quaternion_wxyz[i] = es.camera_quaternion_world_wxyz[i];
  }
  pose.quaternion_wxyz[3] = es.gimbal_quaternion_world_wxyz[3];
  pose.camera_quaternion_wxyz[3] = es.camera_quaternion_world_wxyz[3];
  pose.camera_valid = (es.state_flags & kExposureStateHasCameraWorldPose) != 0;
  return pose;
}

int main(int argc, char** argv) {
  const Options o = parseArgs(argc, argv);
  bool show_window = o.show_window;  // 可在运行时降级为 false

  std::cout << "Daedalus SDK " << kSdkVersion << "  IPC dir: " << o.ipc_directory
            << "\n";

  // ContestClient: 读帧 + 云台命令（瞄准/开火）。
  ContestClientOptions copts;
  copts.ipc_directory = o.ipc_directory;
  ContestClient sim(copts);
  if (!sim.connect()) {
    std::cerr << "could not connect to the simulator in " << o.ipc_directory
              << ".\nStart it first, e.g.:\n"
              << "  /home/xqy/autoaim/linux-x86_64/daedalus-contest.sh start\n";
    return 1;
  }
  std::cout << "connected to simulator.\n";

  // SceneControlClient: 设置靶车运动（与 ContestClient 共用场景控制端口 5603）。
  // 协议要求先以固定 session_id 创建会话，之后 setRangeTargetMotion 才能通过
  // session 校验。
  SceneControlOptions sc_opts;
  sc_opts.session_id = "autoaim002-test";
  SceneControlClient scene(sc_opts);
  {
    auto ses = scene.createSession();
    if (ses.ok() && ses.value &&
        ses.value->status == SceneControlStatus::Ok) {
      std::cout << "scene control session: id=" << ses.value->session_id
                << " status=" << statusName(ses.value->status) << "\n";
    } else {
      std::cerr << "scene control session failed: "
                << (ses.ok() ? statusName(ses.value->status)
                             : ses.status.message.c_str())
                << "\n";
    }
  }

  auto scene_res = sim.selectScene(ContestScene::ShootingRange);
  if (scene_res.ok() && scene_res.value) {
    std::cout << "scene shooting-range selected: status="
              << statusName(scene_res.value->status) << "\n";
  } else {
    std::cerr << "scene selection failed: " << scene_res.status.message << "\n";
    return 1;
  }

  vision::CameraIntrinsics intrinsics;
  vision::CameraExtrinsics extrinsics;
  if (!loadCalibration(o.calibration_path, intrinsics, extrinsics)) {
    std::cerr << "calibration: could not load " << o.calibration_path
              << "; PnP disabled\n";
  } else {
    std::cout << "calibration: fx=" << intrinsics.fx << " fy=" << intrinsics.fy
              << " cx=" << intrinsics.cx << " cy=" << intrinsics.cy
              << " (from " << o.calibration_path << ")\n";
  }

  // 元数据映射：读每帧云台世界位姿，用于世界系滤波/规划。
  TalosMetadataMapping metadata_mapping;
  const std::string meta_path =
      o.ipc_directory + "/" + std::string(kMetaFileName);
  {
    const ClientStatus ms = metadata_mapping.open(meta_path);
    if (!ms.ok()) {
      std::cerr << "metadata: could not open " << meta_path << " ("
                << ms.message << ")\n";
    }
  }
  const double camera_tilt_deg =
      2.0 * std::acos(std::clamp(extrinsics.quaternion_xyzw[3], -1.0, 1.0)) *
      180.0 / CV_PI;

  // 检测器：传统视觉或神经网络（--detector 切换，NN 加载失败自动回退传统）。
  vision::MorphologyParams det_morph;
  det_morph.kernel_size = 3;
  det_morph.rb_threshold = 30;
  std::unique_ptr<detect::IDetector> detector = detect::makeDetector(
      o.detector_backend, o.nn_model_path, o.color, det_morph, o.det);
  std::cout << "detector: " << detector->name() << "\n";

  std::cout << "\nrounds/condition=" << o.rounds
            << " fire=" << (o.fire ? "on" : "off")
            << " color=" << (o.color == vision::LightColor::Blue ? "blue" : "red")
            << " bullet_speed=" << o.bullet_speed << " m/s"
            << " fire_delay=" << o.fire_delay << " s"
            << " kalman=" << (o.use_kalman ? "on" : "off")
            << " armor=" << o.armor_width << "x" << o.armor_height << " m"
            << "\n\n";

  double total_score = 0.0;
  const auto t_total = std::chrono::steady_clock::now();

  for (int ci = 0; ci < static_cast<int>(kConditions.size()); ++ci) {
    if (o.only >= 0 && ci != o.only) continue;
    const Condition& c = kConditions[ci];

    std::cout << "\n========== 工况 " << (ci + 1) << ": " << c.name
              << "  满分=" << std::fixed << std::setprecision(0) << c.full_marks
              << " ==========\n";

    // 1) 设置靶车运动。
    RangeTargetMotion motion;
    motion.mode = c.mode;
    motion.linear_speed_mps = static_cast<float>(c.speed);
    motion.linear_span_m = static_cast<float>(c.span);
    motion.spin_deg_s = static_cast<float>(c.spin_rps * 180.0 / CV_PI);
    if (!o.set_motion) {
      motion.mode = RangeMotionMode::Stationary;
      motion.linear_speed_mps = 0.0F;
      motion.linear_span_m = 0.0F;
      motion.spin_deg_s = 0.0F;
    }
    {
      auto mr = scene.setRangeTargetMotion(motion);
      std::cout << "  set motion: mode=" << motionName(motion.mode)
                << " v=" << motion.linear_speed_mps << " m/s span="
                << motion.linear_span_m << " m spin="
                << motion.spin_deg_s * CV_PI / 180.0 << " rad/s -> "
                << (mr.ok() && mr.value ? statusName(mr.value->status)
                                        : mr.status.message.c_str());
      if (mr.ok() && mr.value) {
        std::cout << " applied_seq=" << mr.value->applied_frame_seq;
      }
      std::cout << "\n";
    }

    // 2) 稳定 3 s。
    std::cout << "  stabilizing 3 s...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // 3) 计分窗口：最多 30 s，发射最多 rounds 发。
    ConditionStats st;
    std::uint64_t last_seq = 0;
    int lock_streak = 0;               // 连续锁定帧数（开火前需稳定）
    const int lock_warmup_frames = 3;  // 云台收敛后再开火（降低防卡住）
    std::uint32_t hit_count_initial = 0;
    {
      auto hit0 = sim.getLatestArmorHit();
      if (hit0.ok() && hit0.value && hit0.value->has_hit) {
        hit_count_initial = hit0.value->accurate_count;
      }
    }
    // 目标运动估计：世界系 Kalman（pos/vel/acc），抑制 NN 检测抖动。
    vision::KalmanFilter3D track_kf;
    track_kf.process_pos_sigma = 0.005;
    track_kf.process_vel_sigma = 0.05;
    track_kf.process_acc_sigma = 0.2;
    track_kf.measurement_sigma = 0.15;
    bool kf_inited = false;
    cv::Vec3d track_pos, track_vel(0, 0, 0), track_acc(0, 0, 0);
    std::uint64_t prev_ts = 0;
    cv::Vec2d smooth_aim(0.0, 90.0);  // 云台命令低通，抑制检测抖动导致乱动
    const auto t0 = std::chrono::steady_clock::now();
    double fire_cooldown_s = 0.06;  // ~16 发/s（模拟器射击冷却）
    auto last_fire = t0;
    int stable_frames = 0;
    constexpr int kStableFramesRequired = 5;
    constexpr double kAimErrorThresholdDeg = 0.8;
    constexpr double kGimbalVelocityThresholdDegS = 6.0;
    constexpr int kLostHoldFrames = 5;
    constexpr double kAssociationGateM = 1.2;
    constexpr double kCenterYawDeg = 0.0;
    constexpr double kCenterPitchDeg = 63.0;
    std::optional<cv::Vec3d> last_target_world;
    cv::Vec2d last_safe_aim(kCenterYawDeg, kCenterPitchDeg);
    bool has_last_safe_aim = false;
    int lost_frames = 0;
    bool window_open = true;

    while (window_open) {
      auto frame_res = sim.nextFrame(last_seq);
      if (!frame_res.ok() || !frame_res.value) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      const ContestFrame& cf = *frame_res.value;
      last_seq = cf.image.header.source_sequence;
      ++st.total_frames;

      // 轮询权威命中：getLatestArmorHit.accurate_count 是累计有效命中数，
      // 每工况命中数 = 工况结束时累计值 - 工况开始时累计值。
      {
        auto hit = sim.getLatestArmorHit();
        if (hit.ok() && hit.value && hit.value->has_hit) {
          st.hits = hit.value->accurate_count > hit_count_initial
                        ? hit.value->accurate_count - hit_count_initial
                        : 0;
        }
      }

      const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t0).count();
      if (elapsed > 30.0 || st.rounds_fired >= o.rounds) {
        window_open = false;
        st.elapsed_s = elapsed;
        break;
      }

      cv::Mat image = frameToBgr(cf.image);
      if (image.empty()) continue;

      // 用所选检测器识别装甲板（传统视觉或神经网络，--detector 切换）。
      vision::MorphologyResult morph;  // 空：形态学网格不用于测试工具
      vision::DetectionResult det = detector->detect(image);
      vision::normalizeArmorVertices(det);

      const vision::GimbalWorldPose wp =
          readGimbalWorldPose(metadata_mapping, cf.image.header.source_sequence);

      // Select the armor associated with the previous world-frame target.
      // A nearest-depth choice switches between visible armor plates as the
      // target rotates, which makes the gimbal oscillate.
      vision::TargetPose best;
      cv::Vec3d best_world;
      bool have_best = false;
      if (!det.armors.empty() && intrinsics.fx > 0.0) {
        double best_score = 1e18;
        for (const auto& armor : det.armors) {
          vision::TargetPose p =
              vision::solveArmorPose(armor, intrinsics, extrinsics,
                                     o.armor_width, o.armor_height);
          if (!p.valid) continue;
          const cv::Vec3d candidate_world =
              (wp.valid && wp.camera_valid)
                  ? vision::cameraToWorld(p.t_cam, wp)
                  : (wp.valid ? vision::gimbalToWorld(p.t_gimbal, wp)
                              : p.t_gimbal);
          const double score = last_target_world
                                   ? cv::norm(candidate_world - *last_target_world)
                                   : p.distance_m;
          if (score < best_score) {
            best_score = score;
            best = p;
            best_world = candidate_world;
            have_best = true;
          }
        }
        if (last_target_world &&
            best_score > kAssociationGateM) {
          have_best = false;
        }
      }

      // 可视化：无论有无目标都显示检测结果 + HUD + 工况信息。
      const double yaw = cf.gimbal.yaw_deg;
      const double pitch = cf.gimbal.pitch_deg;
      cv::Vec2d aim(0.0, 90.0);
      double aim_dist = 0.0;
      cv::Vec3d aim_world(0.0, 0.0, 0.0);
      bool have_aim_world = false;
      if (have_best) {
        // 用每帧云台世界位姿把目标转到世界（odom）系，再经 Kalman 平滑。
        // 关键：云台转动不影响目标世界坐标，Kalman 速度估计不会被云台运动污染。
        const cv::Vec3d cur_w = best_world;
        last_target_world = cur_w;
        lost_frames = 0;
        const std::uint64_t ts = cf.image.header.capture_timestamp_ns;
        const double dt = prev_ts ? (ts - prev_ts) * 1e-9 : 0.0;
        const double kf_dt = (dt > 0.0 && dt < 0.5) ? dt : 1.0 / 60.0;
        if (!kf_inited || !o.use_kalman) {
          track_kf.init(cur_w, kf_dt);
          kf_inited = true;
        } else {
          track_kf.predict(kf_dt);
          track_kf.update(cur_w);
        }
        track_pos = o.use_kalman ? track_kf.position() : cur_w;
        track_vel = o.use_kalman ? track_kf.velocity() : cv::Vec3d(0, 0, 0);
        if (st.total_frames % 30 == 0) {
          const cv::Vec3d filtered = track_kf.position();
          std::cout << "  tracker: raw=(" << cur_w[0] << ", " << cur_w[1]
                    << ", " << cur_w[2] << ") filtered=(" << filtered[0]
                    << ", " << filtered[1] << ", " << filtered[2]
                    << ") vel=(" << track_kf.velocity()[0] << ", "
                    << track_kf.velocity()[1] << ", "
                    << track_kf.velocity()[2] << ") residual="
                    << cv::norm(cur_w - filtered) << " m\n";
        }
        prev_ts = ts;

        // The tracker position is absolute world (odom) coordinates, while
        // the ballistic solver expects a target relative to the muzzle. Do
        // not pass the world origin directly: world +Y is not muzzle height.
        const cv::Vec3d muzzle_world(wp.position_m[0], wp.position_m[1],
                                     wp.position_m[2]);
        const cv::Vec3d target_relative = track_pos - muzzle_world;

        // Predict in the muzzle-relative world frame, then convert the
        // solution back to absolute world coordinates for camera projection.
        const planning::AimSolution sol =
            planning::planAimPoint(target_relative, track_vel, track_acc,
                                   o.bullet_speed, o.fire_delay);
        if (sol.valid && wp.valid) {
          const cv::Vec3d aim_world_point = sol.aim_point + muzzle_world;
          // Convert the planned world point to the exposure camera frame.
          // The SDK yaw/pitch command convention follows image alignment,
          // rather than the calibration geometry's gimbal coordinate axes.
          const cv::Vec2d relative_aim = vision::cameraRelativeAimAngles(
              vision::gimbalToCamera(
                  vision::worldToGimbal(aim_world_point, wp), extrinsics));
          const double line_of_sight_pitch = std::atan2(
              target_relative[1],
              std::hypot(target_relative[0], target_relative[2]));
          const double gravity_pitch =
              sol.pitch_rad - line_of_sight_pitch;
          aim = cv::Vec2d(yaw + relative_aim[0],
                          pitch + relative_aim[1] +
                              gravity_pitch * 180.0 / CV_PI);
          aim_world = aim_world_point;
          have_aim_world = true;
          aim_dist = sol.distance_m;
        } else {
          // PnP already gives the target in the gimbal frame, so use the
          // same absolute-angle conversion as the planned-target path.
          aim = vision::absoluteAimAngles(best.t_gimbal, yaw, pitch,
                                          camera_tilt_deg);
          if (wp.valid) {
            aim_world = (wp.camera_valid)
                           ? vision::cameraToWorld(best.t_cam, wp)
                           : vision::gimbalToWorld(best.t_gimbal, wp);
            have_aim_world = true;
          }
          aim_dist = best.distance_m;
        }
      }
      if (show_window) {
        try {
          cv::Mat display = vision::drawResult(image, morph, det);
          if (have_best) {
            // 在图像上标出目标装甲板中心（相机系坐标直接投影）。
            const cv::Point2f p = vision::projectPoint(best.t_cam, intrinsics);
            if (p.x > 0 && p.y > 0) {
              cv::line(display, cv::Point(int(p.x) - 10, int(p.y)),
                       cv::Point(int(p.x) + 10, int(p.y)),
                       cv::Scalar(0, 255, 0), 2);
              cv::line(display, cv::Point(int(p.x), int(p.y) - 10),
                       cv::Point(int(p.x), int(p.y) + 10),
                       cv::Scalar(0, 255, 0), 2);
            }
          }
          vision::drawAimHud(display, yaw, pitch, have_best, aim,
                             have_best ? aim_dist : 0.0, have_aim_world,
                             aim_world, true, have_best, aim, aim_dist);
          char info[160];
          std::snprintf(info, sizeof(info),
                        "cond %d/6 %s  fired=%u/%u  locked=%llu",
                        ci + 1, c.name, st.rounds_fired, o.rounds,
                        (unsigned long long)st.locked_frames);
          cv::putText(display, info, cv::Point(12, 104),
                      cv::FONT_HERSHEY_SIMPLEX, 0.55,
                      cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
          cv::imshow(o.window_title, display);
          const int key = cv::waitKey(1);
          if (key == 'q' || key == 27) {
            window_open = false;
            st.elapsed_s = elapsed;
            break;
          }
          if (cv::getWindowProperty(o.window_title, cv::WND_PROP_VISIBLE) < 1) {
            show_window = false;
          }
        } catch (const cv::Exception&) {
          show_window = false;  // 无图形环境，降级为纯命令行
        }
      }

      // 有目标时才瞄准开火。连续锁定 warmup 帧（让云台收敛、目标稳定）后才开火。
      if (!have_best) {
        lock_streak = 0;
        stable_frames = 0;
        ++lost_frames;

        UdpGimbalCommand cmd;
        if (has_last_safe_aim && lost_frames <= kLostHoldFrames) {
          cmd.yaw_deg = static_cast<float>(last_safe_aim[0]);
          cmd.pitch_deg = static_cast<float>(last_safe_aim[1]);
        } else {
          cmd.yaw_deg = static_cast<float>(kCenterYawDeg);
          cmd.pitch_deg = static_cast<float>(kCenterPitchDeg);
          smooth_aim = cv::Vec2d(kCenterYawDeg, kCenterPitchDeg);
          has_last_safe_aim = false;
          last_target_world.reset();
        }
        cmd.distance_m = 0.0F;
        cmd.fire_advice = false;
        (void)sim.sendAim(cmd);
        continue;
      }
      ++st.locked_frames;
      ++lock_streak;

      // Command slew-rate limit bounds a single-frame detection jump.
      constexpr double kMaxCommandStepDeg = 1.5;
      if (have_best) {
        const auto limit_step = [&](double current, double requested) {
          return current + std::clamp(requested - current,
                                      -kMaxCommandStepDeg,
                                      kMaxCommandStepDeg);
        };
        smooth_aim = cv::Vec2d(limit_step(yaw, aim[0]),
                               limit_step(pitch, aim[1]));
        last_safe_aim = smooth_aim;
        has_last_safe_aim = true;
        const double yaw_error = std::abs(smooth_aim[0] - yaw);
        const double pitch_error = std::abs(smooth_aim[1] - pitch);
        const bool gimbal_stable =
            yaw_error <= kAimErrorThresholdDeg &&
            pitch_error <= kAimErrorThresholdDeg &&
            std::abs(cf.gimbal.yaw_velocity_deg_s) <=
                kGimbalVelocityThresholdDegS &&
            std::abs(cf.gimbal.pitch_velocity_deg_s) <=
                kGimbalVelocityThresholdDegS;
        stable_frames = gimbal_stable ? stable_frames + 1 : 0;
      } else {
        smooth_aim = aim;  // 无目标直接用（保持最后角度）
        stable_frames = 0;
      }

      UdpGimbalCommand cmd;
      cmd.yaw_deg = static_cast<float>(smooth_aim[0]);
      cmd.pitch_deg = static_cast<float>(smooth_aim[1]);
      cmd.distance_m = static_cast<float>(best.distance_m);
      const bool fire_ready =
          o.fire && lock_streak >= lock_warmup_frames &&
          stable_frames >= kStableFramesRequired;
      cmd.fire_advice = fire_ready;

      const auto now = std::chrono::steady_clock::now();
      const double since = std::chrono::duration<double>(now - last_fire).count();
      if (!o.fire) {
        (void)sim.sendAim(cmd);
      } else if (fire_ready && since >= fire_cooldown_s) {
        (void)sim.sendAim(cmd);
        last_fire = now;
        ++st.rounds_fired;
      } else {
        // Keep tracking the target without requesting a shot.
        cmd.fire_advice = false;
        (void)sim.sendAim(cmd);
      }
    }

    // 4) 统计输出：规则公式 得分 = 满分 × N命中 / 50。
    const double hit_count = static_cast<double>(st.hits);
    const double score = c.full_marks * std::min(hit_count, static_cast<double>(o.rounds)) / o.rounds;
    total_score += score;
    std::cout << "  [" << c.name << "] frames=" << st.total_frames
              << " locked=" << st.locked_frames
              << " rounds_fired=" << st.rounds_fired << "/" << o.rounds
              << " hits=" << st.hits << " (getLatestArmorHit)"
              << " window=" << std::fixed << std::setprecision(1)
              << st.elapsed_s << " s\n"
              << "  score=" << std::fixed << std::setprecision(2)
              << score << " / " << c.full_marks
              << "  (= 满分 × hits/50)\n";
  }

  const double total_elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t_total).count();
  std::cout << "\n===== 装甲板自瞄任务汇总 ====="
            << "  total_score=" << std::fixed << std::setprecision(2)
            << total_score << " / 60"
            << "  elapsed=" << std::setprecision(1) << total_elapsed << " s\n";

  sim.close();
  return 0;
}
