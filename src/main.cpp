#include <daedalus_sim_sdk/contest_client.hpp>
#include <daedalus_sim_sdk/runtime_capabilities.hpp>
#include <daedalus_sim_sdk/talos_metadata_reader.hpp>

#include "detector.hpp"
#include "vision_processing.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

using namespace daedalus::sim::sdk::v1;

namespace {

struct Options {
  std::string ipc_directory;
  std::string scene = "shooting-range";
  std::string window_title = "Daedalus Simulator";
  std::string save_dir;
  std::string calibration_path =
      "/home/xqy/autoaim/linux-x86_64/camera-calibration.json";
  std::uint64_t max_frames = 0;
  bool use_morph = true;
  vision::LightColor color = vision::LightColor::Red;
  int kernel_size = 3;
  int rb_threshold = 30;
  double max_contour_area = 0.0;
  bool enable_open = false;
  int grid_width = 480;
  double armor_width = 0.135;
  double armor_height = 0.056;
  bool use_kalman = true;
  bool send_commands = false;
  double kf_pos_sigma = 0.01;
  double kf_vel_sigma = 0.1;
  double kf_acc_sigma = 0.2;
  double kf_meas_sigma = 0.03;
  bool det_debug = true;
  detect::Backend detector_backend = detect::Backend::NeuralNetwork;  // 默认神经网络
  std::string nn_model_path =
      "/home/xqy/桌面/tongji/assets/yolo11.xml";
  vision::DetectionParams det;
};

// Read a double value that follows the JSON key "\"name\"".
std::optional<double> readJsonDouble(const std::string& text,
                                     const std::string& key) {
  const std::string needle = "\"" + key + "\"";
  std::size_t pos = text.find(needle);
  if (pos == std::string::npos) return std::nullopt;
  pos = text.find(':', pos + needle.size());
  if (pos == std::string::npos) return std::nullopt;
  ++pos;
  while (pos < text.size() &&
         (text[pos] == ' ' || text[pos] == '\t' || text[pos] == '\n')) {
    ++pos;
  }
  return std::strtod(text.c_str() + pos, nullptr);
}

// Read a JSON number array that follows the key "\"name\"".
bool readJsonArray(const std::string& text, const std::string& key,
                   double* out, std::size_t count) {
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

// Load the fixed camera calibration shipped with the simulator release.
bool loadCalibration(const std::string& path, vision::CameraIntrinsics& k,
                     vision::CameraExtrinsics& e) {
  std::ifstream in(path);
  if (!in) return false;
  const std::string text((std::istreambuf_iterator<char>(in)),
                         std::istreambuf_iterator<char>());

  const auto fx = readJsonDouble(text, "fx");
  const auto fy = readJsonDouble(text, "fy");
  const auto cx = readJsonDouble(text, "cx");
  const auto cy = readJsonDouble(text, "cy");
  if (!fx || !fy || !cx || !cy) return false;

  k.fx = *fx;
  k.fy = *fy;
  k.cx = *cx;
  k.cy = *cy;
  readJsonArray(text, "distortion", k.distortion, 5);
  readJsonArray(text, "translation_m", e.translation_m, 3);
  readJsonArray(text, "quaternion_xyzw", e.quaternion_xyzw, 4);
  return true;
}

void printUsage(const char* argv0) {
  std::cout
      << "Usage: " << argv0 << " [options]\n"
      << "Read camera frames and simulator status from the Daedalus contest\n"
      << "simulator through its C++ SDK, show the image in an OpenCV window and\n"
      << "print per-frame information to the terminal.\n"
      << "When --morph is active (default) every frame is also processed by the\n"
      << "morphological filter pipeline (erode, dilate, open, close) defined in\n"
      << "src/vision_processing.cpp and the intermediate stages are shown.\n\n"
      << "Options:\n"
      << "  --ipc-dir PATH    IPC directory used by the simulator.\n"
      << "                    (default: $XDG_RUNTIME_DIR or /tmp,\n"
      << "                     appended with /daedalus-contest-<uid>)\n"
      << "  --scene NAME      scene to select: shooting-range | energy | large-energy\n"
      << "                    (default: shooting-range)\n"
      << "  --max-frames N    exit after receiving N frames\n"
      << "                    (default: run until the window is closed)\n"
      << "  --save-dir PATH   also write the received frame (and morphology PNGs\n"
      << "                    when --morph) into PATH\n"
      << "  --window NAME     title of the display window\n"
      << "  --morph           enable morphological filtering (default)\n"
      << "  --no-morph        disable morphological filtering\n"
      << "  --kernel N        structuring element size for morphology (default 5)\n"
      << "  --rb-threshold N  R-B/B-R threshold for the mask (default 30)\n"
      << "  --color NAME      light bar color: red | blue (default red)\n"
      << "  --detector NAME   detector backend: traditional | nn (default nn,\n"
      << "                    the YOLO neural network via OpenVINO)\n"
      << "  --nn-model PATH   OpenVINO/ONNX model for --detector nn (default\n"
      << "                    tongji's yolo11.xml)\n"
      << "  --open            enable the opening (MORPH_OPEN) step (default)\n"
       << "  --no-open         disable the opening step\n"
       << "  --max-area N      drop contours whose area exceeds N pixels\n"
      << "                    (default 0 = keep all)\n"
      << "  --grid-width N    width of each cell in the morphology grid\n"
      << "                    (default 480)\n"
      << "  --min-light-area N   min contour area to be a light bar (default 100)\n"
      << "  --min-solidity N     min light contour solidity (default 0.3)\n"
      << "  --max-light-ratio N  max light bar width/length ratio (default 0.8)\n"
      << "  --max-angle-diff N   max angle diff between two lights, deg (default 15)\n"
      << "  --max-height-diff N  max length diff ratio of two lights (default 0.2)\n"
      << "  --max-y-diff N       max y-diff ratio of two light centers (default 0.5)\n"
      << "  --min-x-diff N       min x-diff ratio of two light centers (default 0.5)\n"
      << "  --max-armor-ratio N  max armor distance/height ratio (default 2.5)\n"
       << "  --min-armor-ratio N  min armor distance/height ratio (default 1.0)\n"
       << "  --det-debug         print every light-pair candidate armor\n"
       << "                    (constraints) each frame\n"
       << "  --calibration PATH  camera-calibration.json path for PnP\n"
       << "                    (default: /home/xqy/autoaim/linux-x86_64/\n"
       << "                     camera-calibration.json)\n"
       << "  --armor-width M    real armor plate width in meters (default 0.135)\n"
       << "  --armor-height M   real armor plate height in meters (default 0.056)\n"
       << "  --kalman           enable Kalman smoothing of the PnP position\n"
       << "                    (default)\n"
       << "  --no-kalman        disable Kalman smoothing\n"
      << "  --control          send calculated absolute yaw/pitch to the simulator\n"
      << "                    (default: display and log only)\n"
       << "  --kf-pos-sigma N   process noise std-dev for position (m, default 0.01)\n"
       << "  --kf-vel-sigma N   process noise std-dev for velocity (m/s, default 0.1)\n"
       << "  --kf-acc-sigma N   process noise std-dev for accel (m/s^2, default 0.2)\n"
       << "  --kf-meas-sigma N  measurement noise std-dev (m, default 0.03)\n"
       << "  -h, --help        show this help and exit\n";
}

Options parseArgs(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next = [&](const char* flag) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << flag << "\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "--ipc-dir") {
      o.ipc_directory = next("--ipc-dir");
    } else if (arg == "--scene") {
      o.scene = next("--scene");
    } else if (arg == "--max-frames") {
      o.max_frames =
          static_cast<std::uint64_t>(std::strtoull(next("--max-frames").c_str(), nullptr, 10));
    } else if (arg == "--save-dir") {
      o.save_dir = next("--save-dir");
    } else if (arg == "--window") {
      o.window_title = next("--window");
    } else if (arg == "--morph") {
      o.use_morph = true;
    } else if (arg == "--no-morph") {
      o.use_morph = false;
    } else if (arg == "--kernel") {
      o.kernel_size = std::atoi(next("--kernel").c_str());
      if (o.kernel_size < 1 || (o.kernel_size % 2) == 0) {
        std::cerr << "--kernel must be a positive odd integer\n";
        std::exit(2);
      }
    } else if (arg == "--rb-threshold") {
      o.rb_threshold = std::atoi(next("--rb-threshold").c_str());
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
    } else if (arg == "--open") {
      o.enable_open = true;
    } else if (arg == "--no-open") {
      o.enable_open = false;
    } else if (arg == "--max-area") {
      o.max_contour_area = std::strtod(next("--max-area").c_str(), nullptr);
    } else if (arg == "--grid-width") {
      o.grid_width = std::atoi(next("--grid-width").c_str());
      if (o.grid_width < 160) {
        std::cerr << "--grid-width must be at least 160\n";
        std::exit(2);
      }
    } else if (arg == "--min-light-area") {
      o.det.min_light_area = std::strtod(next("--min-light-area").c_str(), nullptr);
    } else if (arg == "--min-solidity") {
      o.det.min_contour_solidity = std::strtod(next("--min-solidity").c_str(), nullptr);
    } else if (arg == "--max-light-ratio") {
      o.det.max_light_ratio = std::strtod(next("--max-light-ratio").c_str(), nullptr);
    } else if (arg == "--max-angle-diff") {
      o.det.max_angle_diff = std::strtod(next("--max-angle-diff").c_str(), nullptr);
    } else if (arg == "--max-height-diff") {
      o.det.max_height_diff_ratio = std::strtod(next("--max-height-diff").c_str(), nullptr);
    } else if (arg == "--max-y-diff") {
      o.det.max_y_diff_ratio = std::strtod(next("--max-y-diff").c_str(), nullptr);
    } else if (arg == "--min-x-diff") {
      o.det.min_x_diff_ratio = std::strtod(next("--min-x-diff").c_str(), nullptr);
    } else if (arg == "--max-armor-ratio") {
      o.det.max_armor_ratio = std::strtod(next("--max-armor-ratio").c_str(), nullptr);
    } else if (arg == "--min-armor-ratio") {
      o.det.min_armor_ratio = std::strtod(next("--min-armor-ratio").c_str(), nullptr);
    } else if (arg == "--det-debug") {
      o.det_debug = true;
    } else if (arg == "--calibration") {
      o.calibration_path = next("--calibration");
    } else if (arg == "--armor-width") {
      o.armor_width = std::strtod(next("--armor-width").c_str(), nullptr);
      if (o.armor_width <= 0.0) {
        std::cerr << "--armor-width must be positive\n";
        std::exit(2);
      }
    } else if (arg == "--armor-height") {
      o.armor_height = std::strtod(next("--armor-height").c_str(), nullptr);
      if (o.armor_height <= 0.0) {
        std::cerr << "--armor-height must be positive\n";
        std::exit(2);
      }
    } else if (arg == "--kalman") {
      o.use_kalman = true;
    } else if (arg == "--no-kalman") {
      o.use_kalman = false;
    } else if (arg == "--control") {
      o.send_commands = true;
    } else if (arg == "--kf-pos-sigma") {
      o.kf_pos_sigma = std::strtod(next("--kf-pos-sigma").c_str(), nullptr);
    } else if (arg == "--kf-vel-sigma") {
      o.kf_vel_sigma = std::strtod(next("--kf-vel-sigma").c_str(), nullptr);
    } else if (arg == "--kf-acc-sigma") {
      o.kf_acc_sigma = std::strtod(next("--kf-acc-sigma").c_str(), nullptr);
    } else if (arg == "--kf-meas-sigma") {
      o.kf_meas_sigma = std::strtod(next("--kf-meas-sigma").c_str(), nullptr);
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

ContestScene toScene(const std::string& name) {
  if (name == "shooting-range") return ContestScene::ShootingRange;
  if (name == "energy" || name == "large-energy") return ContestScene::Energy;
  std::cerr << "unknown scene '" << name
            << "' (expected shooting-range, energy or large-energy)\n";
  std::exit(2);
}

const char* sceneStatusName(SceneControlStatus s) {
  switch (s) {
    case SceneControlStatus::Ok: return "Ok";
    case SceneControlStatus::InvalidRequest: return "InvalidRequest";
    case SceneControlStatus::Unsupported: return "Unsupported";
    case SceneControlStatus::NotReady: return "NotReady";
    case SceneControlStatus::InternalError: return "InternalError";
  }
  return "Unknown";
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

const char* formatName(tcp_image::PixelFormat f) {
  switch (f) {
    case tcp_image::PixelFormat::Rgb24: return "RGB24";
    case tcp_image::PixelFormat::Rgba32: return "RGBA32";
  }
  return "UNKNOWN";
}

void printBigRuneScore(ContestClient& sim) {
  for (const RuneTeam team : {RuneTeam::Red, RuneTeam::Blue}) {
    auto res = sim.getBigRuneScore(team);
    if (!res.ok() || !res.value) {
      std::cout << "big-rune " << (team == RuneTeam::Red ? "red" : "blue")
                << ": unavailable (" << res.status.message << ")\n";
      continue;
    }
    const BigRuneScore& s = *res.value;
    std::cout << "big-rune " << (team == RuneTeam::Red ? "red" : "blue") << ": "
              << "run_id=" << s.run_id << " active=" << s.run_active
              << " arms=" << static_cast<unsigned>(s.activated_arms)
              << " has_hit=" << s.has_hit
              << " avg_ring=" << std::fixed << std::setprecision(2) << s.average_ring
              << " last_ring=" << static_cast<unsigned>(s.last_ring)
              << " last_radius_mm=" << s.last_radius_mm
              << " last_target=" << static_cast<int>(s.last_target) << "\n";
  }
}

void printMetadata(const TalosMetadataMapping& mapping) {
  auto reader_res = mapping.reader();
  if (!reader_res.ok()) {
    std::cout << "metadata: reader unavailable\n";
    return;
  }
  const TalosMetadataReader& reader = *reader_res.value;

  auto header = reader.readHeader();
  if (header.ok() && header.value) {
    const ShmHeader& h = *header.value;
    std::cout << "metadata: magic=0x" << std::hex << h.magic << std::dec
              << " version=" << h.version << " abi_rev=" << h.sdk_abi_revision
              << " image=" << h.image_width << "x" << h.image_height << "\n";
  }

  auto camera = reader.readCameraInfo();
  if (camera.ok() && camera.value) {
    const CameraInfo& c = *camera.value;
    std::cout << "camera: fx=" << c.fx << " fy=" << c.fy << " cx=" << c.cx
              << " cy=" << c.cy << " " << c.width << "x" << c.height << "\n";
  }

  auto gimbal = reader.readGimbalState();
  if (gimbal.ok() && gimbal.value) {
    const GimbalState& g = *gimbal.value;
    std::cout << "gimbal(latest): frame_seq=" << g.frame_seq
              << " yaw=" << g.yaw_deg << " deg pitch=" << g.pitch_deg << " deg\n";
  }

  auto chassis = reader.readChassisObservation();
  if (chassis.ok() && chassis.value) {
    const ChassisObservation& c = *chassis.value;
    std::cout << "chassis: frame_seq=" << c.frame_seq
              << " v_body=(" << c.v_body[0] << ", " << c.v_body[1] << ") m/s"
              << " wz=" << c.wz_radps << " rad/s\n";
  }
}

}  // namespace

// Stateful single-target Kalman tracker fusing per-frame PnP world coordinates.
struct KalmanTracker {
  vision::KalmanFilter3D kf;
  cv::Vec3d measurement;      // last raw PnP world (odom) coordinate
  cv::Vec3d filtered;         // last Kalman-filtered world coordinate
  cv::Vec3d velocity;         // Kalman velocity estimate (world frame)
  bool has_target = false;
  std::uint64_t last_seq = 0;
  std::uint64_t last_timestamp_ns = 0;
  int lost_frames = 0;
};

// Compute PnP poses for all detected armors.
std::vector<vision::TargetPose> computePoses(
    const vision::DetectionResult& det, const vision::CameraIntrinsics& k,
    const vision::CameraExtrinsics& e, double armor_width,
    double armor_height) {
  std::vector<vision::TargetPose> poses;
  if (k.fx <= 0.0 || k.fy <= 0.0) return poses;
  poses.reserve(det.armors.size());
  for (const auto& armor : det.armors) {
    poses.push_back(
        vision::solveArmorPose(armor, k, e, armor_width, armor_height));
  }
  return poses;
}

// Read the exposure-synced gimbal world (odom) pose for a frame sequence.
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
    pose.quaternion_wxyz[i] = es.gimbal_quaternion_world_wxyz[i];
  }
  pose.quaternion_wxyz[3] = es.gimbal_quaternion_world_wxyz[3];
  return pose;
}

// Print raw PnP poses for all detected armors: position and rotation in the
// camera/gimbal frames and (when a world pose is available) the absolute world
// (odom) frame, plus ZYX Euler angles and world spherical (yaw/pitch/distance).
void printPoses(const std::vector<vision::TargetPose>& poses,
                const vision::GimbalWorldPose& world_pose) {
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const vision::TargetPose& pose = poses[i];
    if (!pose.valid) {
      std::cout << "pnp armor[" << i << "]: solvePnP failed\n";
      continue;
    }
    std::cout << "pnp armor[" << i << "]:"
              << " distance=" << std::fixed << std::setprecision(2)
              << pose.distance_m << " m"
              << " cam=(" << pose.t_cam[0] << ", " << pose.t_cam[1] << ", "
              << pose.t_cam[2] << ")"
              << " gimbal=(" << pose.t_gimbal[0] << ", " << pose.t_gimbal[1]
              << ", " << pose.t_gimbal[2] << ")";

    // Rotation chain: camera -> gimbal -> world (R_armor2x). Euler ypr is
    // extracted in the gimbal frame (always available) and world frame.
    {
      cv::Mat R_armor2gimbal;
      cv::Rodrigues(pose.r_gimbal, R_armor2gimbal);
      const cv::Vec3d ypr_gimbal =
          vision::rotationMatrixToYpr(R_armor2gimbal);
      std::cout << " ypr_gimbal=("
                << ypr_gimbal[0] * 180.0 / CV_PI << ", "
                << ypr_gimbal[1] * 180.0 / CV_PI << ", "
                << ypr_gimbal[2] * 180.0 / CV_PI << ") deg";
    }

    if (world_pose.valid) {
      const cv::Vec3d w = vision::gimbalToWorld(pose.t_gimbal, world_pose);
      std::cout << " world=(" << w[0] << ", " << w[1] << ", " << w[2] << ")";

      const cv::Vec3d r_world =
          vision::rotationGimbalToWorld(pose.r_gimbal, world_pose);
      cv::Mat R_armor2world;
      cv::Rodrigues(r_world, R_armor2world);
      const cv::Vec3d ypr_world = vision::rotationMatrixToYpr(R_armor2world);
      const cv::Vec3d ypd_world = vision::xyzToYpd(w);
      std::cout << " ypr_world=("
                << ypr_world[0] * 180.0 / CV_PI << ", "
                << ypr_world[1] * 180.0 / CV_PI << ", "
                << ypr_world[2] * 180.0 / CV_PI << ") deg"
                << " ypd_world=("
                << ypd_world[0] * 180.0 / CV_PI << ", "
                << ypd_world[1] * 180.0 / CV_PI << ", "
                << ypd_world[2] << " m)";
    }
    std::cout << "\n";
    for (int j = 0; j < 2; ++j) {
      const cv::Point3f& l = pose.light_center_gimbal[j];
      std::cout << "  light[" << j << "]: gimbal=(" << l.x << ", " << l.y
                << ", " << l.z << ")";
      if (world_pose.valid) {
        const cv::Vec3d lw = vision::gimbalToWorld(
            cv::Vec3d(l.x, l.y, l.z), world_pose);
        std::cout << " world=(" << lw[0] << ", " << lw[1] << ", " << lw[2]
                  << ")";
      }
      std::cout << "\n";
    }
  }
}

// Update the Kalman tracker with this frame's poses. Filtering happens in the
// absolute world (odom) frame: the target's world position is unaffected by
// gimbal rotation, so the velocity estimate stays stable and the gimbal does
// not chase its own motion. Picks the detected armor nearest to the
// prediction, advances the constant-acceleration model by dt and fuses the
// observation. Returns true and fills the tracker when a target is followed.
bool updateKalman(KalmanTracker& t, const std::vector<vision::TargetPose>& poses,
                  const vision::GimbalWorldPose& world_pose,
                  std::uint64_t timestamp_ns, double default_dt) {
  // dt from exposure timestamps (nanoseconds).
  double dt = default_dt;
  if (t.has_target && t.last_timestamp_ns != 0 && timestamp_ns >= t.last_timestamp_ns) {
    dt = static_cast<double>(timestamp_ns - t.last_timestamp_ns) * 1e-9;
    if (dt <= 0.0 || dt > 0.5) dt = default_dt;
  }
  t.last_timestamp_ns = timestamp_ns;

  // Convert a pose's gimbal-frame position into the world frame.
  const auto to_world = [&](const cv::Vec3d& p_gimbal) -> cv::Vec3d {
    return world_pose.valid ? vision::gimbalToWorld(p_gimbal, world_pose)
                            : p_gimbal;
  };

  if (!t.has_target) {
    // Start tracking the closest valid armor (world frame).
    int best = -1;
    double best_score = 1e18;
    for (std::size_t i = 0; i < poses.size(); ++i) {
      if (!poses[i].valid) continue;
      const double score = to_world(poses[i].t_gimbal)[2];  // depth
      if (score < best_score) {
        best_score = score;
        best = static_cast<int>(i);
      }
    }
    if (best < 0) {
      t.lost_frames = 0;
      return false;
    }
    t.measurement = to_world(poses[best].t_gimbal);
    t.kf.init(t.measurement, dt);
    t.filtered = t.measurement;
    t.velocity = cv::Vec3d(0, 0, 0);
    t.has_target = true;
    t.lost_frames = 0;
    return true;
  }

  // Advance the model.
  t.kf.predict(dt);
  const cv::Vec3d predicted = t.kf.predictedPosition();

  // Associate the pose closest to the prediction (world frame).
  int best = -1;
  double best_dist = 1e18;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    if (!poses[i].valid) continue;
    const double d = cv::norm(to_world(poses[i].t_gimbal) - predicted);
    if (d < best_dist) {
      best_dist = d;
      best = static_cast<int>(i);
    }
  }

  if (best >= 0 && best_dist < 2.0) {  // gate: < 2 m from prediction
    t.measurement = to_world(poses[best].t_gimbal);
    if (t.kf.update(t.measurement)) {
      t.filtered = t.kf.position();
      t.velocity = t.kf.velocity();
    }
    t.lost_frames = 0;
  } else {
    t.filtered = t.kf.position();
    t.velocity = t.kf.velocity();
    if (++t.lost_frames > 60) t.has_target = false;  // lost target
  }
  return t.has_target;
}

int main(int argc, char** argv) {
  const Options o = parseArgs(argc, argv);

  std::cout << "Daedalus SDK " << kSdkVersion << "  IPC dir: " << o.ipc_directory << "\n";

  ContestClientOptions opts;
  opts.ipc_directory = o.ipc_directory;
  ContestClient sim(opts);

  if (!sim.connect()) {
    std::cerr << "could not connect to the simulator in " << o.ipc_directory << ".\n"
              << "Start it first, e.g.:\n"
              << "  /home/xqy/autoaim/linux-x86_64/daedalus-contest.sh start\n"
              << "  /home/xqy/autoaim/linux-x86_64/start-simulator.sh --visible --ipc-dir "
              << o.ipc_directory << "\n";
    return 1;
  }
  std::cout << "connected to simulator.\n";

  auto caps = sim.health();
  if (caps.ok() && caps.value) {
    const RuntimeCapabilities& c = *caps.value;
    std::cout << "simulator health: schema=" << c.schema_version
              << " product=" << c.product_version
              << " backend=" << c.render_backend << " adapter=" << c.adapter_name
              << " driver=" << c.driver << "\n";
  } else {
    std::cout << "simulator health: unavailable (" << caps.status.message << ")\n";
  }

  auto caps_file = readRuntimeCapabilities(o.ipc_directory);
  if (caps_file.ok() && caps_file.value) {
    const RuntimeCapabilities& c = *caps_file.value;
    std::cout << "runtime capabilities: locked=" << (c.distribution_locked ? "yes" : "no")
              << " backend=" << c.render_backend
              << " adapter=" << c.adapter_name
              << " vendor=" << c.vendor_id << " device=" << c.device_id
              << " type=" << c.device_type << "\n";
  }

  auto scene_res = sim.selectScene(toScene(o.scene));
  if (scene_res.ok() && scene_res.value) {
    const SceneControlResponse& r = *scene_res.value;
    std::cout << "scene '" << o.scene << "' selected: status="
              << sceneStatusName(r.status) << " command_id=" << r.command_id
              << " applied_frame_seq=" << r.applied_frame_seq << "\n";
  } else {
    std::cout << "scene selection failed: " << scene_res.status.message << "\n";
  }

  const std::string meta_path =
      o.ipc_directory + "/" + std::string(kMetaFileName);
  TalosMetadataMapping metadata_mapping;
  const ClientStatus meta_status = metadata_mapping.open(meta_path);
  if (meta_status.ok()) {
    printMetadata(metadata_mapping);
  } else {
    std::cout << "metadata: could not open " << meta_path << " ("
              << meta_status.message << ")\n";
  }

  vision::CameraIntrinsics camera_intrinsics;
  vision::CameraExtrinsics camera_extrinsics;
  if (loadCalibration(o.calibration_path, camera_intrinsics,
                      camera_extrinsics)) {
    std::cout << "calibration: fx=" << camera_intrinsics.fx
              << " fy=" << camera_intrinsics.fy
              << " cx=" << camera_intrinsics.cx
              << " cy=" << camera_intrinsics.cy << " (from "
              << o.calibration_path << ")\n";
  } else {
    std::cerr << "calibration: could not load " << o.calibration_path
              << "; PnP world coordinates disabled\n";
  }
  const double camera_tilt_deg =
      2.0 * std::acos(camera_extrinsics.quaternion_xyzw[3]) * 180.0 / CV_PI;

  std::cout << "morph: " << (o.use_morph ? "enabled" : "disabled");
  if (o.use_morph) {
    std::cout << " kernel=" << o.kernel_size << " rb_threshold=" << o.rb_threshold
              << " color=" << (o.color == vision::LightColor::Blue ? "blue" : "red")
              << " open=" << (o.enable_open ? "on" : "off")
              << " max_area=" << o.max_contour_area
              << " grid_width=" << o.grid_width
              << " det(min_area=" << o.det.min_light_area
              << " solidity=" << o.det.min_contour_solidity
              << " max_ratio=" << o.det.max_light_ratio
              << " angle_diff=" << o.det.max_angle_diff
              << " hdiff=" << o.det.max_height_diff_ratio
              << " ydiff=" << o.det.max_y_diff_ratio
              << " xdiff=" << o.det.min_x_diff_ratio
              << " armor_ratio=" << o.det.min_armor_ratio << "-"
              << o.det.max_armor_ratio << ")";
  }
  std::cout << "\n";

  std::uint64_t last_seq = 0;
  std::uint64_t frame_count = 0;
  const auto t0 = std::chrono::steady_clock::now();
  bool show_window = true;
  KalmanTracker tracker;

  // 检测器：传统视觉或神经网络，--detector 切换。
  vision::MorphologyParams detector_morph;
  detector_morph.kernel_size = o.kernel_size;
  detector_morph.rb_threshold = o.rb_threshold;
  detector_morph.max_contour_area = o.max_contour_area;
  detector_morph.enable_open = o.enable_open;
  std::unique_ptr<detect::IDetector> detector = detect::makeDetector(
      o.detector_backend, o.nn_model_path, o.color, detector_morph, o.det);
  std::cout << "detector: " << detector->name()
            << (detector->name() == "traditional"
                    ? " (morph + light-bar pairing)"
                    : " (YOLO ONNX, score/nms tunable)")
            << "\n";

  std::cout << "\nstreaming frames (press 'q' or close the window to exit)...\n";

  while (true) {
    auto frame_res = sim.nextFrame(last_seq);
    if (!frame_res.ok() || !frame_res.value) {
      std::cout << "nextFrame: " << frame_res.status.message << "\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    const ContestFrame& cf = *frame_res.value;
    const tcp_image::FrameHeader& h = cf.image.header;
    last_seq = h.source_sequence;
    ++frame_count;

    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();

    std::cout << "\n===== frame " << frame_count << " ====="
              << std::fixed << std::setprecision(1)
              << "  (" << (frame_count / elapsed) << " fps)\n";
    std::cout << "image: seq=" << h.source_sequence
              << " producer_epoch=" << h.producer_epoch
              << " timestamp_ns=" << h.capture_timestamp_ns << "\n"
              << "       " << h.width << "x" << h.height
              << " format=" << formatName(h.format)
              << " payload=" << h.payload_bytes << " bytes\n";

    const GimbalState& g = cf.gimbal;
    std::cout << "gimbal: yaw=" << g.yaw_deg << " deg"
              << " pitch=" << g.pitch_deg << " deg"
              << " yaw_vel=" << g.yaw_velocity_deg_s << " deg/s"
              << " pitch_vel=" << g.pitch_velocity_deg_s << " deg/s"
              << " last_cmd=" << g.last_applied_command_id << "\n";

    // Read this frame's world pose before detector/PnP work. The simulator
    // retains only the latest 16 exposure poses.
    const vision::GimbalWorldPose world_pose =
        readGimbalWorldPose(metadata_mapping, h.source_sequence);

    if (o.scene == "energy" || o.scene == "large-energy") {
      printBigRuneScore(sim);
    }

    cv::Mat image = frameToBgr(cf.image);
    if (image.empty()) {
      std::cerr << "unsupported pixel format, cannot render frame\n";
      continue;
    }

    vision::MorphologyResult morph;
    vision::DetectionResult det;
    const bool is_traditional = (detector->name() == "traditional");
    if (is_traditional && o.use_morph) {
      vision::MorphologyParams params;
      params.kernel_size = o.kernel_size;
      params.rb_threshold = o.rb_threshold;
      params.max_contour_area = o.max_contour_area;
      params.enable_open = o.enable_open;
      morph = vision::applyMorphology(
          vision::splitColorMask(image, o.color, params.rb_threshold), params);

      const int before = cv::countNonZero(morph.mask);
      const int after = cv::countNonZero(morph.final_mask);
      const double noise_removed =
          100.0 * (before - after) / (before > 0 ? before : 1);
      std::cout << "morph: kernel=" << o.kernel_size
                << " rb_threshold=" << o.rb_threshold
                << " mask_before=" << before << " mask_after=" << after
                << " noise_removed=" << std::fixed << std::setprecision(1)
                << noise_removed << "%"
                << " contours=" << morph.contours.size();
      if (morph.contours_filtered > 0) {
        std::cout << " (filtered=" << morph.contours_filtered << ")";
      }
      std::cout << "\n";
    }

    // 用所选检测器识别装甲板。
    det = detector->detect(image);
    std::cout << "detect[" << detector->name() << "]: armors=" << det.armors.size()
              << "\n";

    if (o.det_debug && is_traditional) {
      for (const auto& c : det.candidates) {
        std::cout << "  cand L" << c.left_idx << "+L" << c.right_idx
                  << (c.accepted ? " [OK] " : " [rej] ")
                  << "angle_diff=" << std::fixed << std::setprecision(1)
                  << c.angle_diff
                  << " len_diff_ratio=" << c.len_diff_ratio
                  << " y_diff_ratio=" << c.y_diff_ratio
                  << " x_diff_ratio=" << c.x_diff_ratio
                  << " ratio=" << c.ratio
                  << " dist=" << c.distance
                  << " mean_len=" << c.mean_len << "\n";
      }
    }

      std::vector<vision::TargetPose> poses =
          computePoses(det, camera_intrinsics, camera_extrinsics,
                       o.armor_width, o.armor_height);
      printPoses(poses, world_pose);

      bool kalman_active = false;
      if (o.use_kalman && camera_intrinsics.fx > 0.0) {
        tracker.kf.process_pos_sigma = o.kf_pos_sigma;
        tracker.kf.process_vel_sigma = o.kf_vel_sigma;
        tracker.kf.process_acc_sigma = o.kf_acc_sigma;
        tracker.kf.measurement_sigma = o.kf_meas_sigma;
        kalman_active = updateKalman(tracker, poses, world_pose,
                                     h.capture_timestamp_ns, 1.0 / 60.0);
        if (kalman_active) {
          const cv::Vec3d& r = tracker.measurement;
          const cv::Vec3d& f = tracker.filtered;
          const cv::Vec3d& v = tracker.velocity;
          std::cout << "kf: raw=(" << std::fixed << std::setprecision(2)
                    << r[0] << ", " << r[1] << ", " << r[2] << ")"
                    << " filtered=(" << f[0] << ", " << f[1] << ", " << f[2]
                    << ")"
                    << " vel=(" << v[0] << ", " << v[1] << ", " << v[2]
                    << ") m/s"
                    << " lost=" << tracker.lost_frames;
          // Convert the filtered world point to the current gimbal frame;
          // absoluteWorldAimAngles then uses the synchronized world pose.
          if (world_pose.valid) {
            const cv::Vec3d f_gimbal = vision::worldToGimbal(f, world_pose);
            const cv::Vec2d abs_aim =
                vision::absoluteWorldAimAngles(f_gimbal, world_pose);
            std::cout << " aim_yaw=" << std::fixed << std::setprecision(2)
                      << abs_aim[0] << " aim_pitch=" << abs_aim[1];
          }
          std::cout << "\n";
        }
      }

      // Keep one target-angle value for the HUD and optional control output.
      // Tracking is done in absolute world coordinates, so convert the
      // filtered point back into this exposure's gimbal frame before aiming.
      cv::Vec2d target_aim(0.0, 90.0);
      double target_dist = 0.0;
      bool hud_target = false;
      cv::Vec3d target_world(0.0, 0.0, 0.0);
      bool hud_world_target = false;
      if (kalman_active && world_pose.valid) {
        const cv::Vec3d target_gimbal =
            vision::worldToGimbal(tracker.filtered, world_pose);
        target_world = tracker.filtered;
        target_aim = vision::absoluteWorldAimAngles(target_gimbal, world_pose);
        target_dist = cv::norm(target_gimbal);
        hud_target = true;
        hud_world_target = true;
      } else if (camera_intrinsics.fx > 0.0) {
        for (const auto& p : poses) {
          if (!p.valid) continue;
          if (world_pose.valid) {
            target_world = vision::gimbalToWorld(p.t_gimbal, world_pose);
            target_aim = vision::absoluteWorldAimAngles(p.t_gimbal, world_pose);
            hud_world_target = true;
          } else {
            target_aim = vision::absoluteAimAngles(
                p.t_gimbal, g.yaw_deg, g.pitch_deg, camera_tilt_deg);
          }
          target_dist = cv::norm(p.t_gimbal);
          hud_target = true;
          break;
        }
      }

      if (o.send_commands && hud_target) {
        UdpGimbalCommand command;
        command.yaw_deg = static_cast<float>(target_aim[0]);
        command.pitch_deg = static_cast<float>(target_aim[1]);
        command.distance_m = static_cast<float>(target_dist);
        const auto sent = sim.sendAim(command);
        if (!sent.ok()) {
          std::cout << "sendAim: " << sent.status.message << "\n";
        }
      }

      if (show_window) {
        try {
          cv::Mat display = vision::drawResult(image, morph, det);
          if (camera_intrinsics.fx > 0.0) {
            for (std::size_t i = 0; i < det.armors.size(); ++i) {
              if (i >= poses.size() || !poses[i].valid) continue;
              const vision::TargetPose& pose = poses[i];
              const cv::Point2f center =
                  (det.armors[i].left.center + det.armors[i].right.center) *
                  0.5F;
              std::ostringstream label;
              label << std::fixed << std::setprecision(2) << pose.distance_m
                    << " m (" << pose.t_gimbal[0] << ", " << pose.t_gimbal[1]
                    << ", " << pose.t_gimbal[2] << ")";
              cv::putText(display, label.str(),
                          cv::Point(static_cast<int>(center.x) - 40,
                                    static_cast<int>(center.y) - 25),
                          cv::FONT_HERSHEY_SIMPLEX, 0.5,
                          cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            }
          }
          if (kalman_active && world_pose.valid) {
            // Project the filtered world position back into the image and
            // draw a green cross (raw PnP label is yellow).
            const cv::Vec3d p_cam =
                vision::gimbalToCamera(
                    vision::worldToGimbal(tracker.filtered, world_pose),
                    camera_extrinsics);
            const cv::Point2f p =
                vision::projectPoint(p_cam, camera_intrinsics);
            if (p.x > 0 && p.y > 0) {
              cv::line(display, cv::Point(int(p.x) - 8, int(p.y)),
                       cv::Point(int(p.x) + 8, int(p.y)),
                       cv::Scalar(0, 255, 0), 2);
              cv::line(display, cv::Point(int(p.x), int(p.y) - 8),
                       cv::Point(int(p.x), int(p.y) + 8),
                       cv::Scalar(0, 255, 0), 2);
              cv::putText(display, "KF",
                          cv::Point(int(p.x) + 10, int(p.y) - 8),
                          cv::FONT_HERSHEY_SIMPLEX, 0.5,
                          cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            }
          }
          // Aiming HUD: current gimbal angles + target aim (from the Kalman
          // filtered world position when active, otherwise the closest raw pose).
          vision::drawAimHud(display, g.yaw_deg, g.pitch_deg, hud_target,
                             target_aim, target_dist, hud_world_target,
                             target_world);

          cv::imshow(o.window_title, display);
          cv::imshow("Morphology Stages",
                     vision::makeGrid(image, morph, det, o.grid_width));
          const int key = cv::waitKey(1);
          if (key == 'q' || key == 27) break;
          if (cv::getWindowProperty(o.window_title, cv::WND_PROP_VISIBLE) < 1) break;
        } catch (const cv::Exception& e) {
          std::cout << "window display unavailable: " << e.what() << "\n";
          show_window = false;
        }
      }

      if (!o.save_dir.empty()) {
        cv::Mat saved = vision::drawResult(image, morph, det);
        if (camera_intrinsics.fx > 0.0) {
          for (std::size_t i = 0; i < det.armors.size(); ++i) {
            if (i >= poses.size() || !poses[i].valid) continue;
            const vision::TargetPose& pose = poses[i];
            const cv::Point2f center =
                (det.armors[i].left.center + det.armors[i].right.center) *
                0.5F;
            std::ostringstream label;
            label << std::fixed << std::setprecision(2) << pose.distance_m
                  << " m (" << pose.t_gimbal[0] << ", " << pose.t_gimbal[1]
                  << ", " << pose.t_gimbal[2] << ")";
            cv::putText(saved, label.str(),
                        cv::Point(static_cast<int>(center.x) - 40,
                                  static_cast<int>(center.y) - 25),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
          }
        }
        if (kalman_active && world_pose.valid) {
          const cv::Vec3d p_cam =
              vision::gimbalToCamera(
                  vision::worldToGimbal(tracker.filtered, world_pose),
                  camera_extrinsics);
          const cv::Point2f p =
              vision::projectPoint(p_cam, camera_intrinsics);
          if (p.x > 0 && p.y > 0) {
            cv::line(saved, cv::Point(int(p.x) - 8, int(p.y)),
                     cv::Point(int(p.x) + 8, int(p.y)), cv::Scalar(0, 255, 0),
                     2);
            cv::line(saved, cv::Point(int(p.x), int(p.y) - 8),
                     cv::Point(int(p.x), int(p.y) + 8), cv::Scalar(0, 255, 0),
                     2);
            cv::putText(saved, "KF", cv::Point(int(p.x) + 10, int(p.y) - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2,
                        cv::LINE_AA);
          }
        }
        vision::drawAimHud(saved, g.yaw_deg, g.pitch_deg, hud_target,
                           target_aim, target_dist, hud_world_target,
                           target_world);
        cv::imwrite(o.save_dir + "/result.png", saved);
      }

    if (!o.save_dir.empty()) {
      std::ostringstream path;
      path << o.save_dir << "/frame_" << std::setw(6) << std::setfill('0')
           << frame_count << "_seq_" << h.source_sequence << ".png";
      cv::imwrite(path.str(), image);
      if (o.use_morph) {
        cv::imwrite(o.save_dir + "/mask.png", morph.mask);
        cv::imwrite(o.save_dir + "/eroded.png", morph.eroded);
        cv::imwrite(o.save_dir + "/dilated.png", morph.dilated);
        cv::imwrite(o.save_dir + "/opened.png", morph.opened);
        cv::imwrite(o.save_dir + "/closed.png", morph.closed);
        cv::imwrite(o.save_dir + "/final_mask.png", morph.final_mask);
      }
    }

    if (o.max_frames > 0 && frame_count >= o.max_frames) break;
  }

  sim.close();
  std::cout << "\ndone. received " << frame_count << " frames.\n";
  return 0;
}
