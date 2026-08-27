// 识别 + PnP 曲线测试工具
//
// 仅做识别与 PnP 解算，不做云台控制/开火。把每一帧检测到的装甲板 PnP 结果
// （相机系/云台系/世界系坐标、距离、检测框像素）写入 CSV，供 tools/plot_pnp.py
// 画曲线图。用于评估识别稳定性、PnP 抖动、Kalman 平滑效果。
//
// 用法：
//   ./build/pnp_curve_test [--ipc-dir DIR] [--calibration PATH] [--frames N]
//                          [--detector nn|traditional] [--out out.csv]
//                          [--kalman] [--no-kalman] [--color red|blue]
// 需先启动模拟器。

#include <daedalus_sim_sdk/contest_client.hpp>
#include <daedalus_sim_sdk/talos_metadata_reader.hpp>

#include "tasks/detection/detector.hpp"
#include "tasks/planning/planning.hpp"
#include "tasks/vision/vision_processing.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace daedalus::sim::sdk::v1;

namespace {

struct Options {
  std::string ipc_directory;
  std::string calibration_path =
      "/home/xqy/下载/linux-x86_64(1)/camera-calibration.json";
  std::string out_path = "/tmp/pnp_curve.csv";
  std::uint64_t frames = 200;
  detect::Backend backend = detect::Backend::NeuralNetwork;
  std::string nn_model = "/home/xqy/桌面/tongji/assets/yolo11.xml";
  vision::LightColor color = vision::LightColor::Red;
  bool use_kalman = true;
  bool show = false;  // 实时可视化识别框窗口
  double bullet_speed = 25.0;
  double armor_width = 0.135;
  double armor_height = 0.056;
  vision::DetectionParams det;
};

void printUsage(const char* argv0) {
  std::cout
      << "Usage: " << argv0 << " [options]\n"
      << "Run detection + PnP only, dump per-frame results to CSV.\n\n"
      << "Options:\n"
      << "  --ipc-dir PATH      simulator IPC directory (default $XDG_RUNTIME_DIR)\n"
      << "  --calibration PATH  camera-calibration.json (default: r4 release's)\n"
      << "  --frames N          frames to record (default 200)\n"
      << "  --detector NAME     nn | traditional (default nn)\n"
      << "  --nn-model PATH     OpenVINO model for nn (default tongji yolo11.xml)\n"
      << "  --out PATH          CSV output path (default /tmp/pnp_curve.csv)\n"
      << "  --kalman            enable world-frame Kalman smoothing (default)\n"
      << "  --no-kalman         disable Kalman smoothing\n"
      << "  --show              show a real-time OpenCV window with detection\n"
      << "                      boxes + PnP distance labels\n"
      << "  --color NAME        light bar color red|blue (default red)\n"
      << "  -h, --help          show this help and exit\n";
}

cv::Mat frameToBgr(const TcpImageFrame& f) {
  const auto& h = f.header;
  if (h.format == tcp_image::PixelFormat::Rgba32) {
    cv::Mat rgba(h.height, h.width, CV_8UC4, (void*)f.payload.data());
    cv::Mat b;
    cv::cvtColor(rgba, b, cv::COLOR_RGBA2BGR);
    return b;
  }
  if (h.format == tcp_image::PixelFormat::Rgb24) {
    cv::Mat rgb(h.height, h.width, CV_8UC3, (void*)f.payload.data());
    cv::Mat b;
    cv::cvtColor(rgb, b, cv::COLOR_RGB2BGR);
    return b;
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
    if (arg == "--ipc-dir") o.ipc_directory = next("--ipc-dir");
    else if (arg == "--calibration") o.calibration_path = next("--calibration");
    else if (arg == "--frames") o.frames = std::strtoull(next("--frames").c_str(), nullptr, 10);
    else if (arg == "--detector") {
      const std::string n = next("--detector");
      o.backend = (n == "nn") ? detect::Backend::NeuralNetwork
                              : detect::Backend::Traditional;
    } else if (arg == "--nn-model") o.nn_model = next("--nn-model");
    else if (arg == "--out") o.out_path = next("--out");
    else if (arg == "--kalman") o.use_kalman = true;
    else if (arg == "--no-kalman") o.use_kalman = false;
    else if (arg == "--show") o.show = true;
    else if (arg == "--color") {
      const std::string n = next("--color");
      o.color = (n == "blue") ? vision::LightColor::Blue : vision::LightColor::Red;
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
    const char* r = std::getenv("XDG_RUNTIME_DIR");
    std::ostringstream ss;
    if (r && *r) ss << r;
    else ss << "/tmp";
    ss << "/daedalus-contest-" << ::getuid();
    o.ipc_directory = ss.str();
  }
  return o;
}

// Simple JSON field reader (reuse for calibration).
double readJsonDouble(const std::string& t, const std::string& key) {
  const std::string needle = "\"" + key + "\"";
  std::size_t p = t.find(needle);
  if (p == std::string::npos) return 0.0;
  p = t.find(':', p + needle.size());
  if (p == std::string::npos) return 0.0;
  return std::strtod(t.c_str() + p + 1, nullptr);
}
bool readJsonArray(const std::string& t, const std::string& key, double* out, std::size_t n) {
  const std::string needle = "\"" + key + "\"";
  std::size_t p = t.find(needle);
  if (p == std::string::npos) return false;
  p = t.find('[', p + needle.size());
  if (p == std::string::npos) return false;
  ++p;
  for (std::size_t i = 0; i < n; ++i) {
    while (p < t.size() && (t[p] == ' ' || t[p] == ',' || t[p] == '\n' || t[p] == '\t')) ++p;
    out[i] = std::strtod(t.c_str() + p, nullptr);
    while (p < t.size() && t[p] != ',' && t[p] != ']') ++p;
    if (p >= t.size()) return false;
  }
  return true;
}

bool loadCalibration(const std::string& path, vision::CameraIntrinsics& k,
                     vision::CameraExtrinsics& e) {
  std::ifstream in(path);
  if (!in) return false;
  const std::string t((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  k.fx = readJsonDouble(t, "fx");
  k.fy = readJsonDouble(t, "fy");
  k.cx = readJsonDouble(t, "cx");
  k.cy = readJsonDouble(t, "cy");
  if (k.fx <= 0.0) return false;
  readJsonArray(t, "distortion", k.distortion, 5);
  readJsonArray(t, "translation_m", e.translation_m, 3);
  readJsonArray(t, "quaternion_xyzw", e.quaternion_xyzw, 4);
  return true;
}

vision::GimbalWorldPose readWorldPose(const TalosMetadataMapping& m,
                                      std::uint64_t seq) {
  vision::GimbalWorldPose pose;
  auto r = m.reader();
  if (!r.ok()) return pose;
  auto exp = r.value->readExposureStateForFrame(seq);
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

}  // namespace

int main(int argc, char** argv) {
  const Options o = parseArgs(argc, argv);
  bool show_ok = o.show;  // 可在运行时降级（无图形环境）

  std::cout << "Daedalus SDK " << kSdkVersion << "  IPC dir: " << o.ipc_directory
            << "\n";

  ContestClientOptions copts;
  copts.ipc_directory = o.ipc_directory;
  ContestClient sim(copts);
  if (!sim.connect()) {
    std::cerr << "could not connect to the simulator in " << o.ipc_directory << "\n";
    return 1;
  }

  auto scene_res = sim.selectScene(ContestScene::ShootingRange);
  if (scene_res.ok() && scene_res.value) {
    std::cout << "scene shooting-range: status="
              << static_cast<int>(scene_res.value->status) << "\n";
  }

  vision::CameraIntrinsics k;
  vision::CameraExtrinsics e;
  if (!loadCalibration(o.calibration_path, k, e)) {
    std::cerr << "calibration load failed: " << o.calibration_path << "\n";
    return 1;
  }

  TalosMetadataMapping meta;
  {
    const ClientStatus ms = meta.open(o.ipc_directory + "/talos_ipc_meta");
    if (!ms.ok()) {
      std::cerr << "metadata open failed: " << ms.message << "\n";
    }
  }

  vision::MorphologyParams dm;
  dm.kernel_size = 3;
  dm.rb_threshold = 30;
  auto detector = detect::makeDetector(o.backend, o.nn_model, o.color, dm, o.det);
  std::cout << "detector: " << detector->name() << "\n";

  vision::KalmanFilter3D kf;
  kf.process_pos_sigma = 0.005;
  kf.process_vel_sigma = 0.05;
  kf.process_acc_sigma = 0.2;
  kf.measurement_sigma = 0.15;
  bool kf_inited = false;
  std::uint64_t prev_ts = 0;

  std::ofstream csv(o.out_path);
  csv << "seq,t_ns,n_armors,best_cam_x,best_cam_y,best_cam_z,"
         "best_g_x,best_g_y,best_g_z,best_w_x,best_w_y,best_w_z,"
         "best_dist,kf_x,kf_y,kf_z,"
         "box_cx,box_cy,box_w,box_h\n";

  std::uint64_t last_seq = 0;
  std::uint64_t recorded = 0;
  std::cout << "recording " << o.frames << " frames...\n";

  while (recorded < o.frames) {
    auto fr = sim.nextFrame(last_seq);
    if (!fr.ok() || !fr.value) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    const ContestFrame& cf = *fr.value;
    last_seq = cf.image.header.source_sequence;
    const std::uint64_t ts = cf.image.header.capture_timestamp_ns;

    cv::Mat img = frameToBgr(cf.image);
    if (img.empty()) continue;
    vision::DetectionResult det = detector->detect(img);

    const auto wp = readWorldPose(meta, last_seq);

    // Best (closest) armor.
    vision::TargetPose best;
    bool have_best = false;
    double best_dist = 1e18;
    int best_armor = -1;
    for (std::size_t i = 0; i < det.armors.size(); ++i) {
      vision::TargetPose p = vision::solveArmorPose(det.armors[i], k, e,
                                                    o.armor_width, o.armor_height);
      if (!p.valid) continue;
      if (p.distance_m < best_dist) {
        best_dist = p.distance_m;
        best = p;
        have_best = true;
        best_armor = static_cast<int>(i);
      }
    }

    // Kalman world-frame smoothing.
    cv::Vec3d kf_pos(0, 0, 0);
    bool have_kf = false;
    if (have_best && o.use_kalman) {
      const cv::Vec3d cur_w = wp.valid ? vision::gimbalToWorld(best.t_gimbal, wp)
                                       : best.t_gimbal;
      const double dt = prev_ts ? (ts - prev_ts) * 1e-9 : 1.0 / 60.0;
      if (!kf_inited) {
        kf.init(cur_w, dt);
        kf_inited = true;
      } else {
        kf.predict((dt > 0 && dt < 0.5) ? dt : 1.0 / 60.0);
        kf.update(cur_w);
      }
      kf_pos = kf.position();
      have_kf = true;
      prev_ts = ts;
    }

    // Box of the best armor (image pixels).
    double bx = 0, by = 0, bw = 0, bh = 0;
    if (have_best && best_armor >= 0) {
      const auto& a = det.armors[best_armor];
      cv::Point2f c = (a.left.center + a.right.center) * 0.5f;
      bx = c.x; by = c.y;
      bw = cv::norm(a.left.center - a.right.center);
      bh = (a.left.length + a.right.length) * 0.5;
      if (bh <= 0) bh = bw * 0.5;
    }

    const cv::Vec3d w = wp.valid ? vision::gimbalToWorld(best.t_gimbal, wp)
                                 : best.t_gimbal;
    csv << last_seq << "," << ts << "," << det.armors.size() << ","
        << (have_best ? best.t_cam[0] : 0) << "," << (have_best ? best.t_cam[1] : 0) << ","
        << (have_best ? best.t_cam[2] : 0) << ","
        << (have_best ? best.t_gimbal[0] : 0) << "," << (have_best ? best.t_gimbal[1] : 0) << ","
        << (have_best ? best.t_gimbal[2] : 0) << ","
        << (have_best && wp.valid ? w[0] : 0) << "," << (have_best && wp.valid ? w[1] : 0) << ","
        << (have_best && wp.valid ? w[2] : 0) << ","
        << (have_best ? best.distance_m : 0) << ","
        << (have_kf ? kf_pos[0] : 0) << "," << (have_kf ? kf_pos[1] : 0) << ","
        << (have_kf ? kf_pos[2] : 0) << ","
        << bx << "," << by << "," << bw << "," << bh << "\n";

    // 实时可视化：绘制所有检测框 + 最近目标的 PnP 距离/坐标。
    if (show_ok) {
      try {
        cv::Mat vis = img.clone();
        for (const auto& armor : det.armors) {
          const std::vector<cv::Point> box(armor.vertex, armor.vertex + 4);
          cv::polylines(vis, box, true, cv::Scalar(255, 0, 0), 2);
        }
        if (have_best) {
          const cv::Point2f c = (det.armors[best_armor].left.center +
                                 det.armors[best_armor].right.center) * 0.5f;
          cv::circle(vis, c, 4, cv::Scalar(0, 255, 0), -1);
          char info[128];
          std::snprintf(info, sizeof(info), "%.2f m  w=(%.2f, %.2f, %.2f)",
                        best.distance_m, w[0], w[1], w[2]);
          cv::putText(vis, info, cv::Point(int(c.x) - 60, int(c.y) - 12),
                      cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255),
                      1, cv::LINE_AA);
        }
        cv::imshow("pnp_curve_test", vis);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
      } catch (const cv::Exception&) {
        show_ok = false;  // 无图形环境，降级为纯录制
      }
    }

    ++recorded;
    if (recorded % 20 == 0) {
      std::cout << "  " << recorded << "/" << o.frames
                << " armors=" << det.armors.size()
                << " dist=" << (have_best ? best.distance_m : 0)
                << "\n";
    }
  }

  csv.close();
  std::cout << "wrote " << o.out_path << " (" << recorded << " rows)\n";
  sim.close();
  return 0;
}
