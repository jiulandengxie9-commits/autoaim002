#include <daedalus_sim_sdk/contest_client.hpp>
#include <daedalus_sim_sdk/runtime_capabilities.hpp>
#include <daedalus_sim_sdk/talos_metadata_reader.hpp>

#include "vision_processing.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
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
  std::uint64_t max_frames = 0;
  bool use_morph = true;
  int kernel_size = 5;
  int rb_threshold = 30;
  double max_contour_area = 0.0;
  int grid_width = 480;
  vision::DetectionParams det;
};

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
      << "  --rb-threshold N  R-B threshold for the red mask (default 30)\n"
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

void printMetadata(const std::string& ipc_directory) {
  TalosMetadataMapping mapping;
  const std::string meta_path = ipc_directory + "/" + std::string(kMetaFileName);
  const ClientStatus status = mapping.open(meta_path);
  if (!status.ok()) {
    std::cout << "metadata: could not open " << meta_path << " (" << status.message << ")\n";
    return;
  }
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

  printMetadata(o.ipc_directory);

  std::cout << "morph: " << (o.use_morph ? "enabled" : "disabled");
  if (o.use_morph) {
    std::cout << " kernel=" << o.kernel_size << " rb_threshold=" << o.rb_threshold
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

    if (o.scene == "energy" || o.scene == "large-energy") {
      printBigRuneScore(sim);
    }

    cv::Mat image = frameToBgr(cf.image);
    if (image.empty()) {
      std::cerr << "unsupported pixel format, cannot render frame\n";
      continue;
    }

    vision::MorphologyResult morph;
    if (o.use_morph) {
      vision::MorphologyParams params;
      params.kernel_size = o.kernel_size;
      params.rb_threshold = o.rb_threshold;
      params.max_contour_area = o.max_contour_area;
      morph = vision::applyMorphology(
          vision::splitRedMask(image, params.rb_threshold), params);

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

      vision::DetectionResult det = vision::detect(morph.contours, o.det);
      std::cout << "detect: lights=" << det.lights.size()
                << " armors=" << det.armors.size() << "\n";

      if (show_window) {
        try {
          const cv::Mat display = vision::drawResult(image, morph, det);
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
        cv::imwrite(o.save_dir + "/result.png", vision::drawResult(image, morph, det));
      }
    } else {
      if (show_window) {
        try {
          cv::imshow(o.window_title, image);
          if (cv::getWindowProperty("Morphology Stages", cv::WND_PROP_VISIBLE) >= 0) {
            cv::destroyWindow("Morphology Stages");
          }
          const int key = cv::waitKey(1);
          if (key == 'q' || key == 27) break;
          if (cv::getWindowProperty(o.window_title, cv::WND_PROP_VISIBLE) < 1) break;
        } catch (const cv::Exception& e) {
          std::cout << "window display unavailable: " << e.what() << "\n";
          show_window = false;
        }
      }
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
