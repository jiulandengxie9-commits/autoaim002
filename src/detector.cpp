#include "detector.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <signal.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace detect {

// ---------------- VisionDetector ----------------

vision::DetectionResult VisionDetector::detect(const cv::Mat& bgr) {
  vision::MorphologyResult morph = vision::applyMorphology(
      vision::splitColorMask(bgr, color_, morph_.rb_threshold), morph_);
  return vision::detect(morph.contours, det_);
}

// ---------------- NNDetector ----------------

NNDetector::NNDetector(const std::string& model_path, double score_threshold,
                       double nms_threshold)
    : model_path_(model_path),
      score_threshold_(score_threshold),
      nms_threshold_(nms_threshold) {
  ready_ = startService();
  if (!ready_) {
    std::fprintf(stderr,
                 "[NNDetector] could not start OpenVINO Python service (%s); "
                 "fall back to --detector traditional\n",
                 model_path_.c_str());
  }
}

NNDetector::~NNDetector() {
  if (pipe_in_ >= 0) close(pipe_in_);
  if (pipe_out_ >= 0) close(pipe_out_);
  if (child_ > 0) {
    kill(child_, SIGTERM);
    waitpid(child_, nullptr, 0);
  }
}

bool NNDetector::startService() {
  // tools/ov_armor_service.py located next to this source file's project.
  std::string script = "tools/ov_armor_service.py";
  // If cwd is not the project root, locate relative to the executable.
  if (access(script.c_str(), R_OK) != 0) {
    script = "/home/xqy/桌面/autoaim002/tools/ov_armor_service.py";
  }
  if (access(script.c_str(), R_OK) != 0) return false;

  int to_child[2], from_child[2];
  if (pipe(to_child) != 0 || pipe(from_child) != 0) return false;

  pid_t pid = fork();
  if (pid < 0) return false;
  if (pid == 0) {
    // Child: run the Python service, wiring pipes to stdin/stdout.
    dup2(to_child[0], STDIN_FILENO);
    dup2(from_child[1], STDOUT_FILENO);
    close(to_child[0]);
    close(to_child[1]);
    close(from_child[0]);
    close(from_child[1]);
    setenv("AUTO_AIM_NN_MODEL", model_path_.c_str(), 1);
    std::string score = std::to_string(score_threshold_);
    setenv("AUTO_AIM_NN_SCORE", score.c_str(), 1);
    std::string nms = std::to_string(nms_threshold_);
    setenv("AUTO_AIM_NN_NMS", nms.c_str(), 1);
    execlp("python3", "python3", script.c_str(), nullptr);
    _exit(127);
  }

  pipe_in_ = to_child[1];
  pipe_out_ = from_child[0];
  close(to_child[0]);
  close(from_child[1]);
  child_ = pid;
  return true;
}

vision::DetectionResult NNDetector::parseResponse(const std::string& line,
                                                  double scale) {
  vision::DetectionResult result;
  if (line.empty()) return result;

  // Simple JSON parse: find all "x":..., "y":..., "w":..., "h":... pairs.
  std::size_t pos = 0;
  while ((pos = line.find("\"x\":", pos)) != std::string::npos) {
    const std::size_t xb = line.find(':', pos);
    const std::size_t yb = line.find("\"y\":", pos);
    const std::size_t wb = line.find("\"w\":", pos);
    const std::size_t hb = line.find("\"h\":", pos);
    if (xb == std::string::npos || yb == std::string::npos ||
        wb == std::string::npos || hb == std::string::npos) {
      break;
    }
    // The value starts right after the colon; skip spaces.
    const std::size_t xv = line.find_first_not_of(" :", xb + 1);
    const std::size_t yv = line.find_first_not_of(" :", line.find(':', yb) + 1);
    const std::size_t wv = line.find_first_not_of(" :", line.find(':', wb) + 1);
    const std::size_t hv = line.find_first_not_of(" :", line.find(':', hb) + 1);
    if (xv == std::string::npos || yv == std::string::npos ||
        wv == std::string::npos || hv == std::string::npos) {
      break;
    }
    const double x = std::strtod(line.c_str() + xv, nullptr);
    const double y = std::strtod(line.c_str() + yv, nullptr);
    const double w = std::strtod(line.c_str() + wv, nullptr);
    const double h = std::strtod(line.c_str() + hv, nullptr);

    // Drop degenerate/invalid boxes (NaN, non-positive size).
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(w) ||
        !std::isfinite(h) || w <= 0.0 || h <= 0.0) {
      pos = line.find("\"cls\":", pos);
      if (pos == std::string::npos) break;
      pos = line.find(',', pos);
      if (pos == std::string::npos) break;
      ++pos;
      continue;
    }

    // Convert 640-letterbox coords back to full image pixels.
    const float px = static_cast<float>(x / scale);
    const float py = static_cast<float>(y / scale);
    const float pw = static_cast<float>(w / scale);
    const float ph = static_cast<float>(h / scale);

    vision::ArmorDescriptor armor;
    armor.vertex[0] = cv::Point2f(px - pw * 0.5f, py - ph * 0.5f);  // TL
    armor.vertex[1] = cv::Point2f(px + pw * 0.5f, py - ph * 0.5f);  // TR
    armor.vertex[2] = cv::Point2f(px + pw * 0.5f, py + ph * 0.5f);  // BR
    armor.vertex[3] = cv::Point2f(px - pw * 0.5f, py + ph * 0.5f);  // BL
    armor.left.center = (armor.vertex[0] + armor.vertex[3]) * 0.5f;
    armor.right.center = (armor.vertex[1] + armor.vertex[2]) * 0.5f;
    result.armors.push_back(armor);
    pos = line.find("\"cls\":", pos);
    if (pos == std::string::npos) break;
    pos = line.find(',', pos);
    if (pos == std::string::npos) break;
    ++pos;
  }
  return result;
}

vision::DetectionResult NNDetector::detect(const cv::Mat& bgr) {
  vision::DetectionResult result;
  if (!ready_ || bgr.empty()) return result;
  if (bgr.channels() != 3) return result;

  // Compute letterbox scale (must match the Python service).
  const double scale = std::min(640.0 / bgr.rows, 640.0 / bgr.cols);

  // Send raw BGR frame with a 4-byte length prefix (loop for short writes).
  const size_t bytes = static_cast<size_t>(bgr.rows) * bgr.cols * 3;
  const std::uint32_t len = static_cast<std::uint32_t>(bytes);
  std::string header;
  header.push_back(static_cast<char>(len & 0xff));
  header.push_back(static_cast<char>((len >> 8) & 0xff));
  header.push_back(static_cast<char>((len >> 16) & 0xff));
  header.push_back(static_cast<char>((len >> 24) & 0xff));
  {
    size_t off = 0;
    while (off < 4) {
      const ssize_t n = write(pipe_in_, header.data() + off, 4 - off);
      if (n <= 0) return result;
      off += static_cast<size_t>(n);
    }
  }
  {
    const unsigned char* data = bgr.data;
    size_t off = 0;
    while (off < bytes) {
      const ssize_t n = write(pipe_in_, data + off, bytes - off);
      if (n <= 0) return result;
      off += static_cast<size_t>(n);
    }
  }

  // Read one JSON line from the service (with a short timeout).
  std::string line;
  char ch;
  const int kTimeoutMs = 500;
  for (;;) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(pipe_out_, &rfds);
    timeval tv;
    tv.tv_sec = kTimeoutMs / 1000;
    tv.tv_usec = (kTimeoutMs % 1000) * 1000;
    const int sel = select(pipe_out_ + 1, &rfds, nullptr, nullptr, &tv);
    if (sel <= 0) break;  // timeout or error
    const ssize_t n = read(pipe_out_, &ch, 1);
    if (n != 1) break;
    if (ch == '\n') break;
    line.push_back(ch);
    if (line.size() > 1 << 20) break;
  }
  return parseResponse(line, scale);
}

// ---------------- factory ----------------

std::unique_ptr<IDetector> makeDetector(Backend backend,
                                        const std::string& nn_model_path,
                                        vision::LightColor color,
                                        const vision::MorphologyParams& morph,
                                        const vision::DetectionParams& det) {
  if (backend == Backend::NeuralNetwork) {
    auto nn = std::make_unique<NNDetector>(nn_model_path);
    if (nn->ready()) return nn;
    // Fall back to traditional if the model cannot be loaded.
    return std::make_unique<VisionDetector>(color, morph, det);
  }
  return std::make_unique<VisionDetector>(color, morph, det);
}

}  // namespace detect
