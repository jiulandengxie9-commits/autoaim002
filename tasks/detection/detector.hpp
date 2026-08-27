#pragma once

#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

#include "tasks/vision/vision_processing.hpp"

namespace detect {

// Detector backend selector.
enum class Backend { Traditional, NeuralNetwork };

// Base interface for armor detectors. Both the traditional morphological
// pipeline and the YOLO neural-network backend implement it, so the caller
// can switch at runtime with --detector traditional|nn.
class IDetector {
 public:
  virtual ~IDetector() = default;

  // Detect armors in a BGR image. Returns the same DetectionResult structure
  // used by the traditional pipeline (armors with 4 vertices, light bars).
  virtual vision::DetectionResult detect(const cv::Mat& bgr) = 0;

  // True if the backend is ready (e.g. NN model loaded).
  virtual bool ready() const = 0;

  // Human-readable backend name for logging.
  virtual std::string name() const = 0;
};

// Traditional morphological + light-bar/armor-pair detector.
class VisionDetector : public IDetector {
 public:
  VisionDetector(vision::LightColor color, const vision::MorphologyParams& morph,
                 const vision::DetectionParams& det)
      : color_(color), morph_(morph), det_(det) {}

  vision::DetectionResult detect(const cv::Mat& bgr) override;
  bool ready() const override { return true; }
  std::string name() const override { return "traditional"; }

 private:
  vision::LightColor color_;
  vision::MorphologyParams morph_;
  vision::DetectionParams det_;
};

// YOLO neural-network detector backed by an OpenVINO Python subprocess service
// (tools/ov_armor_service.py). The C++ OpenVINO runtime produces wrong
// results on this INT8 model on this host, so we use the proven Python path.
// Falls back gracefully (ready() == false) if the service cannot start.
class NNDetector : public IDetector {
 public:
  NNDetector(const std::string& model_path, double score_threshold = 0.5,
             double nms_threshold = 0.3);

  ~NNDetector() override;

  vision::DetectionResult detect(const cv::Mat& bgr) override;
  bool ready() const override { return ready_; }
  std::string name() const override { return "neural-network"; }

 private:
  std::string model_path_;
  double score_threshold_;
  double nms_threshold_;
  int pipe_in_ = -1;   // write BGR frames to the Python service
  int pipe_out_ = -1;  // read JSON detections from the service
  pid_t child_ = 0;
  bool ready_ = false;

  bool startService();
  vision::DetectionResult parseResponse(const std::string& json_line,
                                        double scale);
};

// Factory: create a detector for the given backend.
std::unique_ptr<IDetector> makeDetector(Backend backend,
                                        const std::string& nn_model_path,
                                        vision::LightColor color,
                                        const vision::MorphologyParams& morph,
                                        const vision::DetectionParams& det);

}  // namespace detect
