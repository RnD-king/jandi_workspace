// Test-only inference backend: no model, CUDA initialization, or camera hardware.
#include <vision/yolo_trt_engine.hpp>
#include <vision_core/config_loader.hpp>

namespace vision {
YoloTrtEngine::YoloTrtEngine(const std::string&, int, int, float) {}
YoloTrtEngine::~YoloTrtEngine() = default;
bool YoloTrtEngine::Infer(const cv::Mat&, std::vector<Detection>& detections) {
  const auto config = vision_core::LoadDefaultAlgorithmConfig();
  detections = {
      {{180, 120, 240, 200}, .99f, config.object_targets.backboard_class_id},
      {{30, 200, 60, 60}, .99f, config.object_targets.ball_class_id},
      {{450, 180, 80, 70}, .99f, config.object_targets.goal_class_id},
      {{30, 350, 80, 60}, .99f, config.object_targets.hurdle_class_id},
      {{450, 350, 100, 40}, .99f, config.line_detection.class_id}};
  return true;
}
}  // namespace vision
