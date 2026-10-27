/**
 * @file lane_segmenter.cpp
 * @brief replay backend, shared pre / post-processing and the segmentation factory.
 */
#include "object_detection/lane_segmenter.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <stdexcept>

#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>

namespace object_detection
{

#ifdef HAVE_ONNXRUNTIME
/// @brief ONNX Runtime backend (onnxruntime_lane.cpp).
std::unique_ptr<LaneSegmenter> makeOnnxRuntimeLaneSegmenter(const LaneConfig & cfg);
#endif
#ifdef HAVE_TENSORRT
/// @brief TensorRT backend (tensorrt_lane.cpp).
std::unique_ptr<LaneSegmenter> makeTensorRtLaneSegmenter(const LaneConfig & cfg);
#endif

namespace
{
/// Replays pre-computed masks (<dir>/<stamp_ns>.png), matched to the image by stamp.
class ReplayLaneSegmenter : public LaneSegmenter
{
public:
  /**
   * @brief Indexes the mask files of the directory.
   * @param[in] cfg  replay_dir, replay_max_time_diff
   * @throws std::runtime_error if the directory is missing or has no masks
   */
  explicit ReplayLaneSegmenter(const LaneConfig & cfg) : cfg_(cfg)
  {
    namespace fs = std::filesystem;
    if (!fs::is_directory(cfg.replay_dir)) {
      throw std::runtime_error("lane replay directory '" + cfg.replay_dir + "' not found "
        "(bag_to_detections_json --lane-model ...)");
    }
    for (const auto & e : fs::directory_iterator(cfg.replay_dir)) {
      if (e.path().extension() != ".png") {continue;}
      char * end = nullptr;
      const std::string stem = e.path().stem().string();
      const long long ns = std::strtoll(stem.c_str(), &end, 10);
      if (end && *end == '\0') {files_[ns] = e.path().string();}
    }
    if (files_.empty()) {throw std::runtime_error("no masks in '" + cfg.replay_dir + "'");}
    std::cerr << "[lane replay] " << files_.size() << " masks in " << cfg.replay_dir << "\n";
  }
  /// @brief Backend name.
  std::string name() const override {return "replay";}
  /// @brief replay works on stamps only.
  bool needsImage() const override {return false;}
  /**
   * @brief Mask of the nearest stamp.
   * @param[in] stamp_ns  image stamp [ns]
   * @return mask, empty if none within replay_max_time_diff
   */
  cv::Mat segment(const cv::Mat &, int64_t stamp_ns) override
  {
    auto it = files_.lower_bound(stamp_ns);
    int64_t best_dt = INT64_MAX;
    const std::string * best = nullptr;
    for (auto c : {it, it == files_.begin() ? files_.end() : std::prev(it)}) {
      if (c == files_.end()) {continue;}
      const int64_t dt = std::llabs(c->first - stamp_ns);
      if (dt < best_dt) {best_dt = dt; best = &c->second;}
    }
    if (!best || best_dt > static_cast<int64_t>(cfg_.replay_max_time_diff * 1e9)) {return {};}
    return cv::imread(*best, cv::IMREAD_GRAYSCALE);
  }

private:
  LaneConfig cfg_;
  std::map<int64_t, std::string> files_;
};
}  // namespace

const std::vector<std::string> & laneClassNames()
{
  static const std::vector<std::string> n = {"background", "lane", "road"};
  return n;
}

// (pixel - 127) / 127 = blobFromImage(scale 1/127, mean 127), RGB like the reference (imageio).
cv::Mat lanePreprocess(const cv::Mat & bgr, const LaneConfig & cfg)
{
  return cv::dnn::blobFromImage(bgr, 1.0 / 127.0, cv::Size(cfg.input_width, cfg.input_height),
           cv::Scalar(127, 127, 127), true, false, CV_32F);
}

// Class ids are 0..2: element-wise narrowing.
cv::Mat laneMaskFromTensor(const void * data, bool int64, const LaneConfig & cfg)
{
  cv::Mat m(cfg.input_height, cfg.input_width, CV_8UC1);
  const std::size_t n = m.total();
  for (std::size_t i = 0; i < n; ++i) {
    const int64_t v = int64 ? static_cast<const int64_t *>(data)[i] : static_cast<const int32_t *>(data)[i];
    m.data[i] = static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
  }
  return m;
}

std::unique_ptr<LaneSegmenter> makeLaneSegmenter(const LaneConfig & cfg)
{
  if (cfg.backend == "replay") {return std::make_unique<ReplayLaneSegmenter>(cfg);}
  if (cfg.backend == "onnxruntime") {
#ifdef HAVE_ONNXRUNTIME
    return makeOnnxRuntimeLaneSegmenter(cfg);
#else
    throw std::runtime_error("built without ONNX Runtime (set ONNXRUNTIME_ROOT when building)");
#endif
  }
  if (cfg.backend == "tensorrt") {
#ifdef HAVE_TENSORRT
    return makeTensorRtLaneSegmenter(cfg);
#else
    throw std::runtime_error("built without TensorRT (build on the Orin / with TensorRT installed)");
#endif
  }
  throw std::runtime_error("unknown lane backend '" + cfg.backend + "'");
}

}  // namespace object_detection
