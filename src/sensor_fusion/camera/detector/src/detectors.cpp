/**
 * @file detectors.cpp
 * @brief json_replay + opencv_dnn backends and the detector factory.
 */
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>

#include <opencv2/core/persistence.hpp>
#include <opencv2/dnn.hpp>

#include "object_detection/detector.hpp"

namespace object_detection
{

#ifdef HAVE_ONNXRUNTIME
/// @brief ONNX Runtime backend (onnxruntime_detector.cpp).
std::unique_ptr<Detector> makeOnnxRuntimeDetector(const DetectorConfig & cfg);
#endif
#ifdef HAVE_TENSORRT
/// @brief TensorRT backend (tensorrt_detector.cpp).
std::unique_ptr<Detector> makeTensorRtDetector(const DetectorConfig & cfg);
#endif

namespace
{

// ---------------------------------------------------------------- json_replay
// File format (written by bag_to_detections_json):
// {"image_width": W, "image_height": H, "rectified": true,
//  "frames": [{"stamp_sec": s, "stamp_nanosec": ns, "frame_index": i,
//              "detections": [{"bbox": [x1, y1, x2, y2], "score": p,
//                              "class_id": k, "class_name": "person"}]}]}
/// Replays pre-computed detections, matched to the image by stamp.
class JsonReplayDetector : public Detector
{
public:
  /**
   * @brief Loads all frames of the JSON file.
   * @param[in] cfg  json_file, json_expect_rectified, thresholds, class filter
   * @throws std::runtime_error if the file is missing, empty or made on the other image type
   */
  explicit JsonReplayDetector(const DetectorConfig & cfg) : cfg_(cfg)
  {
    cv::FileStorage fs(cfg.json_file, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
    if (!fs.isOpened()) {throw std::runtime_error("cannot open detections JSON '" + cfg.json_file + "'");}
    const bool rectified = !fs["rectified"].empty() && static_cast<int>(fs["rectified"]) != 0;
    if (rectified != cfg.json_expect_rectified) {
      throw std::runtime_error("'" + cfg.json_file + "' was made on " + (rectified ? "rectified" : "raw") +
        " images but the node " + (cfg.json_expect_rectified ? "rectifies" : "does not rectify") +
        " (re-run precompute_detections.launch.xml)");
    }
    const cv::FileNode frames = fs["frames"];
    for (const auto & f : frames) {
      int64_t stamp = 0;
      if (!f["stamp_sec"].empty()) {
        stamp = static_cast<int64_t>(static_cast<int>(f["stamp_sec"])) * 1000000000LL +
          static_cast<int64_t>(static_cast<int>(f["stamp_nanosec"]));
      } else {
        stamp = static_cast<int64_t>(static_cast<double>(f["stamp_ns"]));
      }
      std::vector<Detection> dets;
      for (const auto & d : f["detections"]) {
        Detection x;
        const cv::FileNode b = d["bbox"];
        x.x1 = static_cast<float>(b[0]);
        x.y1 = static_cast<float>(b[1]);
        x.x2 = static_cast<float>(b[2]);
        x.y2 = static_cast<float>(b[3]);
        x.score = static_cast<float>(d["score"]);
        x.class_id = static_cast<int>(d["class_id"]);
        x.class_name = static_cast<std::string>(d["class_name"]);
        if (x.score < cfg.conf_threshold) {continue;}
        if (!cfg.keep_classes.empty() && !cfg.keep_classes.count(x.class_name)) {continue;}
        dets.push_back(x);
      }
      frames_[stamp] = std::move(dets);
    }
    if (frames_.empty()) {throw std::runtime_error("no frames in '" + cfg.json_file + "'");}
    std::cerr << "[json_replay] loaded " << frames_.size() << " frames from " << cfg.json_file << "\n";
  }
  /// @brief Backend name.
  std::string name() const override {return "json_replay";}
  /// @brief json_replay works on stamps only.
  bool needsImage() const override {return false;}
  /**
   * @brief Detections of the frame nearest to the stamp.
   * @param[in] stamp_ns  image stamp [ns]
   * @return detections, empty if no frame within json_max_time_diff
   */
  std::vector<Detection> detect(const cv::Mat &, int64_t stamp_ns) override
  {
    auto it = frames_.lower_bound(stamp_ns);
    int64_t best_dt = INT64_MAX;
    const std::vector<Detection> * best = nullptr;
    for (auto c : {it, it == frames_.begin() ? frames_.end() : std::prev(it)}) {
      if (c == frames_.end()) {continue;}
      const int64_t dt = std::llabs(c->first - stamp_ns);
      if (dt < best_dt) {best_dt = dt; best = &c->second;}
    }
    if (!best || best_dt > static_cast<int64_t>(cfg_.json_max_time_diff * 1e9)) {return {};}
    return *best;
  }

private:
  DetectorConfig cfg_;
  std::map<int64_t, std::vector<Detection>> frames_;
};

// ---------------------------------------------------------------- opencv_dnn
/// YOLO ONNX through cv::dnn (optional CUDA target).
class OpenCvDnnDetector : public Detector
{
public:
  /**
   * @param[in] cfg  model_path, use_gpu, thresholds
   */
  explicit OpenCvDnnDetector(const DetectorConfig & cfg) : cfg_(cfg)
  {
    net_ = cv::dnn::readNetFromONNX(cfg.model_path);   // YOLOv8/11 needs OpenCV >= 4.7
    if (cfg.use_gpu) {
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    }
  }
  /// @brief Backend name.
  std::string name() const override {return "opencv_dnn";}
  /**
   * @brief Letterbox, forward pass, decode + NMS.
   * @param[in] bgr  BGR8 image
   * @return boxes in image pixels
   */
  std::vector<Detection> detect(const cv::Mat & bgr, int64_t) override
  {
    Letterbox lb;
    net_.setInput(yoloPreprocess(bgr, cfg_, cfg_.input_size, lb));
    cv::Mat out = net_.forward();
    return yoloPostprocess(out.ptr<float>(), out.size[1], out.size[2], lb, cfg_, bgr.cols, bgr.rows);
  }

private:
  DetectorConfig cfg_;
  cv::dnn::Net net_;
};

}  // namespace

// Backends depend on the libraries found at build time.
std::vector<std::string> availableBackends()
{
  std::vector<std::string> b{"json_replay", "opencv_dnn"};
#ifdef HAVE_ONNXRUNTIME
  b.push_back("onnxruntime");
#endif
#ifdef HAVE_TENSORRT
  b.push_back("tensorrt");
#endif
  return b;
}

// Backend by name.
std::unique_ptr<Detector> makeDetector(const DetectorConfig & cfg)
{
  if (cfg.backend == "json_replay") {return std::make_unique<JsonReplayDetector>(cfg);}
  if (cfg.backend == "opencv_dnn") {return std::make_unique<OpenCvDnnDetector>(cfg);}
  if (cfg.backend == "onnxruntime") {
#ifdef HAVE_ONNXRUNTIME
    return makeOnnxRuntimeDetector(cfg);
#else
    throw std::runtime_error("built without ONNX Runtime (set ONNXRUNTIME_ROOT when building)");
#endif
  }
  if (cfg.backend == "tensorrt") {
#ifdef HAVE_TENSORRT
    return makeTensorRtDetector(cfg);
#else
    throw std::runtime_error("built without TensorRT (build on the Orin / with TensorRT installed)");
#endif
  }
  throw std::runtime_error("unknown detector backend '" + cfg.backend + "'");
}

}  // namespace object_detection
