/**
 * @file detector.hpp
 * @brief Detector interface + factory.
 *
 * Backends:
 *   - json_replay  : pre-computed detections from a JSON file (PC, offline)
 *   - onnxruntime  : YOLO ONNX on CPU (or CUDA EP) via ONNX Runtime
 *   - opencv_dnn   : YOLO ONNX via cv::dnn (needs OpenCV >= 4.7)
 *   - tensorrt     : YOLO TensorRT engine (Jetson Orin)
 * Models (model_type): yolox (18 classes, default, models/yolox_s_18class.onnx) | yolov8 (COCO).
 */
#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "object_detection/types.hpp"

namespace object_detection
{

/// Detector parameters (detector.* in object_detection_<platform>.yaml).
struct DetectorConfig
{
  std::string backend{"json_replay"};
  std::string json_file;
  double json_max_time_diff{0.02};       // s, stamp matching tolerance
  bool json_expect_rectified{true};      // the JSON boxes must be on the same (rectified) image
  std::string model_path;
  // yolox: letterbox top-left, BGR 0..255, output [N, 4 + 1 + C] (objectness x class score) |
  // yolov8: letterbox centred, RGB / 255, output [4 + C, N]
  std::string model_type{"yolox"};
  std::vector<std::string> class_names;  // index = model class; empty = the model type's default list
  int input_size{640};
  float conf_threshold{0.3f};
  float nms_threshold{0.5f};
  bool use_gpu{false};                   // CUDA EP / DNN_TARGET_CUDA where available
  std::set<std::string> keep_classes;    // empty = all
};

/// Interface of every detector backend.
class Detector
{
public:
  virtual ~Detector() = default;
  /// @brief Backend name.
  virtual std::string name() const = 0;
  /// @brief Does this backend need the image pixels? (json_replay does not)
  virtual bool needsImage() const {return true;}
  /**
   * @brief Detects objects in one image.
   * @param[in] bgr       BGR8 image (may be empty if needsImage() is false)
   * @param[in] stamp_ns  image stamp [ns] (json_replay matches on it)
   * @return boxes in image pixels
   */
  virtual std::vector<Detection> detect(const cv::Mat & bgr, int64_t stamp_ns) = 0;
};

/**
 * @brief Creates the configured backend.
 * @param[in] cfg  detector configuration
 * @return detector
 * @throws std::runtime_error for an unknown / not compiled backend or a load failure
 */
std::unique_ptr<Detector> makeDetector(const DetectorConfig & cfg);
/// @brief Backends compiled into this build.
std::vector<std::string> availableBackends();

// ---- shared YOLO helpers ----
/// Letterbox transform: model pixel = image pixel * scale + pad.
struct Letterbox
{
  float scale{1};
  int pad_x{0}, pad_y{0};
};
/// @brief Class names of the model (cfg.class_names, else the model type's default list).
const std::vector<std::string> & classNames(const DetectorConfig & cfg);
/**
 * @brief BGR image -> NCHW float blob letterboxed to size x size, as the model type expects
 *        (yolox: top-left, BGR 0..255; yolov8: centred, RGB [0,1]; padding 114).
 * @param[in]  bgr   input image
 * @param[in]  cfg   model_type
 * @param[in]  size  model input size [px]
 * @param[out] lb    letterbox scale and padding
 * @return 1x3xsize x size blob
 */
cv::Mat yoloPreprocess(const cv::Mat & bgr, const DetectorConfig & cfg, int size, Letterbox & lb);
/**
 * @brief Decodes the head output + per-class NMS. yolov8: [1, 4 + C, N] (or [1, N, 4 + C]);
 *        yolox: [1, N, 4 + 1 + C] raw grid outputs (decoded with strides 8 / 16 / 32).
 * @param[in] data   raw output tensor
 * @param[in] dim1   first non-batch dimension
 * @param[in] dim2   second non-batch dimension
 * @param[in] lb     letterbox used in preprocessing
 * @param[in] cfg    thresholds and class filter
 * @param[in] img_w  original image width [px]
 * @param[in] img_h  original image height [px]
 * @return boxes in original image pixels
 */
std::vector<Detection> yoloPostprocess(
  const float * data, int dim1, int dim2, const Letterbox & lb, const DetectorConfig & cfg,
  int img_w, int img_h);

}  // namespace object_detection
