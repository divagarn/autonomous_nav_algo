/**
 * @file lane_segmenter.hpp
 * @brief Lane / road segmentation of the camera image + factory.
 *
 * Backends:
 *   - replay       : pre-computed masks <dir>/<stamp_ns>.png (PC, bag_to_detections_json --lane-model)
 *   - onnxruntime  : ONNX on CPU (or CUDA EP) via ONNX Runtime
 *   - tensorrt     : TensorRT engine (Jetson Orin)
 * Model: input RGB (pixel - 127) / 127, plain resize to the model input size (taken from the model);
 * the output used is the per-pixel integer class map (0 background, 1 lane, 2 road), argmax inside
 * the model. models/lane_seg_up.onnx (lane + road, default) and models/road_bisenetv2_ade20k.onnx
 * (open source, road only) both have this interface.
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace object_detection
{

/// Segmentation parameters (lane.* in object_detection_<platform>.yaml).
struct LaneConfig
{
  std::string backend{"replay"};
  std::string model_path;
  std::string replay_dir;                // replay: directory of <stamp_ns>.png masks
  double replay_max_time_diff{0.02};     // s, stamp matching tolerance
  int input_width{768};
  int input_height{512};
  bool use_gpu{false};
};

/// Interface of every segmentation backend.
class LaneSegmenter
{
public:
  virtual ~LaneSegmenter() = default;
  /// @brief Backend name.
  virtual std::string name() const = 0;
  /// @brief Does this backend need the image pixels? (replay does not)
  virtual bool needsImage() const {return true;}
  /**
   * @brief Class mask of one image.
   * @param[in] bgr       BGR8 image (rectified, as for detection)
   * @param[in] stamp_ns  image stamp [ns]
   * @return CV_8UC1 class per pixel at the model resolution (input_width x input_height);
   *         empty if there is none (replay without a mask for this stamp)
   */
  virtual cv::Mat segment(const cv::Mat & bgr, int64_t stamp_ns) = 0;
};

/**
 * @brief Creates the segmentation backend.
 * @param[in] cfg  configuration
 * @return segmenter
 * @throws std::runtime_error for an unknown / not compiled backend or a load failure
 */
std::unique_ptr<LaneSegmenter> makeLaneSegmenter(const LaneConfig & cfg);
/// @brief Class names of the mask values (0 background, 1 lane, 2 road).
const std::vector<std::string> & laneClassNames();
/**
 * @brief Model input: plain resize, RGB, (pixel - 127) / 127, NCHW float.
 * @param[in] bgr  BGR8 image
 * @param[in] cfg  input size
 * @return 1x3xHxW blob
 */
cv::Mat lanePreprocess(const cv::Mat & bgr, const LaneConfig & cfg);
/**
 * @brief Integer class map (int32 or int64 elements) -> CV_8UC1 mask.
 * @param[in] data      output tensor
 * @param[in] int64     true for int64 elements, false for int32
 * @param[in] cfg       input size (= mask size)
 */
cv::Mat laneMaskFromTensor(const void * data, bool int64, const LaneConfig & cfg);

}  // namespace object_detection
