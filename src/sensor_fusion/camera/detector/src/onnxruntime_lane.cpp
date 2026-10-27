/**
 * @file onnxruntime_lane.cpp
 * @brief Lane / road segmentation with ONNX Runtime (CPU, or CUDA EP if use_gpu and available).
 */
#include <onnxruntime_cxx_api.h>

#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "object_detection/lane_segmenter.hpp"

namespace object_detection
{

namespace
{
/// Segmentation model through ONNX Runtime; only the class-map output is fetched.
class OnnxRuntimeLaneSegmenter : public LaneSegmenter
{
public:
  /**
   * @brief Creates the session and finds the integer class-map output.
   * @param[in] cfg  model_path, use_gpu; the input size is taken from the model if fixed there
   * @throws std::runtime_error if the model has no integer [1, 1, H, W] output
   */
  explicit OnnxRuntimeLaneSegmenter(const LaneConfig & cfg)
  : cfg_(cfg), env_(ORT_LOGGING_LEVEL_WARNING, "lane")
  {
    Ort::SessionOptions so;
    so.SetIntraOpNumThreads(static_cast<int>(std::max(1u, std::thread::hardware_concurrency() / 2)));
    so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    if (cfg.use_gpu) {
      try {
        OrtCUDAProviderOptions cuda{};
        so.AppendExecutionProvider_CUDA(cuda);
      } catch (const Ort::Exception &) {
        // CPU-only ONNX Runtime build: fall back silently
      }
    }
    session_ = std::make_unique<Ort::Session>(env_, cfg.model_path.c_str(), so);
    Ort::AllocatorWithDefaultOptions alloc;
    in_name_ = session_->GetInputNameAllocated(0, alloc).get();
    {   // input size from the model when it is fixed (as the TensorRT backend)
      const Ort::TypeInfo in_info = session_->GetInputTypeInfo(0);
      const auto dims = in_info.GetTensorTypeAndShapeInfo().GetShape();
      if (dims.size() == 4 && dims[2] > 0 && dims[3] > 0) {
        cfg_.input_height = static_cast<int>(dims[2]);
        cfg_.input_width = static_cast<int>(dims[3]);
      }
    }
    for (std::size_t i = 0; i < session_->GetOutputCount(); ++i) {
      const Ort::TypeInfo type_info = session_->GetOutputTypeInfo(i);   // owns what the view below reads
      const auto t = type_info.GetTensorTypeAndShapeInfo().GetElementType();
      if (t == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64 || t == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32) {
        out_name_ = session_->GetOutputNameAllocated(i, alloc).get();
        int64_ = t == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64;
      }
    }
    if (out_name_.empty()) {throw std::runtime_error("no integer class-map output in " + cfg.model_path);}
  }
  /// @brief Backend name.
  std::string name() const override {return "onnxruntime";}
  /**
   * @brief Resize + normalise, session run, class map -> mask.
   * @param[in] bgr  BGR8 image
   * @return CV_8UC1 mask at the model resolution
   */
  cv::Mat segment(const cv::Mat & bgr, int64_t) override
  {
    cv::Mat blob = lanePreprocess(bgr, cfg_);
    const std::array<int64_t, 4> shape{1, 3, cfg_.input_height, cfg_.input_width};
    auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value in = Ort::Value::CreateTensor<float>(mem, blob.ptr<float>(), blob.total(), shape.data(), shape.size());
    const char * in_names[] = {in_name_.c_str()};
    const char * out_names[] = {out_name_.c_str()};
    auto out = session_->Run(Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 1);
    return laneMaskFromTensor(out[0].GetTensorRawData(), int64_, cfg_);
  }

private:
  LaneConfig cfg_;
  Ort::Env env_;
  std::unique_ptr<Ort::Session> session_;
  std::string in_name_, out_name_;
  bool int64_{true};
};
}  // namespace

/// @brief Factory used by makeLaneSegmenter().
std::unique_ptr<LaneSegmenter> makeOnnxRuntimeLaneSegmenter(const LaneConfig & cfg)
{
  return std::make_unique<OnnxRuntimeLaneSegmenter>(cfg);
}

}  // namespace object_detection
