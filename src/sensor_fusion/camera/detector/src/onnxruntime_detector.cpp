/**
 * @file onnxruntime_detector.cpp
 * @brief YOLOv8/11 ONNX inference with ONNX Runtime (CPU, or CUDA EP if use_gpu and available).
 */
#include <onnxruntime_cxx_api.h>

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "object_detection/detector.hpp"

namespace object_detection
{

namespace
{
/// YOLO ONNX through ONNX Runtime.
class OnnxRuntimeDetector : public Detector
{
public:
  /**
   * @brief Creates the session (half of the CPU cores, full graph optimisation).
   * @param[in] cfg  model_path, use_gpu, thresholds
   * @throws Ort::Exception if the model cannot be loaded
   */
  explicit OnnxRuntimeDetector(const DetectorConfig & cfg)
  : cfg_(cfg), env_(ORT_LOGGING_LEVEL_WARNING, "yolo")
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
    out_name_ = session_->GetOutputNameAllocated(0, alloc).get();
  }
  /// @brief Backend name.
  std::string name() const override {return "onnxruntime";}

  /**
   * @brief Letterbox, session run, decode + NMS.
   * @param[in] bgr  BGR8 image
   * @return boxes in image pixels
   */
  std::vector<Detection> detect(const cv::Mat & bgr, int64_t) override
  {
    Letterbox lb;
    cv::Mat blob = yoloPreprocess(bgr, cfg_, cfg_.input_size, lb);
    const std::array<int64_t, 4> shape{1, 3, cfg_.input_size, cfg_.input_size};
    auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value in = Ort::Value::CreateTensor<float>(
      mem, blob.ptr<float>(), blob.total(), shape.data(), shape.size());
    const char * in_names[] = {in_name_.c_str()};
    const char * out_names[] = {out_name_.c_str()};
    auto out = session_->Run(Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 1);
    const auto oshape = out[0].GetTensorTypeAndShapeInfo().GetShape();   // [1, 84, 8400]
    return yoloPostprocess(out[0].GetTensorData<float>(), static_cast<int>(oshape[1]),
             static_cast<int>(oshape[2]), lb, cfg_, bgr.cols, bgr.rows);
  }

private:
  DetectorConfig cfg_;
  Ort::Env env_;
  std::unique_ptr<Ort::Session> session_;
  std::string in_name_, out_name_;
};
}  // namespace

/// @brief Factory used by makeDetector().
std::unique_ptr<Detector> makeOnnxRuntimeDetector(const DetectorConfig & cfg)
{
  return std::make_unique<OnnxRuntimeDetector>(cfg);
}

}  // namespace object_detection
