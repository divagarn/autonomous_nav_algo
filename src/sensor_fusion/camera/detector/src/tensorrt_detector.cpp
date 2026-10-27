/**
 * @file tensorrt_detector.cpp
 * @brief YOLOv8/11 TensorRT engine inference (Jetson Orin, TensorRT 8.6 / 10 API).
 *
 * Build the engine on the target:
 *   /usr/src/tensorrt/bin/trtexec --onnx=yolov8n.onnx --saveEngine=yolov8n_fp16.engine --fp16
 */
#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "object_detection/detector.hpp"

namespace object_detection
{

namespace
{

/// Throws std::runtime_error when a CUDA call fails.
#define CUDA_CHECK(x) \
  do { cudaError_t e = (x); if (e != cudaSuccess) { \
      throw std::runtime_error(std::string("CUDA: ") + cudaGetErrorString(e)); } } while (0)

/// TensorRT logger: warnings and errors to stderr.
class TrtLogger : public nvinfer1::ILogger
{
  /**
   * @param[in] s    severity
   * @param[in] msg  message
   */
  void log(Severity s, const char * msg) noexcept override
  {
    if (s <= Severity::kWARNING) {std::cerr << "[TensorRT] " << msg << "\n";}
  }
};

/// YOLO TensorRT engine with one CUDA stream and fixed device buffers.
class TensorRtDetector : public Detector
{
public:
  /**
   * @brief Deserialises the engine and allocates one device buffer per I/O tensor.
   * @param[in] cfg  model_path (engine), thresholds; input_size is taken from the engine
   * @throws std::runtime_error if the engine cannot be read or CUDA fails
   */
  explicit TensorRtDetector(const DetectorConfig & cfg) : cfg_(cfg)
  {
    std::ifstream f(cfg.model_path, std::ios::binary);
    if (!f) {throw std::runtime_error("cannot open engine " + cfg.model_path);}
    std::vector<char> data((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    engine_.reset(runtime_->deserializeCudaEngine(data.data(), data.size()));
    if (!engine_) {throw std::runtime_error("failed to deserialize engine " + cfg.model_path);}
    ctx_.reset(engine_->createExecutionContext());
    CUDA_CHECK(cudaStreamCreate(&stream_));
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char * name = engine_->getIOTensorName(i);
      const auto dims = engine_->getTensorShape(name);
      std::size_t n = 1;
      for (int k = 0; k < dims.nbDims; ++k) {n *= static_cast<std::size_t>(dims.d[k]);}
      void * dev = nullptr;
      CUDA_CHECK(cudaMalloc(&dev, n * sizeof(float)));
      ctx_->setTensorAddress(name, dev);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        in_dev_ = dev;
        in_count_ = n;
        cfg_.input_size = static_cast<int>(dims.d[dims.nbDims - 1]);
      } else {
        out_dev_ = dev;
        out_count_ = n;
        out_d1_ = static_cast<int>(dims.d[1]);
        out_d2_ = static_cast<int>(dims.d[2]);
      }
      bufs_.push_back(dev);
    }
    out_host_.resize(out_count_);
  }
  /// @brief Frees the device buffers and the stream.
  ~TensorRtDetector() override
  {
    for (void * b : bufs_) {cudaFree(b);}
    if (stream_) {cudaStreamDestroy(stream_);}
  }
  /// @brief Backend name.
  std::string name() const override {return "tensorrt";}

  /**
   * @brief Letterbox, host -> device copy, enqueueV3, device -> host copy, decode + NMS.
   * @param[in] bgr  BGR8 image
   * @return boxes in image pixels
   * @throws std::runtime_error on a CUDA / TensorRT failure
   */
  std::vector<Detection> detect(const cv::Mat & bgr, int64_t) override
  {
    Letterbox lb;
    cv::Mat blob = yoloPreprocess(bgr, cfg_, cfg_.input_size, lb);
    CUDA_CHECK(cudaMemcpyAsync(in_dev_, blob.ptr<float>(), in_count_ * sizeof(float),
      cudaMemcpyHostToDevice, stream_));
    if (!ctx_->enqueueV3(stream_)) {throw std::runtime_error("TensorRT enqueueV3 failed");}
    CUDA_CHECK(cudaMemcpyAsync(out_host_.data(), out_dev_, out_count_ * sizeof(float),
      cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaStreamSynchronize(stream_));
    return yoloPostprocess(out_host_.data(), out_d1_, out_d2_, lb, cfg_, bgr.cols, bgr.rows);
  }

private:
  DetectorConfig cfg_;
  TrtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> ctx_;
  cudaStream_t stream_{nullptr};
  std::vector<void *> bufs_;
  void * in_dev_{nullptr};
  void * out_dev_{nullptr};
  std::size_t in_count_{0}, out_count_{0};
  int out_d1_{0}, out_d2_{0};
  std::vector<float> out_host_;
};

}  // namespace

/// @brief Factory used by makeDetector().
std::unique_ptr<Detector> makeTensorRtDetector(const DetectorConfig & cfg)
{
  return std::make_unique<TensorRtDetector>(cfg);
}

}  // namespace object_detection
