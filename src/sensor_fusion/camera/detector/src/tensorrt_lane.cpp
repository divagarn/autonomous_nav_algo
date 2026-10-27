/**
 * @file tensorrt_lane.cpp
 * @brief Lane / road segmentation TensorRT engine (Jetson Orin, TensorRT 8.6 / 10 API).
 *
 * Build the engine on the target (the int64 class map becomes int32 in TensorRT 8):
 *   /usr/src/tensorrt/bin/trtexec --onnx=lane_seg_up.onnx --saveEngine=lane_seg_up_fp16.engine --fp16
 */
#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "object_detection/lane_segmenter.hpp"

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

/**
 * @brief Bytes per element of a TensorRT data type.
 * @param[in] t  data type
 * @return size [B]
 */
std::size_t elementSize(nvinfer1::DataType t)
{
  switch (t) {
    case nvinfer1::DataType::kHALF: return 2;
    case nvinfer1::DataType::kINT8: return 1;
    case nvinfer1::DataType::kBOOL: return 1;
#if NV_TENSORRT_MAJOR >= 10
    case nvinfer1::DataType::kINT64: return 8;
#endif
    default: return 4;   // kFLOAT, kINT32
  }
}

/// Segmentation engine: one device buffer per I/O tensor, only the class map is copied back.
class TensorRtLaneSegmenter : public LaneSegmenter
{
public:
  /**
   * @brief Deserialises the engine, allocates the buffers and finds the class-map output.
   * @param[in] cfg  model_path (engine); the input size is taken from the engine
   * @throws std::runtime_error if the engine cannot be read, has no integer output, or CUDA fails
   */
  explicit TensorRtLaneSegmenter(const LaneConfig & cfg) : cfg_(cfg)
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
      const auto type = engine_->getTensorDataType(name);
      std::size_t n = 1;
      for (int k = 0; k < dims.nbDims; ++k) {n *= static_cast<std::size_t>(dims.d[k]);}
      void * dev = nullptr;
      CUDA_CHECK(cudaMalloc(&dev, n * elementSize(type)));
      ctx_->setTensorAddress(name, dev);
      bufs_.push_back(dev);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        in_dev_ = dev;
        in_count_ = n;
        cfg_.input_height = static_cast<int>(dims.d[2]);
        cfg_.input_width = static_cast<int>(dims.d[3]);
        continue;
      }
      // the class map: integer output (int32 in TensorRT 8, int32 / int64 in 10)
      const bool is_int = type == nvinfer1::DataType::kINT32
#if NV_TENSORRT_MAJOR >= 10
        || type == nvinfer1::DataType::kINT64
#endif
      ;
      if (is_int) {
        out_dev_ = dev;
        out_bytes_ = n * elementSize(type);
        int64_ = elementSize(type) == 8;
      }
    }
    if (!out_dev_) {throw std::runtime_error("no integer class-map output in " + cfg.model_path);}
    out_host_.resize(out_bytes_);
  }
  /// @brief Frees the device buffers and the stream.
  ~TensorRtLaneSegmenter() override
  {
    for (void * b : bufs_) {cudaFree(b);}
    if (stream_) {cudaStreamDestroy(stream_);}
  }
  /// @brief Backend name.
  std::string name() const override {return "tensorrt";}
  /**
   * @brief Resize + normalise, host -> device, enqueueV3, class map device -> host.
   * @param[in] bgr  BGR8 image
   * @return CV_8UC1 mask at the model resolution
   * @throws std::runtime_error on a CUDA / TensorRT failure
   */
  cv::Mat segment(const cv::Mat & bgr, int64_t) override
  {
    cv::Mat blob = lanePreprocess(bgr, cfg_);
    CUDA_CHECK(cudaMemcpyAsync(in_dev_, blob.ptr<float>(), in_count_ * sizeof(float),
      cudaMemcpyHostToDevice, stream_));
    if (!ctx_->enqueueV3(stream_)) {throw std::runtime_error("TensorRT enqueueV3 failed");}
    CUDA_CHECK(cudaMemcpyAsync(out_host_.data(), out_dev_, out_bytes_, cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaStreamSynchronize(stream_));
    return laneMaskFromTensor(out_host_.data(), int64_, cfg_);
  }

private:
  LaneConfig cfg_;
  TrtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> ctx_;
  cudaStream_t stream_{nullptr};
  std::vector<void *> bufs_;
  void * in_dev_{nullptr};
  void * out_dev_{nullptr};
  std::size_t in_count_{0}, out_bytes_{0};
  bool int64_{false};
  std::vector<uint8_t> out_host_;
};

}  // namespace

/// @brief Factory used by makeLaneSegmenter().
std::unique_ptr<LaneSegmenter> makeTensorRtLaneSegmenter(const LaneConfig & cfg)
{
  return std::make_unique<TensorRtLaneSegmenter>(cfg);
}

}  // namespace object_detection
