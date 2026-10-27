/**
 * @file types.hpp
 * @brief Detector output types.
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace object_detection
{

/// One detected box in image pixels.
struct Detection
{
  float x1{0}, y1{0}, x2{0}, y2{0};
  float score{0};
  int class_id{-1};
  std::string class_name;
};

/// @brief The 80 COCO class names, index = YOLO class id.
const std::vector<std::string> & cocoClassNames();
/// @brief The 18 classes of models/yolox_s_18class.onnx (index = model class).
const std::vector<std::string> & yolox18ClassNames();

}  // namespace object_detection
