#include "object_detection/types.hpp"

namespace object_detection
{
// Static COCO label table.
const std::vector<std::string> & cocoClassNames()
{
  static const std::vector<std::string> n = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
    "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
    "teddy bear", "hair drier", "toothbrush"};
  return n;
}

// Classes of the YOLOX-S 18-class model (dataset_twillight_18class_clean, category id - 1).
const std::vector<std::string> & yolox18ClassNames()
{
  static const std::vector<std::string> n = {
    "animal", "autorickshaw", "barricade", "bicycle", "bicyclist", "bus", "car", "debris",
    "motorcycle", "motorcyclist", "partial pedestrian", "pedestrian", "pothole", "road damage",
    "traffic light", "traffic sign", "truck", "vehicle fallback"};
  return n;
}
}  // namespace object_detection
