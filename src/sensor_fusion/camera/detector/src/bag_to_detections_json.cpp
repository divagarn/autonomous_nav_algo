/**
 * @file bag_to_detections_json.cpp
 * @brief Offline (PC) preprocessing: rectify every image of a rosbag topic, run YOLO on the
 *        CPU (ONNX Runtime) and write the JSON replayed by object_detection (json_replay);
 *        with --lane-model also the lane / road masks (<lane-out>/<stamp_ns>.png, lane.backend replay).
 *
 * Usually started through fusion_bringup/launch/precompute_detections.launch.xml.
 */
//
//   ros2 run object_detection bag_to_detections_json --bag <bag_dir> --model yolov8n.onnx
//        --intrinsics front_intrinsics.yaml [--out <bag_dir>_detections.json]
//        [--balance 0] [--fov_scale 1] [--backend onnxruntime] [--topic /camera/image_raw] [--conf 0.15]
//        [--model-type yolox|yolov8] [--lane-model lane_seg_up.onnx] [--lane-out <bag_dir>_lane]
//        [--lane-every 3]   (lane mask of every 3rd frame: 10 Hz at 30 fps, as lane.rate online)
// --intrinsics none skips rectification (the online node must then run with rectify.enabled false).
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

#include "fusion_common/camera_model.hpp"
#include "object_detection/detector.hpp"
#include "object_detection/image_utils.hpp"
#include "object_detection/lane_segmenter.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/imgcodecs.hpp>

using namespace object_detection;

/**
 * @brief Escapes quotes and backslashes for a JSON string.
 * @param[in] s  raw text
 * @return escaped text
 */
static std::string jsonEscape(const std::string & s)
{
  std::string o;
  for (char c : s) {if (c == '"' || c == '\\') {o += '\\';} o += c;}
  return o;
}

/**
 * @brief Command-line tool: reads the bag, writes one JSON frame per image.
 * @param[in] argc, argv  --bag, --model, --intrinsics (required) and options, see usage
 * @return 0 on success, 1 on a usage error
 */
int main(int argc, char ** argv)
{
  std::map<std::string, std::string> a{{"--topic", "/camera/image_raw"}, {"--backend", "onnxruntime"},
    {"--conf", "0.15"}, {"--nms", "0.5"}, {"--every", "1"}, {"--balance", "0"}, {"--fov_scale", "1"},
    {"--model-type", "yolox"}, {"--lane-every", "3"}};
  for (int i = 1; i + 1 < argc; i += 2) {a[argv[i]] = argv[i + 1];}
  if (!a.count("--bag") || !a.count("--model") || !a.count("--intrinsics")) {
    std::cerr << "usage: bag_to_detections_json --bag DIR --model MODEL --intrinsics FILE|none "
      "[--out FILE.json] [--balance B] [--fov_scale S] [--backend onnxruntime|opencv_dnn|tensorrt] "
      "[--topic T] [--conf C] [--every N] [--model-type yolox|yolov8] [--lane-model ONNX] "
      "[--lane-out DIR] [--lane-every N]\n";
    return 1;
  }
  if (!a.count("--out") || a["--out"].empty()) {
    std::string bag = a["--bag"];
    while (bag.size() > 1 && bag.back() == '/') {bag.pop_back();}
    a["--out"] = bag + "_detections.json";
  }
  if (a.count("--lane-model") && (!a.count("--lane-out") || a["--lane-out"].empty())) {
    std::string bag = a["--bag"];
    while (bag.size() > 1 && bag.back() == '/') {bag.pop_back();}
    a["--lane-out"] = bag + "_lane";
  }
  DetectorConfig dc;
  dc.backend = a["--backend"];
  dc.model_path = a["--model"];
  dc.conf_threshold = std::stof(a["--conf"]);
  dc.nms_threshold = std::stof(a["--nms"]);
  dc.model_type = a["--model-type"];
  const int every = std::max(1, std::stoi(a["--every"]));
  auto det = makeDetector(dc);
  std::unique_ptr<LaneSegmenter> lane;
  const int lane_every = std::max(1, std::stoi(a["--lane-every"]));
  if (a.count("--lane-model")) {
    LaneConfig lc;
    lc.backend = a["--backend"] == "tensorrt" ? "tensorrt" : "onnxruntime";
    lc.model_path = a["--lane-model"];
    lane = makeLaneSegmenter(lc);
    std::filesystem::create_directories(a["--lane-out"]);
  }
  std::unique_ptr<fusion_common::ImageRectifier> rect;
  if (a["--intrinsics"] != "none") {
    rect = std::make_unique<fusion_common::ImageRectifier>(
      fusion_common::CameraModel::loadIntrinsics(a["--intrinsics"]), std::stod(a["--balance"]), std::stod(a["--fov_scale"]));
  }

  rosbag2_cpp::Reader reader;
  reader.open(a["--bag"]);
  rosbag2_storage::StorageFilter filter;
  filter.topics = {a["--topic"]};
  reader.set_filter(filter);
  rclcpp::Serialization<sensor_msgs::msg::Image> ser;

  std::ofstream out(a["--out"]);
  out << std::fixed << std::setprecision(2);
  out << "{\n\"bag\": \"" << jsonEscape(a["--bag"]) << "\",\n\"topic\": \"" << jsonEscape(a["--topic"])
      << "\",\n\"model\": \"" << jsonEscape(a["--model"]) << "\",\n\"model_type\": \"" << dc.model_type
      << "\",\n\"backend\": \"" << det->name()
      << "\",\n\"rectified\": " << (rect ? 1 : 0) << ",\n";
  bool first = true, size_written = false;
  int n = 0, written = 0, masks = 0;
  const auto t0 = std::chrono::steady_clock::now();
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    if (n++ % every) {continue;}
    rclcpp::SerializedMessage sm(*bag_msg->serialized_data);
    sensor_msgs::msg::Image img;
    ser.deserialize_message(&sm, &img);
    if (!size_written) {
      out << "\"image_width\": " << img.width << ",\n\"image_height\": " << img.height << ",\n\"frames\": [\n";
      size_written = true;
    }
    const int64_t stamp_ns = static_cast<int64_t>(img.header.stamp.sec) * 1000000000LL + img.header.stamp.nanosec;
    cv::Mat bgr = toBgr(img);
    if (rect) {
      cv::Mat r;
      rect->rectify(bgr, r);
      bgr = r;
    }
    const auto dets = det->detect(bgr, stamp_ns);
    if (lane && written % lane_every == 0) {
      cv::imwrite(a["--lane-out"] + "/" + std::to_string(stamp_ns) + ".png", lane->segment(bgr, stamp_ns));
      ++masks;
    }
    out << (first ? "" : ",\n") << "{\"stamp_sec\": " << img.header.stamp.sec << ", \"stamp_nanosec\": "
        << img.header.stamp.nanosec << ", \"frame_index\": " << (n - 1) << ", \"detections\": [";
    for (std::size_t i = 0; i < dets.size(); ++i) {
      const auto & d = dets[i];
      out << (i ? ", " : "") << "{\"bbox\": [" << d.x1 << ", " << d.y1 << ", " << d.x2 << ", " << d.y2
          << "], \"score\": " << std::setprecision(3) << d.score << std::setprecision(2)
          << ", \"class_id\": " << d.class_id << ", \"class_name\": \"" << jsonEscape(d.class_name) << "\"}";
    }
    out << "]}";
    first = false;
    if (++written % 200 == 0) {
      const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      std::fprintf(stderr, "%d frames  %.1f fps\n", written, written / s);
    }
  }
  if (!size_written) {out << "\"frames\": [\n";}
  out << "\n]\n}\n";
  std::cerr << "wrote " << written << " frames -> " << a["--out"] << "\n";
  if (lane) {std::cerr << "wrote " << masks << " lane masks -> " << a["--lane-out"] << "\n";}
  return 0;
}
