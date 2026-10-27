/**
 * @file object_detection_node.cpp
 * @brief ROS 2 node: lens distortion correction + camera object detection (no tracking:
 *        the 2D boxes are tracked inside sensor_fusion) + lane / road segmentation.
 */
//   platform = pc   -> backend json_replay (detections pre-computed on the rectified
//                      images by bag_to_detections_json, see precompute_detections.launch.xml)
//   platform = orin -> backend tensorrt    (YOLO engine on the GPU)
// detector.backend can override (json_replay | onnxruntime | opencv_dnn | tensorrt).
// Lane segmentation (lane.enabled) on the same rectified image every 1 / lane.rate s:
//   platform = pc   -> backend replay   (masks pre-computed by bag_to_detections_json --lane-model)
//   platform = orin -> backend tensorrt
//
// Sub: image_topic (sensor_msgs/Image, raw fisheye)
// Pub: ~/objects    (perception_msgs/CameraObjectArray, boxes in the rectified image)
//      ~/lane_mask  (perception_msgs/SegmentationMask, 0 background / 1 lane / 2 road, model resolution)
//      ~/image_rect (sensor_msgs/Image bgr8, display only: best-effort, display_rate,
//                    only while someone subscribes - a slow viewer never blocks detection)
#include <chrono>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "fusion_common/camera_model.hpp"
#include "object_detection/detector.hpp"
#include "object_detection/image_utils.hpp"
#include "object_detection/lane_segmenter.hpp"
#include "perception_msgs/msg/camera_object_array.hpp"
#include "perception_msgs/msg/segmentation_mask.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace object_detection
{

/// Rectifies each camera image, runs the detector and publishes the boxes.
class ObjectDetectionNode : public rclcpp::Node
{
public:
  /**
   * @brief Declares the parameters, loads the rectifier and the detector, creates the topics.
   * @throws std::runtime_error if rectification is on without intrinsics, or the detector fails to load
   */
  ObjectDetectionNode()
  : Node("object_detection")
  {
    const auto platform = declare_parameter<std::string>("platform", "pc");
    const auto image_topic = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    // reliable: large images over best-effort DDS lose fragments (measured: 7 of 30 frames
    // arrive). best_effort only if the camera driver publishes best-effort.
    const auto image_qos = declare_parameter<std::string>("image_qos", "reliable");
    // lens distortion correction (same model as sensor_fusion / fusion_visualization)
    const bool rectify = declare_parameter<bool>("rectify.enabled", true);
    const auto intr = declare_parameter<std::string>("rectify.intrinsics_file", "");
    const double balance = declare_parameter<double>("rectify.balance", 0.0);
    const double fov_scale = declare_parameter<double>("rectify.fov_scale", 1.0);
    if (rectify) {
      if (intr.empty()) {throw std::runtime_error("rectify.enabled but rectify.intrinsics_file is empty");}
      rectifier_ = std::make_unique<fusion_common::ImageRectifier>(
        fusion_common::CameraModel::loadIntrinsics(intr), balance, fov_scale);
    }

    DetectorConfig dc;
    dc.backend = declare_parameter<std::string>("detector.backend", "auto");
    if (dc.backend == "auto") {dc.backend = platform == "orin" ? "tensorrt" : "json_replay";}
    dc.json_file = declare_parameter<std::string>("detector.json_file", "");
    dc.json_max_time_diff = declare_parameter<double>("detector.json_max_time_diff", dc.json_max_time_diff);
    dc.json_expect_rectified = rectify;
    dc.model_path = declare_parameter<std::string>("detector.model_path", "");
    dc.model_type = declare_parameter<std::string>("detector.model_type", dc.model_type);
    if (dc.model_type != "yolox" && dc.model_type != "yolov8") {
      throw std::runtime_error("detector.model_type must be yolox or yolov8, got '" + dc.model_type + "'");
    }
    dc.input_size = declare_parameter<int>("detector.input_size", dc.input_size);
    // low threshold: the fusion's ByteTrack uses low-score boxes in its second stage
    dc.conf_threshold = static_cast<float>(declare_parameter<double>("detector.conf_threshold", 0.15));
    dc.nms_threshold = static_cast<float>(declare_parameter<double>("detector.nms_threshold", dc.nms_threshold));
    dc.use_gpu = declare_parameter<bool>("detector.use_gpu", platform == "orin");
    // published classes per model type (labels differ: YOLOX 18 classes / COCO)
    const auto keep_yolox = declare_parameter<std::vector<std::string>>("detector.classes",
      {"animal", "autorickshaw", "barricade", "bicycle", "bicyclist", "bus", "car", "debris", "motorcycle",
        "motorcyclist", "partial pedestrian", "pedestrian", "pothole", "road damage", "truck", "vehicle fallback"});
    const auto keep_coco = declare_parameter<std::vector<std::string>>(
      "detector.classes_yolov8", {"person", "bicycle", "car", "motorcycle", "bus", "truck", "dog", "cat"});
    const auto & keep = dc.model_type == "yolov8" ? keep_coco : keep_yolox;
    dc.keep_classes = std::set<std::string>(keep.begin(), keep.end());
    detector_ = makeDetector(dc);

    LaneConfig lc;
    const bool lane = declare_parameter<bool>("lane.enabled", true);
    lc.backend = declare_parameter<std::string>("lane.backend", "auto");
    if (lc.backend == "auto") {lc.backend = platform == "orin" ? "tensorrt" : "replay";}
    lc.model_path = declare_parameter<std::string>("lane.model_path", "");
    lc.replay_dir = declare_parameter<std::string>("lane.replay_dir", "");
    lc.replay_max_time_diff = declare_parameter<double>("lane.replay_max_time_diff", lc.replay_max_time_diff);
    lc.use_gpu = declare_parameter<bool>("lane.use_gpu", platform == "orin");
    lane_period_ = 1.0 / std::max(0.1, declare_parameter<double>("lane.rate", 10.0));
    if (lane) {
      try {
        lane_ = makeLaneSegmenter(lc);
      } catch (const std::exception & e) {   // detection keeps running without the lane
        RCLCPP_ERROR(get_logger(), "lane segmentation off: %s", e.what());
      }
    }

    pub_ = create_publisher<perception_msgs::msg::CameraObjectArray>("~/objects", 10);
    // reliable publisher: serves reliable and best-effort subscribers
    // display stream: best-effort so a slow viewer / recorder can never block this node
    // (reliable large images back-pressured the detector: 2-3 s stalls in fusion_run1)
    rect_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_rect", rclcpp::SensorDataQoS().keep_last(1));
    lane_pub_ = create_publisher<perception_msgs::msg::SegmentationMask>("~/lane_mask", 5);
    display_rate_ = declare_parameter<double>("display_rate", 10.0);
    // depth 2: on a slow platform the newest frames replace older ones
    auto qos = rclcpp::QoS(2);
    if (image_qos == "best_effort") {qos.best_effort();} else {qos.reliable();}
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {onImage(*m);});
    // which models run (printed so a run can be checked at a glance)
    const auto & names = classNames(dc);
    std::string classes;
    for (const auto & n : names) {classes += (classes.empty() ? "" : ", ") + n + (dc.keep_classes.count(n) ? "" : " (not published)");}
    const std::string det_file = dc.backend == "json_replay" ? dc.json_file + " (replay)" : dc.model_path;
    const std::string lane_file = !lane_ ? std::string("off") : lane_->name() == "replay" ? lc.replay_dir + " (replay)" :
      lc.model_path;
    RCLCPP_INFO(get_logger(),
      "\n======== object_detection models (platform %s) ========\n"
      "  detector : %s\n"
      "             backend %s, type %s, conf >= %.2f, %zu classes: %s\n"
      "  lane seg : %s\n"
      "             backend %s, %.0f Hz, mask classes: background, lane, road\n"
      "  image    : %s, rectify %s\n"
      "=====================================================",
      platform.c_str(), det_file.c_str(), detector_->name().c_str(), dc.model_type.c_str(), dc.conf_threshold,
      names.size(), classes.c_str(), lane_file.c_str(), lane_ ? lane_->name().c_str() : "-",
      lane_ ? 1.0 / lane_period_ : 0.0, image_topic.c_str(), rectify ? intr.c_str() : "off");
  }

private:
  /**
   * @brief One camera frame: convert, rectify, detect, publish boxes (+ display image if due).
   * @param[in] m  raw camera image
   */
  void onImage(const sensor_msgs::msg::Image & m)
  {
    const auto t0 = std::chrono::steady_clock::now();
    const int64_t stamp_ns = static_cast<int64_t>(m.header.stamp.sec) * 1000000000LL + m.header.stamp.nanosec;
    const double t_img = stamp_ns * 1e-9;
    // display image if someone subscribes and |t - t_last_display| >= 1 / display_rate
    const bool want_image = rect_pub_->get_subscription_count() > 0 &&
      (display_rate_ <= 0 || std::abs(t_img - last_display_t_) >= 1.0 / display_rate_);
    if (want_image) {last_display_t_ = t_img;}
    // lane every lane.rate (by image stamp; a bag restart starts over)
    const bool want_lane = lane_ && (t_img - last_lane_t_ >= lane_period_ - 1e-3 || t_img < last_lane_t_);
    std::vector<Detection> dets;
    cv::Mat img;
    try {
      if (detector_->needsImage() || want_image || (want_lane && lane_->needsImage())) {
        img = toBgr(m);
        if (rectifier_) {
          cv::Mat r;
          rectifier_->rectify(img, r);
          img = r;
        }
      }
      dets = detector_->detect(img, stamp_ns);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "detection failed: %s", e.what());
      return;
    }

    perception_msgs::msg::CameraObjectArray out;
    out.header = m.header;
    out.source = detector_->name();
    out.image_width = m.width;     // rectified image keeps the raw size
    out.image_height = m.height;
    for (const auto & d : dets) {
      perception_msgs::msg::CameraObject o;
      o.class_id = d.class_id;
      o.class_name = d.class_name;
      o.score = d.score;
      o.x_min = d.x1; o.y_min = d.y1; o.x_max = d.x2; o.y_max = d.y2;
      out.objects.push_back(o);
    }
    pub_->publish(out);

    if (want_lane) {
      try {
        const cv::Mat mask = lane_->segment(img, stamp_ns);
        if (!mask.empty()) {   // replay: frames without a dumped mask are retried on the next frame
          last_lane_t_ = t_img;
          perception_msgs::msg::SegmentationMask lm;
          lm.header = m.header;
          lm.source = lane_->name();
          lm.image_width = m.width;
          lm.image_height = m.height;
          lm.class_names = laneClassNames();
          lm.mask.header = m.header;
          lm.mask.height = mask.rows;
          lm.mask.width = mask.cols;
          lm.mask.encoding = "mono8";
          lm.mask.step = mask.cols;
          lm.mask.data.assign(mask.data, mask.data + mask.total());
          lane_pub_->publish(lm);
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "lane segmentation failed: %s", e.what());
      }
    }

    if (want_image && !img.empty()) {
      auto im = std::make_unique<sensor_msgs::msg::Image>();
      im->header = m.header;
      im->height = img.rows;
      im->width = img.cols;
      im->encoding = "bgr8";
      im->step = static_cast<uint32_t>(img.step);
      im->data.assign(img.data, img.data + img.total() * img.elemSize());
      rect_pub_->publish(std::move(im));
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    RCLCPP_DEBUG(get_logger(), "%zu dets, %.1f ms", dets.size(), ms);
  }

  std::unique_ptr<fusion_common::ImageRectifier> rectifier_;
  std::unique_ptr<Detector> detector_;
  std::unique_ptr<LaneSegmenter> lane_;
  double lane_period_{0.1}, last_lane_t_{-1e18};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<perception_msgs::msg::CameraObjectArray>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_pub_;
  rclcpp::Publisher<perception_msgs::msg::SegmentationMask>::SharedPtr lane_pub_;
  double display_rate_{10.0}, last_display_t_{-1e18};
};

}  // namespace object_detection

/// Node entry point.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<object_detection::ObjectDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
