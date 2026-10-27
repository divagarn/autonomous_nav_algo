/**
 * @file lane_viewer.cpp
 * @brief Optional offline viewer: lane / road segmentation of a rosbag drawn on the rectified camera
 *        image (mask, searched rows, road edges, lane markings, tracked path + lane) with a top view.
 *
 * Same chain as online: rectify (object_detection) -> segmentation model -> LaneEstimator + PathTracker
 * (as in sensor_fusion, ego motion from the odometry topic) -> projected back into the image with
 * the camera calibration (flat ground at the radar height). Usually started through ./view_lane.sh.
 */
//
//   ros2 run object_detection lane_viewer --bag <bag_dir> --intrinsics I.yaml --extrinsics E.yaml
//        [--lane-model models/lane_seg_up.onnx | --replay <masks_dir>] [--backend onnxruntime|tensorrt]
//        [--start S] [--duration S] [--every 3] [--radar-height 0.41] [--radar-mount-x 2.2]
//        [--balance 0] [--fov_scale 1] [--topic /camera/image_raw] [--odom /odom]
//        [--detections <bag>_detections_<model>.json] [--occluder-classes a,b,..] [--score 0.15]
//        [--out video.mp4] [--window 1]
// --detections: boxes of people / two-wheelers (occluder classes) are no road edge (as sensor_fusion
// with the online detector); drawn in orange.
// Keys: space pause / resume, n next frame while paused, s save a PNG, q / Esc quit.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "fusion_common/camera_model.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "object_detection/detector.hpp"
#include "object_detection/image_utils.hpp"
#include "object_detection/lane_segmenter.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_fusion/ego_motion.hpp"
#include "sensor_fusion/lane_estimator.hpp"
#include "sensor_fusion/path_tracker.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace object_detection;
using sensor_fusion::LaneBoundary;
using sensor_fusion::LaneEstimator;

namespace
{
// BGR colours, as fusion_visualization: road blue, marking yellow, edges cyan, markings white
const cv::Scalar kRoad(255, 90, 0), kMarking(0, 230, 255), kEdge(255, 255, 0), kLaneLine(255, 255, 255),
kPath(0, 220, 0), kRowEnd(255, 0, 255), kOccluder(0, 140, 255);

/**
 * @brief Mask colours blended into the image.
 * @param[in,out] img   rectified image
 * @param[in]     mask  class mask (any size, 1 marking, 2 road)
 */
void drawMask(cv::Mat & img, const cv::Mat & mask)
{
  cv::Mat m;
  cv::resize(mask, m, img.size(), 0, 0, cv::INTER_NEAREST);
  cv::Mat colour = img.clone();
  colour.setTo(kRoad, m == LaneEstimator::kRoad);
  colour.setTo(kMarking, m == LaneEstimator::kLane);
  cv::addWeighted(colour, 0.35, img, 0.65, 0.0, img);
}

/**
 * @brief Ground curve projected into the image (segments with both ends visible).
 * @param[in,out] img       image
 * @param[in]     b         boundary (radar frame)
 * @param[in]     cam       camera model of the image
 * @param[in]     ground_z  ground height in the radar frame [m]
 */
void drawCurve(cv::Mat & img, const LaneBoundary & b, const fusion_common::CameraModel & cam, double ground_z,
  const cv::Scalar & colour, int thickness)
{
  if (!b.valid) {return;}
  std::optional<Eigen::Vector2d> prev;
  for (const auto & p : b.points) {
    const auto uv = cam.projectRadarPoint({p.x(), p.y(), ground_z});
    if (uv && prev) {
      cv::line(img, {static_cast<int>(prev->x()), static_cast<int>(prev->y())},
        {static_cast<int>(uv->x()), static_cast<int>(uv->y())}, colour, thickness, cv::LINE_AA);
    }
    prev = uv;
  }
}

/**
 * @brief Ground polyline projected into the image (segments with both ends visible).
 * @param[in,out] img       image
 * @param[in]     line      points (radar frame)
 * @param[in]     cam       camera model of the image
 * @param[in]     ground_z  ground height in the radar frame [m]
 */
void drawLine(cv::Mat & img, const std::vector<Eigen::Vector2d> & line, const fusion_common::CameraModel & cam,
  double ground_z, const cv::Scalar & colour, int thickness)
{
  LaneBoundary b;
  b.valid = true;
  b.points = line;
  drawCurve(img, b, cam, ground_z, colour, thickness);
}

/**
 * @brief Top view of the boundaries and the tracked path: x forward (up), y left; radar at 5 m.
 * @param[in] e          estimate (radar frame)
 * @param[in] path       tracked path (radar frame)
 * @param[in] height     panel height [px]
 * @param[in] max_range  forward extent [m]
 * @return panel image
 */
cv::Mat topView(const sensor_fusion::LaneEstimate & e, const sensor_fusion::TrackedPath & path, int height,
  double max_range)
{
  const int width = height * 3 / 5;
  const double scale = height / (max_range + 6.0);   // px per m
  cv::Mat img(height, width, CV_8UC3, cv::Scalar(30, 30, 30));
  // radar frame (x, y) -> pixel: u = w/2 - y s, v = h - (x + 5) s
  auto px = [&](double x, double y) {
      return cv::Point(static_cast<int>(width / 2 - y * scale), static_cast<int>(height - (x + 5.0) * scale));
    };
  for (int r = 5; r <= static_cast<int>(max_range); r += 5) {
    cv::line(img, px(r, 10), px(r, -10), cv::Scalar(70, 70, 70), 1);
    cv::putText(img, std::to_string(r) + " m", px(r, width / (2 * scale) - 0.3) + cv::Point(2, -3),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);
  }
  cv::line(img, px(0, 0), px(max_range, 0), cv::Scalar(70, 70, 70), 1);
  cv::rectangle(img, px(0, 0.7), px(-2.8, -0.7), cv::Scalar(200, 200, 200), 2);   // vehicle (radar at the front)
  const std::pair<const LaneBoundary *, cv::Scalar> curves[] = {{&e.road_left, kEdge}, {&e.road_right, kEdge},
    {&e.lane_left, kLaneLine}, {&e.lane_right, kLaneLine}};
  for (const auto & [b, colour] : curves) {
    for (std::size_t i = 1; b->valid && i < b->points.size(); ++i) {
      cv::line(img, px(b->points[i - 1].x(), b->points[i - 1].y()), px(b->points[i].x(), b->points[i].y()),
        colour, 2, cv::LINE_AA);
    }
  }
  const std::pair<const std::vector<Eigen::Vector2d> *, int> lines[] = {{&path.left, 1}, {&path.right, 1},
    {&path.centre, 3}};
  for (const auto & [line, thick] : lines) {
    for (std::size_t i = 1; i < line->size(); ++i) {
      cv::line(img, px((*line)[i - 1].x(), (*line)[i - 1].y()), px((*line)[i].x(), (*line)[i].y()), kPath, thick,
        cv::LINE_AA);
    }
  }
  for (const auto & q : path.centre) {cv::circle(img, px(q.x(), q.y()), 2, kPath, cv::FILLED);}
  return img;
}
}  // namespace

/**
 * @brief Command-line tool: reads the bag, segments every n-th image, shows / writes the drawing.
 * @param[in] argc, argv  --bag, --intrinsics, --extrinsics, --lane-model or --replay, options see usage
 * @return 0 on success, 1 on a usage error
 */
int main(int argc, char ** argv)
{
  std::map<std::string, std::string> a{{"--topic", "/camera/image_raw"}, {"--odom", "/odom"},
    {"--backend", "onnxruntime"}, {"--every", "3"}, {"--start", "0"}, {"--duration", "0"},
    {"--radar-height", "0.41"}, {"--radar-mount-x", "2.2"}, {"--balance", "0"},
    {"--fov_scale", "1"}, {"--window", "1"}, {"--max-range", "25"}, {"--score", "0.15"},
    {"--occluder-classes", "person,pedestrian,partial pedestrian,bicycle,bicyclist,motorcycle,motorcyclist,animal"}};
  for (int i = 1; i + 1 < argc; i += 2) {a[argv[i]] = argv[i + 1];}
  if (!a.count("--bag") || !a.count("--intrinsics") || !a.count("--extrinsics") ||
    (!a.count("--lane-model") && !a.count("--replay")))
  {
    std::cerr << "usage: lane_viewer --bag DIR --intrinsics FILE --extrinsics FILE (--lane-model MODEL | "
      "--replay DIR) [--backend onnxruntime|tensorrt] [--start S] [--duration S] [--every N] "
      "[--radar-height M] [--radar-mount-x M] [--max-range M] [--balance B] "
      "[--fov_scale S] [--topic T] [--odom T] [--detections FILE.json] [--occluder-classes A,B] [--score S] "
      "[--out FILE.mp4] [--window 0|1]\n";
    return 1;
  }
  LaneConfig lc;
  if (a.count("--replay")) {
    lc.backend = "replay";
    lc.replay_dir = a["--replay"];
  } else {
    lc.backend = a["--backend"];
    lc.model_path = a["--lane-model"];
  }
  auto seg = makeLaneSegmenter(lc);
  std::unique_ptr<Detector> det;   // precomputed boxes of the bag (json_replay)
  std::set<std::string> occluder_classes;
  if (a.count("--detections") && !a["--detections"].empty()) {
    DetectorConfig dc;
    dc.json_file = a["--detections"];
    det = makeDetector(dc);
    std::stringstream ss(a["--occluder-classes"]);
    for (std::string c; std::getline(ss, c, ',');) {occluder_classes.insert(c);}
  }
  const float min_score = std::stof(a["--score"]);
  const double balance = std::stod(a["--balance"]), fov_scale = std::stod(a["--fov_scale"]);
  const auto base = fusion_common::CameraModel::load(a["--intrinsics"], a["--extrinsics"]);
  fusion_common::ImageRectifier rect(base, balance, fov_scale);
  sensor_fusion::LaneEstimatorConfig lec;
  lec.max_range = std::stod(a["--max-range"]);
  LaneEstimator est(lec);
  sensor_fusion::PathTracker tracker;   // defaults = sensor_fusion.yaml lane.path
  // base_link in the radar frame (radar mount_x ahead of it, same heading)
  const Eigen::Vector2d base_link(-std::stod(a["--radar-mount-x"]), 0.0);
  sensor_fusion::EgoMotion ego(sensor_fusion::MountPose2D{std::stod(a["--radar-mount-x"]), 0.0, 0.0});
  const double ground_z = -std::stod(a["--radar-height"]);
  const int every = std::max(1, std::stoi(a["--every"]));
  const double start = std::stod(a["--start"]), duration = std::stod(a["--duration"]);
  const bool window = a["--window"] != "0";
  const std::string title = "lane_viewer (space pause, n next, s save, q quit)";

  rosbag2_cpp::Reader reader;
  reader.open(a["--bag"]);
  rosbag2_storage::StorageFilter filter;
  filter.topics = {a["--topic"], a["--odom"]};
  reader.set_filter(filter);
  rclcpp::Serialization<sensor_msgs::msg::Image> img_ser;
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_ser;

  fusion_common::CameraModel cam;
  cv::Size cam_size;
  cv::VideoWriter writer;
  double t_first = -1, prev = -1;
  int images = 0, shown = 0, with_path = 0;
  bool paused = false;
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    rclcpp::SerializedMessage sm(*bag_msg->serialized_data);
    if (bag_msg->topic_name == a["--odom"]) {
      nav_msgs::msg::Odometry o;
      odom_ser.deserialize_message(&sm, &o);
      ego.add({o.header.stamp.sec + o.header.stamp.nanosec * 1e-9, o.twist.twist.linear.x, o.twist.twist.angular.z});
      continue;
    }
    sensor_msgs::msg::Image img;
    img_ser.deserialize_message(&sm, &img);
    const double t = img.header.stamp.sec + img.header.stamp.nanosec * 1e-9;
    if (t_first < 0) {t_first = t;}
    if (t - t_first < start) {continue;}
    if (duration > 0 && t - t_first > start + duration) {break;}
    if (images++ % every) {continue;}
    const int64_t stamp_ns = static_cast<int64_t>(img.header.stamp.sec) * 1000000000LL + img.header.stamp.nanosec;
    cv::Mat bgr;
    rect.rectify(toBgr(img), bgr);
    if (bgr.size() != cam_size) {
      cam = fusion_common::cameraForImage(base, bgr.cols, bgr.rows, true, balance, fov_scale);
      cam_size = bgr.size();
    }
    const auto t0 = std::chrono::steady_clock::now();
    const cv::Mat mask = seg->segment(seg->needsImage() ? bgr : cv::Mat(), stamp_ns);
    const double seg_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (mask.empty()) {continue;}   // replay: no mask for this image
    // radar frame at the previous mask -> now (ego motion)
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    Eigen::Vector2d tr = Eigen::Vector2d::Zero();
    if (prev >= 0 && t >= prev && t - prev < 2.0 && !ego.empty()) {ego.frameTransform(prev, t, R, tr);}
    prev = t;
    std::vector<Eigen::Vector4d> occluders;
    if (det) {
      for (const auto & d : det->detect(bgr, stamp_ns)) {
        if (d.score >= min_score && occluder_classes.count(d.class_name)) {occluders.emplace_back(d.x1, d.y1, d.x2, d.y2);}
      }
    }
    const auto & e = est.update(t, mask.data, mask.cols, mask.rows, bgr.cols, bgr.rows, cam, ground_z, R, tr,
      occluders);
    // path from base_link; curvature of the vehicle from odometry (speed > 0.3 m/s)
    const auto es = ego.empty() ? sensor_fusion::EgoSample{} : ego.at(t);
    const auto & tp = tracker.update(e, base_link, 0.0, R, tr, es.speed > 0.3 ? es.yaw_rate / es.speed : 0.0);
    with_path += tp.measured > 0;

    // drawing: mask, row ends (the edge pixels found), curves, distance marks on the driving path
    drawMask(bgr, mask);
    for (const auto & b : occluders) {
      cv::rectangle(bgr, {static_cast<int>(b(0)), static_cast<int>(b(1))}, {static_cast<int>(b(2)), static_cast<int>(b(3))},
        kOccluder, 2);
    }
    const double sx = static_cast<double>(bgr.cols) / mask.cols, sy = static_cast<double>(bgr.rows) / mask.rows;
    for (const auto & r : est.rows()) {
      const int v = static_cast<int>((r.v + 0.5) * sy);
      for (const int u : {r.left, r.right}) {
        if (u >= 0) {cv::circle(bgr, {static_cast<int>((u + 0.5) * sx), v}, 3, kRowEnd, cv::FILLED);}
      }
    }
    drawCurve(bgr, e.road_left, cam, ground_z, kEdge, 2);
    drawCurve(bgr, e.road_right, cam, ground_z, kEdge, 2);
    drawCurve(bgr, e.lane_left, cam, ground_z, kLaneLine, 2);
    drawCurve(bgr, e.lane_right, cam, ground_z, kLaneLine, 2);
    drawLine(bgr, tp.left, cam, ground_z, kPath, 1);
    drawLine(bgr, tp.right, cam, ground_z, kPath, 1);
    drawLine(bgr, tp.centre, cam, ground_z, kPath, 3);
    for (std::size_t i = 0; i < tp.centre.size(); ++i) {   // the tracked points, distance from base_link
      const auto & p = tp.centre[i];
      if (const auto uv = cam.projectRadarPoint({p.x(), p.y(), ground_z})) {
        const cv::Point c(static_cast<int>(uv->x()), static_cast<int>(uv->y()));
        cv::circle(bgr, c, 4, kPath, cv::FILLED);
        if (i % 5 == 0) {
          cv::putText(bgr, std::to_string(i) + " m", c + cv::Point(8, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, kPath, 2);
        }
      }
    }
    char width[16] = "--", hud[300];
    if (e.road_width > 0) {std::snprintf(width, sizeof(width), "%.1f m", e.road_width);}
    std::snprintf(hud, sizeof(hud), "t %.1f s  edges L %s R %s  width %s  path: %d pts corrected, ref %s %.1f m, "
      "%.0f m unmeasured  seg %.0f ms [%s]", t - t_first, e.road_left.valid ? "ok" : "--",
      e.road_right.valid ? "ok" : "--", width, tp.measured, tp.reference == 0 ? "L" : tp.reference == 1 ? "R" : "-",
      tp.offset, tp.unmeasured_m, seg_ms, seg->name().c_str());
    cv::rectangle(bgr, {0, 0}, {bgr.cols, 26}, cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(bgr, hud, {8, 18}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    cv::Mat frame;
    cv::hconcat(bgr, topView(e, tp, bgr.rows, lec.max_range), frame);
    ++shown;

    if (a.count("--out")) {
      if (!writer.isOpened()) {
        writer.open(a["--out"], cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30.0 / every, frame.size());
        if (!writer.isOpened()) {std::cerr << "cannot write " << a["--out"] << "\n"; return 1;}
      }
      writer.write(frame);
    }
    if (window) {
      cv::imshow(title, frame);
      for (;;) {
        const int key = cv::waitKey(paused ? 0 : 1) & 0xff;
        if (key == 'q' || key == 27) {return 0;}
        if (key == ' ') {paused = !paused;}
        if (key == 's') {
          const std::string png = "lane_viewer_" + std::to_string(stamp_ns) + ".png";
          cv::imwrite(png, frame);
          std::cerr << "saved " << png << "\n";
        }
        if (!paused || key == 'n') {break;}
      }
    } else if (shown % 100 == 0) {
      std::fprintf(stderr, "%d frames, path corrected in %d\n", shown, with_path);
    }
  }
  std::fprintf(stderr, "%d frames, path corrected in %d (%.0f %%)%s%s\n", shown, with_path,
    shown ? 100.0 * with_path / shown : 0.0, a.count("--out") ? " -> " : "",
    a.count("--out") ? a["--out"].c_str() : "");
  return 0;
}
