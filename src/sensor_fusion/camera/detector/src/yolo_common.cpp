#include <algorithm>
#include <cmath>
#include <map>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include "object_detection/detector.hpp"

namespace object_detection
{

// Letterbox resize with grey (114) padding, then blobFromImage (BGR -> RGB, / 255).
// Model type's default class list unless configured.
const std::vector<std::string> & classNames(const DetectorConfig & cfg)
{
  if (!cfg.class_names.empty()) {return cfg.class_names;}
  return cfg.model_type == "yolov8" ? cocoClassNames() : yolox18ClassNames();
}

// yolox: top-left letterbox, BGR 0..255 (YOLOX preproc); yolov8: centred, RGB / 255.
cv::Mat yoloPreprocess(const cv::Mat & bgr, const DetectorConfig & cfg, int size, Letterbox & lb)
{
  const bool yolox = cfg.model_type != "yolov8";
  lb.scale = std::min(static_cast<float>(size) / bgr.cols, static_cast<float>(size) / bgr.rows);
  const int nw = static_cast<int>(yolox ? bgr.cols * lb.scale : std::round(bgr.cols * lb.scale));
  const int nh = static_cast<int>(yolox ? bgr.rows * lb.scale : std::round(bgr.rows * lb.scale));
  lb.pad_x = yolox ? 0 : (size - nw) / 2;
  lb.pad_y = yolox ? 0 : (size - nh) / 2;
  cv::Mat resized, canvas(size, size, CV_8UC3, cv::Scalar(114, 114, 114));
  cv::resize(bgr, resized, cv::Size(nw, nh));
  resized.copyTo(canvas(cv::Rect(lb.pad_x, lb.pad_y, nw, nh)));
  if (yolox) {return cv::dnn::blobFromImage(canvas, 1.0, cv::Size(), cv::Scalar(), false, false, CV_32F);}
  return cv::dnn::blobFromImage(canvas, 1.0 / 255.0, cv::Size(), cv::Scalar(), true, false, CV_32F);
}

namespace
{
/// One candidate box before NMS (model input pixels, centre / size).
struct Candidate
{
  float cx, cy, w, h, score;
  int cls;
};

// Class filter, letterbox undo, per-class NMS.
std::vector<Detection> finish(const std::vector<Candidate> & cand, const Letterbox & lb,
  const DetectorConfig & cfg, int img_w, int img_h)
{
  const auto & names = classNames(cfg);
  std::map<int, std::vector<cv::Rect2d>> boxes;
  std::map<int, std::vector<float>> scores;
  for (const auto & c : cand) {
    if (!cfg.keep_classes.empty()) {
      const std::string nm = c.cls < static_cast<int>(names.size()) ? names[c.cls] : std::to_string(c.cls);
      if (!cfg.keep_classes.count(nm)) {continue;}
    }
    // image pixel = (model pixel - pad) / scale
    const float cx = (c.cx - lb.pad_x) / lb.scale, cy = (c.cy - lb.pad_y) / lb.scale;
    const float w = c.w / lb.scale, h = c.h / lb.scale;
    boxes[c.cls].emplace_back(cx - w / 2, cy - h / 2, w, h);
    scores[c.cls].push_back(c.score);
  }
  std::vector<Detection> out;
  for (auto & [cls, b] : boxes) {
    std::vector<int> keep;
    cv::dnn::NMSBoxes(b, scores[cls], cfg.conf_threshold, cfg.nms_threshold, keep);
    for (int k : keep) {
      Detection d;
      d.x1 = std::clamp(static_cast<float>(b[k].x), 0.f, static_cast<float>(img_w - 1));
      d.y1 = std::clamp(static_cast<float>(b[k].y), 0.f, static_cast<float>(img_h - 1));
      d.x2 = std::clamp(static_cast<float>(b[k].x + b[k].width), 0.f, static_cast<float>(img_w - 1));
      d.y2 = std::clamp(static_cast<float>(b[k].y + b[k].height), 0.f, static_cast<float>(img_h - 1));
      d.score = scores[cls][k];
      d.class_id = cls;
      d.class_name = cls < static_cast<int>(names.size()) ? names[cls] : std::to_string(cls);
      out.push_back(d);
    }
  }
  return out;
}
}  // namespace

std::vector<Detection> yoloPostprocess(
  const float * data, int dim1, int dim2, const Letterbox & lb, const DetectorConfig & cfg,
  int img_w, int img_h)
{
  std::vector<Candidate> cand;
  if (cfg.model_type != "yolov8") {
    // YOLOX: rows = anchors of strides 8 / 16 / 32 (grid row-major), cols = x y w h obj c0..cN-1
    // xy = (xy + grid) * stride, wh = exp(wh) * stride, score = obj * class (sigmoids in the model)
    const int n = dim1, ch = dim2, nc = ch - 5;
    const int size = static_cast<int>(std::lround(std::sqrt(n / (1.0 / 64 + 1.0 / 256 + 1.0 / 1024))));
    int i = 0;
    for (const int stride : {8, 16, 32}) {
      const int gs = size / stride;
      for (int gy = 0; gy < gs && i < n; ++gy) {
        for (int gx = 0; gx < gs && i < n; ++gx, ++i) {
          const float * r = data + static_cast<std::size_t>(i) * ch;
          int best = 0;
          float bs = r[5];
          for (int c = 1; c < nc; ++c) {
            if (r[5 + c] > bs) {bs = r[5 + c]; best = c;}
          }
          bs *= r[4];
          if (bs < cfg.conf_threshold) {continue;}
          cand.push_back({(r[0] + gx) * stride, (r[1] + gy) * stride, std::exp(r[2]) * stride,
              std::exp(r[3]) * stride, bs, best});
        }
      }
    }
    return finish(cand, lb, cfg, img_w, img_h);
  }
  const bool channels_first = dim1 < dim2;
  const int n = channels_first ? dim2 : dim1;
  const int ch = channels_first ? dim1 : dim2;
  const int nc = ch - 4;
  auto at = [&](int i, int c) {return channels_first ? data[c * n + i] : data[i * ch + c];};
  for (int i = 0; i < n; ++i) {
    int best = 0;
    float bs = at(i, 4);
    for (int c = 1; c < nc; ++c) {
      const float s = at(i, 4 + c);
      if (s > bs) {bs = s; best = c;}
    }
    if (bs < cfg.conf_threshold) {continue;}
    cand.push_back({at(i, 0), at(i, 1), at(i, 2), at(i, 3), bs, best});
  }
  return finish(cand, lb, cfg, img_w, img_h);
}

}  // namespace object_detection
