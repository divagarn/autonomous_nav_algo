/**
 * @file image_utils.hpp
 * @brief sensor_msgs/Image <-> cv::Mat without cv_bridge (avoids the OpenCV version
 *        clash between ROS apt packages and JetPack's OpenCV on Orin).
 */
#pragma once

#include <stdexcept>
#include <string>

#include <opencv2/imgproc.hpp>

#include "sensor_msgs/msg/image.hpp"

namespace object_detection
{

/**
 * @brief Converts an image message to BGR8.
 * @param[in] m  image message (bgr8, rgb8, bgra8, rgba8, mono8, uyvy, yuyv)
 * @return BGR8 image; shares memory with the message when no conversion is needed
 * @throws std::runtime_error for an unsupported encoding
 */
inline cv::Mat toBgr(const sensor_msgs::msg::Image & m)
{
  const std::string & e = m.encoding;
  auto * data = const_cast<uint8_t *>(m.data.data());
  if (e == "bgr8") {return cv::Mat(m.height, m.width, CV_8UC3, data, m.step);}
  cv::Mat out;
  if (e == "rgb8") {
    cv::cvtColor(cv::Mat(m.height, m.width, CV_8UC3, data, m.step), out, cv::COLOR_RGB2BGR);
  } else if (e == "bgra8") {
    cv::cvtColor(cv::Mat(m.height, m.width, CV_8UC4, data, m.step), out, cv::COLOR_BGRA2BGR);
  } else if (e == "rgba8") {
    cv::cvtColor(cv::Mat(m.height, m.width, CV_8UC4, data, m.step), out, cv::COLOR_RGBA2BGR);
  } else if (e == "mono8") {
    cv::cvtColor(cv::Mat(m.height, m.width, CV_8UC1, data, m.step), out, cv::COLOR_GRAY2BGR);
  } else if (e == "yuv422" || e == "uyvy") {
    cv::cvtColor(cv::Mat(m.height, m.width, CV_8UC2, data, m.step), out, cv::COLOR_YUV2BGR_UYVY);
  } else if (e == "yuyv" || e == "yuv422_yuy2") {
    cv::cvtColor(cv::Mat(m.height, m.width, CV_8UC2, data, m.step), out, cv::COLOR_YUV2BGR_YUYV);
  } else {
    throw std::runtime_error("unsupported image encoding '" + e + "'");
  }
  return out;
}

}  // namespace object_detection
