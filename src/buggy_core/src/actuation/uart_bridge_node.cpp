// C++ version of buggy_uart/uart_bridge.py: ECU serial feedback -> /wheel_uart (SteerSpeed).
// Polls the port every 10 ms without blocking the executor (the Python node waits up to 50 ms
// per read); the packet framing is in actuation/feedback_parser.cpp.
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <ground_segmentation_msgs/msg/steer_speed.hpp>
#include <rclcpp/rclcpp.hpp>

#include "buggy_core/actuation/feedback_parser.hpp"
#include "buggy_core/actuation/serial_port.hpp"

class UartBridge : public rclcpp::Node
{
public:
  UartBridge()
  : Node("uart_bridge")
  {
    const auto port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const int baud = static_cast<int>(declare_parameter<int>("baud", 115200));
    serial_ = std::make_unique<buggy_core::SerialPort>(port, baud);
    if (!serial_->ensure_open()) {
      RCLCPP_FATAL(get_logger(), "UART open failed: %s", serial_->last_error().c_str());
      throw std::runtime_error("UART open failed");
    }
    RCLCPP_INFO(get_logger(), "UART bridge: %s @ %d baud", port.c_str(), baud);
    pub_ = create_publisher<ground_segmentation_msgs::msg::SteerSpeed>("/wheel_uart", 50);
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { poll(); });
  }

private:
  void poll()
  {
    uint8_t buf[128];
    const int n = serial_->read_some(buf, sizeof(buf), 0);
    if (n < 0) {
      if (!read_failed_) {RCLCPP_ERROR(get_logger(), "Serial read error: %s", serial_->last_error().c_str());}
      read_failed_ = true;
      parser_.clear();
      return;
    }
    if (read_failed_) {RCLCPP_INFO(get_logger(), "Serial port is back"); read_failed_ = false;}
    if (n == 0) {return;}
    std::vector<std::string> warnings;
    for (const auto & f : parser_.feed(buf, static_cast<size_t>(n), &warnings)) {
      ground_segmentation_msgs::msg::SteerSpeed m;
      m.header.stamp = now();
      m.steering = static_cast<float>(f.steering_deg);
      m.speed = static_cast<float>(f.speed_kmh);
      pub_->publish(m);
    }
    for (const auto & w : warnings) {RCLCPP_WARN(get_logger(), "%s", w.c_str());}
  }

  std::unique_ptr<buggy_core::SerialPort> serial_;
  buggy_core::FeedbackParser parser_;
  bool read_failed_ = false;
  rclcpp::Publisher<ground_segmentation_msgs::msg::SteerSpeed>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<UartBridge>());
  } catch (const std::runtime_error &) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
