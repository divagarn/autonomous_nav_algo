// The ONLY node that writes the ECU serial port (C++ version of buggy_pcd_lanelet_map/uart_writer.py).
//
// Subscribes the commands  /steering_angle [deg, +right], /final_red_detected (stop),
// /final_yellow_detected (2 km/h preset) and sends the 6-byte frame at 10 Hz.
//   * WATCHDOG: if any of the three topics is older than watchdog_sec the frame is forced to STOP
//     (red=1, steering 0).
//   * Starts in STOP and sends STOP (3 frames) on shutdown.
//   * loopback (default TRUE): builds every frame but opens no port and writes nothing.
// Publishes /uart/frame ("yellow=0;red=1;angle=0.00", every frame sent) and /uart/status.
// Never run together with another writer on the same port (uart_sender_node, the Python uart_writer).
#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include "buggy_core/actuation/serial_port.hpp"
#include "buggy_core/actuation/uart_frame.hpp"

using namespace std::chrono_literals;

class UartWriter : public rclcpp::Node
{
public:
  UartWriter()
  : Node("uart_writer")
  {
    const auto port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const auto baud = declare_parameter<int>("baudrate", 115200);
    loopback_ = declare_parameter<bool>("loopback", true);
    watchdog_ = declare_parameter<double>("watchdog_sec", 0.5);
    if (!loopback_) {
      serial_ = std::make_unique<buggy_core::SerialPort>(port, baud);
    }
    const auto now = get_clock()->now();
    t_steer_ = t_red_ = t_yellow_ = now;

    sub_steer_ = create_subscription<std_msgs::msg::Float32>(
      "/steering_angle", 1, [this](std_msgs::msg::Float32::ConstSharedPtr m) {
        steer_ = m->data; t_steer_ = get_clock()->now();
      });
    sub_red_ = create_subscription<std_msgs::msg::Bool>(
      "/final_red_detected", 1, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        red_ = m->data; t_red_ = get_clock()->now();
      });
    sub_yellow_ = create_subscription<std_msgs::msg::Bool>(
      "/final_yellow_detected", 1, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        yellow_ = m->data; t_yellow_ = get_clock()->now();
      });
    status_pub_ = create_publisher<std_msgs::msg::String>("/uart/status", 1);
    frame_pub_ = create_publisher<std_msgs::msg::String>("/uart/frame", 10);
    send_timer_ = create_wall_timer(100ms, [this]() {send();});
    status_timer_ = create_wall_timer(500ms, [this]() {publish_status();});
    RCLCPP_WARN(
      get_logger(), "uart_writer (C++) on %s: %s", port.c_str(),
      loopback_ ? "LOOPBACK (nothing is written)" : "REAL WRITE ENABLED");
  }

  // Called after the executor stopped (Ctrl+C / SIGTERM): leave the buggy explicitly commanded to stop.
  void final_stop()
  {
    RCLCPP_WARN(get_logger(), "shutdown: sending final STOP");
    for (int i = 0; i < 3; ++i) {
      write_frame(false, true, 0.0);
      std::this_thread::sleep_for(20ms);
    }
    if (serial_) {
      serial_->close();
    }
  }

private:
  double age(const rclcpp::Time & t) {return (get_clock()->now() - t).seconds();}

  bool write_frame(bool yellow, bool red, double steer)
  {
    const auto f = buggy_core::encode_frame(yellow, red, steer);
    if (loopback_) {
      return true;
    }
    const bool ok = serial_->write_all(f.data(), f.size());
    if (!ok) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "UART write failed: %s (will retry)",
        serial_->last_error().c_str());
    }
    return ok;
  }

  void send()
  {
    const bool stale = std::max({age(t_steer_), age(t_red_), age(t_yellow_)}) > watchdog_;
    if (stale != stale_) {
      if (stale) {
        RCLCPP_WARN(get_logger(), "commands STALE - forcing STOP");
      } else {
        RCLCPP_WARN(get_logger(), "commands fresh again");
      }
    }
    stale_ = stale;
    const bool yellow = stale ? false : yellow_;
    const bool red = stale ? true : red_;
    const double steer = stale ? 0.0 : steer_;
    write_frame(yellow, red, steer);
    char text[64];
    std::snprintf(text, sizeof(text), "yellow=%d;red=%d;angle=%.2f", yellow, red, steer);
    std_msgs::msg::String msg;
    msg.data = text;
    frame_pub_->publish(msg);
    ++frames_;
  }

  void publish_status()
  {
    std_msgs::msg::String msg;
    msg.data = std::string("mode=") + (loopback_ ? "loopback" : "real") +
      ";state=" + (stale_ ? "stale" : "ok") + ";frames=" + std::to_string(frames_);
    status_pub_->publish(msg);
  }

  bool loopback_{true};
  double watchdog_{0.5};
  std::unique_ptr<buggy_core::SerialPort> serial_;
  double steer_{0.0};
  bool red_{true}, yellow_{false}, stale_{true};
  rclcpp::Time t_steer_, t_red_, t_yellow_;
  uint64_t frames_{0};
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_steer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_red_, sub_yellow_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, frame_pub_;
  rclcpp::TimerBase::SharedPtr send_timer_, status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UartWriter>();
  rclcpp::spin(node);
  node->final_stop();
  rclcpp::shutdown();
  return 0;
}
