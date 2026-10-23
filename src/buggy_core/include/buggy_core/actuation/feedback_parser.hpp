// ECU feedback stream -> steering / speed. 6-byte packets '(' s_hi s_lo speed ... ')' - the same
// framing and resync rules as buggy_uart/uart_bridge.py (the Python file is the reference):
//   steering = int16 big-endian (bytes 1-2) * 0.01 - 30, clamped to +-20 deg (+right)
//   speed    = byte 3 (km/h)
// A packet whose end byte is not ')' is reported and its 6 bytes dropped; with no start byte in
// the buffer everything is dropped.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace buggy_core
{
struct Feedback { double steering_deg; double speed_kmh; };

class FeedbackParser
{
public:
  static constexpr uint8_t kStart = 0x28, kEnd = 0x29;
  static constexpr size_t kPacketSize = 6;

  // Append bytes and return every complete valid packet. Problems are appended to `warnings`.
  std::vector<Feedback> feed(const uint8_t * data, size_t n, std::vector<std::string> * warnings = nullptr);
  void clear() { buf_.clear(); }
  size_t buffered() const { return buf_.size(); }

private:
  std::vector<uint8_t> buf_;
};
}  // namespace buggy_core
