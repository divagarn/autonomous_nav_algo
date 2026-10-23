#include "buggy_core/actuation/feedback_parser.hpp"

#include <algorithm>
#include <cstdio>

namespace buggy_core
{
std::vector<Feedback> FeedbackParser::feed(const uint8_t * data, size_t n, std::vector<std::string> * warnings)
{
  std::vector<Feedback> out;
  buf_.insert(buf_.end(), data, data + n);
  while (buf_.size() >= kPacketSize) {
    const auto it = std::find(buf_.begin(), buf_.end(), kStart);
    if (it == buf_.end()) {buf_.clear(); break;}                 // no start byte: drop everything
    if (it != buf_.begin()) {buf_.erase(buf_.begin(), it); continue;}   // discard data before the start byte
    if (buf_.size() < kPacketSize) {break;}
    if (buf_[5] != kEnd) {
      if (warnings) {
        char b[160];
        std::snprintf(b, sizeof(b), "[UART] Invalid packet markers: start=0x%x end=0x%x (expected start=0x%x, end=0x%x)",
          buf_[0], buf_[5], kStart, kEnd);
        warnings->push_back(b);
      }
    } else {
      const int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(buf_[1]) << 8) | buf_[2]);
      double deg = raw * 0.01 + (-30.0);
      deg = std::max(std::min(deg, 20.0), -20.0);
      out.push_back({deg, static_cast<double>(buf_[3])});
    }
    buf_.erase(buf_.begin(), buf_.begin() + kPacketSize);
  }
  return out;
}
}  // namespace buggy_core
