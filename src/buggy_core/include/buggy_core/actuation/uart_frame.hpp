// The 6-byte frame the buggy ECU expects:  '(' yellow red angle_hi angle_lo ')'
//   yellow/red : 0 or 1  (red = stop, yellow = 2 km/h speed preset instead of 4 km/h)
//   angle      : steering in degrees, +right, clamped to +-20; sent as int16 big-endian of
//                (angle + 30) / 0.01   (identical to buggy_uart/uart_interface.py)
#ifndef BUGGY_CORE__ACTUATION__UART_FRAME_HPP_
#define BUGGY_CORE__ACTUATION__UART_FRAME_HPP_

#include <algorithm>
#include <array>
#include <cstdint>

namespace buggy_core
{

constexpr uint8_t kFrameStart = 40;   // '('
constexpr uint8_t kFrameEnd = 41;     // ')'
constexpr double kAngleOffset = -30.0;
constexpr double kAngleResolution = 0.01;
constexpr double kMaxSteerDeg = 20.0;

inline std::array<uint8_t, 6> encode_frame(bool yellow, bool red, double degree)
{
  degree = std::max(std::min(degree, kMaxSteerDeg), -kMaxSteerDeg);
  // Python: int((degree - OFFSET) / RESOLUTION): truncation toward zero, same as the cast
  int raw = static_cast<int>((degree - kAngleOffset) / kAngleResolution);
  raw = std::max(std::min(raw, 32767), -32768);
  const auto u = static_cast<uint16_t>(static_cast<int16_t>(raw));
  return {kFrameStart, static_cast<uint8_t>(yellow ? 1 : 0), static_cast<uint8_t>(red ? 1 : 0),
    static_cast<uint8_t>(u >> 8), static_cast<uint8_t>(u & 0xFF), kFrameEnd};
}

}  // namespace buggy_core

#endif  // BUGGY_CORE__ACTUATION__UART_FRAME_HPP_
