// Minimal POSIX serial port (8N1, raw, no flow control) that reopens itself after an error.
#ifndef BUGGY_CORE__ACTUATION__SERIAL_PORT_HPP_
#define BUGGY_CORE__ACTUATION__SERIAL_PORT_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace buggy_core
{

class SerialPort
{
public:
  SerialPort(std::string path, int baud);
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Open if closed. Retries at most once per second. True when the port is open.
  bool ensure_open();
  // Write everything or fail; a failure closes the port (ensure_open() reopens it later).
  bool write_all(const uint8_t * data, size_t size);
  // Read up to `size` bytes, waiting at most timeout_ms for the first byte. Returns the count (0 when
  // nothing arrived), -1 on error (the port is closed; ensure_open() reopens it later).
  int read_some(uint8_t * data, size_t size, int timeout_ms);
  void close();
  bool is_open() const {return fd_ >= 0;}
  const std::string & last_error() const {return last_error_;}

private:
  std::string path_;
  int baud_;
  int fd_{-1};
  std::string last_error_;
  std::chrono::steady_clock::time_point last_try_{};
};

}  // namespace buggy_core

#endif  // BUGGY_CORE__ACTUATION__SERIAL_PORT_HPP_
