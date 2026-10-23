#include "buggy_core/actuation/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <utility>

namespace buggy_core
{

namespace
{
speed_t baud_constant(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B115200;
  }
}
}  // namespace

SerialPort::SerialPort(std::string path, int baud)
: path_(std::move(path)), baud_(baud) {}

SerialPort::~SerialPort() {close();}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::ensure_open()
{
  if (fd_ >= 0) {
    return true;
  }
  const auto now = std::chrono::steady_clock::now();
  if (now - last_try_ < std::chrono::seconds(1) && last_try_.time_since_epoch().count() != 0) {
    return false;
  }
  last_try_ = now;
  const int fd = ::open(path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    last_error_ = "open " + path_ + ": " + std::strerror(errno);
    return false;
  }
  termios tio{};
  if (tcgetattr(fd, &tio) != 0) {
    last_error_ = "tcgetattr " + path_ + ": " + std::strerror(errno);
    ::close(fd);
    return false;
  }
  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
  tio.c_cflag |= CS8;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, baud_constant(baud_));
  cfsetospeed(&tio, baud_constant(baud_));
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    last_error_ = "tcsetattr " + path_ + ": " + std::strerror(errno);
    ::close(fd);
    return false;
  }
  tcflush(fd, TCIOFLUSH);
  fd_ = fd;
  last_error_.clear();
  return true;
}

bool SerialPort::write_all(const uint8_t * data, size_t size)
{
  if (!ensure_open()) {
    return false;
  }
  size_t done = 0;
  int waits = 0;
  while (done < size) {
    const ssize_t n = ::write(fd_, data + done, size - done);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      if ((errno == EAGAIN || errno == EWOULDBLOCK) && waits++ < 5) {
        pollfd pfd{fd_, POLLOUT, 0};
        ::poll(&pfd, 1, 20);          // output buffer full: wait a moment, then retry
        continue;
      }
      last_error_ = std::string("write ") + path_ + ": " + std::strerror(errno);
      close();
      return false;
    }
    done += static_cast<size_t>(n);
  }
  return true;
}

int SerialPort::read_some(uint8_t * data, size_t size, int timeout_ms)
{
  if (!ensure_open()) {
    return -1;
  }
  pollfd pfd{fd_, POLLIN, 0};
  const int pr = ::poll(&pfd, 1, timeout_ms);
  if (pr < 0 && errno != EINTR) {
    last_error_ = std::string("poll ") + path_ + ": " + std::strerror(errno);
    close();
    return -1;
  }
  if (pr <= 0) {
    return 0;
  }
  const ssize_t n = ::read(fd_, data, size);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return 0;
    }
    last_error_ = std::string("read ") + path_ + ": " + std::strerror(errno);
    close();
    return -1;
  }
  if (n == 0 && (pfd.revents & (POLLHUP | POLLERR))) {      // unplugged adapter
    last_error_ = "read " + path_ + ": device closed";
    close();
    return -1;
  }
  return static_cast<int>(n);
}

}  // namespace buggy_core
