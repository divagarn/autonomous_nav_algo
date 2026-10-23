// Bicycle-model dead reckoning from the ECU feedback (steering deg +right, speed km/h).
// Port of buggy_perception/wheel_odometry.py; no ROS types.
#pragma once

#include <cmath>

namespace buggy_core
{
struct WheelOdomState
{
  double x = 0.0, y = 0.0, yaw = 0.0;
  double speed_ms = 0.0, angular_z = 0.0;     // of the last update
};

class WheelIntegrator
{
public:
  explicit WheelIntegrator(double wheelbase = 1.6)
  : wheelbase_(wheelbase) {}

  // dt in seconds (> 0). Positive steering = right, ROS angular.z positive = left, hence the minus.
  const WheelOdomState & update(double dt, double steering_deg, double speed_kmh)
  {
    const double v = speed_kmh / 3.6;
    const double steering_rad = steering_deg * M_PI / 180.0;
    const double angular_z = std::abs(v) > 1e-6 ? -std::tan(steering_rad) * v / wheelbase_ : 0.0;
    s_.x += v * std::cos(s_.yaw) * dt;
    s_.y += v * std::sin(s_.yaw) * dt;
    s_.yaw += angular_z * dt;
    s_.speed_ms = v;
    s_.angular_z = angular_z;
    return s_;
  }
  const WheelOdomState & state() const { return s_; }

private:
  double wheelbase_;
  WheelOdomState s_;
};
}  // namespace buggy_core
