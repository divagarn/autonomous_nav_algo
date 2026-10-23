// NDT localization supervisor logic without ROS types: state machine, TF watchdog (two parents
// for one frame, map->base_link jumps) and the /localization/state text.
// Port of the logic of buggy_pcd_lanelet_map/localization_manager.py.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace buggy_core
{
struct LocParams
{
  std::string map_frame = "map", base_frame = "base_link";
  double score_ok = 1.4, lost_timeout = 3.0, jump_m = 0.5;
};

struct TfEdge { std::string child, parent; double x, y; };
struct MapPose { double x, y, yaw_deg; };

class LocalizationCore
{
public:
  explicit LocalizationCore(LocParams p = {}) : p_(std::move(p)) {}

  // ---- inputs ----
  void on_tf(const std::vector<TfEdge> & edges, double now);
  void on_ndt_pose(double now) { last_ndt_ = now; }
  void reset_ndt() { last_ndt_.reset(); }
  void set_score(double v) { score_ = v; }
  void set_iterations(double v) { iters_ = v; }
  void set_exe_ms(double v) { exe_ms_ = v; }
  void set_state(const std::string & state, const std::string & msg) { state_ = state; msg_ = msg; }
  void set_activated(bool a) { activated_ = a; }
  void set_msg(const std::string & m) { msg_ = m; }

  // ---- cycle ----
  void update_state(double now);
  // "k=v;..." for /localization/state; `pose` is the map->base_link lookup if available
  std::string state_string(double now, const std::optional<MapPose> & pose);
  // 'child<-parentA+parentB' for frames with several recent parents, joined by ', '
  std::string tf_conflict(double now) const;
  // true once, when the conflict text has just changed (for the error log)
  bool conflict_changed(const std::string & conflict);

  const std::string & state() const { return state_; }
  const std::string & msg() const { return msg_; }
  bool activated() const { return activated_; }

private:
  LocParams p_;
  std::string state_ = "uninitialized", msg_ = "Give an initial pose (2D Pose Estimate)";
  bool activated_ = false;
  double score_ = 0.0, iters_ = 0.0, exe_ms_ = 0.0;
  std::optional<double> last_ndt_;
  // child -> {parent -> last receive time}, children in insertion order like the Python dict
  std::vector<std::pair<std::string, std::map<std::string, double>>> parents_;
  std::vector<double> jumps_;
  std::optional<std::pair<double, double>> last_xy_;
  std::string last_conflict_;
};
}  // namespace buggy_core
