// Live-mode logic of operation_manager.py without any ROS types: AUTO state machine, stop model,
// obstacle memory, hold / slow decisions, pure pursuit and the detour hand-over.
// Time is always passed in as seconds, so the logic is testable and independent of the clock.
// Port of buggy_pcd_lanelet_map/operation_manager.py (live:=true branch); the Python file is the
// reference.
#pragma once

#include <array>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace buggy_core
{
constexpr int kSearchWindow = 60;       // path points searched ahead when matching

struct PathPt { double x, y, z, qz, qw; };
struct Obj { double x, y, yaw, length, width; int label; };
struct NameTag { double x, y; std::string name; };
struct Pose { double x, y, yaw, z; };       // rear axle
struct Command { double steer; bool red; bool yellow; };
struct Obstacle { double dist; int label; std::string cam; };

struct CoreParams
{
  double limit_kmh = 5.0, wheelbase = 1.6, max_steer = 20.0, lookahead = 4.0;
  double front_offset = 2.2;      // rear axle -> front bumper
  double detect_range = 30.0;     // obstacles up to this far ahead along the path count
  double steer_rate = 40.0;
  bool drive_enabled = false, base_at_center = false;
  double safe_half_width = 1.25, max_cte = 2.5, loc_grace = 1.0;
  double react = 1.0, decel = 0.8, stop_margin = 2.5, slow_zone = 6.0, goal_margin = 0.5;
  bool require_detector = true, radar_enabled = false, camera_enabled = false;
  bool avoid_enabled = false;
  double avoid_after = 5.0, avoid_hw = 1.05, avoid_retry = 5.0;
  // replan only after the vehicle STOPPED for avoid_after s; while detouring go back to the original
  // route when the obstacle has been gone for avoid_clear_sec. The buggy has no reverse and a ~4.4 m
  // turning radius, so a detour from a standstill needs ~7 m in front of the obstacle: with avoidance
  // on the stop line is kept at least avoid_stop_margin away.
  double avoid_clear_sec = 2.0, avoid_stop_margin = 7.5, stop_speed = 0.15;
  bool avoid_stop_early = false;  // true: with avoidance stop far enough before the obstacle for a detour
  // with avoid_stop_early the stop line is ALWAYS avoid_stop_margin. avoid_probe: ask the planner how much room a
  // detour needs while approaching, show it (state key detour_room), warn when the stop line is closer.
  // avoid_stop_follow_planner: that answer also moves the stop line back (up to 20 m); default false.
  bool avoid_probe = true;
  bool avoid_stop_follow_planner = false;
  int avoid_max_tries = 6;
  double memory_sec = 4.0, memory_max_sec = 25.0, creep_sec = 10.0;
  int memory_min_hits = 5;
  double dt = 0.05;
};

enum class Level { Info, Warn, Error };

// what the core needs from the outside world
struct Hooks
{
  std::function<void(Level, const std::string &)> log;
  std::function<void(const std::string &)> avoid_request;       // JSON for /planning/avoid_request
  std::function<void()> detour_clear;
  // detour points + per-point "inside the lane" flags (cyan inside, magenta outside)
  std::function<void(const std::vector<std::array<double, 3>> &, const std::vector<bool> &)> detour_show;
  std::function<std::optional<Pose>()> pose;                    // fresh map->base_link, or none
};

class OperationCore
{
public:
  struct Pending { int id; double t; size_t idx; };

  OperationCore(CoreParams p, Hooks h);

  // ---- calibration ----
  bool load_calibration(const std::string & path);    // react_sec / decel_ms2 from the yaml
  double stopping_distance(double v) const { return v * p_.react + v * v / (2.0 * p_.decel); }

  // ---- inputs ----
  void on_path(const std::vector<PathPt> & pts);
  void on_loc(const std::string & data, double now);
  void on_uart(const std::string & data, double now);
  void on_wheel(double steering_deg, double speed_kmh);
  void on_dummy(std::vector<Obj> o) { dummy_ = std::move(o); }
  void on_lidar(std::vector<Obj> o, double now) { lidar_ = std::move(o); lidar_time_ = now; }
  void on_radar(std::vector<Obj> o, double now) { radar_ = std::move(o); radar_time_ = now; }
  void on_names(const std::string & json, double now);
  void on_calib(bool active, double now) { if (active) {calib_time_ = now;} }
  void on_request(const std::string & data);
  void on_avoid_result(const std::string & json, double now);

  // ---- cycle ----
  // One control step. nullopt = brake_calibration owns the command topics right now.
  std::optional<Command> step_live(double now);
  void match();                                       // advance idx to the nearest path point
  std::string state_string(double now) const;
  bool path_ready() const { return routing_ == "set" && path_.size() >= 2; }

  // ---- queries used by the zone display ----
  std::optional<Obstacle> obstacle_ahead(double now) const;
  static std::string obs_name(const Obstacle & o);
  const std::vector<PathPt> & path() const { return path_; }
  const std::vector<double> & seg() const { return seg_; }
  size_t idx() const { return idx_; }
  bool localized() const { return localized_; }
  const CoreParams & params() const { return p_; }
  double v() const { return v_; }
  const std::string & routing() const { return routing_; }

  // ---- exposed for the unit tests ----
  double pure_pursuit() const;
  double cross_track() const;
  std::optional<double> block_dist(const Obj & o) const;
  void set_pose(double x, double y, double yaw) { rx_ = x; ry_ = y; ryaw_ = yaw; }
  void set_speed(double v) { v_ = v; }
  void set_idx(size_t i) { idx_ = i; }
  void set_avoiding(bool a) { avoiding_ = a; }
  void set_pending(int id, double t, size_t idx) { avoid_pending_ = Pending{id, t, idx}; }
  size_t avoid_end_idx() const { return avoid_end_idx_; }
  bool avoiding() const { return avoiding_; }
  const std::string & message() const { return msg_; }
  const std::string & avoid_status() const { return avoid_status_; }

private:
  struct Mem { Obj obj; double t; std::optional<double> hold_t; int n; };

  void rebuild_seg();
  std::vector<Obj> all_objects(double now) const;
  std::vector<Obj> remembered(double now) const;
  std::optional<std::string> name_near(double x, double y, double now) const;
  void update_memory(double now);
  void update_avoidance(const std::optional<Obstacle> & obs, double now, bool stopped);
  void check_obstacle_gone(double now);
  void revert_to_original();
  void request_avoidance(double now, bool probe = false);
  std::string live_fault(const std::optional<Pose> & pose, double now) const;
  bool auto_available() const;
  std::string drive_text(double now) const;
  std::string sensors_text(double now) const;
  void log(Level l, const std::string & s) const { if (h_.log) {h_.log(l, s);} }

  CoreParams p_;
  Hooks h_;
  std::string calib_source_ = "default (NOT measured)";

  double cmd_steer_ = 0.0, preset_kmh_ = 4.0;
  bool cmd_go_ = false, hold_obstacle_ = false;
  std::optional<double> clear_since_;
  double calib_time_ = -10.0, lidar_time_ = -10.0, radar_time_ = -10.0, names_time_ = -10.0;
  double creep_until_ = 0.0, diag_t_ = 0.0, loc_ok_time_ = 0.0, uart_time_ = 0.0;
  std::optional<double> last_gap_;

  bool avoiding_ = false;
  size_t avoid_end_idx_ = 0;
  std::optional<double> block_since_;
  double block_last_ = 0.0, avoid_next_ = 0.0;
  int avoid_id_ = 0, avoid_tries_ = 0;
  std::optional<Pending> avoid_pending_;
  std::vector<PathPt> orig_path_;
  std::vector<double> orig_seg_;
  size_t avoid_req_idx_ = 0, avoid_ib_ = 0;
  std::vector<std::array<double, 3>> avoid_blockers_, pending_blockers_;   // x, y, half extent
  std::optional<double> gone_since_;
  double restored_until_ = 0.0, last_now_ = 0.0, avoid_wait_ = 0.0;
  // planner probe: how much room a detour needs; moves the stop line (see update_avoidance)
  double stop_margin_base_ = 2.5, room_time_ = -10.0, probe_next_ = 0.0;
  std::optional<double> room_needed_;
  bool room_warned_ = false;
  double last_obs_time_ = 0.0;
  std::optional<std::pair<int, double>> probe_pending_;
  std::string avoid_status_ = "idle", avoid_note_;

  std::vector<Mem> memory_;
  std::vector<Obj> dummy_, lidar_, radar_;
  std::vector<NameTag> names_;
  std::map<std::string, std::string> loc_, uart_status_;

  std::string mode_ = "stop", routing_ = "unset", gear_ = "park", turn_signal_ = "none", msg_;
  bool localized_ = false, emergency_ = false;
  std::vector<PathPt> path_;
  std::vector<double> seg_;
  size_t idx_ = 0;
  double v_ = 0.0, rx_ = 0.0, ry_ = 0.0, ryaw_ = 0.0, rz_ = 0.0, steer_ = 0.0, cte_ = 0.0;
};

std::map<std::string, std::string> parse_kv(const std::string & s);
}  // namespace buggy_core
