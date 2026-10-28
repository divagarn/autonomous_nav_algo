// Mission = an ordered list of named stops (home, A, B ...) on the map. The buggy drives to the first
// stop, waits wait_sec there, drives on to the next one, ... and after the last one either starts again
// (loop) or ends. Pure logic without ROS types; time is passed in as seconds.
//
// It does not drive anything itself: it sends each leg's goal to the route planner (/goal_pose) and asks
// the operation manager for AUTO (/system/request mode:auto) - the same two things the operator does by
// hand - and it watches /system/state (routing=arrived means the stop is reached). Obstacle stops and
// detours stay with the operation manager, they happen inside a leg.
#pragma once

#include <optional>
#include <string>
#include <vector>

namespace buggy_core
{
struct MissionStop
{
  std::string name;
  double x = 0.0, y = 0.0;
  double wait_sec = 10.0;
};

struct MissionParams
{
  std::vector<MissionStop> stops;
  bool loop = true;                 // after the last stop go to the first again
  bool auto_restart = true;         // false: wait for a "resume" request after each stop
  double plan_timeout = 5.0;        // s to wait for the route planner after a goal was sent
  double abort_stop_sec = 2.0;      // AUTO dropped to stop (operator, fault) for this long -> mission aborted
  double auto_grace = 3.0;          // s after "mode:auto" was requested before a stop counts as a drop
  double at_stop_radius = 4.0;      // at start: already this close to a stop -> that stop counts as reached
  bool start_as_home = false;       // the pose the vehicle has at 'start' (set in RViz) becomes the last stop "home"
};

enum class MissionPhase { Idle, Planning, Driving, Waiting, Paused, Done, Aborted };

struct MissionInput
{
  double now = 0.0;
  std::string mode, routing, msg;       // from /system/state
  bool state_fresh = false;             // a /system/state was received in the last 2 s
  bool new_path = false;                // a /planning/route_path arrived since the previous step
  std::optional<std::pair<double, double>> pose;     // vehicle x, y in the map
};

struct MissionActions
{
  std::optional<MissionStop> send_goal;     // publish this as /goal_pose
  bool request_auto = false;                // publish "mode:auto" on /system/request
  bool request_stop = false;                // publish "mode:stop" on /system/request
};

class MissionCore
{
public:
  explicit MissionCore(MissionParams p);

  // "start" (or "start:N" = begin at stop N, 0-based), "stop", "resume"
  void on_request(const std::string & req);
  MissionActions step(const MissionInput & in);

  MissionPhase phase() const {return phase_;}
  const std::string & message() const {return msg_;}
  size_t target_index() const {return target_;}
  const MissionParams & params() const {return p_;}
  // the stops in use: the file's stops, plus "home" (the start pose) when start_as_home is on and a mission started
  const std::vector<MissionStop> & stops() const {return stops_;}
  double wait_remaining(double now) const;
  static const char * phase_name(MissionPhase p);
  // "phase=waiting;target=A;index=1;stops=3;wait_left=6.2;loop=true;msg=..."
  std::string state_string(double now) const;

private:
  size_t next_index(size_t i) const {return (i + 1) % stops_.size();}
  void begin_leg(size_t idx, MissionActions & a, double now);

  MissionParams p_;
  std::vector<MissionStop> stops_;
  MissionPhase phase_ = MissionPhase::Idle;
  size_t target_ = 0;
  bool start_requested_ = false;
  size_t start_index_ = 0;
  bool resume_requested_ = false;
  bool stop_requested_ = false;
  double leg_started_ = 0.0, auto_requested_ = -1e9, last_auto_ = -1e9, wait_until_ = 0.0, drop_since_ = -1.0;
  bool path_seen_ = false;
  size_t legs_done_ = 0;
  std::string msg_ = "idle: send 'start' to /mission/request";
};
}  // namespace buggy_core
