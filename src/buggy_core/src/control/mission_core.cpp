#include "buggy_core/control/mission_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace buggy_core
{
MissionCore::MissionCore(MissionParams p)
: p_(std::move(p)), stops_(p_.stops)
{
  if (stops_.empty()) {
    phase_ = MissionPhase::Aborted;
    msg_ = "no stops in the mission file";
  }
}

const char * MissionCore::phase_name(MissionPhase p)
{
  switch (p) {
    case MissionPhase::Idle: return "idle";
    case MissionPhase::Planning: return "planning";
    case MissionPhase::Driving: return "driving";
    case MissionPhase::Waiting: return "waiting";
    case MissionPhase::Paused: return "paused";
    case MissionPhase::Done: return "done";
    case MissionPhase::Aborted: return "aborted";
  }
  return "?";
}

void MissionCore::on_request(const std::string & req)
{
  if (req.rfind("start", 0) == 0) {
    if (stops_.empty()) {return;}
    start_index_ = 0;
    const auto c = req.find(':');
    if (c != std::string::npos) {
      start_index_ = static_cast<size_t>(std::max(0, std::atoi(req.c_str() + c + 1)));
      if (start_index_ >= stops_.size()) {start_index_ = 0;}
    }
    start_requested_ = true;
  } else if (req == "stop") {
    stop_requested_ = true;
  } else if (req == "resume") {
    resume_requested_ = true;
  }
}

double MissionCore::wait_remaining(double now) const
{
  return phase_ == MissionPhase::Waiting ? std::max(wait_until_ - now, 0.0) : 0.0;
}

void MissionCore::begin_leg(size_t idx, MissionActions & a, double now)
{
  target_ = idx;
  phase_ = MissionPhase::Planning;
  leg_started_ = now;
  path_seen_ = false;
  drop_since_ = -1.0;
  a.send_goal = stops_[idx];
  msg_ = "going to " + stops_[idx].name;
}

MissionActions MissionCore::step(const MissionInput & in)
{
  MissionActions a;
  if (stop_requested_) {
    stop_requested_ = false;
    start_requested_ = resume_requested_ = false;
    if (phase_ != MissionPhase::Idle && phase_ != MissionPhase::Done && phase_ != MissionPhase::Aborted) {
      a.request_stop = true;
      msg_ = "mission stopped by the operator";
    }
    phase_ = MissionPhase::Idle;
    return a;
  }
  if (start_requested_) {
    start_requested_ = false;
    if (stops_.empty()) {return a;}
    if (!in.state_fresh) {
      msg_ = "cannot start: no /system/state (is the operation manager running?)";
      phase_ = MissionPhase::Aborted;
      return a;
    }
    stops_ = p_.stops;                                        // a new run: drop the previous start pose
    if (p_.start_as_home) {
      // a fixed "home" stop in the file wins over the start pose
      const bool has_home = std::any_of(stops_.begin(), stops_.end(), [](const MissionStop & s) {return s.name == "home";});
      if (has_home) {
        // nothing to add
      } else if (in.pose) {
        stops_.push_back({"home", in.pose->first, in.pose->second, stops_.back().wait_sec});
      } else {
        msg_ = "no vehicle pose: set the start pose in RViz (2D Pose Estimate) first";
        phase_ = MissionPhase::Aborted;
        return a;
      }
    }
    size_t idx = start_index_;
    // already standing at that stop: it counts as reached, go on to the next one
    if (in.pose && stops_.size() > 1 &&
      std::hypot(in.pose->first - stops_[idx].x, in.pose->second - stops_[idx].y) < p_.at_stop_radius)
    {
      idx = next_index(idx);
    }
    legs_done_ = 0;
    begin_leg(idx, a, in.now);
    return a;
  }

  switch (phase_) {
    case MissionPhase::Planning: {
      if (in.new_path) {path_seen_ = true;}
      if (path_seen_ && in.routing == "set") {
        phase_ = MissionPhase::Driving;
        auto_requested_ = last_auto_ = in.now;
        a.request_auto = true;
        msg_ = "driving to " + stops_[target_].name;
      } else if (in.now - leg_started_ > p_.plan_timeout) {
        phase_ = MissionPhase::Aborted;
        msg_ = "no route to " + stops_[target_].name + " (the route planner gave no path)";
      }
      break;
    }
    case MissionPhase::Driving: {
      if (in.routing == "arrived") {
        ++legs_done_;
        const auto & s = stops_[target_];
        phase_ = MissionPhase::Waiting;
        wait_until_ = in.now + s.wait_sec;
        msg_ = "arrived at " + s.name + ", waiting";
        break;
      }
      if (in.mode == "auto") {
        drop_since_ = -1.0;
      } else {
        // AUTO not (yet) on: retry the request during the grace time, afterwards it is a drop
        if (in.now - auto_requested_ < p_.auto_grace) {
          if (in.now - last_auto_ >= 1.0) {a.request_auto = true; last_auto_ = in.now;}
        } else {
          if (drop_since_ < 0.0) {drop_since_ = in.now;}
          if (in.now - drop_since_ >= p_.abort_stop_sec) {
            phase_ = MissionPhase::Aborted;
            msg_ = "AUTO stopped on the way to " + stops_[target_].name + (in.msg.empty() ? "" : ": " + in.msg);
          }
        }
      }
      break;
    }
    case MissionPhase::Waiting: {
      if (in.now < wait_until_) {break;}
      const bool last = target_ + 1 == stops_.size();
      if (last && !p_.loop) {
        phase_ = MissionPhase::Done;
        msg_ = "mission finished at " + stops_[target_].name;
        break;
      }
      if (!p_.auto_restart) {
        phase_ = MissionPhase::Paused;
        resume_requested_ = false;
        msg_ = "waiting at " + stops_[target_].name + ": send 'resume' to go on";
        break;
      }
      begin_leg(next_index(target_), a, in.now);
      break;
    }
    case MissionPhase::Paused: {
      if (resume_requested_) {
        resume_requested_ = false;
        begin_leg(next_index(target_), a, in.now);
      }
      break;
    }
    default: break;
  }
  return a;
}

std::string MissionCore::state_string(double now) const
{
  const std::string tgt = stops_.empty() ? "-" : stops_[std::min(target_, stops_.size() - 1)].name;
  char b[400];
  std::snprintf(b, sizeof(b), "phase=%s;target=%s;index=%zu;stops=%zu;legs_done=%zu;wait_left=%.1f;loop=%s;msg=%s",
    phase_name(phase_), tgt.c_str(), target_, stops_.size(), legs_done_, wait_remaining(now),
    p_.loop ? "true" : "false", msg_.c_str());
  return b;
}
}  // namespace buggy_core
