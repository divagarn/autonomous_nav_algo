// Automatic start pose from GNSS (what Autoware's gnss_poser + pose_initializer do), without any click.
//
// GNSS fix (latitude, longitude, dual-antenna heading) -> pose in the map frame through the map projection
// (LocalCartesian, see projection.hpp) -> /initialpose, the same input as the RViz "2D Pose Estimate": the
// localization_manager then lets NDT refine it (search around the seed) and starts the EKF.
//
// It only sends a pose when
//   * the localization is not running (uninitialized, or lost / align_failed for lost_sec),
//   * the fix has been good for stable_sec without a break (state in accepted_states, position std <= max_std_m,
//     heading valid, fresh), and
//   * the last attempt is older than retry_sec and fewer than max_attempts were made since the last convergence.
// Pure logic without ROS types; time is passed in as seconds.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "buggy_core/localization/projection.hpp"

namespace buggy_core
{
struct GnssFix
{
  double lat = 0.0, lon = 0.0;
  double heading_cw_deg = 0.0;       // dual-antenna heading, degrees clockwise from north (devpvt.heading2)
  int state = 0;                     // GNSS state (devpvt.stat[1]); 4 = RTK fixed with heading
  double std_m = 1e9;                // horizontal position std [m]
  double time = 0.0;                 // receive time [s]
};

struct GnssInitParams
{
  std::vector<int> accepted_states{4};     // 4 = RTK fixed + heading; add 5 for RTK float + heading
  double max_std_m = 0.3;
  double stable_sec = 3.0;
  double fix_timeout = 1.0;                // a fix older than this is no fix
  double retry_sec = 20.0;
  int max_attempts = 3;
  bool reinit_when_lost = true;
  double lost_sec = 5.0;                   // lost / align_failed for this long -> try again
  // GNSS position -> rear axle (base_link), in the vehicle frame [m]: x forward, y left. Antenna 0.8 m ahead of the
  // rear axle: lever_x = -0.8. Zero = the GNSS position is used as the rear axle position.
  double lever_x = 0.0, lever_y = 0.0;
  double heading_offset_deg = 0.0;         // added to the vehicle yaw (antenna baseline not parallel to the vehicle)
};

struct InitPose { double x, y, yaw; double std_m; };

class GnssInitCore
{
public:
  GnssInitCore(const GnssInitParams & p, const Projector & proj);

  void on_fix(const GnssFix & f) {fix_ = f;}
  // the operator's switch (RViz button): off = never send a pose; turning it on starts the stable-fix time again
  void set_enabled(bool on);
  bool enabled() const {return enabled_;}
  const std::optional<GnssFix> & fix() const {return fix_;}
  // is the last fix one that would be accepted now (state, std, fresh)? why not -> text
  bool fix_good(double now, std::string * why) const {return fix_ok(now, why);}
  // loc_state: "uninitialized" | "aligning" | "initialized" | "converged" | "lost" | "align_failed"
  std::optional<InitPose> step(double now, const std::string & loc_state);

  // the pose a fix means in the map frame (lever arm and heading applied), without any gating
  InitPose to_map(const GnssFix & f) const;
  bool usable() const {return proj_.georeferenced();}
  const std::string & status() const {return status_;}
  int attempts() const {return attempts_;}

private:
  bool fix_ok(double now, std::string * why) const;

  GnssInitParams p_;
  Projector proj_;
  std::optional<GnssFix> fix_;
  bool enabled_ = true;
  double good_since_ = -1.0, bad_since_ = -1.0, last_attempt_ = -1e9;
  int attempts_ = 0;
  std::string status_ = "waiting for a GNSS fix";
};
}  // namespace buggy_core
