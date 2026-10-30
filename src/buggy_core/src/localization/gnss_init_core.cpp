#include "buggy_core/localization/gnss_init_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace buggy_core
{
GnssInitCore::GnssInitCore(const GnssInitParams & p, const Projector & proj)
: p_(p), proj_(proj)
{
  if (!proj_.georeferenced()) {
    status_ = "disabled: the map is Local (map_projector_info.yaml has no LocalCartesian origin), GNSS cannot place the vehicle";
  }
}

void GnssInitCore::set_enabled(bool on)
{
  if (on == enabled_) {return;}
  enabled_ = on;
  good_since_ = -1.0;                       // after switching on the fix must be stable again
  bad_since_ = -1.0;
  attempts_ = 0;
  status_ = on ? "enabled: waiting for a stable GNSS fix" : "disabled by the operator";
}

InitPose GnssInitCore::to_map(const GnssFix & f) const
{
  const auto xy = proj_.to_xy(f.lat, f.lon);
  // heading: degrees clockwise from north -> radians counter-clockwise from east (the map x axis is East)
  double yaw = (90.0 - f.heading_cw_deg + p_.heading_offset_deg) * M_PI / 180.0;
  yaw = std::atan2(std::sin(yaw), std::cos(yaw));
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {xy.first + c * p_.lever_x - s * p_.lever_y, xy.second + s * p_.lever_x + c * p_.lever_y, yaw, f.std_m};
}

bool GnssInitCore::fix_ok(double now, std::string * why) const
{
  if (!fix_) {*why = "no GNSS message yet"; return false;}
  if (now - fix_->time > p_.fix_timeout) {*why = "GNSS fix is old (no message for > fix_timeout)"; return false;}
  if (std::find(p_.accepted_states.begin(), p_.accepted_states.end(), fix_->state) == p_.accepted_states.end()) {
    char b[120];
    std::snprintf(b, sizeof(b), "GNSS state %d is not accepted (need RTK fixed with heading)", fix_->state);
    *why = b;
    return false;
  }
  if (!(fix_->std_m <= p_.max_std_m)) {
    char b[120];
    std::snprintf(b, sizeof(b), "position std %.2f m > %.2f m", fix_->std_m, p_.max_std_m);
    *why = b;
    return false;
  }
  return true;
}

std::optional<InitPose> GnssInitCore::step(double now, const std::string & loc_state)
{
  if (!proj_.georeferenced()) {return std::nullopt;}
  if (!enabled_) {return std::nullopt;}
  if (loc_state == "converged") {
    attempts_ = 0; bad_since_ = -1.0;
    status_ = "localized";
    return std::nullopt;
  }
  if (loc_state == "aligning" || loc_state == "initialized") {
    status_ = "NDT is aligning the GNSS pose";
    bad_since_ = -1.0;
    return std::nullopt;
  }
  // uninitialized, lost or align_failed
  if (bad_since_ < 0.0) {bad_since_ = now;}
  std::string why;
  if (!fix_ok(now, &why)) {
    good_since_ = -1.0;
    status_ = "waiting: " + why;
    return std::nullopt;
  }
  if (good_since_ < 0.0) {good_since_ = now;}
  if (now - good_since_ < p_.stable_sec) {
    char b[120];
    std::snprintf(b, sizeof(b), "GNSS fix good for %.1f s of %.1f s", now - good_since_, p_.stable_sec);
    status_ = b;
    return std::nullopt;
  }
  if (loc_state != "uninitialized") {
    if (!p_.reinit_when_lost) {status_ = "lost: reinit_when_lost is off"; return std::nullopt;}
    if (now - bad_since_ < p_.lost_sec) {status_ = "lost: waiting before a new GNSS start pose"; return std::nullopt;}
  }
  if (attempts_ >= p_.max_attempts) {
    status_ = "giving up: " + std::to_string(attempts_) + " GNSS start poses did not converge (click 2D Pose Estimate)";
    return std::nullopt;
  }
  if (now - last_attempt_ < p_.retry_sec) {status_ = "waiting for NDT to converge from the last GNSS start pose"; return std::nullopt;}
  last_attempt_ = now;
  ++attempts_;
  status_ = "sent GNSS start pose (attempt " + std::to_string(attempts_) + ")";
  return to_map(*fix_);
}
}  // namespace buggy_core
