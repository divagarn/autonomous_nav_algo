#include "buggy_core/localization/localization_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace buggy_core
{
namespace
{
std::string fmt(const char * f, double v)
{
  char b[64];
  std::snprintf(b, sizeof(b), f, v);
  return b;
}
}  // namespace

void LocalizationCore::on_tf(const std::vector<TfEdge> & edges, double now)
{
  for (const auto & e : edges) {
    auto it = std::find_if(parents_.begin(), parents_.end(), [&](const auto & c) { return c.first == e.child; });
    if (it == parents_.end()) {parents_.push_back({e.child, {}}); it = parents_.end() - 1;}
    it->second[e.parent] = now;
    if (e.parent == p_.map_frame && e.child == p_.base_frame) {
      if (last_xy_ && std::hypot(e.x - last_xy_->first, e.y - last_xy_->second) > p_.jump_m) {jumps_.push_back(now);}
      last_xy_ = {e.x, e.y};
    }
  }
}

std::string LocalizationCore::tf_conflict(double now) const
{
  std::string out;
  for (const auto & [child, parents] : parents_) {
    std::vector<std::string> live;
    for (const auto & [par, t] : parents) {if (now - t < 3.0) {live.push_back(par);}}   // map keeps them sorted
    if (live.size() > 1) {
      std::string joined;
      for (size_t i = 0; i < live.size(); ++i) {joined += (i ? "+" : "") + live[i];}
      out += (out.empty() ? "" : ", ") + child + "<-" + joined;
    }
  }
  return out;
}

bool LocalizationCore::conflict_changed(const std::string & conflict)
{
  const bool changed = !conflict.empty() && conflict != last_conflict_;
  last_conflict_ = conflict;
  return changed;
}

void LocalizationCore::update_state(double now)
{
  if (state_ == "uninitialized" || state_ == "aligning" || state_ == "align_failed" || !activated_) {return;}
  if (!last_ndt_) {return;}                       // initialized, waiting for the first result
  const double age = now - *last_ndt_;
  if (age > p_.lost_timeout) {
    state_ = "lost"; msg_ = "No NDT pose for " + fmt("%.0f", age) + " s";
  } else if (age > 1.0) {
    state_ = "degraded"; msg_ = "NDT pose stale (" + fmt("%.1f", age) + " s)";
  } else if (score_ < p_.score_ok) {
    state_ = "degraded"; msg_ = "Low match score " + fmt("%.2f", score_);
  } else {
    state_ = "converged"; msg_.clear();
  }
}

std::string LocalizationCore::state_string(double now, const std::optional<MapPose> & pose)
{
  update_state(now);
  const double nan = std::nan("");
  const double x = pose ? pose->x : nan, y = pose ? pose->y : nan, yaw = pose ? pose->yaw_deg : nan;
  const double age = last_ndt_ ? now - *last_ndt_ : -1.0;
  jumps_.erase(std::remove_if(jumps_.begin(), jumps_.end(), [&](double t) { return now - t >= 30.0; }), jumps_.end());
  const std::string conflict = tf_conflict(now);
  const std::vector<std::pair<std::string, std::string>> kv = {
    {"state", state_}, {"score", fmt("%.2f", score_)}, {"iterations", fmt("%.0f", iters_)},
    {"exe_ms", fmt("%.0f", exe_ms_)}, {"age", fmt("%.1f", age)}, {"x", fmt("%.2f", x)},
    {"y", fmt("%.2f", y)}, {"yaw", fmt("%.1f", yaw)},
    {"tf_conflict", conflict}, {"jumps", std::to_string(jumps_.size())},
    {"msg", conflict.empty() ? msg_ : "TF CONFLICT: " + conflict}};
  std::string out;
  for (const auto & p : kv) {out += (out.empty() ? "" : ";") + p.first + "=" + p.second;}
  return out;
}
}  // namespace buggy_core
