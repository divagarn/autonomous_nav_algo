#include "buggy_core/control/operation_core.hpp"

#include "buggy_core/planning/avoidance.hpp"
#include "buggy_core/planning/path_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace buggy_core
{
namespace
{
std::string fmt(const char * f, ...)
{
  char buf[512];
  va_list a;
  va_start(a, f);
  std::vsnprintf(buf, sizeof(buf), f, a);
  va_end(a);
  return buf;
}

bool starts_with(const std::string & s, const char * p) { return s.rfind(p, 0) == 0; }

double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

// Python str(None/True/False/other) of a JSON value, for the log line
std::string py_str(const nlohmann::json & j, const char * key)
{
  if (!j.contains(key) || j[key].is_null()) {return "None";}
  if (j[key].is_boolean()) {return j[key].get<bool>() ? "True" : "False";}
  if (j[key].is_string()) {return j[key].get<std::string>();}
  return j[key].dump();
}
}  // namespace

std::map<std::string, std::string> parse_kv(const std::string & s)
{
  std::map<std::string, std::string> out;
  std::stringstream ss(s);
  std::string kv;
  while (std::getline(ss, kv, ';')) {
    const auto p = kv.find('=');
    if (p != std::string::npos) {out[kv.substr(0, p)] = kv.substr(p + 1);}
  }
  return out;
}

OperationCore::OperationCore(CoreParams p, Hooks h)
: p_(std::move(p)), h_(std::move(h))
{
  if (p_.avoid_enabled && p_.avoid_stop_early) {p_.stop_margin = std::max(p_.stop_margin, p_.avoid_stop_margin);}
  stop_margin_base_ = p_.stop_margin;
}

bool OperationCore::load_calibration(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {return false;}
  std::optional<double> react, decel;
  std::string line;
  while (std::getline(f, line)) {
    const auto c = line.find(':');
    if (c == std::string::npos || line.empty() || line[0] == ' ') {continue;}
    const std::string key = line.substr(0, c);
    char * end = nullptr;
    const std::string val = line.substr(c + 1);
    const double d = std::strtod(val.c_str(), &end);
    if (end == val.c_str()) {continue;}
    if (key == "react_sec") {react = d;}
    if (key == "decel_ms2") {decel = d;}
  }
  if (!react || !decel) {return false;}
  p_.react = *react;
  p_.decel = *decel;
  const auto slash = path.find_last_of('/');
  calib_source_ = "measured (" + (slash == std::string::npos ? path : path.substr(slash + 1)) + ")";
  return true;
}

// ---- inputs ----------------------------------------------------------------------------------
void OperationCore::rebuild_seg()
{
  seg_.assign(1, 0.0);
  for (size_t i = 1; i < path_.size(); ++i) {
    seg_.push_back(seg_.back() + std::hypot(path_[i].x - path_[i - 1].x, path_[i].y - path_[i - 1].y));
  }
}

void OperationCore::on_path(const std::vector<PathPt> & pts)
{
  path_ = pts;
  rebuild_seg();
  idx_ = 0;
  v_ = 0.0;
  avoiding_ = false;
  avoid_pending_.reset();
  avoid_tries_ = 0;
  block_since_.reset();
  avoid_status_ = "idle";
  orig_path_.clear();
  gone_since_.reset();
  if (h_.detour_clear) {h_.detour_clear();}
  if (mode_ == "auto") {mode_ = "stop";}
  routing_ = path_.size() > 1 ? "set" : "unset";
  msg_.clear();
}

void OperationCore::on_loc(const std::string & data, double now)
{
  loc_ = parse_kv(data);
  const auto it = loc_.find("state");
  localized_ = it != loc_.end() && it->second == "converged";
  if (localized_) {loc_ok_time_ = now;}
}

void OperationCore::on_uart(const std::string & data, double now)
{
  uart_status_ = parse_kv(data);
  uart_time_ = now;
}

void OperationCore::on_wheel(double steering_deg, double speed_kmh)
{
  steer_ = steering_deg;
  v_ = speed_kmh / 3.6;
}

void OperationCore::on_names(const std::string & json, double now)
{
  names_.clear();
  try {
    const auto j = nlohmann::json::parse(json);
    if (j.is_array()) {
      for (const auto & n : j) {
        names_.push_back({n.value("x", 0.0), n.value("y", 0.0), n.value("name", std::string("unknown"))});
      }
    }
  } catch (const nlohmann::json::exception &) {
    names_.clear();
  }
  names_time_ = now;
}

std::optional<std::string> OperationCore::name_near(double x, double y, double now) const
{
  if (now - names_time_ > 1.5 || names_.empty()) {return std::nullopt;}
  const auto best = std::min_element(names_.begin(), names_.end(), [&](const NameTag & a, const NameTag & b) {
        return std::hypot(a.x - x, a.y - y) < std::hypot(b.x - x, b.y - y);
      });
  if (std::hypot(best->x - x, best->y - y) < 1.5 && best->name != "unknown") {return best->name;}
  return std::nullopt;
}

// ---- obstacles -------------------------------------------------------------------------------
std::vector<Obj> OperationCore::all_objects(double now) const
{
  std::vector<Obj> out = dummy_;
  if (now - lidar_time_ < 0.7) {out.insert(out.end(), lidar_.begin(), lidar_.end());}
  if (now - radar_time_ < 1.0) {out.insert(out.end(), radar_.begin(), radar_.end());}
  return out;
}

std::optional<double> OperationCore::block_dist(const Obj & o) const
{
  const size_t hi = path_.size();
  if (hi < 2 || idx_ >= hi) {return std::nullopt;}
  size_t j = idx_;
  double best = 1e300;
  for (size_t i = idx_; i < hi; ++i) {
    const double d = std::hypot(path_[i].x - o.x, path_[i].y - o.y);
    if (d < best) {best = d; j = i;}
  }
  const double lat = best;
  const size_t k = std::min(j, hi - 2);
  const double tang = std::atan2(path_[k + 1].y - path_[k].y, path_[k + 1].x - path_[k].x);
  const double th = o.yaw - tang;
  const double half_lat = std::abs(std::cos(th)) * o.width / 2 + std::abs(std::sin(th)) * o.length / 2;
  const double half_lon = std::abs(std::cos(th)) * o.length / 2 + std::abs(std::sin(th)) * o.width / 2;
  const double corridor = avoiding_ ? p_.avoid_hw : p_.safe_half_width;
  if (lat - half_lat > corridor) {return std::nullopt;}
  const double dist = seg_[j] - seg_[idx_] - p_.front_offset - half_lon;
  if (-half_lon * 2 < dist && dist < p_.detect_range) {return dist;}
  return std::nullopt;
}

void OperationCore::update_memory(double now)
{
  if (path_.empty() || p_.memory_sec <= 0.0) {memory_.clear(); return;}
  std::vector<Obj> fresh;
  if (now - lidar_time_ < 0.7) {fresh.insert(fresh.end(), lidar_.begin(), lidar_.end());}
  if (now - radar_time_ < 1.0) {fresh.insert(fresh.end(), radar_.begin(), radar_.end());}
  for (const auto & o : fresh) {
    if (!block_dist(o)) {continue;}
    bool found = false;
    for (auto & m : memory_) {
      if (std::hypot(m.obj.x - o.x, m.obj.y - o.y) < 1.0) {
        m.obj = o; m.t = now; m.hold_t.reset(); m.n += 1;
        found = true;
        break;
      }
    }
    if (!found) {memory_.push_back({o, now, std::nullopt, 1});}
  }
  const double s_stop = stopping_distance(std::max(v_, 0.55));
  std::vector<Mem> keep;
  for (auto & m : memory_) {
    const auto d = block_dist(m.obj);
    if (!d) {continue;}
    if (now - m.t > 1.0 && m.n < p_.memory_min_hits) {continue;}
    if (now - m.t > 0.35) {
      if (!m.hold_t && *d - p_.stop_margin < s_stop) {m.hold_t = now;}
      if (m.hold_t) {
        if (now - *m.hold_t > p_.memory_sec) {creep_until_ = now + p_.creep_sec; continue;}
      } else if (now - m.t > p_.memory_max_sec) {
        continue;
      }
    }
    keep.push_back(m);
  }
  memory_ = std::move(keep);
}

std::vector<Obj> OperationCore::remembered(double now) const
{
  std::vector<Obj> out;
  for (const auto & m : memory_) {
    if (now - m.t > 0.35 && m.n >= p_.memory_min_hits) {out.push_back(m.obj);}
  }
  return out;
}

std::optional<Obstacle> OperationCore::obstacle_ahead(double now) const
{
  std::optional<Obstacle> best;
  auto objs = all_objects(now);
  const auto rem = remembered(now);
  objs.insert(objs.end(), rem.begin(), rem.end());
  for (const auto & o : objs) {
    const auto d = block_dist(o);
    if (d && (!best || *d < best->dist)) {
      best = Obstacle{*d, o.label, name_near(o.x, o.y, now).value_or("")};
    }
  }
  return best;
}

std::string OperationCore::obs_name(const Obstacle & o)
{
  if (!o.cam.empty()) {return o.cam;}
  switch (o.label) {
    case 1: return "car";
    case 2: return "truck";
    case 3: return "bus";
    case 5: return "motorcycle";
    case 6: return "bicycle";
    case 7: return "pedestrian";
    case 8: return "animal";
    default: return "obstacle";
  }
}

// ---- detour ----------------------------------------------------------------------------------
void OperationCore::update_avoidance(const std::optional<Obstacle> & obs, double now, bool stopped)
{
  if (!p_.avoid_enabled) {return;}
  last_now_ = now;
  if (starts_with(msg_, "Route restored") && now > restored_until_) {msg_.clear();}
  const bool fresh = room_needed_ && now - room_time_ < 6.0;
  p_.stop_margin = (fresh && obs && p_.avoid_stop_early && p_.avoid_probe) ? std::max(stop_margin_base_, std::min(20.0, *room_needed_ + 0.5)) : stop_margin_base_;
  if (p_.avoid_stop_early && p_.avoid_probe && !avoiding_ && obs && !stopped && !avoid_pending_ && now >= probe_next_ &&
    (!probe_pending_ || now - probe_pending_->second > 4.0))
  {
    probe_next_ = now + 1.5;
    request_avoidance(now, true);
  }
  if (avoiding_) {
    if (idx_ >= avoid_end_idx_) {
      avoiding_ = false; avoid_tries_ = 0; block_since_.reset();
      avoid_status_ = "idle";
      orig_path_.clear();
      if (h_.detour_clear) {h_.detour_clear();}
      log(Level::Info, "detour finished: back on the lanelet path");
      msg_.clear();
    } else {
      check_obstacle_gone(now);
    }
    return;
  }
  if (obs && stopped) {
    if (!block_since_) {block_since_ = now;}
    block_last_ = now;
  } else if (now - block_last_ > 1.0) {
    block_since_.reset();
    if (avoid_status_ != "idle") {avoid_status_ = "idle";}
    return;
  }
  if (avoid_pending_) {
    if (now - avoid_pending_->t > 4.0) {
      avoid_pending_.reset();
      avoid_next_ = now + p_.avoid_retry;
      avoid_status_ = "planner not answering";
    }
    return;
  }
  if (block_since_ && avoid_status_ != "no detour" && avoid_status_ != "planning") {
    avoid_status_ = fmt("stopped %.0f/%.0f s", now - *block_since_, p_.avoid_after);
    avoid_wait_ = now - *block_since_;
  }
  if (stopped && block_since_ && now - *block_since_ >= p_.avoid_after && now >= avoid_next_ &&
    avoid_tries_ < p_.avoid_max_tries)
  {
    request_avoidance(now);
  }
}

// ---- obstacle gone while detouring: back to the original route ---------------------------------
void OperationCore::check_obstacle_gone(double now)
{
  if (orig_path_.empty() || avoid_blockers_.empty()) {return;}
  const bool sensors_alive = now - lidar_time_ < 0.7;
  bool present = false;
  for (const auto & o : all_objects(now)) {
    for (const auto & b : avoid_blockers_) {
      if (std::hypot(o.x - b[0], o.y - b[1]) < 1.5) {present = true;}
    }
  }
  const auto & op = orig_path_;
  const size_t lo = avoid_req_idx_, hi = std::min(op.size(), avoid_req_idx_ + avoid_ib_ + 60);
  size_t j = lo;
  double best = 1e300;
  for (size_t i = lo; i < hi; ++i) {
    const double d = std::hypot(op[i].x - rx_, op[i].y - ry_);
    if (d < best) {best = d; j = i;}
  }
  double sblock = 1e300;
  for (const auto & b : avoid_blockers_) {
    size_t kk = 0;
    double bd = 1e300;
    for (size_t i = 0; i < op.size(); ++i) {
      const double d = std::hypot(op[i].x - b[0], op[i].y - b[1]);
      if (d < bd) {bd = d; kk = i;}
    }
    sblock = std::min(sblock, orig_seg_[kk]);
  }
  // only while the vehicle is still before the obstacle (beside / past it the detector no longer sees it)
  const bool before = orig_seg_[j] + p_.front_offset + 1.0 < sblock - 1.5;
  if (present || !sensors_alive || !before) {gone_since_.reset(); return;}
  if (!gone_since_) {gone_since_ = now;}
  if (now - *gone_since_ >= p_.avoid_clear_sec) {revert_to_original();}
}

void OperationCore::revert_to_original()
{
  const auto & op = orig_path_;
  const auto & oseg = orig_seg_;
  const size_t lo = avoid_req_idx_, hi = std::min(op.size(), avoid_req_idx_ + avoid_ib_ + 60);
  size_t j = lo;
  double best = 1e300;
  for (size_t i = lo; i < hi; ++i) {
    const double d = std::hypot(op[i].x - rx_, op[i].y - ry_);
    if (d < best) {best = d; j = i;}
  }
  const size_t k0 = j > 0 ? j - 1 : 0, k1 = std::min(j + 1, op.size() - 1);
  const double tx = op[k1].x - op[k0].x, ty = op[k1].y - op[k0].y;
  double n = std::hypot(tx, ty);
  if (n == 0.0) {n = 1.0;}
  const double nx = -ty / n, ny = tx / n;
  const double e = (rx_ - op[j].x) * nx + (ry_ - op[j].y) * ny;      // + = vehicle left of the route
  size_t tail_from = j;
  Poly trans;
  if (std::abs(e) >= 0.15) {
    const double R0 = p_.wheelbase / std::tan(p_.max_steer * 0.75 * M_PI / 180.0);
    for (double kk : {1.0, 1.4, 2.0, 2.8, 4.0}) {
      const double R = R0 * kk;
      const double L = shift_length(e, R);
      Poly pts;
      size_t i = j;
      while (i < op.size() && oseg[i] - oseg[j] <= L) {
        const auto & a = op[i > 0 ? i - 1 : 0];
        const auto & b = op[std::min(i + 1, op.size() - 1)];
        double m = std::hypot(b.x - a.x, b.y - a.y);
        if (m == 0.0) {m = 1.0;}
        const double off = e - arc_shift_one(oseg[i] - oseg[j], L, e, R);
        pts.push_back({op[i].x - (b.y - a.y) / m * off, op[i].y + (b.x - a.x) / m * off, op[i].z});
        ++i;
      }
      if (i + 1 >= op.size()) {break;}
      trans = pts;
      tail_from = i;
      Poly probe{{rx_, ry_, 0.0}};
      probe.insert(probe.end(), pts.begin(), pts.end());
      for (size_t q = i; q < std::min(op.size(), i + 8); ++q) {probe.push_back({op[q].x, op[q].y, op[q].z});}
      if (max_steer_deg(probe, p_.wheelbase) <= p_.max_steer * 0.95) {break;}
    }
  }
  Poly all = trans;
  for (size_t q = tail_from; q < op.size(); ++q) {all.push_back({op[q].x, op[q].y, op[q].z});}
  std::vector<PathPt> np(path_.begin(), path_.begin() + std::min(idx_, path_.size()));
  for (size_t i = 0; i < all.size(); ++i) {
    const auto a = i + 1 < all.size() ? all[i] : all[i - 1];
    const auto b = i + 1 < all.size() ? all[i + 1] : all[i];
    const double yaw = std::atan2(b[1] - a[1], b[0] - a[0]);
    np.push_back({all[i][0], all[i][1], all[i][2], std::sin(yaw / 2), std::cos(yaw / 2)});
  }
  path_ = std::move(np);
  rebuild_seg();
  avoiding_ = false; avoid_tries_ = 0; block_since_.reset();
  avoid_status_ = "idle";
  orig_path_.clear();
  gone_since_.reset();
  hold_obstacle_ = false; clear_since_.reset();
  msg_ = "Route restored - obstacle cleared, back to the original route";
  restored_until_ = last_now_ + 6.0;
  log(Level::Info, msg_ + fmt(" (was %.1f m off the lane centre)", std::abs(e)));
  if (h_.detour_clear) {h_.detour_clear();}
}

void OperationCore::request_avoidance(double now, bool probe)
{
  nlohmann::json pts = nlohmann::json::array(), obstacles = nlohmann::json::array();
  const size_t hi = std::min(path_.size(), idx_ + 130);
  for (size_t i = idx_; i < hi; ++i) {pts.push_back({path_[i].x, path_[i].y, path_[i].z});}
  if (pts.size() < 6) {return;}
  auto objs = all_objects(now);
  const auto rem = remembered(now);
  objs.insert(objs.end(), rem.begin(), rem.end());
  for (const auto & o : objs) {obstacles.push_back({o.x, o.y, o.yaw, o.length, o.width});}
  avoid_id_ += 1;
  if (probe) {                       // "how much room would a detour need?" - the answer moves the stop line
    probe_pending_ = std::make_pair(avoid_id_, now);
    nlohmann::json preq = {{"id", avoid_id_}, {"points", pts}, {"obstacles", obstacles}, {"probe", true}};
    if (h_.avoid_request) {h_.avoid_request(preq.dump());}
    return;
  }
  pending_blockers_.clear();
  for (const auto & o : objs) {pending_blockers_.push_back({o.x, o.y});}
  avoid_pending_ = Pending{avoid_id_, now, idx_};
  avoid_status_ = "planning";
  nlohmann::json req = {{"id", avoid_id_}, {"points", pts}, {"obstacles", obstacles}};
  if (h_.avoid_request) {h_.avoid_request(req.dump());}
  log(Level::Info, fmt("stopped for %.0f s in front of the obstacle: asking for a detour", now - block_since_.value_or(now)));
}

void OperationCore::on_avoid_result(const std::string & json, double now)
{
  nlohmann::json d;
  try {
    d = nlohmann::json::parse(json);
  } catch (const nlohmann::json::exception &) {
    return;
  }
  if (d.value("probe", false)) {
    if (probe_pending_ && d.contains("id") && d["id"].get<int>() == probe_pending_->first) {
      probe_pending_.reset();
      if (d.contains("room_needed") && d["room_needed"].is_number()) {
        room_needed_ = d["room_needed"].get<double>();
        room_time_ = now;
      }
    }
    return;
  }
  if (!avoid_pending_) {return;}
  const Pending pend = *avoid_pending_;
  if (!d.contains("id") || d["id"].get<int>() != pend.id) {return;}
  avoid_pending_.reset();
  if (!d.value("ok", false)) {
    avoid_tries_ += 1;
    avoid_next_ = now + p_.avoid_retry;
    avoid_status_ = "no detour";
    avoid_note_ = d.value("reason", std::string());
    msg_ = "Blocked, no detour: " + avoid_note_.substr(0, 140);
    log(Level::Warn, msg_);
    return;
  }
  std::vector<std::array<double, 3>> seg;
  for (const auto & q : d["segment"]) {seg.push_back({q[0].get<double>(), q[1].get<double>(), q[2].get<double>()});}
  const size_t ib = static_cast<size_t>(d["ib"].get<double>());
  std::vector<PathPt> add;
  for (size_t i = 0; i < seg.size(); ++i) {
    auto a = seg[i], b = seg[std::min(i + 1, seg.size() - 1)];
    if (i == seg.size() - 1) {a = seg[i - 1]; b = seg[i];}
    const double yaw = std::atan2(b[1] - a[1], b[0] - a[0]);
    add.push_back({seg[i][0], seg[i][1], seg[i][2], std::sin(yaw / 2), std::cos(yaw / 2)});
  }
  orig_path_ = path_; orig_seg_ = seg_;
  avoid_req_idx_ = pend.idx; avoid_ib_ = ib;
  avoid_blockers_ = pending_blockers_; gone_since_.reset();
  std::vector<PathPt> np(path_.begin(), path_.begin() + std::min(pend.idx, path_.size()));
  np.insert(np.end(), add.begin(), add.end());
  if (pend.idx + ib < path_.size()) {np.insert(np.end(), path_.begin() + pend.idx + ib, path_.end());}
  path_ = std::move(np);
  rebuild_seg();
  avoid_end_idx_ = pend.idx + add.size();
  avoiding_ = true;
  const std::string side = d.value("side", std::string());
  const double offset = std::abs(d.value("offset", 0.0));
  avoid_status_ = fmt("avoiding (%s %.1f m)", side.c_str(), offset);
  hold_obstacle_ = false;
  clear_since_.reset();
  msg_ = fmt("Detour %s by %.1f m around the obstacle", side.c_str(), offset);
  log(Level::Info, msg_ + fmt(" (%.0f m, LiDAR free space: ", d.value("length", 0.0)) + py_str(d, "sensed") + ")");
  std::vector<bool> inside(seg.size(), true);
  if (d.contains("inside") && d["inside"].size() == seg.size()) {
    for (size_t i = 0; i < seg.size(); ++i) {inside[i] = d["inside"][i].get<bool>();}
  }
  if (h_.detour_show) {h_.detour_show(seg, inside);}
}

// ---- requests --------------------------------------------------------------------------------
bool OperationCore::auto_available() const
{
  if (!p_.drive_enabled) {return false;}
  return routing_ == "set" && localized_ && !emergency_;
}

void OperationCore::on_request(const std::string & data)
{
  const auto c = data.find(':');
  const std::string key = data.substr(0, c);
  const std::string val = c == std::string::npos ? "" : data.substr(c + 1);
  if (key == "mode") {
    if (val == "auto" && !p_.drive_enabled) {
      msg_ = "AUTO disabled: not armed (launch with drive:=true)";
      return;
    }
    if (val == "auto" && !(h_.pose && h_.pose())) {
      msg_ = "AUTO unavailable: no fresh map->base_link pose";
      return;
    }
    if (val == "auto" && !auto_available()) {
      msg_ = "AUTO unavailable: needs localization + route";
      return;
    }
    mode_ = val;
    msg_.clear();
    if (val == "auto") {gear_ = "drive";}
  } else if (key == "emergency") {
    emergency_ = val == "true";
    if (emergency_) {mode_ = "stop"; v_ = 0.0; msg_ = "EMERGENCY";} else {msg_.clear();}
  } else if (key == "velocity_limit") {
    char * end = nullptr;
    const double d = std::strtod(val.c_str(), &end);
    if (end != val.c_str()) {p_.limit_kmh = clampd(d, 0.0, 30.0);}
  } else if (key == "gear" && (val == "park" || val == "reverse" || val == "neutral" || val == "drive")) {
    gear_ = val;
  } else if (key == "turn_signal" && (val == "none" || val == "left" || val == "right" || val == "hazard")) {
    turn_signal_ = val;
  }
}

// ---- tracking --------------------------------------------------------------------------------
void OperationCore::match()
{
  const size_t hi = std::min(path_.size(), idx_ + static_cast<size_t>(kSearchWindow));
  size_t best_i = idx_;
  double best = 1e300;
  for (size_t i = idx_; i < hi; ++i) {
    const double d = std::hypot(path_[i].x - rx_, path_[i].y - ry_);
    if (d < best) {best = d; best_i = i;}
  }
  idx_ = best_i;
}

double OperationCore::pure_pursuit() const
{
  size_t i = idx_;
  while (i < path_.size() - 1 && seg_[i] - seg_[idx_] < p_.lookahead) {++i;}
  const double dx = path_[i].x - rx_, dy = path_[i].y - ry_;
  const double ld = std::max(std::hypot(dx, dy), 1e-3);
  double alpha = std::atan2(dy, dx) - ryaw_;
  alpha = std::atan2(std::sin(alpha), std::cos(alpha));
  const double delta_left = std::atan(2.0 * p_.wheelbase * std::sin(alpha) / ld);
  return clampd(-delta_left * 180.0 / M_PI, -p_.max_steer, p_.max_steer);
}

double OperationCore::cross_track() const
{
  const size_t i = std::min(idx_, path_.size() - 2);
  const double ax = path_[i].x, ay = path_[i].y, bx = path_[i + 1].x, by = path_[i + 1].y;
  double seg_len = std::hypot(bx - ax, by - ay);
  if (seg_len == 0.0) {seg_len = 1.0;}
  return ((bx - ax) * (ry_ - ay) - (by - ay) * (rx_ - ax)) / seg_len;
}

std::string OperationCore::live_fault(const std::optional<Pose> & pose, double now) const
{
  if (emergency_) {return "EMERGENCY";}
  if (!pose) {return "FAULT: pose stale - stopped";}
  if (now - loc_ok_time_ > p_.loc_grace) {return "FAULT: localization not converged - stopped";}
  const auto tf = loc_.find("tf_conflict");
  if (tf != loc_.end() && !tf->second.empty()) {
    return "FAULT: TF conflict (" + tf->second + ") - stopped";
  }
  if (p_.require_detector && now - lidar_time_ > 0.7) {return "FAULT: obstacle detector not running - stopped";}
  if (routing_ != "set") {return "FAULT: no route - stopped";}
  if (std::abs(cte_) > p_.max_cte) {return fmt("FAULT: %.1f m off the path - stopped", std::abs(cte_));}
  return "";
}

std::optional<Command> OperationCore::step_live(double now)
{
  double steer = 0.0;
  bool red = true, yellow = false;     // safe default = STOP
  if (mode_ != "auto") {
    hold_obstacle_ = false; clear_since_.reset(); memory_.clear();
    creep_until_ = 0.0;
  }
  const auto pose = h_.pose ? h_.pose() : std::nullopt;
  if (pose) {rx_ = pose->x; ry_ = pose->y; ryaw_ = pose->yaw; rz_ = pose->z;}
  if (mode_ == "auto" && p_.drive_enabled) {
    if (pose && !path_.empty()) {match(); cte_ = cross_track();}
    const std::string fault = live_fault(pose, now);
    if (!fault.empty()) {
      mode_ = "stop"; msg_ = fault;
      log(Level::Error, fault);
    } else {
      const double remaining = seg_.back() - seg_[idx_];
      update_memory(now);
      auto obs = obstacle_ahead(now);
      update_avoidance(obs, now, hold_obstacle_ && v_ < p_.stop_speed);
      if (avoiding_) {obs = obstacle_ahead(now);}
      const double preset = p_.limit_kmh <= 3.0 ? 2.0 : 4.0;
      // the ECU takes seconds to ramp up: plan with the larger of measured speed and the preset asked for
      const double v_eff = std::max(v_, cmd_go_ ? preset_kmh_ / 3.6 : 0.0);
      const double s_stop = stopping_distance(v_eff);
      const double s_slow = stopping_distance(preset / 3.6) + p_.slow_zone;
      const bool far = !obs || obs->dist - p_.stop_margin > stopping_distance(preset / 3.6) + 1.0;
      if (far) {
        if (!clear_since_) {clear_since_ = now;}
        if (hold_obstacle_ && now - *clear_since_ > 1.0) {hold_obstacle_ = false;}
      } else {
        clear_since_.reset();
        if (obs->dist - p_.stop_margin < s_stop) {hold_obstacle_ = true;}
      }
      const bool slow = (obs && obs->dist - p_.stop_margin < s_slow) || remaining - p_.goal_margin < s_slow;
      last_gap_ = obs ? std::optional<double>(obs->dist) : std::nullopt;
      const bool stop_goal = remaining - p_.goal_margin < s_stop;
      if (now - diag_t_ > 1.0) {
        diag_t_ = now;
        const std::string gap = obs ? fmt("%.1f", obs->dist) : "-";
        log(Level::Info, fmt(
            "AUTO: v=%.1f km/h gap=%s m stop_below=%.1f m slow_below=%.1f m goal_left=%.1f m hold=%s "
            "remembered=%d creep=%.0fs -> %s",
            v_ * 3.6, gap.c_str(), p_.stop_margin + s_stop, p_.stop_margin + s_slow, remaining,
            hold_obstacle_ ? "True" : "False", static_cast<int>(remembered(now).size()),
            std::max(creep_until_ - now, 0.0),
            (hold_obstacle_ || stop_goal) ? "RED" : (slow ? "SLOW" : "GO")));
      }
      if (stop_goal) {
        routing_ = "arrived"; mode_ = "stop"; msg_ = "Arrived at goal";
      } else if (hold_obstacle_) {
        const std::string gap = obs ? fmt("%.1f m", std::max(obs->dist, 0.0)) : "clearing";
        const std::string what = obs ? ": " + obs_name(*obs) : "";
        msg_ = "Obstacle ahead" + what + " (" + gap + ") - stopped";
      } else {
        if (starts_with(msg_, "Obstacle")) {msg_.clear();}
        const double demand = pure_pursuit();
        const double max_d = p_.steer_rate * p_.dt;
        cmd_steer_ += std::max(-max_d, std::min(max_d, demand - cmd_steer_));
        steer = cmd_steer_;
        red = false;
        yellow = slow || preset <= 2.0 || now < creep_until_ || avoiding_;   // 2 km/h
        preset_kmh_ = yellow ? 2.0 : 4.0;
        if (slow && obs) {
          msg_ = fmt("Slowing: %s %.1f m ahead", obs_name(*obs).c_str(), std::max(obs->dist, 0.0));
        } else if (slow) {
          msg_ = "Slowing for goal";
        } else if (starts_with(msg_, "Slowing")) {
          msg_.clear();
        }
      }
    }
  }
  cmd_go_ = !red;
  if (red) {cmd_steer_ = 0.0;}
  if (now - calib_time_ < 1.0) {return std::nullopt;}   // brake_calibration owns the topics
  return Command{steer, red, yellow};
}

// ---- state text ------------------------------------------------------------------------------
std::string OperationCore::sensors_text(double now) const
{
  std::string s = fmt("lidar %s (%d)", now - lidar_time_ < 0.7 ? "ok" : "STALE", static_cast<int>(lidar_.size()));
  if (p_.radar_enabled) {
    s += fmt(" | radar %s (%d)", now - radar_time_ < 1.0 ? "ok" : "STALE", static_cast<int>(radar_.size()));
  }
  if (p_.camera_enabled) {
    int named = 0;
    for (const auto & n : names_) {if (n.name != "unknown") {++named;}}
    s += fmt(" | camera %s (%d named)", now - names_time_ < 1.5 ? "ok" : "STALE/no objects", named);
  }
  return s;
}

std::string OperationCore::drive_text(double now) const
{
  if (!p_.drive_enabled) {return "DISARMED (AUTO off)";}
  if (!(now - uart_time_ < 2.0)) {return "armed - no uart_writer running";}
  const auto get = [&](const char * k) {
      const auto it = uart_status_.find(k);
      return it == uart_status_.end() ? std::string("?") : it->second;
    };
  return (get("mode") == "loopback" ? "armed - UART LOOPBACK (not writing)" : "ARMED - UART WRITING") +
         std::string(get("state") == "ok" ? "" : " [STALE]");
}

std::string OperationCore::state_string(double now) const
{
  const double total = seg_.empty() ? 0.0 : seg_.back();
  const double progress = (total > 0 && !seg_.empty()) ? 100.0 * seg_[idx_] / total : 0.0;
  const auto get = [&](const char * k, const char * dflt) {
      const auto it = loc_.find(k);
      return it == loc_.end() ? std::string(dflt) : it->second;
    };
  std::string loc = get("state", "no_data");
  const std::string tfc = get("tf_conflict", "");
  if (!tfc.empty()) {loc += " TF-CONFLICT " + tfc;}
  const std::string jumps = get("jumps", "0");
  if (std::atoi(jumps.c_str()) > 3) {loc += " (" + jumps + " pose jumps/30s)";}
  const double v_stop = std::max(v_, 0.55);
  std::string msg = msg_;
  if (p_.avoid_enabled && starts_with(avoid_status_, "stopped")) {
    msg += fmt(" | replanning in %.0f s", std::max(p_.avoid_after - avoid_wait_, 0.0));
  }
  if (p_.avoid_enabled && !avoid_note_.empty() && avoid_status_ == "no detour" && starts_with(msg_, "Obstacle")) {
    msg += " | no detour: " + avoid_note_.substr(0, 110);
  }
  const std::vector<std::pair<std::string, std::string>> kv = {
    {"mode", mode_}, {"routing", routing_}, {"localization", loc},
    {"drive", drive_text(now)},
    {"detector", now - lidar_time_ < 0.7 ? "ok" : "STALE"},
    {"objects", std::to_string(all_objects(now).size())},
    {"avoid", p_.avoid_enabled ? avoid_status_ : "off"},
    {"obs_gap", last_gap_ ? fmt("%.1f", *last_gap_) : "-"},
    {"stop_below", fmt("%.1f", p_.stop_margin + stopping_distance(v_stop))},
    {"sensors", sensors_text(now)},
    {"stop_model", fmt("react %.1fs decel %.2f -> %.1f m @4km/h, ", p_.react, p_.decel,
      stopping_distance(4.0 / 3.6)) + calib_source_},
    {"cmd_steer", fmt("%.1f", cmd_steer_)},
    {"ndt_score", get("score", "-")},
    {"ndt_age", get("age", "-")},
    {"motion", v_ > 0.05 ? "moving" : "stopped"},
    {"emergency", emergency_ ? "true" : "false"},
    {"velocity_limit", fmt("%.1f", p_.limit_kmh)}, {"speed", fmt("%.1f", v_ * 3.6)},
    {"gear", gear_}, {"turn_signal", turn_signal_},
    {"auto_available", auto_available() ? "true" : "false"},
    {"progress", fmt("%.0f", progress)}, {"steering", fmt("%.1f", steer_)},
    {"cte", fmt("%.2f", cte_)}, {"msg", msg}};
  std::string out;
  for (const auto & p : kv) {
    if (!out.empty()) {out += ";";}
    out += p.first + "=" + p.second;
  }
  return out;
}
}  // namespace buggy_core
