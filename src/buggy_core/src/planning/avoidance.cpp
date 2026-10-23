#include "buggy_core/planning/avoidance.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <numeric>
#include <queue>
#include <set>

namespace buggy_core
{
namespace
{
std::string fmt1(const char * f, double v)
{
  char b[256];
  std::snprintf(b, sizeof(b), f, v);
  return b;
}

// Frenet helper for a polyline
struct Route
{
  std::vector<std::array<double, 3>> p;
  std::vector<double> s, tx, ty, nx, ny;

  explicit Route(const Poly & pts)
  : p(pts)
  {
    const size_t n = p.size();
    s.assign(n, 0.0);
    for (size_t i = 1; i < n; ++i) {s[i] = s[i - 1] + std::hypot(p[i][0] - p[i - 1][0], p[i][1] - p[i - 1][1]);}
    tx.resize(n); ty.resize(n); nx.resize(n); ny.resize(n);
    for (size_t i = 0; i < n; ++i) {
      // numpy.gradient (spacing 1): central differences inside, one-sided at the ends
      double gx, gy;
      if (i == 0) {gx = p[1][0] - p[0][0]; gy = p[1][1] - p[0][1];}
      else if (i == n - 1) {gx = p[n - 1][0] - p[n - 2][0]; gy = p[n - 1][1] - p[n - 2][1];}
      else {gx = (p[i + 1][0] - p[i - 1][0]) / 2.0; gy = (p[i + 1][1] - p[i - 1][1]) / 2.0;}
      double nn = std::hypot(gx, gy);
      if (nn < 1e-9) {nn = 1.0;}
      tx[i] = gx / nn; ty[i] = gy / nn;
      nx[i] = -ty[i]; ny[i] = tx[i];
    }
  }

  void to_frenet(double x, double y, double & ss, double & e) const
  {
    size_t j = 0;
    double bd = 1e300;
    for (size_t i = 0; i < p.size(); ++i) {
      const double d = (x - p[i][0]) * (x - p[i][0]) + (y - p[i][1]) * (y - p[i][1]);
      if (d < bd) {bd = d; j = i;}
    }
    const double dx = x - p[j][0], dy = y - p[j][1];
    ss = s[j] + dx * tx[j] + dy * ty[j];
    e = dx * nx[j] + dy * ny[j];
  }
};

std::array<std::array<double, 2>, 4> box_corners(const Box & b)
{
  const double c = std::cos(b.yaw), s = std::sin(b.yaw);
  const double l = b.length, w = b.width;
  const double d[4][2] = {{-l / 2, -w / 2}, {l / 2, -w / 2}, {l / 2, w / 2}, {-l / 2, w / 2}};
  std::array<std::array<double, 2>, 4> out;
  for (int i = 0; i < 4; ++i) {out[i] = {b.x + c * d[i][0] - s * d[i][1], b.y + s * d[i][0] + c * d[i][1]};}
  return out;
}

void merge_into(std::vector<Zone> & out, const Zone & z, double gap)
{
  if (!out.empty() && z.s0 - out.back().s1 < gap) {
    Zone & m = out.back();
    m.s1 = std::max(m.s1, z.s1); m.lo = std::min(m.lo, z.lo); m.hi = std::max(m.hi, z.hi);
  } else {
    out.push_back(z);
  }
}

std::vector<Zone> zones_on_route(const Route & route, const std::vector<Box> & obstacles, const AvoidParams & prm)
{
  std::vector<Zone> raw;
  for (const auto & o : obstacles) {
    double s0 = 1e300, s1 = -1e300, lo = 1e300, hi = -1e300;
    for (const auto & c : box_corners(o)) {
      double s, e;
      route.to_frenet(c[0], c[1], s, e);
      s0 = std::min(s0, s); s1 = std::max(s1, s); lo = std::min(lo, e); hi = std::max(hi, e);
    }
    if (s1 < 0.0 || s0 > prm.look_ahead) {continue;}
    if (hi < -prm.avoid_half_width || lo > prm.avoid_half_width) {continue;}
    raw.push_back({s0, s1, lo, hi});
  }
  std::stable_sort(raw.begin(), raw.end(), [](const Zone & a, const Zone & b) { return a.s0 < b.s0; });
  std::vector<Zone> zones;
  for (const auto & z : raw) {merge_into(zones, z, prm.merge_gap);}
  return zones;
}

std::vector<Zone> merge_zones(std::vector<Zone> zones, const AvoidParams & prm)
{
  std::stable_sort(zones.begin(), zones.end(), [](const Zone & a, const Zone & b) { return a.s0 < b.s0; });
  std::vector<Zone> out;
  for (const auto & z : zones) {merge_into(out, z, prm.merge_gap);}
  return out;
}

// 8-connected labelling in raster order (labels 1..n like scipy.ndimage.label)
std::vector<int> label8(const std::vector<uint8_t> & g, int nx, int ny, int & n)
{
  std::vector<int> lab(g.size(), 0);
  n = 0;
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      if (!g[i * ny + j] || lab[i * ny + j]) {continue;}
      ++n;
      std::queue<std::pair<int, int>> q;
      q.push({i, j});
      lab[i * ny + j] = n;
      while (!q.empty()) {
        const auto [a, b] = q.front();
        q.pop();
        for (int da = -1; da <= 1; ++da) {
          for (int db = -1; db <= 1; ++db) {
            const int u = a + da, v = b + db;
            if (u < 0 || v < 0 || u >= nx || v >= ny) {continue;}
            if (g[u * ny + v] && !lab[u * ny + v]) {lab[u * ny + v] = n; q.push({u, v});}
          }
        }
      }
    }
  }
  return lab;
}

std::vector<Zone> barrier_zones(const Route & route, const FreeSpace & free, const AvoidParams & prm)
{
  int n = 0;
  const auto lab = label8(free.occ, free.nx, free.ny, n);
  if (n == 0) {return {};}
  struct Acc { double s0 = 1e300, s1 = -1e300, lo = 1e300, hi = -1e300; int count = 0; bool touching = false; };
  std::map<int, Acc> acc;
  for (int i = 0; i < free.nx; ++i) {
    for (int j = 0; j < free.ny; ++j) {
      if (!free.occ[i * free.ny + j]) {continue;}
      const double x = free.ox + (i + 0.5) * free.res, y = free.oy + (j + 0.5) * free.res;
      double s, e;
      route.to_frenet(x, y, s, e);
      const bool ahead = s >= 0.0 && s <= prm.look_ahead + 10.0;
      Acc & a = acc[lab[i * free.ny + j]];
      if (ahead) {
        a.s0 = std::min(a.s0, s); a.s1 = std::max(a.s1, s);
        a.lo = std::min(a.lo, e); a.hi = std::max(a.hi, e);
        a.count++;
        if (std::abs(e) <= prm.avoid_half_width) {a.touching = true;}
      }
    }
  }
  std::vector<Zone> zones;
  for (const auto & [k, a] : acc) {
    if (!a.touching || a.count < 3) {continue;}
    zones.push_back({a.s0, a.s1, a.lo, a.hi});
  }
  return zones;
}

}  // namespace

double arc_shift_one(double u, double total, double d, double R)
{
  const double sign = d >= 0 ? 1.0 : -1.0;
  const double half = total / 2.0;
  const double a = std::min(std::max(u, 0.0), total);
  const double first = R - std::sqrt(std::max(R * R - std::pow(std::min(a, half), 2), 0.0));
  const double second = R - std::sqrt(std::max(R * R - std::pow(std::max(total - a, 0.0), 2), 0.0));
  return sign * (a <= half ? first : std::abs(d) - second);
}

double shift_length(double d, double R)
{
  const double th = std::acos(std::max(-1.0, std::min(1.0, 1.0 - std::abs(d) / (2.0 * R))));
  return 2.0 * R * std::sin(th);
}

namespace
{
struct Built { Poly seg; size_t ib_count; double L, s_a0; };

std::optional<Built> build_detour(const Route & route, const Zone & z, double d, double R, const AvoidParams & prm)
{
  const double L = shift_length(d, R);
  const double s_a0 = std::max(0.0, z.s0 - prm.front - prm.lead - L);
  const double s_hold_end = z.s1 + prm.rear + prm.tail;
  const double s_end = s_hold_end + L;
  if (s_end >= route.s.back() - 0.5) {return std::nullopt;}
  size_t ib = static_cast<size_t>(std::lower_bound(route.s.begin(), route.s.end(), s_end) - route.s.begin()) + 1;
  ib = std::min(ib, route.s.size() - 1);
  Built b;
  for (size_t i = 0; i <= ib; ++i) {
    const double s = route.s[i];
    double off = 0.0;
    if (s >= s_a0 && s < s_a0 + L) {off = arc_shift_one(s - s_a0, L, d, R);}
    if (s >= s_a0 + L && s <= s_hold_end) {off = d;}
    if (s > s_hold_end && s < s_end) {off = d - arc_shift_one(s - s_hold_end, L, d, R);}
    b.seg.push_back({route.p[i][0] + off * route.nx[i], route.p[i][1] + off * route.ny[i], route.p[i][2]});
  }
  b.ib_count = ib + 1; b.L = L; b.s_a0 = s_a0;
  return b;
}

std::vector<std::array<double, 2>> footprint_samples(const AvoidParams & prm)
{
  std::vector<std::array<double, 2>> out;
  const int nxs = static_cast<int>(std::ceil((prm.front + 1e-6 + prm.rear) / 0.35));      // numpy.arange count
  const double hw = prm.veh_half_width + prm.footprint_margin;
  for (int i = 0; i < nxs; ++i) {
    const double x = -prm.rear + i * 0.35;
    for (int k = 0; k < 7; ++k) {out.push_back({x, -hw + (2 * hw) * k / 6.0});}
  }
  return out;
}

std::pair<bool, std::string> swept_check(const Poly & seg, const std::vector<Box> & obstacles,
  const LaneModel * lane, const FreeSpace * free, const AvoidParams & prm)
{
  const size_t n = seg.size();
  std::vector<double> gx(n), gy(n);
  for (size_t i = 0; i < n; ++i) {
    if (i == 0) {gx[i] = seg[1][0] - seg[0][0]; gy[i] = seg[1][1] - seg[0][1];}
    else if (i == n - 1) {gx[i] = seg[n - 1][0] - seg[n - 2][0]; gy[i] = seg[n - 1][1] - seg[n - 2][1];}
    else {gx[i] = (seg[i + 1][0] - seg[i - 1][0]) / 2.0; gy[i] = (seg[i + 1][1] - seg[i - 1][1]) / 2.0;}
  }
  const auto F = footprint_samples(prm);
  std::vector<std::array<double, 2>> W;
  W.reserve(n * F.size());
  for (size_t i = 0; i < n; ++i) {
    const double yaw = std::atan2(gy[i], gx[i]);
    const double c = std::cos(yaw), s = std::sin(yaw);
    for (const auto & f : F) {W.push_back({seg[i][0] + c * f[0] - s * f[1], seg[i][1] + s * f[0] + c * f[1]});}
  }
  for (const auto & o : obstacles) {
    const double cc = std::cos(o.yaw), ss = std::sin(o.yaw);
    for (const auto & w : W) {
      const double rx = w[0] - o.x, ry = w[1] - o.y;
      const double lx = cc * rx + ss * ry, ly = -ss * rx + cc * ry;
      if (std::abs(lx) <= o.length / 2 + 0.1 && std::abs(ly) <= o.width / 2 + 0.1) {
        return {false, "the detour would hit an obstacle"};
      }
    }
  }
  if (free) {
    for (const auto & w : W) {
      if (free->lookup(free->occ, w[0], w[1])) {
        return {false, "LiDAR sees something solid beside the obstacle (no room to pass)"};
      }
    }
  }
  if (lane) {
    std::vector<std::array<double, 2>> out;
    for (const auto & w : W) {if (!lane->inside(w[0], w[1])) {out.push_back(w);}}
    if (!out.empty()) {
      const bool sensed = free && free->has_ground;
      const double limit = sensed ? prm.offlane_allow_sensed : prm.offlane_allow;
      for (const auto & w : out) {
        if (!lane->within(w[0], w[1], limit)) {return {false, "would leave the lane by more than " + fmt1("%.1f", limit) + " m"};}
      }
      if (sensed) {
        size_t unseen = 0;
        for (const auto & w : out) {if (!free->lookup(free->gnd, w[0], w[1])) {++unseen;}}
        if (static_cast<double>(unseen) / out.size() > 1.0 - prm.ground_fraction) {
          return {false, "the LiDAR has not seen free ground where the detour leaves the lane"};
        }
      }
    }
  }
  return {true, ""};
}
}  // namespace

// ---- LaneModel / FreeSpace -------------------------------------------------------------------
LaneModel::LaneModel(const std::vector<Lanelet> & lanelets)
{
  for (const auto & l : lanelets) {
    Poly2 p;
    for (const auto & q : l.left) {p.v.push_back({q[0], q[1]}); bounds_.push_back({q[0], q[1]});}
    for (auto it = l.right.rbegin(); it != l.right.rend(); ++it) {p.v.push_back({(*it)[0], (*it)[1]});}
    for (const auto & q : l.right) {bounds_.push_back({q[0], q[1]});}
    p.x0 = p.y0 = 1e300; p.x1 = p.y1 = -1e300;
    for (const auto & q : p.v) {p.x0 = std::min(p.x0, q[0]); p.x1 = std::max(p.x1, q[0]); p.y0 = std::min(p.y0, q[1]); p.y1 = std::max(p.y1, q[1]);}
    polys_.push_back(std::move(p));
  }
}

bool LaneModel::inside(double x, double y) const
{
  for (const auto & p : polys_) {
    if (x < p.x0 || x > p.x1 || y < p.y0 || y > p.y1) {continue;}
    bool in = false;
    for (size_t i = 0, j = p.v.size() - 1; i < p.v.size(); j = i++) {
      const auto & a = p.v[i];
      const auto & b = p.v[j];
      if ((a[1] > y) != (b[1] > y) && x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]) {in = !in;}
    }
    if (in) {return true;}
  }
  return false;
}

double LaneModel::dist(double x, double y) const
{
  double best = 1e300;
  for (const auto & b : bounds_) {best = std::min(best, (b[0] - x) * (b[0] - x) + (b[1] - y) * (b[1] - y));}
  return bounds_.empty() ? 0.0 : std::sqrt(best);
}

bool LaneModel::within(double x, double y, double r) const
{
  const double r2 = r * r;
  for (const auto & b : bounds_) {
    if ((b[0] - x) * (b[0] - x) + (b[1] - y) * (b[1] - y) <= r2) {return true;}
  }
  return bounds_.empty();
}

bool FreeSpace::lookup(const std::vector<uint8_t> & g, double x, double y, bool * known) const
{
  const int i = static_cast<int>(std::floor((x - ox) / res));
  const int j = static_cast<int>(std::floor((y - oy) / res));
  const bool ok = i >= 0 && j >= 0 && i < nx && j < ny;
  if (known) {*known = ok;}
  return ok && g[static_cast<size_t>(i) * ny + j];
}

std::vector<uint8_t> dilate(const std::vector<uint8_t> & g, int nx, int ny, int iterations)
{
  std::vector<uint8_t> cur = g;
  for (int it = 0; it < iterations; ++it) {
    std::vector<uint8_t> nw = cur;
    for (int i = 0; i < nx; ++i) {
      for (int j = 0; j < ny; ++j) {
        if (cur[i * ny + j]) {continue;}
        if ((i > 0 && cur[(i - 1) * ny + j]) || (i + 1 < nx && cur[(i + 1) * ny + j]) ||
          (j > 0 && cur[i * ny + j - 1]) || (j + 1 < ny && cur[i * ny + j + 1]))
        {
          nw[i * ny + j] = 1;
        }
      }
    }
    cur = std::move(nw);
  }
  return cur;
}

// ---- planning --------------------------------------------------------------------------------
nlohmann::json plan_detour(const Poly & points, const std::vector<Box> & obstacles,
  const LaneModel * lane, const FreeSpace * free, const AvoidParams & prm)
{
  using nlohmann::json;
  const Route route(points);
  auto zones = zones_on_route(route, obstacles, prm);
  if (free) {
    auto bz = barrier_zones(route, *free, prm);
    zones.insert(zones.end(), bz.begin(), bz.end());
    zones = merge_zones(zones, prm);
  }
  if (zones.empty()) {return {{"ok", false}, {"reason", "no obstacle on the route ahead"}};}
  const Zone z = zones[0];
  const double clr = prm.veh_half_width + prm.side_margin;
  const double R0 = prm.wheelbase / std::tan(prm.max_steer_deg * prm.steer_frac * M_PI / 180.0);
  // bumper-to-obstacle gap a forward-only shift around this zone needs (also reported when it works)
  const double need = shift_length(clr + (z.hi - z.lo) / 2, R0) + prm.lead;
  std::optional<json> best;
  std::vector<std::string> why;
  const std::pair<const char *, double> sides[2] = {{"left", z.hi + clr}, {"right", z.lo - clr}};
  for (const auto & [side, d0] : sides) {
    std::string last;
    const double sign = d0 > 0 ? 1.0 : -1.0;
    const int n_off = static_cast<int>(std::max(0.0, prm.max_offset - std::abs(d0)) / prm.offset_step) + 1;
    json best_side;
    for (int step = 0; step < n_off; ++step) {
      const double d = d0 + sign * prm.offset_step * step;
      bool built_none = false;
      for (double k : {1.0, 1.4, 2.0, 2.8, 4.0}) {
        const double R = R0 * k;
        const auto built = build_detour(route, z, d, R, prm);
        if (!built) {
          if (last.empty()) {last = "the route ends before the vehicle could rejoin it";}
          built_none = true;
          break;
        }
        built_none = false;
        if (max_steer_deg(built->seg, prm.wheelbase) > prm.max_steer_deg * prm.check_steer_frac) {
          last = "needs more steering than the buggy has";
          continue;
        }
        const auto [ok, reason] = swept_check(built->seg, obstacles, lane, free, prm);
        if (!ok) {last = reason; continue;}
        bool outside = false;
        if (lane) {for (const auto & q : built->seg) {if (!lane->inside(q[0], q[1])) {outside = true; break;}}}
        json segj = json::array();
        for (const auto & q : built->seg) {segj.push_back({q[0], q[1], q[2]});}
        json insidej = json::array();
        for (const auto & q : built->seg) {insidej.push_back(lane ? lane->inside(q[0], q[1]) : true);}
        best_side = {{"ok", true}, {"side", side}, {"offset", d}, {"segment", segj}, {"ib", built->ib_count},
          {"length", route.s[built->ib_count - 1]}, {"shift_length", built->L}, {"radius", R},
          {"room_before", z.s0 - prm.front}, {"outside_lane", outside}, {"inside", insidej},
          {"zone", {{"s0", z.s0}, {"s1", z.s1}, {"lo", z.lo}, {"hi", z.hi}}}, {"room_needed", need}};
        last.clear();
        break;
      }
      if (last.empty()) {
        if (!best || std::abs(d) < std::abs((*best)["offset"].get<double>())) {best = best_side;}
        break;
      }
      if (built_none) {break;}
    }
    if (!last.empty()) {why.push_back(std::string(side) + ": " + last);}
  }
  if (best) {return *best;}
  const double have = z.s0 - prm.front;
  std::string extra;
  if (have < need) {
    char b[160];
    std::snprintf(b, sizeof(b), " (needs ~%.1f m before the obstacle, have %.1f m)", need, have);
    extra = b;
  }
  std::string reason;
  if (!why.empty()) {
    for (size_t i = 0; i < why.size(); ++i) {reason += (i ? "; " : "") + why[i];}
    reason += extra;
  } else {
    reason = "no feasible detour" + extra;
  }
  return {{"ok", false}, {"reason", reason}, {"room_before", have}, {"room_needed", need},
    {"zone", {{"s0", z.s0}, {"s1", z.s1}, {"lo", z.lo}, {"hi", z.hi}}}};
}

std::vector<std::array<double, 2>> free_space_cells(const Poly & points, const Zone & zone,
  const FreeSpace * free, const LaneModel * lane, const AvoidParams & prm, int stride, size_t max_cells)
{
  std::vector<std::array<double, 2>> out;
  if (!free || !free->has_ground) {return out;}
  const Route route(points);
  for (int i = 0; i < free->nx; i += stride) {
    for (int j = 0; j < free->ny; j += stride) {
      const size_t k = static_cast<size_t>(i) * free->ny + j;
      if (!free->gnd[k] || free->occ[k]) {continue;}
      const double x = free->ox + (i + 0.5) * free->res, y = free->oy + (j + 0.5) * free->res;
      double s, e;
      route.to_frenet(x, y, s, e);
      if (s < zone.s0 - 6.0 || s > zone.s1 + 6.0 || std::abs(e) > prm.max_offset) {continue;}
      if (lane && lane->inside(x, y)) {continue;}
      out.push_back({x, y});
    }
  }
  if (out.size() > max_cells) {
    std::vector<std::array<double, 2>> thin;
    const double step = static_cast<double>(out.size() - 1) / (max_cells - 1);
    for (size_t i = 0; i < max_cells; ++i) {thin.push_back(out[static_cast<size_t>(i * step)]);}
    out = std::move(thin);
  }
  return out;
}
}  // namespace buggy_core
