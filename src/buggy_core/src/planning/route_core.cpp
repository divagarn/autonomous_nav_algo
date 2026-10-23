#include "buggy_core/planning/route_core.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

namespace buggy_core
{
RouteCore::RouteCore(std::vector<Lanelet> lanelets, RouteParams p)
: lanelets_(std::move(lanelets)), p_(p)
{
  std::map<std::pair<std::string, std::string>, std::vector<std::string>> by_first;
  for (size_t i = 0; i < lanelets_.size(); ++i) {
    index_[lanelets_[i].id] = i;
    by_first[lanelets_[i].first].push_back(lanelets_[i].id);
  }
  for (const auto & l : lanelets_) {
    const auto it = by_first.find(l.last);
    succ_[l.id] = it == by_first.end() ? std::vector<std::string>{} : it->second;
  }
  link_geometrically();
}

void RouteCore::link_geometrically()
{
  const double link_angle = p_.link_angle_deg * M_PI / 180.0;
  const auto tangent = [](const Poly & c, bool end) {
      const size_t k = std::min<size_t>(4, c.size() - 1);       // ~2 m apart
      const Pt & a = end ? c[c.size() - 1 - k] : c[0];
      const Pt & b = end ? c[c.size() - 1] : c[k];
      return std::atan2(b[1] - a[1], b[0] - a[0]);
    };
  for (const auto & a : lanelets_) {
    for (const auto & b : lanelets_) {
      const auto & sa = succ_[a.id];
      if (std::find(sa.begin(), sa.end(), b.id) != sa.end() || a.center.size() < 2 || b.center.size() < 2) {continue;}
      if (std::hypot(a.center.back()[0] - b.center.front()[0], a.center.back()[1] - b.center.front()[1]) > p_.link_gap) {continue;}
      const double d = tangent(a.center, true) - tangent(b.center, false);
      if (std::abs(std::atan2(std::sin(d), std::cos(d))) > link_angle) {continue;}
      succ_[a.id].push_back(b.id);
      linked_.emplace_back(a.id, b.id);
    }
  }
}

const Lanelet & RouteCore::nearest(double x, double y) const
{
  size_t best = 0;
  double bd = 1e300;
  for (size_t i = 0; i < lanelets_.size(); ++i) {
    const double d = project(lanelets_[i], x, y).second;
    if (d < bd) {bd = d; best = i;}
  }
  return lanelets_[best];
}

RouteCore::Dij RouteCore::route(const std::string & a, const std::string & b) const
{
  // Python heap of (d, u, pu): ties broken by lane id string, then predecessor (None first)
  using Item = std::tuple<double, std::string, std::optional<std::string>>;
  const auto cmp = [](const Item & x, const Item & y) { return y < x; };
  std::priority_queue<Item, std::vector<Item>, decltype(cmp)> pq(cmp);
  Dij r;
  pq.emplace(0.0, a, std::nullopt);
  while (!pq.empty()) {
    auto [d, u, pu] = pq.top();
    pq.pop();
    if (r.dist.count(u)) {continue;}
    r.dist[u] = d;
    r.prev[u] = pu;
    if (u == b) {break;}
    for (const auto & v : succ_.at(u)) {
      if (!r.dist.count(v)) {pq.emplace(d + lanelet(v).length, v, u);}
    }
  }
  return r;
}

std::optional<Plan> RouteCore::plan(double sx, double sy, double gx, double gy) const
{
  const Lanelet & ls = nearest(sx, sy);
  const Lanelet & lg = nearest(gx, gy);
  const size_t si = project(ls, sx, sy).first;
  const size_t gi = project(lg, gx, gy).first;
  if (ls.id == lg.id && gi >= si) {return Plan{{ls.id}, si, gi};}
  std::optional<std::pair<double, std::vector<std::string>>> best;
  for (const auto & s : succ_.at(ls.id)) {
    const auto dj = route(s, lg.id);
    const auto it = dj.dist.find(lg.id);
    if (it != dj.dist.end() && (!best || it->second < best->first)) {
      std::vector<std::string> chain;
      std::optional<std::string> u = lg.id;
      while (u) {chain.push_back(*u); u = dj.prev.at(*u);}
      std::reverse(chain.begin(), chain.end());
      chain.insert(chain.begin(), ls.id);
      best = {it->second, chain};
    }
  }
  if (!best) {return std::nullopt;}
  return Plan{best->second, si, gi};
}

Poly RouteCore::chain_points(const std::vector<std::string> & chain, size_t si, size_t gi) const
{
  Poly pts;
  for (size_t k = 0; k < chain.size(); ++k) {
    const Poly & c = lanelet(chain[k]).center;
    const size_t a = k == 0 ? si : 0;
    const size_t b = k == chain.size() - 1 ? gi : c.size() - 1;
    for (size_t i = a; i <= b && i < c.size(); ++i) {pts.push_back(c[i]);}
  }
  return pts;
}

SmoothResult RouteCore::smooth_pts(const Poly & pts) const
{
  return smooth(pts, p_.wheelbase, p_.steer_limit(), p_.smoothing_max_deviation);
}

std::optional<Lap> RouteCore::lap_points(double hx, double hy) const
{
  const Lanelet & lh = nearest(hx, hy);
  const size_t h = project(lh, hx, hy).first;
  std::optional<std::pair<double, std::vector<std::string>>> best;
  for (const auto & s0 : succ_.at(lh.id)) {
    const auto dj = route(s0, lh.id);
    const auto it = dj.dist.find(lh.id);
    if (it != dj.dist.end() && (!best || it->second < best->first)) {
      std::vector<std::string> chain;
      std::optional<std::string> u = lh.id;
      while (u) {chain.push_back(*u); u = dj.prev.at(*u);}
      std::reverse(chain.begin(), chain.end());
      best = {it->second, chain};
    }
  }
  if (!best) {return std::nullopt;}
  const auto & chain = best->second;
  Poly pts(lh.center.begin() + h, lh.center.end());
  for (size_t k = 0; k < chain.size(); ++k) {
    const Poly & c = lanelet(chain[k]).center;
    if (k == chain.size() - 1 && chain[k] == lh.id) {
      pts.insert(pts.end(), c.begin(), c.begin() + std::min(h + 1, c.size()));
    } else {
      pts.insert(pts.end(), c.begin(), c.end());
    }
  }
  std::vector<std::string> out{lh.id};
  out.insert(out.end(), chain.begin(), chain.end());
  return Lap{out, pts};
}
}  // namespace buggy_core
