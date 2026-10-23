// Lanelet routing without ROS types: successor graph (shared nodes + geometric ring links),
// shortest lane chain, centreline extraction, smoothing, one-lap path. Port of the planning
// methods of route_planner.py.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "buggy_core/planning/lanelet_map.hpp"
#include "buggy_core/planning/path_utils.hpp"

namespace buggy_core
{
struct RouteParams
{
  double wheelbase = 1.6, max_steer_deg = 20.0, steering_margin = 0.9;
  double smoothing_max_deviation = 0.6;
  double link_gap = 4.0, link_angle_deg = 60.0, home_reach = 2.5;
  double steer_limit() const { return max_steer_deg * steering_margin; }
};

struct Plan { std::vector<std::string> chain; size_t si, gi; };
struct Lap { std::vector<std::string> chain; Poly pts; };

class RouteCore
{
public:
  RouteCore(std::vector<Lanelet> lanelets, RouteParams p);

  // ring links added by position: (a, b) pairs, for the log
  const std::vector<std::pair<std::string, std::string>> & linked() const { return linked_; }

  const Lanelet & lanelet(const std::string & id) const { return lanelets_[index_.at(id)]; }
  const std::vector<Lanelet> & lanelets() const { return lanelets_; }
  const std::vector<std::string> & succ(const std::string & id) const { return succ_.at(id); }
  const RouteParams & params() const { return p_; }

  const Lanelet & nearest(double x, double y) const;
  std::optional<Plan> plan(double sx, double sy, double gx, double gy) const;
  Poly chain_points(const std::vector<std::string> & chain, size_t si, size_t gi) const;
  SmoothResult smooth_pts(const Poly & pts) const;
  std::optional<Lap> lap_points(double hx, double hy) const;

private:
  struct Dij { std::map<std::string, double> dist; std::map<std::string, std::optional<std::string>> prev; };
  Dij route(const std::string & a, const std::string & b) const;
  void link_geometrically();

  std::vector<Lanelet> lanelets_;
  std::unordered_map<std::string, size_t> index_;
  std::map<std::string, std::vector<std::string>> succ_;
  RouteParams p_;
  std::vector<std::pair<std::string, std::string>> linked_;
};
}  // namespace buggy_core
