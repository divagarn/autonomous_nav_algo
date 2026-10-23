#include "buggy_core/planning/lanelet_map.hpp"

#include <cmath>
#include <map>
#include <stdexcept>

#include <tinyxml2.h>

namespace buggy_core
{
Lanelet::Lanelet(std::string lid, const Poly & l, const Poly & r,
  std::pair<std::string, std::string> f, std::pair<std::string, std::string> la)
: id(std::move(lid)), first(std::move(f)), last(std::move(la))
{
  Poly sl = resample(l, 0.5), sr = resample(r, 0.5);
  const size_t k = std::max(sl.size(), sr.size());
  Poly nl, nr;
  for (size_t i = 0; i < k; ++i) {
    // Python: sl[min(len-1, round(i * (len-1) / (k-1)))]  (round = half to even)
    const auto pick = [&](const Poly & s) {
        const double v = std::nearbyint(static_cast<double>(i) * (s.size() - 1) / (k - 1));
        return s[std::min<size_t>(s.size() - 1, static_cast<size_t>(v))];
      };
    nl.push_back(pick(sl));
    nr.push_back(pick(sr));
  }
  left = nl;
  right = nr;
  for (size_t i = 0; i < k; ++i) {
    center.push_back({(left[i][0] + right[i][0]) / 2, (left[i][1] + right[i][1]) / 2, (left[i][2] + right[i][2]) / 2});
  }
  length = length3(center);
}

std::vector<Lanelet> load_lanelets(const std::string & path)
{
  using namespace tinyxml2;
  XMLDocument doc;
  if (doc.LoadFile(path.c_str()) != XML_SUCCESS) {throw std::runtime_error("cannot read lanelet map: " + path);}
  XMLElement * root = doc.RootElement();
  auto tags = [](XMLElement * e) {
      std::map<std::string, std::string> t;
      for (XMLElement * x = e->FirstChildElement("tag"); x; x = x->NextSiblingElement("tag")) {
        if (x->Attribute("k") && x->Attribute("v")) {t[x->Attribute("k")] = x->Attribute("v");}
      }
      return t;
    };
  std::map<std::string, Pt> nodes;
  for (XMLElement * n = root->FirstChildElement("node"); n; n = n->NextSiblingElement("node")) {
    const auto t = tags(n);
    if (t.count("local_x")) {
      const auto ele = t.find("ele");
      nodes[n->Attribute("id")] = {std::stod(t.at("local_x")), std::stod(t.at("local_y")),
        ele == t.end() ? 0.0 : std::stod(ele->second)};
    }
  }
  std::map<std::string, std::vector<std::string>> ways;
  for (XMLElement * w = root->FirstChildElement("way"); w; w = w->NextSiblingElement("way")) {
    std::vector<std::string> refs;
    for (XMLElement * r = w->FirstChildElement("nd"); r; r = r->NextSiblingElement("nd")) {refs.push_back(r->Attribute("ref"));}
    ways[w->Attribute("id")] = refs;
  }
  std::vector<Lanelet> out;
  for (XMLElement * rel = root->FirstChildElement("relation"); rel; rel = rel->NextSiblingElement("relation")) {
    const auto t = tags(rel);
    const auto ty = t.find("type");
    if (ty == t.end() || ty->second != "lanelet") {continue;}
    std::map<std::string, std::string> m;
    for (XMLElement * x = rel->FirstChildElement("member"); x; x = x->NextSiblingElement("member")) {
      if (x->Attribute("role") && x->Attribute("ref")) {m[x->Attribute("role")] = x->Attribute("ref");}
    }
    const auto & L = ways.at(m.at("left"));
    const auto & R = ways.at(m.at("right"));
    Poly pl, pr;
    for (const auto & i : L) {pl.push_back(nodes.at(i));}
    for (const auto & i : R) {pr.push_back(nodes.at(i));}
    out.emplace_back(rel->Attribute("id"), pl, pr, std::make_pair(L.front(), R.front()),
      std::make_pair(L.back(), R.back()));
  }
  return out;
}

std::pair<size_t, double> project(const Lanelet & l, double px, double py)
{
  size_t best = 0;
  double bd = 1e300;
  for (size_t i = 0; i < l.center.size(); ++i) {
    const double d = std::hypot(l.center[i][0] - px, l.center[i][1] - py);
    if (d < bd) {bd = d; best = i;}
  }
  return {best, bd};
}
}  // namespace buggy_core
