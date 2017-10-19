// -*- mode: c++ -*-
#ifndef MMP_MATCH_RESULT_H_
#define MMP_MATCH_RESULT_H_

#include <vector>
#include <functional>
#include <unordered_set>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/meili/stateid.h>

namespace valhalla {
namespace meili {

struct MatchResult
{
  // Coordinate of the match point
  midgard::PointLL lnglat;
  // Distance from measurement to the match point
  float distance_from;
  // Which edge this match point stays
  baldr::GraphId edgeid;
  // Percentage distance along the edge
  float distance_along;
  // Optional epoch time for this match point copied from original measurement
  double epoch_time;
  // Sequential state id
  StateId stateid;

  bool HasState() const { return stateid.IsValid(); }
};

}
}

namespace std {
  template <>
  struct equal_to<std::vector<valhalla::meili::MatchResult> > {
    bool operator()(const std::vector<valhalla::meili::MatchResult>& a, const std::vector<valhalla::meili::MatchResult>& b) const {
      size_t i = 0, j = 0;
      uint64_t x = -1, y = -1;
      while(i < a.size() && j < b.size()) {
        if(a[i].edgeid == x){ ++i; continue; }
        if(b[j].edgeid == y){ ++j; continue; }
        x = a[i].edgeid;
        y = b[j].edgeid;
        if(x != y) return false;
        i += i < a.size() - 1;
        j += j < b.size() - 1;
      }
      return true;
    }
  };

  template <>
  struct hash<std::vector<valhalla::meili::MatchResult> > {
    size_t operator()(const std::vector<valhalla::meili::MatchResult>& r) const {
      //TODO: worry about using edges at the beginning of the path when the correlated point is 100% along the edge
      //or using edges at the end of the path when the correlated point is 0% along the edge, we probably dont want
      //those results either
      std::hash<uint64_t> hasher;
      size_t seed = 0;
      uint64_t last = -1;
      for (const auto& m : r)
        if(m.edgeid != last)
          seed ^= hasher(m.edgeid) + 0x9e3779b9 + (seed<<6) + (seed>>2);
      return seed;
    }
  };
}

#endif // MMP_MATCH_RESULT_H_
