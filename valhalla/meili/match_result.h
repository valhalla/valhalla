// -*- mode: c++ -*-
#ifndef MMP_MATCH_RESULT_H_
#define MMP_MATCH_RESULT_H_

#include <vector>
#include <functional>
#include <unordered_set>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
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

struct EdgeSegment
{
  EdgeSegment(baldr::GraphId the_edgeid,
              float the_source = 0.f,
              float the_target = 1.f);

  std::vector<midgard::PointLL>
  Shape(baldr::GraphReader& graphreader) const;

  bool
  Adjoined(baldr::GraphReader& graphreader, const EdgeSegment& other) const;

  // TODO make them private
  baldr::GraphId edgeid;

  float source;

  float target;
};

struct MatchResults {
  MatchResults(std::vector<MatchResult>&& results, std::vector<EdgeSegment>&& segments):
    results(results), segments(segments), edges(unique()) { }
  MatchResults(const MatchResults&) = delete;
  MatchResults& operator=(const MatchResults&) = delete;
  MatchResults(MatchResults&& o) {
    results = std::move(o.results);
    segments = std::move(o.segments);
    edges = std::move(o.edges);
    score = o.score;
  }
  MatchResults& operator=(MatchResults&& o) {
    results = std::move(o.results);
    segments = std::move(o.segments);
    edges = std::move(o.edges);
    score = o.score;
    return *this;
  }
  std::vector<MatchResult> results;
  std::vector<EdgeSegment> segments;
  std::vector<uint64_t> edges;
  float score;
 private:
  std::vector<uint64_t> unique() {
    std::vector<uint64_t> edges;
    edges.reserve(segments.size());
    for(const auto& segment : segments)
      if(edges.empty() || edges.back() != segment.edgeid)
        edges.push_back(segment.edgeid);
    return edges;
  }
};

}
}

namespace std {
  template <>
  struct equal_to<valhalla::meili::MatchResults> {
    bool operator()(const valhalla::meili::MatchResults& a, const valhalla::meili::MatchResults& b) const {
      //equal if they are the same edges but TODO: if one has an extra on either end
      //then they dont count if they are 100% along at the beginning edge or 0% along
      //at the ending edge
      return a.edges == b.edges;
    }
  };

  template <>
  struct hash<valhalla::meili::MatchResults> {
    size_t operator()(const valhalla::meili::MatchResults& r) const {
      std::hash<uint64_t> hasher;
      size_t seed = 0;
      uint64_t last = -1;
      for(auto id : r.edges)
        seed ^= hasher(id) + 0x9e3779b9 + (seed<<6) + (seed>>2);
      return seed;
    }
  };
}

#endif // MMP_MATCH_RESULT_H_
