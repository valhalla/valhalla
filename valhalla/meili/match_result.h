// -*- mode: c++ -*-
#ifndef MMP_MATCH_RESULT_H_
#define MMP_MATCH_RESULT_H_

#include <vector>
#include <algorithm>

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
  bool operator==(const MatchResults& o) const {
    //account for node snaps at the beginning and end of this result
    auto e1 = segments.empty() || segments.front().source < 1.0f || edges.size() ? edges.cbegin() : edges.cbegin() + 1;
    auto e2 = segments.empty() || segments.back().target > 0.0f || edges.size() ? edges.cend() : edges.cend() - 1;
    //account for node snaps at the beginning and end of the other result
    auto e3 = o.segments.empty() || o.segments.front().source < 1.0f || o.edges.size() ? o.edges.cbegin() : o.edges.cbegin() + 1;
    auto e4 = o.segments.empty() || o.segments.back().target > 0.0f || o.edges.size() ? o.edges.cend() : o.edges.cend() - 1;
    //if we can find the one path contained within the other then they are the same
    return std::search(e1, e2, e3, e4) != e2 || std::search(e3, e4, e1, e2) != e4;
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

#endif // MMP_MATCH_RESULT_H_
