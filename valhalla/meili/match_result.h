// -*- mode: c++ -*-
#ifndef MMP_MATCH_RESULT_H_
#define MMP_MATCH_RESULT_H_

#include <algorithm>
#include <vector>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/meili/stateid.h>
#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace meili {

struct MatchResult {
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

  bool HasState() const {
    return stateid.IsValid();
  }
};

struct EdgeSegment {
  EdgeSegment(baldr::GraphId the_edgeid, float the_source = 0.f, float the_target = 1.f);

  std::vector<midgard::PointLL> Shape(baldr::GraphReader& graphreader) const;

  bool Adjoined(baldr::GraphReader& graphreader, const EdgeSegment& other) const;

  // TODO make them private
  baldr::GraphId edgeid;

  float source;

  float target;
};

struct MatchResults {
  MatchResults(std::vector<MatchResult>&& results, std::vector<EdgeSegment>&& segments, float score)
      : results(results), segments(segments), score(score) {
    edges.reserve(this->segments.size());
    for (const auto& segment : this->segments)
      if (edges.empty() || edges.back() != segment.edgeid)
        edges.push_back(segment.edgeid);
    e1 = this->segments.empty() || this->segments.front().source < 1.0f ? edges.cbegin()
                                                                        : edges.cbegin() + 1;
    e2 = this->segments.empty() || this->segments.back().target > 0.0f ? edges.cend()
                                                                       : edges.cend() - 1;
  }
  // equality here means that the sequence of edges within p either contains or is contained in
  // this's sequence of edges, so we do basically a substring search algorithm
  bool operator==(const MatchResults& p) const {
    // if p's edge sequence is small enough to fit into this edge sequence and we can find it OR
    // if this edge sequence is small enough to fit into p's edge seuqence and we can find it
    return (p.edges.size() <= edges.size() && std::search(e1, e2, p.e1, p.e2) != edges.cend()) ||
           (edges.size() <= p.edges.size() && std::search(p.e1, p.e2, e1, e2) != p.edges.cend());
  }

  MatchResults(const MatchResults&) = delete;
  MatchResults& operator=(const MatchResults&) = delete;
  MatchResults(MatchResults&& o) {
    results = std::move(o.results);
    segments = std::move(o.segments);
    edges = std::move(o.edges);
    score = o.score;
    e1 = segments.empty() || segments.front().source < 1.0f ? edges.cbegin() : edges.cbegin() + 1;
    e2 = segments.empty() || segments.back().target > 0.0f ? edges.cend() : edges.cend() - 1;
  }
  MatchResults& operator=(MatchResults&& o) {
    results = std::move(o.results);
    segments = std::move(o.segments);
    edges = std::move(o.edges);
    score = o.score;
    e1 = segments.empty() || segments.front().source < 1.0f ? edges.cbegin() : edges.cbegin() + 1;
    e2 = segments.empty() || segments.back().target > 0.0f ? edges.cend() : edges.cend() - 1;
    return *this;
  }

  std::vector<MatchResult> results;
  std::vector<EdgeSegment> segments;
  std::vector<uint64_t> edges;
  float score;
  std::vector<uint64_t>::const_iterator e1;
  std::vector<uint64_t>::const_iterator e2;
};

} // namespace meili
} // namespace valhalla

#endif // MMP_MATCH_RESULT_H_
