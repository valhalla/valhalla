// -*- mode: c++ -*-
#ifndef MMP_MATCH_RESULT_H_
#define MMP_MATCH_RESULT_H_

#include <algorithm>
#include <limits>
#include <vector>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/meili/stateid.h>
#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace meili {

constexpr size_t kInvalidEdgeIndex = std::numeric_limits<size_t>::max();

struct MatchResult {
  // Coordinate of the match point
  midgard::PointLL lnglat;
  // Distance from measurement to the match point
  double distance_from;
  // Which edge this match point stays
  baldr::GraphId edgeid;
  // Percentage distance along the edge
  double distance_along;
  // Optional epoch time for this match point copied from original measurement
  double epoch_time;
  // Sequential state id
  StateId stateid;
  // Whether or not this was a break or break_through type location
  bool is_break_point;
  // Whether or not there is a discontinuity starting from this point
  bool begins_discontinuity;
  // Whether or not a discontinuity ends at this point
  bool ends_discontinuity;
  // An index into the full list of edges in the path even across discontinuities (for
  // trace_attributes)
  size_t edge_index = kInvalidEdgeIndex;

  bool HasState() const {
    return stateid.IsValid();
  }

  enum class Type { kUnmatched, kInterpolated, kMatched };
  Type GetType() const {
    // Set the type based on edge id and state
    if (edgeid.Is_Valid() && HasState()) {
      return Type::kMatched;
    } else if (edgeid.Is_Valid()) {
      return Type::kInterpolated;
    } else {
      return Type::kUnmatched;
    }
  }

  // Stream output
  friend std::ostream& operator<<(std::ostream& os, const MatchResult& r) {
    os << std::fixed << std::setprecision(6);
    os << "lnglat: " << r.lnglat.lng() << "," << r.lnglat.lat() << std::setprecision(3)
       << ", distance_from: " << r.distance_from << ", edgeid: " << r.edgeid
       << ", distance_along: " << r.distance_along << ", epoch_time: " << r.epoch_time
       << ", stateid.time: " << r.stateid.time() << ", stateid.id: " << r.stateid.id()
       << ", is_break_point:" << r.is_break_point
       << ", begins_discontinuity:" << r.begins_discontinuity
       << ", ends_discontinuity:" << r.ends_discontinuity << ", edge_index: " << r.edge_index;
    return os;
  }
};

struct EdgeSegment {
  EdgeSegment(baldr::GraphId the_edgeid,
              double the_source = 0.f,
              double the_target = 1.f,
              int the_first_match_idx = -1,
              int the_last_match_idx = -1,
              bool disconnect = false,
              int restriction_idx = -1);

  baldr::GraphId edgeid;
  double source{0.f};
  double target{1.f};
  int first_match_idx{-1};
  int last_match_idx{-1};
  uint8_t restriction_idx{baldr::kInvalidRestriction};
  bool discontinuity{false};

  // Stream output
  friend std::ostream& operator<<(std::ostream& os, const EdgeSegment& segment) {
    os << std::fixed << std::setprecision(3);
    os << "edgeid: " << segment.edgeid << ", source: " << segment.source
       << ", target: " << segment.target << ", first match idx: " << segment.first_match_idx
       << ", last match idx: " << segment.last_match_idx
       << ", discontinuity: " << segment.discontinuity;
    return os;
  }
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

  // Stream output
  friend std::ostream& operator<<(std::ostream& os, const MatchResults& mrs) {
    os << "MatchResults:" << std::endl;
    for (const auto& r : mrs.results) {
      os << r << std::endl;
    }
    os << "EdgeSegments:" << std::endl;
    for (const auto& s : mrs.segments) {
      os << s << std::endl;
    }
    return os;
  }
};

} // namespace meili
} // namespace valhalla

#endif // MMP_MATCH_RESULT_H_
