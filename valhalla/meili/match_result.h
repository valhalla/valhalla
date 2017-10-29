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
  MatchResults(std::vector<MatchResult>&& results, std::vector<EdgeSegment>&& segments, float score):
    results(results), segments(segments), score(score) { }
  MatchResults(const MatchResults&) = delete;
  MatchResults& operator=(const MatchResults&) = delete;
  MatchResults(MatchResults&& o) {
    results = std::move(o.results);
    segments = std::move(o.segments);
    score = o.score;
  }
  MatchResults& operator=(MatchResults&& o) {
    results = std::move(o.results);
    segments = std::move(o.segments);
    score = o.score;
    return *this;
  }

  std::vector<MatchResult> results;
  std::vector<EdgeSegment> segments;
  float score;
};

}
}

#endif // MMP_MATCH_RESULT_H_
