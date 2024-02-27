#ifndef VALHALLA_TYR_ROUTE_SUMMARY_CACHE_H_
#define VALHALLA_TYR_ROUTE_SUMMARY_CACHE_H_
#pragma once

#include "proto/directions.pb.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"
#include "proto_conversions.h"

namespace valhalla {
namespace tyr {

struct NamedSegment {
  std::string name;
  uint32_t index;
  float distance;

  NamedSegment(const std::string& _n, uint32_t _i, float _d) : name(_n), index(_i), distance(_d) {
  }

  NamedSegment(const NamedSegment& ns) : name(ns.name), index(ns.index), distance(ns.distance) {
  }

  NamedSegment(NamedSegment&& ns) : name(std::move(ns.name)), index(ns.index), distance(ns.distance) {
  }

  NamedSegment& operator=(const NamedSegment& ns);

  NamedSegment& operator=(NamedSegment&& ns) noexcept;
};

//=============================================================================
// Cache the summaries by route/leg/#-of-named-segs. Has a couple benefits:
// 1) lets the business logic be business-y (ask for the same thing repeatedly
//    without performance concerns)
// 2) only computes/caches what is needed (doesn't precompute everything
//    needlessly)
//
// Here's an example of what would be cached for a fictional route/leg.
//
// Incoming routes data:
//--------------------------------------------------
// maneuver   name      length     maneuver index
//--------------------------------------------------
// 0          R0        8.8        0
// 1          R1        5.5        1
// 2          R2        10.0       2
// 3          R3        9.0        3
//
// route_leg_segs_by_dist would store:
//--------------------------------------------------
// name       length       maneuver index
//--------------------------------------------------
// R2         10.0         2
// R3         9.0          3
// R0         8.8          0
// R1         5.5          1
//
// Since there are four named-segments there can be up to four summaries.
// It just depends on how many named-segments you wish to comprise the
// summary. Here are the summaries that would be stored in the cache:
//--------------------------------------------------
// number of segments in summary      summary
//--------------------------------------------------
// 1 (index 0)                        R2
// 2 (index 1)                        R2, R3
// 3 (index 2)                        R0, R2, R3
// 4 (index 3)                        R0, R1, R2, R3
//=============================================================================
class route_summary_cache {
  // vector 1: routes
  // vector 2: legs for the route
  // vector 3: named segments sorted by longest segment distance
  std::vector<std::vector<std::vector<NamedSegment>>> route_leg_segs_by_dist;

  // vector 1: routes
  // vector 2: legs for the route
  // vector 3: summary for the desired # of named segments (offset by -1)
  std::vector<std::vector<std::vector<std::string>>> cache;

  int hits = 0;
  int misses = 0;

public:
  route_summary_cache(const google::protobuf::RepeatedPtrField<DirectionsRoute>& routes);
  size_t num_named_segments_for_route_leg(size_t route_idx, size_t leg_idx);

  // Return the n-part named-segment summary for the given route/leg.
  // The summary returned is guaranteed to be comprised of as few named
  // segments as possible, while also being unique among all route/same-leg
  // summaries.
  std::string get_n_segment_summary(size_t route_idx, size_t leg_idx, size_t num_named_segs);
};
} // namespace tyr
} // namespace valhalla

#endif // VALHALLA_TYR_ROUTE_SUMMARY_CACHE_H
