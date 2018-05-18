// -*- mode: c++ -*-
#ifndef VALHALLA_THOR_MATCH_RESULT_H_
#define VALHALLA_THOR_MATCH_RESULT_H_

#include <valhalla/baldr/graphid.h>
#include <valhalla/meili/match_result.h>
#include <valhalla/midgard/pointll.h>

#include <utility>

namespace valhalla {
namespace thor {

constexpr uint32_t kInvalidEdgeIndex = std::numeric_limits<uint32_t>::max();

struct MatchResult : meili::MatchResult {
  enum class Type { kUnmatched, kInterpolated, kMatched };

  MatchResult(meili::MatchResult result) : meili::MatchResult(std::move(result)) {
    // Set the type based on edge id and state
    if (edgeid.Is_Valid() && HasState()) {
      type = Type::kMatched;
    } else if (edgeid.Is_Valid()) {
      type = Type::kInterpolated;
    } else {
      type = Type::kUnmatched;
    }

    // Default values for edge index and begin/end route discontinuity
    edge_index = kInvalidEdgeIndex;
    begin_route_discontinuity = false;
    end_route_discontinuity = false;
  }

  // The result type of the point based on edge id and state
  Type type;

  // Index of the edge
  uint32_t edge_index = kInvalidEdgeIndex;

  // True if begin location of route discontinuity, otherwise false
  bool begin_route_discontinuity;

  // True if end location of route discontinuity, otherwise false
  bool end_route_discontinuity;

  bool HasEdgeIndex() const {
    return edge_index != kInvalidEdgeIndex;
  }
};

struct RouteDiscontinuity {
  bool exists;
  midgard::PointLL vertex;
  float distance_along;
};

} // namespace thor
} // namespace valhalla
#endif // VALHALLA_THOR_MATCH_RESULT_H_
