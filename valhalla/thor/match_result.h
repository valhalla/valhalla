// -*- mode: c++ -*-
#ifndef VALHALLA_THOR_MATCH_RESULT_H_
#define VALHALLA_THOR_MATCH_RESULT_H_

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/meili/match_result.h>


namespace valhalla {
namespace thor {

constexpr uint32_t kInvalidEdgeIndex = std::numeric_limits<uint32_t>::max();

struct MatchResult : meili::MatchResult {
  enum class Type {
      kUnmatched,
      kInterpolated,
      kMatched
  };

  MatchResult(meili::MatchResult result) {
    lnglat = result.lnglat;
    distance_from = result.distance_from;
    edgeid = result.edgeid;
    distance_along = result.distance_along;
    epoch_time = result.epoch_time;
    stateid = result.stateid;

    // Set the type based on edge id and state
    if (edgeid.Is_Valid() && HasState())
      type = Type::kMatched;
    else if (edgeid.Is_Valid())
      type = Type::kInterpolated;
    else
      type = Type::kUnmatched;

    // Default values for edge index and disconnected route boundary
    edge_index = kInvalidEdgeIndex;
    disconnected_route_boundary = false;
  }

  // The result type of the point based on edge id and state
  Type type;

  // Index of the edge
  uint32_t edge_index = kInvalidEdgeIndex;

  // True if disconnected path at this point, otherwise false
  bool disconnected_route_boundary;

  bool HasEdgeIndex() const { return edge_index != kInvalidEdgeIndex; }
};

}
}
#endif // VALHALLA_THOR_MATCH_RESULT_H_
