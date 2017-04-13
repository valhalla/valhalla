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
  MatchResult(meili::MatchResult result, uint32_t index = kInvalidEdgeIndex) {
    lnglat = result.lnglat;
    distance_from = result.distance_from;
    edgeid = result.edgeid;
    distance_along = result.distance_along;
    epoch_time = result.epoch_time;
    stateid = result.stateid;
    edge_index = index;
  }

  // Index of the edge
  uint32_t edge_index = kInvalidEdgeIndex;

  bool HasEdgeIndex() const { return edge_index != kInvalidEdgeIndex; }
};

}
}
#endif // VALHALLA_THOR_MATCH_RESULT_H_
