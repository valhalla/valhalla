#ifndef VALHALLA_THOR_MAP_MATCHER_H_
#define VALHALLA_THOR_MAP_MATCHER_H_

#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/meili/map_matcher.h>
#include <valhalla/meili/match_result.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathinfo.h>

namespace valhalla {
namespace thor {

class MapMatcher {
public:
  static std::vector<PathInfo>
  FormPath(meili::MapMatcher* matcher,
           const std::vector<meili::MatchResult>& results,
           const std::vector<meili::EdgeSegment>& edge_segments,
           const std::shared_ptr<sif::DynamicCost>* mode_costing,
           const sif::TravelMode mode,
           std::vector<std::pair<baldr::GraphId, baldr::GraphId>>& disconnected_edges,
           Options& options);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_MAP_MATCHER_H_
