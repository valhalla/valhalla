#ifndef VALHALLA_THOR_MAP_MATCHER_H_
#define VALHALLA_THOR_MAP_MATCHER_H_

#include <cstdint>
#include <deque>
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

  static std::deque<std::vector<std::pair<PathInfo, const meili::EdgeSegment*>>>
  FormPathNew(meili::MapMatcher* matcher,
              const std::vector<meili::MatchResult>& results,
              const std::vector<meili::EdgeSegment>& edge_segments,
              const std::shared_ptr<sif::DynamicCost>* mode_costing,
              const sif::TravelMode mode,
              std::vector<std::pair<baldr::GraphId, baldr::GraphId>>& disconnected_edges,
              Options& options);

private:
  struct interpolation_t {
    baldr::GraphId edge;   // edge id
    float total_distance;  // distance along the path
    float edge_distance;   // ratio of the distance along the edge
    size_t original_index; // index into the original measurements
    double epoch_time;     // seconds from epoch
  };

  static uint32_t compute_origin_epoch(const std::vector<meili::EdgeSegment>& edge_segments,
                                       meili::MapMatcher* matcher,
                                       Options& options);

  static std::vector<std::vector<interpolation_t>>
  interpolate_matches(const std::vector<meili::MatchResult>& matches,
                      const std::vector<meili::EdgeSegment>& edges,
                      meili::MapMatcher* matcher);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_MAP_MATCHER_H_
