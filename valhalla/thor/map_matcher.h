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
  /**
   * Similar to the rest of the routing algorithms returns a vector of PathInfo. This one is
   * slightly more complicated in 2 ways. First it can return multiple vectors of PathInfo.
   * There will be one for each leg in the path (determined by break/break_through locations)
   * and it will also return new ones where there are discontinuities in the match. Finally
   * it combines these with a vector of edge segments representing the portion of the total
   * map matched path that applies to a given vector of PathInfos.
   * @param matcher         The map matcher object used to produce the match
   * @param results         The results vector from the match (how tracepoints where matched)
   * @param edge_segments   The edge segments (the path that was matched)
   * @param mode_costing    The costing used to do the match
   * @param mode            Which travel mode was used with the costing
   * @param options         The request object with the input trace points (options.shape())
   * @return
   */
  static std::deque<std::pair<std::vector<PathInfo>, std::vector<const meili::EdgeSegment*>>>
  FormPath(meili::MapMatcher* matcher,
           const std::vector<meili::MatchResult>& results,
           const std::vector<meili::EdgeSegment>& edge_segments,
           const sif::mode_costing_t& mode_costing,
           const sif::TravelMode mode,
           Options& options);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_MAP_MATCHER_H_
