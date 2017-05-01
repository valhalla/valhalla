#ifndef VALHALLA_THOR_ROUTE_MATCHER_H_
#define VALHALLA_THOR_ROUTE_MATCHER_H_

#include <cstdint>
#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <memory>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathinfo.h>

namespace valhalla {
namespace thor {

class RouteMatcher {
 public:

  static bool FormPath(const std::shared_ptr<sif::DynamicCost>* mode_costing,
                       const sif::TravelMode& mode, baldr::GraphReader& reader,
                       const std::vector<midgard::PointLL>& shape,
                       const std::vector<baldr::PathLocation>& correlated,
                       std::vector<PathInfo>& path_infos);
};

}
}

#endif  // VALHALLA_THOR_ROUTE_MATCHER_H_
