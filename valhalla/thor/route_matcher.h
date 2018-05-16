#ifndef VALHALLA_THOR_ROUTE_MATCHER_H_
#define VALHALLA_THOR_ROUTE_MATCHER_H_

#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/meili/measurement.h>
#include <valhalla/proto/tripcommon.pb.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathinfo.h>

namespace valhalla {
namespace thor {

class RouteMatcher {
public:
  static bool FormPath(const std::shared_ptr<sif::DynamicCost>* mode_costing,
                       const sif::TravelMode& mode,
                       baldr::GraphReader& reader,
                       const std::vector<meili::Measurement>& shape,
                       const google::protobuf::RepeatedPtrField<odin::Location>& correlated,
                       std::vector<PathInfo>& path_infos);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_ROUTE_MATCHER_H_
