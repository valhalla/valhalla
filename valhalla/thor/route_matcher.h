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
  /**
   * Form a path by matching shape with graph edges (edge walking).
   * @param mode_costing Dynamic costing methods used to determine allowed edges and costs.
   * @param mode Travel mode (indexes the costing methods).
   * @param reader Graph reader.
   * @param shape Shape used to match edges. Can include timestamps.
   * @param use_timestamps If true the timestamps will be used to compute elapsed time,
   *                       otherwise costing methods determine elapsed time along the path.
   * @param correlated Correlated locations (generally the first and last shape point).
   * @param path_infos Returns PathInfo - list of edges and elapsed times.
   * @return Returns true if the edge walk forms a path, false if shape does not match.
   */
  static bool FormPath(const std::shared_ptr<sif::DynamicCost>* mode_costing,
                       const sif::TravelMode& mode,
                       baldr::GraphReader& reader,
                       const std::vector<meili::Measurement>& shape,
                       valhalla::Options& options,
                       std::vector<PathInfo>& path_infos);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_ROUTE_MATCHER_H_
