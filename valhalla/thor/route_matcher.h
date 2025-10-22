#ifndef VALHALLA_THOR_ROUTE_MATCHER_H_
#define VALHALLA_THOR_ROUTE_MATCHER_H_

#include <valhalla/baldr/graphreader.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/thor/pathinfo.h>

#include <vector>

namespace valhalla {
namespace thor {

class RouteMatcher {
public:
  /**
   * Form a path by matching shape with graph edges (edge walking). Also sets the path_edges
   * on the appropriate locations in the shape so that trip leg builder will have that info
   *
   * @param mode_costing   Dynamic costing methods used to determine allowed edges and costs.
   * @param mode           Travel mode (indexes the costing methods).
   * @param reader         Access to the tiled graph data
   * @param options        All the information about the request
   * @param legs           Vector of vector of path_info, edge-wise cost/time information
   * @return Returns true if the edge walk forms a path, false if shape does not match.
   */
  static bool FormPath(const sif::mode_costing_t& mode_costing,
                       const sif::TravelMode& mode,
                       baldr::GraphReader& reader,
                       valhalla::Options& options,
                       std::vector<std::vector<PathInfo>>& legs);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_ROUTE_MATCHER_H_
