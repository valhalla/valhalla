#ifndef VALHALLA_THOR_TRIPPATHBUILDER_H_
#define VALHALLA_THOR_TRIPPATHBUILDER_H_

#include <cstdint>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/proto/trip.pb.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/thor/attributes_controller.h>
#include <valhalla/thor/match_result.h>
#include <valhalla/thor/pathinfo.h>

namespace valhalla {
namespace thor {

/**
 * Algorithm to create a trip path output from a list of directed edges.
 */
class TripLegBuilder {
public:
  /**
   * Format the trip path output given the edges on the path.
   * For now just return length. TODO - modify to return trip path.
   */
  static TripLeg Build(const AttributesController& controller,
                       baldr::GraphReader& graphreader,
                       const std::shared_ptr<sif::DynamicCost>* mode_costing,
                       const std::vector<PathInfo>::const_iterator path_begin,
                       const std::vector<PathInfo>::const_iterator path_end,
                       valhalla::Location& origin,
                       valhalla::Location& dest,
                       const std::list<valhalla::Location>& through_loc,
                       TripLeg& trip_path,
                       const std::function<void()>* interrupt_callback = nullptr,
                       std::unordered_map<size_t, std::pair<RouteDiscontinuity, RouteDiscontinuity>>*
                           route_discontinuities = nullptr);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_TRIPPATHBUILDER_H_
