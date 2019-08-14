#ifndef VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_
#define VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_

#include <cstdint>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/mjolnir/osmway.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

/**
 * Derived class to build a directed edge given OSM way and other properties.
 */
class DirectedEdgeBuilder : public baldr::DirectedEdge {
public:
  /**
   * Constructor with arguments.
   * @param  way            OSM way info generated from parsing OSM tags with Lua.
   * @param  endnode        GraphId of the end node of this directed edge.
   * @param  length         Length in meters.
   * @param  speed          Average speed in kph.
   * @param  truck_speed    Truck speed limit in kph.
   * @param  use            Use of the edge.
   * @param  rc             Road class / importance
   * @param  localidx       Index of the edge (from the node) on the local level
   * @param  restrictions   Mask of simple turn restrictions at the end node
   *                        of this directed edge.
   * @param  bike_network   Mask of bike_networks from relations.
   * @param  reclass_ferry  Reclassify ferry boolean; Allows us to drop destination only attribution
   * or anything that would prevent seeing a ferry connection
   */
  DirectedEdgeBuilder(const OSMWay& way,
                      const baldr::GraphId& endnode,
                      const bool forward,
                      const uint32_t length,
                      const uint32_t speed,
                      const uint32_t truck_speed,
                      const baldr::Use use,
                      const baldr::RoadClass rc,
                      const uint32_t localidx,
                      const bool signal,
                      const uint32_t restrictions,
                      const uint32_t bike_network,
                      const bool reclass_ferry);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_
