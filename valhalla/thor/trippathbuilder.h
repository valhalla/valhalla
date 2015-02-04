#ifndef VALHALLA_THOR_TRIPPATHBUILDER_H_
#define VALHALLA_THOR_TRIPPATHBUILDER_H_

#include <vector>
#include <map>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/trippath.pb.h>

namespace valhalla {
namespace thor {

/**
 * Algorithm to create a trip path output from a list of directed edges.
 */
class TripPathBuilder {
 public:
  /**
   * Constructor.
   */
  TripPathBuilder();

  /**
   * Destructor
   */
  virtual ~TripPathBuilder();

  /**
   * Format the trip path output given the edges on the path.
   * For now just return length. TODO - modify to return trip path.
   */
  static odin::TripPath Build(baldr::GraphReader& graphreader,
             const std::vector<baldr::GraphId>& pathedges);

  /**
   * Add trip edge. (TOD more comments)
   * @param  idx  Index of the directed edge within the tile.
   * @param  directededge  Directed edge information.
   * @param  trip_node     Trip node to add the edge information to.
   * @param  graphtile     Graph tile for accessing data.
   */
  static odin::TripPath_Edge* AddTripEdge(const uint32_t idx,
                                          const baldr::DirectedEdge* directededge,
                                          odin::TripPath_Node* trip_node,
                                          const baldr::GraphTile* graphtile);
};

}
}

#endif  // VALHALLA_THOR_TRIPPATHBUILDER_H_
