#ifndef VALHALLA_THOR_TRIPPATHBUILDER_H_
#define VALHALLA_THOR_TRIPPATHBUILDER_H_

#include <vector>
#include <map>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/trippath.pb.h>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::odin;

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

  static odin::TripPath_Edge* AddTripEdge(const DirectedEdge* directededge,TripPath_Node* trip_node,
                                          const GraphTile* graphtile);
};

}
}

#endif  // VALHALLA_THOR_TRIPPATHBUILDER_H_
