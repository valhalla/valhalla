#ifndef VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_
#define VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_

#include "baldr/graphid.h"
#include "baldr/directededge.h"

namespace valhalla {
namespace mjolnir {

/**
 * Directed edge within the graph.
 * @author  David W. Nesbitt
 */
class DirectedEdgeBuilder : public baldr::DirectedEdge {
 public:
  /**
   * Constructor
   */
  DirectedEdgeBuilder();

  /**
   * Sets the length of the edge in kilometers.
   * @param  length  Length of the edge in kilometers.
   */
  void set_length(const float length);

  /**
   * Set the end node of this directed edge.
   * @param  endnode  End node of the directed link.
   */
  void set_endnode(const baldr::GraphId& endnode);

  /**
   * Set the offset to the common edge data. The offset is from the start
   * of the common edge data within a tile.
   * @param  offset  Offset from the start of the edge data within a tile.
   */
  void set_edgedataoffset(const unsigned int offset);

  // TODO - methods for access

  /**
   * Sets the speed in KPH.
   * @param  speed  Speed in KPH.
  */
  void set_speed(const float speed);
};

}
}

#endif  // VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_

