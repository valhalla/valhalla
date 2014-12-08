#ifndef VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_
#define VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_

#include "geo/util.h"
#include "baldr/graphid.h"

namespace valhalla{
namespace mjolnir{

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
  void set_endnode(const GraphId& endnode);

  // TODO - methods for access

  /**
   * Sets the speed in KPH.
   * @param  speed  Speed in KPH.
  */
  void set_speed(const float speed);
};

#endif  // VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_
