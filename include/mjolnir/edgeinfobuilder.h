#ifndef VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_
#define VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_

#include <vector>
#include <string>

#include "geo/util.h"
#include "geo/pointll.h"
#include "baldr/graphid.h"
#include "baldr/edgeinfo.h"

using namespace valhalla::geo;

namespace valhalla {
namespace mjolnir {

/**
 * Edge information not required in shortest path algorithm and is
 * common among the 2 directions.
 */
class EdgeInfoBuilder : public baldr::EdgeInfo {
 public:
  /**
   * Constructor
   */
  EdgeInfoBuilder();

  /**
   * Set the reference node (start) of the edge.
   * @param  nodea  the GraphId of the reference node of the edge.
   */
  void set_nodea(const baldr::GraphId& nodea);

  /**
   * Set the end node of the edge.
   * @param  nodeb  the GraphId of the end node of the edge.
   */
  void set_nodeb(const baldr::GraphId& nodeb);

  /**
   * Set the shape of the edge.
   * @param  shape  the the list of lat,lng points describing the
   * *        shape of the edge.
   */
  void set_shape(const std::vector<PointLL>& shape);

  /**
   * Set the indexes to names used by this edge
   * @param  nameindexes  a list of name indexes.
   */
  void set_nameindexes(const std::vector<uint32_t>& nameindexes);

};

}
}

#endif  // VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_
