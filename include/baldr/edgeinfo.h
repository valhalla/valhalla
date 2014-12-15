#ifndef VALHALLA_BALDR_EDGEINFO_H_
#define VALHALLA_BALDR_EDGEINFO_H_

#include <vector>
#include <string>

#include "geo/util.h"
#include "geo/pointll.h"
#include "graphid.h"

using namespace valhalla::geo;

namespace valhalla {
namespace baldr {

/**
 * Edge information not required in shortest path algorithm and is
 * common among the 2 directions.
 * @author  David W. Nesbitt
 */
class EdgeInfo {
 public:
  /**
   * Constructor
   */
  EdgeInfo();

  /**
   * Offset into EdgeInfo collection.
   */
  unsigned int offset() const;

  /**
   * Get the reference node (start) of the edge.
   * @return  Returns the GraphId of the reference node of the edge.
   */
  const GraphId& nodea() const;

  /**
   * Get the end node of the edge.
   * @return  Returns the GraphId of the end node of the edge.
   */
  const GraphId& nodeb() const;

  /**
   * Get the shape of the edge.
   * @return  Returns the the list of lat,lng points describing the
   * *        shape of the edge.
   */
  const std::vector<PointLL>& shape() const;

  /**
   * Get the indexes to names used by this edge
   * @return  Returns a list of name indexes.
   */
  const std::vector<std::string>& nameindexes() const;

  // Operator EqualTo based nodea and nodeb.
  bool operator ==(const EdgeInfo& rhs) const;

 protected:
  unsigned int offset_;

  // GraphIds of the 2 end nodes
  GraphId nodea_;
  GraphId nodeb_;

  // Lat,lng shape of the edge
  std::vector<PointLL> shape_;

  // TODO - add edge information

  // List of roadname indexes
  std::vector<std::string> nameindexes_;
};

}
}

#endif  // VALHALLA_BALDR_EDGEINFO_H_
