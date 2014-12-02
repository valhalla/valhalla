#ifndef __nodeinfo_h_
#define __nodeinfo_h_

#include "geo/util.h"
#include "geo/pointll.h"
#include "graphid.h"

using namespace valhalla::geo;

namespace valhalla{
namespace baldr{

/**
 * Information held for each node within the graph. The graph uses a forward
 * star structure: nodes point to the first outbound directed edge and each
 * directed edge points to the other end node of the edge.
 * @author  David W. Nesbitt
 */
class NodeInfo {
 public:
  /**
   * Constructor
   */
  NodeInfo():nedges_(0) {
    latlng_.Set(0.0f, 0.0f);
  }

  /**
   * Sets the latitude and longitude.
   * @param  ll  Lat,lng position of the node.
   */
  void SetLatLng(const PointLL& ll) {
    latlng_ = ll;
  }

  /**
   * Get the latitude, longitude of the node.
   * @return  Returns the latitude and longitude of the node.
   */
  const PointLL& LatLng() const {
    return latlng_;
  }

  /**
   * Get the GraphId of the first outbound edge from this node.
   * @return  Returns the GraphId of the first outbound edge.
   */
  GraphId Edge() {
    return edge_;
  }

  /**
   * Get the number of outbound directed edges.
   * @return  Returns the number of outbound directed edges.
   */
  unsigned int EdgeCount() {
    return nedges_;
  }

 protected:
   // Latitude, longitude position of the node.
   PointLL latlng_;

   // GraphId of the first directed edge outbound from this node
   GraphId edge_;

   // Number of outbound edges
   // TODO - add this to a bit field to compress with other node data.
   // Rather than number of driveable, we can probably sort by driveability
   // to optimized for drving routes - when the first non driveable edge is
   // encountered the successive edges can be skipped
   unsigned int nedges_;
};

// TODO - do we use the NodeInfo class as a read-only class and create a
// derived class with Set methods?

}
}

#endif
