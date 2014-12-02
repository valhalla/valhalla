#include "baldr/nodeinfo.h"

namespace valhalla{
namespace baldr{
  NodeInfo::NodeInfo():nedges_(0) {
    latlng_.Set(0.0f, 0.0f);
  }

  /**
   * Sets the latitude and longitude.
   * @param  ll  Lat,lng position of the node.
   */
  void NodeInfo::SetLatLng(const PointLL& ll) {
    latlng_ = ll;
  }

  /**
   * Get the latitude, longitude of the node.
   * @return  Returns the latitude and longitude of the node.
   */
  const PointLL& NodeInfo::LatLng() const {
    return latlng_;
  }

  /**
   * Get the GraphId of the first outbound edge from this node.
   * @return  Returns the GraphId of the first outbound edge.
   */
  GraphId NodeInfo::Edge() {
    return edge_;
  }

  /**
   * Get the number of outbound directed edges.
   * @return  Returns the number of outbound directed edges.
   */
  unsigned int NodeInfo::EdgeCount() {
    return nedges_;
  }
}
}
