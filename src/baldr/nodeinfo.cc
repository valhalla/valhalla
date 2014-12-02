#include "baldr/nodeinfo.h"

namespace valhalla{
namespace baldr{
  NodeInfo::NodeInfo():nedges_(0) {
    latlng_.Set(0.0f, 0.0f);
  }

  void NodeInfo::SetLatLng(const PointLL& ll) {
    latlng_ = ll;
  }

  const PointLL& NodeInfo::LatLng() const {
    return latlng_;
  }

  GraphId NodeInfo::Edge() {
    return edge_;
  }

  unsigned int NodeInfo::EdgeCount() {
    return nedges_;
  }
}
}
