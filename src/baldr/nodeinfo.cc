#include "baldr/nodeinfo.h"

namespace valhalla {
namespace baldr {

NodeInfo::NodeInfo()
    : edge_count_(0) {
  latlng_.Set(0.0f, 0.0f);
}

const PointLL& NodeInfo::latlng() const {
  return latlng_;
}

unsigned int NodeInfo::edge_index() const {
  return edge_index_;
}

unsigned int NodeInfo::edge_count() const {
  return edge_count_;
}

}
}
