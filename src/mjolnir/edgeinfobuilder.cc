#include <algorithm>

#include "mjolnir/edgeinfobuilder.h"

namespace valhalla {
namespace mjolnir {

// Constructor
EdgeInfoBuilder::EdgeInfoBuilder() {
}

// Set the reference node (start) of the edge.
void EdgeInfoBuilder::set_nodea(const baldr::GraphId& nodea) {
  nodea_ = nodea;
}

// Set the end node of the edge.
void EdgeInfoBuilder::set_nodeb(const baldr::GraphId& nodeb) {
  nodeb_ = nodeb;
}

// Set the shape of the edge. TODO - move?
void EdgeInfoBuilder::set_shape(const std::vector<PointLL>& shape) {
  shape_.clear();
  if (!shape.empty()) {
    shape_.insert(shape_.end(), shape.begin(), shape.end());
  }
}

// Set the indexes to names used by this edge. TODO - move?
void EdgeInfoBuilder::set_nameindexes(
    const std::vector<uint32_t>& nameindexes) {
  nameindexes_.clear();
  if (!nameindexes.empty()) {
    nameindexes_.insert(nameindexes_.end(), nameindexes.begin(),
                        nameindexes.end());
  }
}

}
}
