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

// Set the shape of the edge.
void EdgeInfoBuilder::set_shape(const std::vector<PointLL>& shape) {
  shape_.clear();
  std::copy(shape.begin(), shape.end(), shape_.begin());
}

// Set the indexes to names used by this edge
void EdgeInfoBuilder::set_nameindexes(
    const std::vector<std::string>& nameindexes) {
  nameindexes_.clear();
  std::copy(nameindexes.begin(), nameindexes.end(), nameindexes_.begin());
}

}
}
