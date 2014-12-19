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
void EdgeInfoBuilder::set_name_indexes(
    const std::vector<uint32_t>& nameindexes) {
  name_indexes_.clear();
  if (!nameindexes.empty()) {
    name_indexes_.insert(name_indexes_.end(), nameindexes.begin(),
                        nameindexes.end());
  }
}

std::size_t EdgeInfoBuilder::SizeOf() const {
  std::size_t size = 0;
  size += sizeof(GraphId);                           // nodea_
  size += sizeof(GraphId);                           // nodeb_
  size += sizeof(std::size_t);                       // shape_ size
  size += (shape_.size() * sizeof(PointLL));         // shape_
  size += sizeof(std::size_t);                       // nameindexes_ size
  size += (name_indexes_.size() * sizeof(uint32_t));  // nameindexes_

  return size;
}

}
}
