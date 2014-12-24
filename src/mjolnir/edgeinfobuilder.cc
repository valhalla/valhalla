#include <algorithm>
#include <ostream>
#include <iostream>

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
void EdgeInfoBuilder::set_street_name_offset_list(
    const std::vector<size_t>& street_name_offset_list) {
  // TODO - move
  street_name_offset_list_.clear();
  if (!street_name_offset_list.empty()) {
    street_name_offset_list_.insert(street_name_offset_list_.end(),
                                    street_name_offset_list.begin(),
                                    street_name_offset_list.end());
  }
}

std::size_t EdgeInfoBuilder::SizeOf() const {
  std::size_t size = 0;
  size += sizeof(GraphId);                                      // nodea_
  size += sizeof(GraphId);                                      // nodeb_
  size += sizeof(PackedItem);                                   // item_
  size += (street_name_offset_list_.size() * sizeof(size_t));   // street_name_offset_list_
  size += (shape_.size() * sizeof(PointLL));                    // shape_
  size += (exit_signs_.size() * sizeof(ExitSign));              // exit_signs_

  return size;
}

void EdgeInfoBuilder::SerializeToOstream(std::ostream& out) {
  for (size_t name_offset : street_name_offset_list_) {
    std::cout << "name_offset=" << name_offset << std::endl;
  }
  out.write(reinterpret_cast<const char*>(&nodea_), sizeof(GraphId));
  out.write(reinterpret_cast<const char*>(&nodeb_), sizeof(GraphId));
  out.write(reinterpret_cast<const char*>(&item_), sizeof(PackedItem));
  out.write(reinterpret_cast<const char*>(&street_name_offset_list_[0]),
            (street_name_offset_list_.size() * sizeof(size_t)));
  out.write(reinterpret_cast<const char*>(&shape_[0]),
            (shape_.size() * sizeof(PointLL)));
  out.write(reinterpret_cast<const char*>(&exit_signs_[0]),
            (exit_signs_.size() * sizeof(ExitSign)));
}

}
}
