#include <algorithm>
#include <ostream>
#include <iostream>

#include "mjolnir/edgeinfobuilder.h"

namespace valhalla {
namespace mjolnir {

// Constructor
EdgeInfoBuilder::EdgeInfoBuilder() {
  item_ = new PackedItem();
}

EdgeInfoBuilder::EdgeInfoBuilder(const EdgeInfoBuilder& other) {
  item_ = new PackedItem(*(other.item_));
  street_name_offset_list_ = other.street_name_offset_list_;
  shape_ = other.shape_;
  exit_signs_ = other.exit_signs_;
}

EdgeInfoBuilder& EdgeInfoBuilder::operator=(const EdgeInfoBuilder& rhs) {
  if (&rhs != this) {
    item_ = new PackedItem(*(rhs.item_));
    street_name_offset_list_ = rhs.street_name_offset_list_;
    shape_ = rhs.shape_;
    exit_signs_ = rhs.exit_signs_;
  }
  return *this;
}

EdgeInfoBuilder::~EdgeInfoBuilder() {
  if (item_) {
    delete item_;
    item_ = nullptr;
  }
}

// Set the indexes to names used by this edge.
void EdgeInfoBuilder::set_street_name_offset_list(
    const std::vector<size_t>& street_name_offset_list) {
  street_name_offset_list_ = street_name_offset_list;

  // Set the street name offset list offset
  item_->fields.street_name_offset_list_offset = sizeof(PackedItem);

  // Set the name count
  item_->fields.name_count = street_name_offset_list_.size();
}

// Set the shape of the edge. TODO - move?
void EdgeInfoBuilder::set_shape(const std::vector<PointLL>& shape) {
  // Set the shape
  shape_.assign(shape.begin(), shape.end());

  // Set the shape count
  item_->fields.shape_count = shape_.size();
}

const size_t EdgeInfoBuilder::GetStreetNameOffset(uint8_t index) const {
  return street_name_offset_list_[index];
}

const PointLL EdgeInfoBuilder::GetShapePoint(uint16_t index) const {
  return shape_[index];
}

std::size_t EdgeInfoBuilder::SizeOf() const {
  std::size_t size = 0;
  size += sizeof(PackedItem);                                  // item_
  size += (street_name_offset_list_.size() * sizeof(size_t));  // street_name_offset_list_
  size += (shape_.size() * sizeof(PointLL));                   // shape_
  size += (exit_signs_.size() * sizeof(ExitSign));             // exit_signs_

  return size;
}

void EdgeInfoBuilder::SerializeToOstream(std::ostream& out) const {
  // TODO - rm later
  /*std::cout << "------------------------------------------------------"
   << std::endl;
   std::cout << __FILE__ << ":" << __LINE__ << std::endl;
   std::cout << "EdgeInfoBuilder::SizeOf=" << SizeOf() << std::endl;
   std::cout << "item_=" << item_->value << std::endl;
   std::cout << "street_name_offset_list_offset="
   << street_name_offset_list_offset() << "  name_count="
   << street_name_offset_list_.size() << std::endl;
   for (auto name_offset : street_name_offset_list_) {
   std::cout << "   name_offset=" << name_offset << std::endl;
   }
   std::cout << "shape_count=" << shape_.size() << std::endl;
   for (const auto& ll : shape_) {
   std::cout << "   ll=" << ll.lat() << "," << ll.lng() << std::endl;
   }
   std::cout << "exit_sign_count=" << exit_signs_.size() << std::endl;
   std::cout << "======================================================="
   << std::endl;*/

  out.write(reinterpret_cast<const char*>(item_), sizeof(PackedItem));
  out.write(reinterpret_cast<const char*>(&street_name_offset_list_[0]),
            (street_name_offset_list_.size() * sizeof(size_t)));
  out.write(reinterpret_cast<const char*>(&shape_[0]),
            (shape_.size() * sizeof(PointLL)));
  out.write(reinterpret_cast<const char*>(&exit_signs_[0]),
            (exit_signs_.size() * sizeof(ExitSign)));
}

}
}
