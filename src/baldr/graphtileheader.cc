#include <string.h>
#include "baldr/graphtileheader.h"
#include "baldr/nodeinfo.h"
#include "baldr/directededge.h"
#include "baldr/edgeinfo.h"
#include "baldr/datetime.h"
#include "config.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace baldr {

// Default constructor. This is written with GraphTileHeaderBuilder and
// read within GraphTile. Set the version and internal version here so a
// "default" GraphTileHeader can be used to compare to what is read from
// a tile.
GraphTileHeader::GraphTileHeader() {
  memset(this, 0, sizeof(GraphTileHeader));
  strncpy(version_, PACKAGE_VERSION, kMaxVersionSize);
  version_[kMaxVersionSize-1] = 0;
  date_created_ = DateTime::seconds_since_epoch();

}

// Get the GraphId (tileid and level) of this tile.
const GraphId& GraphTileHeader::graphid() const {
  return graphid_;
}

// Get the date created
uint64_t GraphTileHeader::date_created() const {
  return date_created_;
}

// Get the version string.
std::string GraphTileHeader::version() const {
  return version_;
}

// Get the relative road density within this tile.
uint32_t GraphTileHeader::density() const {
  return static_cast<uint32_t>(density_);
}

// Get the relative quality of name assignment for this tile.
uint32_t GraphTileHeader::name_quality() const {
  return static_cast<uint32_t>(name_quality_);
}

// Get the relative quality of speed assignment for this tile.
uint32_t GraphTileHeader::speed_quality() const {
  return static_cast<uint32_t>(speed_quality_);
}

// Get the relative quality of exit signs for this tile.
uint32_t GraphTileHeader::exit_quality() const {
  return static_cast<uint32_t>(exit_quality_);
}

// Get the count of nodes in the tile.
uint32_t GraphTileHeader::nodecount() const {
  return nodecount_;
}

// Get the count of directed edges in the tile.
uint32_t GraphTileHeader::directededgecount() const {
  return directededgecount_;
}

// Gets the number of signs in the tile.
uint32_t GraphTileHeader::signcount() const {
  return signcount_;
}

// Gets the number of transit departures in this tile.
uint32_t GraphTileHeader::departurecount() const {
  return departurecount_;
}

// Gets the number of transit stops in this tile.
uint32_t GraphTileHeader::stopcount() const {
  return stopcount_;
}

// Gets the number of transit routes in this tile.
uint32_t GraphTileHeader::routecount() const {
  return routecount_;
}

// Gets the number of transit transfers in this tile.
uint32_t GraphTileHeader::transfercount() const {
  return transfercount_;
}

// Gets the number of access restrictions in this tile.
uint32_t GraphTileHeader::access_restriction_count() const {
  return access_restriction_count_;
}

// Gets the number of admins in the tile.
uint32_t GraphTileHeader::admincount() const {
  return admincount_;
}

// Get the offset in bytes to the start of the edge information.
uint32_t GraphTileHeader::edgeinfo_offset() const {
  return edgeinfo_offset_;
}

// Get the offset in bytes to the start of the text / names list.
uint32_t GraphTileHeader::textlist_offset() const {
  return textlist_offset_;
}

// Get the offset in bytes to the complex restriction list.
uint32_t GraphTileHeader::complex_restriction_offset() const {
  return complex_restriction_offset_;
}

// Get the offset to the given cell in the 5x5 grid.
std::pair<uint32_t, uint32_t> GraphTileHeader::cell_offset(size_t column, size_t row) const {
  auto i = row * kGridDim + column;
  if(i < kCellCount)
    return std::make_pair(i == 0 ? 0 : cell_offsets_[i], cell_offsets_[i]);
  throw std::runtime_error("Cell out of bounds");
}

}
}
