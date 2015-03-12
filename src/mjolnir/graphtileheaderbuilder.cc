#include <string.h>
#include "mjolnir/graphtileheaderbuilder.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Constructor
GraphTileHeaderBuilder::GraphTileHeaderBuilder()
    : GraphTileHeader() {
}

// Set the internal version
void GraphTileHeaderBuilder::set_internal_version(const int64_t version) {
  internal_version_ = version;
}

// Set the date created
void GraphTileHeaderBuilder::set_date_created(const uint64_t date) {
  date_created_ = date;
}

// Set the version string.
void GraphTileHeaderBuilder::set_version(const std::string& version) {
  strncpy(version_, version.c_str(), kMaxVersionSize);
  version_[kMaxVersionSize-1] = 0;
}

// Set the graph Id of this tile.
void GraphTileHeaderBuilder::set_graphid(const baldr::GraphId& graphid) {
  graphid_ = graphid;
}

// Sets the number of nodes in this tile.
void GraphTileHeaderBuilder::set_nodecount(const uint32_t count) {
  nodecount_ = count;
}

// Sets the number of directed edges in this tile.
void GraphTileHeaderBuilder::set_directededgecount(const uint32_t count) {
  directededgecount_ = count;
}

// Sets the sign count.
void GraphTileHeaderBuilder::set_signcount(const uint32_t count) {
  signcount_ = count;
}

// Sets the offset to the edge info.
void GraphTileHeaderBuilder::set_edgeinfo_offset(const uint32_t offset) {
  edgeinfo_offset_ = offset;
}

// Sets the offset to the street list.
void GraphTileHeaderBuilder::set_streetlist_offset(const uint32_t offset) {
  streetlist_offset_ = offset;
}

// Sets the offset to the admin info.
void GraphTileHeaderBuilder::set_admininfo_offset(const uint32_t offset) {
  admininfo_offset_ = offset;
}

// Sets the offset to the street list.
void GraphTileHeaderBuilder::set_namelist_offset(const uint32_t offset) {
  namelist_offset_ = offset;
}

// Sets the offset to the list of Multi-Edge Restrictions.
void GraphTileHeaderBuilder::set_merlist_offset(const uint32_t offset) {
  merlist_offset_ = offset;
}

// Sets the offset to the list of timed restrictions.
void GraphTileHeaderBuilder::set_timedres_offset(const uint32_t offset) {
  timedres_offset_ = offset;
}

 // Sets the offset to the list of transit departures / schedule.
void GraphTileHeaderBuilder::set_transit_offset(const uint32_t offset) {
  transit_offset_ = offset;
}

}
}
