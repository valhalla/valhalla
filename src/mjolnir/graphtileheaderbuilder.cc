#include <string.h>
#include "mjolnir/graphtileheaderbuilder.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Constructor
GraphTileHeaderBuilder::GraphTileHeaderBuilder()
    : GraphTileHeader() {
}

// Set the graph Id of this tile.
void GraphTileHeaderBuilder::set_graphid(const baldr::GraphId& graphid) {
  graphid_ = graphid;
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

// Set the relative road density within this tile.
void GraphTileHeaderBuilder::set_density(const uint32_t density) {
  density_ = density;
}

// Set the relative quality of name assignment for this tile.
void GraphTileHeaderBuilder::set_name_quality(const uint32_t name_quality) {
  name_quality_ = name_quality;
}

// Set the relative quality of speed assignment for this tile.
void GraphTileHeaderBuilder::set_speed_quality(const uint32_t speed_quality) {
  speed_quality_ = speed_quality;
}

// Set the relative quality of exit signs for this tile.
void GraphTileHeaderBuilder::set_exit_quality(const uint32_t exit_quality) {
  exit_quality_ = exit_quality;
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

// Sets the number of transit departures in this tile.
void GraphTileHeaderBuilder::set_departurecount(const uint32_t departures) {
  departurecount_ = departures;
}

// Sets the number of transit stops in this tile.
void GraphTileHeaderBuilder::set_stopcount(const uint32_t stops) {
  stopcount_ = stops;
}

// Sets the number of transit routes in this tile.
void GraphTileHeaderBuilder::set_routecount(const uint32_t routes) {
  routecount_ = routes;
}

// Sets the number of transit transfers in this tile.
void GraphTileHeaderBuilder::set_transfercount(const uint32_t transfers) {
  transfercount_ = transfers;
}

// Sets the number of restrictions in this tile.
void GraphTileHeaderBuilder::set_access_restriction_count(const uint32_t n) {
  access_restriction_count_ = n;
}

// Sets the admin count.
void GraphTileHeaderBuilder::set_admincount(const uint32_t count) {
  admincount_ = count;
}

// Sets the offset to the edge info.
void GraphTileHeaderBuilder::set_edgeinfo_offset(const uint32_t offset) {
  edgeinfo_offset_ = offset;
}

// Sets the offset to the text list.
void GraphTileHeaderBuilder::set_textlist_offset(const uint32_t offset) {
  textlist_offset_ = offset;
}

// Sets the offset to the list of complex restrictions.
void GraphTileHeaderBuilder::set_complex_restriction_offset(const uint32_t offset) {
  complex_restriction_offset_ = offset;
}

// Sets the edge bin offsets
void GraphTileHeaderBuilder::set_edge_cell_offsets(const uint32_t (&offsets)[kCellCount + 1]) {
  memcpy(cell_offsets_, offsets, sizeof(cell_offsets_));
}

}
}
