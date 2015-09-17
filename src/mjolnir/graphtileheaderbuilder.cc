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

// Set the relative road density within this tile.
void GraphTileHeaderBuilder::set_density(const uint32_t density) {
  quality_.density = density;
}

// Set the relative quality of name assignment for this tile.
void GraphTileHeaderBuilder::set_name_quality(const uint32_t name_quality) {
  quality_.name = name_quality;
}

// Set the relative quality of speed assignment for this tile.
void GraphTileHeaderBuilder::set_speed_quality(const uint32_t speed_quality) {
  quality_.speed = speed_quality;
}

// Set the relative quality of exit signs for this tile.
void GraphTileHeaderBuilder::set_exit_quality(const uint32_t exit_quality) {
  quality_.exit = exit_quality;
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

// Sets the number of transit departures in this tile.
void GraphTileHeaderBuilder::set_departurecount(const uint32_t departures) {
  transit1_.departurecount = departures;
}

// Sets the number of transit stops in this tile.
void GraphTileHeaderBuilder::set_stopcount(const uint32_t stops) {
  transit1_.stopcount = stops;
}

// Sets the number of transit routes in this tile.
void GraphTileHeaderBuilder::set_routecount(const uint32_t routes) {
  transit2_.routecount = routes;
}

// Sets the number of transit transfers in this tile.
void GraphTileHeaderBuilder::set_transfercount(const uint32_t transfers) {
  transit2_.transfercount = transfers;
}

// Gets the number of transit calendar exceptions in this tile.
void GraphTileHeaderBuilder::set_calendarcount(const uint32_t calendars) {
  transit2_.calendarcount = calendars;
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

// Sets the offset to the list of Multi-Edge Restrictions.
void GraphTileHeaderBuilder::set_merlist_offset(const uint32_t offset) {
  merlist_offset_ = offset;
}

// Sets the offset to the list of timed restrictions.
void GraphTileHeaderBuilder::set_timedres_offset(const uint32_t offset) {
  timedres_offset_ = offset;
}

}
}
