#include "baldr/graphtileheader.h"
#include "baldr/datetime.h"
#include "baldr/directededge.h"
#include "baldr/edgeinfo.h"
#include "baldr/nodeinfo.h"
#include "config.h"
#include <string.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace baldr {

// Default constructor.
GraphTileHeader::GraphTileHeader() {
  memset(this, 0, sizeof(GraphTileHeader));
  strncpy(version_, PACKAGE_VERSION, kMaxVersionSize);
  version_[kMaxVersionSize - 1] = 0;
  empty_slots_[0] = 0;
}

// Set the graph Id of this tile.
void GraphTileHeader::set_graphid(const baldr::GraphId& graphid) {
  graphid_ = graphid;
}

// Set the date created
void GraphTileHeader::set_date_created(const uint32_t date) {
  date_created_ = date;
}

// Set the version string.
void GraphTileHeader::set_version(const std::string& version) {
  strncpy(version_, version.c_str(), kMaxVersionSize);
  version_[kMaxVersionSize - 1] = 0;
}

// Set the data set Id (latest OSM changeset Id).
void GraphTileHeader::set_dataset_id(const uint64_t id) {
  dataset_id_ = id;
}

// Set the relative road density within this tile.
void GraphTileHeader::set_density(const uint32_t density) {
  density_ = (density <= kMaxDensity) ? density : kMaxDensity;
}

// Set the relative quality of name assignment for this tile.
void GraphTileHeader::set_name_quality(const uint32_t name_quality) {
  name_quality_ = name_quality;
}

// Set the relative quality of speed assignment for this tile.
void GraphTileHeader::set_speed_quality(const uint32_t speed_quality) {
  speed_quality_ = speed_quality;
}

// Set the relative quality of exit signs for this tile.
void GraphTileHeader::set_exit_quality(const uint32_t exit_quality) {
  exit_quality_ = exit_quality;
}

// Sets the number of nodes in this tile.
void GraphTileHeader::set_nodecount(const uint32_t count) {
  nodecount_ = count;
}

// Sets the number of directed edges in this tile.
void GraphTileHeader::set_directededgecount(const uint32_t count) {
  directededgecount_ = count;
}

// Sets the sign count.
void GraphTileHeader::set_signcount(const uint32_t count) {
  signcount_ = count;
}

// Sets the number of transit departures in this tile.
void GraphTileHeader::set_departurecount(const uint32_t departures) {
  // Check against limit
  if (departures > kMaxTransitDepartures) {
    throw std::runtime_error("Exceeding maximum number of transit departures per tile");
  }
  departurecount_ = departures;
}

// Sets the number of transit stops in this tile.
void GraphTileHeader::set_stopcount(const uint32_t stops) {
  // Check against limit
  if (stops > kMaxTransitStops) {
    throw std::runtime_error("Exceeding maximum number of transit stops per tile");
  }
  stopcount_ = stops;
}

// Sets the number of transit routes in this tile.
void GraphTileHeader::set_routecount(const uint32_t routes) {
  // Check against limit
  if (routes > kMaxTransitRoutes) {
    throw std::runtime_error("Exceeding maximum number of transit routes per tile");
  }
  routecount_ = routes;
}

// Sets the number of transit schedules in this tile.
void GraphTileHeader::set_schedulecount(const uint32_t schedules) {
  // Check against limit
  if (schedules > kMaxTransitSchedules) {
    throw std::runtime_error("Exceeding maximum number of transit schedule entries per tile");
  }
  schedulecount_ = schedules;
}

// Sets the number of transit transfers in this tile.
void GraphTileHeader::set_transfercount(const uint32_t transfers) {
  // Check against limit
  if (transfers > kMaxTransfers) {
    throw std::runtime_error("Exceeding maximum number of transit transfer entries per tile");
  }
  transfercount_ = transfers;
}

// Sets the number of restrictions in this tile.
void GraphTileHeader::set_access_restriction_count(const uint32_t n) {
  access_restriction_count_ = n;
}

// Sets the admin count.
void GraphTileHeader::set_admincount(const uint32_t count) {
  admincount_ = count;
}

// Sets the offset to the cr in the forward direction.
void GraphTileHeader::set_complex_restriction_forward_offset(const uint32_t offset) {
  complex_restriction_forward_offset_ = offset;
}

// Sets the offset to the cr in the reverse direction.
void GraphTileHeader::set_complex_restriction_reverse_offset(const uint32_t offset) {
  complex_restriction_reverse_offset_ = offset;
}

// Sets the offset to the edge info.
void GraphTileHeader::set_edgeinfo_offset(const uint32_t offset) {
  edgeinfo_offset_ = offset;
}

// Sets the offset to the text list.
void GraphTileHeader::set_textlist_offset(const uint32_t offset) {
  textlist_offset_ = offset;
}

// Sets the edge bin offsets
void GraphTileHeader::set_edge_bin_offsets(const uint32_t (&offsets)[kBinCount]) {
  memcpy(bin_offsets_, offsets, sizeof(bin_offsets_));
}

// Get the offsets to the given bin in the 5x5 grid.
std::pair<uint32_t, uint32_t> GraphTileHeader::bin_offset(size_t column, size_t row) const {
  return bin_offset(row * kBinsDim + column);
}

// Get the offsets to the given bin in the 5x5 grid.
std::pair<uint32_t, uint32_t> GraphTileHeader::bin_offset(size_t index) const {
  if (index < kBinCount) {
    return std::make_pair(index == 0 ? 0 : bin_offsets_[index - 1], bin_offsets_[index]);
  }
  throw std::runtime_error("Bin out of bounds");
}

// Sets the number of traffic segment Ids in this tile.
void GraphTileHeader::set_traffic_id_count(const uint32_t count) {
  traffic_id_count_ = count;
}

// Sets the offset to the traffic segment Ids.
void GraphTileHeader::set_traffic_segmentid_offset(const uint32_t offset) {
  traffic_segmentid_offset_ = offset;
}

// Sets the offset to the traffic chunks.
void GraphTileHeader::set_traffic_chunk_offset(const uint32_t offset) {
  traffic_chunk_offset_ = offset;
}

// Sets the offset to the lane connectivity data.
void GraphTileHeader::set_lane_connectivity_offset(const uint32_t offset) {
  lane_connectivity_offset_ = offset;
}

// Sets the offset to the edge elevation data.
void GraphTileHeader::set_edge_elevation_offset(const uint32_t offset) {
  edge_elevation_offset_ = offset;
}

// Sets the offset to the predicted traffic data.
void GraphTileHeader::set_predicted_traffic_offset(const uint32_t offset) {
  predicted_traffic_offset_ = offset;
}

// Gets the offset to the end of the tile.
uint32_t GraphTileHeader::end_offset() const {
  return empty_slots_[0];
}

// Sets the offset to the end of the tile. Fills all empty slots with the
// offset value. This allows compatibility with different tile versions.
void GraphTileHeader::set_end_offset(uint32_t offset) {
  for (size_t i = 0; i < kEmptySlots; i++) {
    empty_slots_[i] = offset;
  }
}

} // namespace baldr
} // namespace valhalla
