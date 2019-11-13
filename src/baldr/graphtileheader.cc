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
  tile_size_ = 0;
}

// Set the version string.
void GraphTileHeader::set_version(const std::string& version) {
  strncpy(version_, version.c_str(), kMaxVersionSize);
  version_[kMaxVersionSize - 1] = 0;
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

// Sets the edge bin offsets
void GraphTileHeader::set_edge_bin_offsets(const uint32_t (&offsets)[kBinCount]) {
  memcpy(bin_offsets_, offsets, sizeof(bin_offsets_));
}

// Get the offsets to the given bin in the 5x5 grid.
std::pair<uint32_t, uint32_t> GraphTileHeader::bin_offset(size_t index) const {
  if (index < kBinCount) {
    return std::make_pair(index == 0 ? 0 : bin_offsets_[index - 1], bin_offsets_[index]);
  }
  throw std::runtime_error("Bin out of bounds");
}

} // namespace baldr
} // namespace valhalla
