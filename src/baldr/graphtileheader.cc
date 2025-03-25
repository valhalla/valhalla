#include "baldr/graphtileheader.h"
#include "baldr/datetime.h"
#include "baldr/directededge.h"
#include "baldr/edgeinfo.h"
#include "baldr/nodeinfo.h"
#include "config.h"
#include <algorithm>
#include <string.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace baldr {

// Default constructor.
GraphTileHeader::GraphTileHeader()
    : // initialization of bitfields done here in c++20 can be done in the class definition
      graphid_(0), density_(0), name_quality_(0), speed_quality_(0), exit_quality_(0),
      has_elevation_(0), has_ext_directededge_(0), nodecount_(0), directededgecount_(0),
      predictedspeeds_count_(0), spare1_(0), transitioncount_(0), spare3_(0), turnlane_count_(0),
      spare4_(0), transfercount_(0), spare2_(0), departurecount_(0), stopcount_(0), spare5_(0),
      routecount_(0), schedulecount_(0), signcount_(0), spare6_(0), access_restriction_count_(0),
      admincount_(0), spare7_(0) {
  set_version(PACKAGE_VERSION);
}

// Set the version string.
void GraphTileHeader::set_version(const std::string& version) {
  // reinitializing the version array before copying
  version_ = {};
  std::copy(version.begin(), version.begin() + std::min(kMaxVersionSize, version.size()),
            version_.data());
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
  std::copy(std::begin(offsets), std::end(offsets), bin_offsets_.data());
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
