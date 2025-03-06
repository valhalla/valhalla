#pragma once

#include <array>
#include <cstdint>
#include <future>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "baldr/graphid.h"
#include "baldr/predictedspeeds.h"
#include "filesystem.h"

namespace valhalla {
namespace mjolnir {

// Struct to hold stats information during each threads work
struct TrafficStats {
  uint32_t constrained_count = 0;
  uint32_t free_flow_count = 0;
  uint32_t compressed_count = 0;
  uint32_t updated_count = 0;
  uint32_t dup_count = 0;

  // Accumulate counts from all threads
  void operator()(const TrafficStats& other);
};

struct TrafficSpeeds {
  uint8_t constrained_flow_speed = 0;
  uint8_t free_flow_speed = 0;
  std::optional<std::array<int16_t, valhalla::baldr::kCoefficientCount>> coefficients;
};

// Functions declarations
std::unordered_map<uint32_t, TrafficSpeeds>
ParseTrafficFile(const std::vector<std::string>& filenames, TrafficStats& stat);

void UpdateTile(const std::string& tile_dir,
                const valhalla::baldr::GraphId& tile_id,
                const std::unordered_map<uint32_t, TrafficSpeeds>& speeds,
                TrafficStats& stat);

void UpdateTiles(
    const std::string& tile_dir,
    std::vector<std::pair<valhalla::baldr::GraphId, std::vector<std::string>>>::const_iterator
        tile_start,
    std::vector<std::pair<valhalla::baldr::GraphId, std::vector<std::string>>>::const_iterator
        tile_end,
    std::promise<TrafficStats>& result);

std::vector<std::pair<valhalla::baldr::GraphId, std::vector<std::string>>>
PrepareTrafficTiles(const filesystem::path& traffic_tile_dir);

TrafficStats ProcessTrafficTiles(
    const std::string& tile_dir,
    const std::vector<std::pair<valhalla::baldr::GraphId, std::vector<std::string>>>& traffic_tiles,
    uint32_t concurrency);

void LogProcessingResults(const TrafficStats& final_stats);

void GenerateSummary(const boost::property_tree::ptree& config);

} // namespace mjolnir
} // namespace valhalla
