// -*- mode: c++ -*-
#pragma once

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/traffictile.h"
#include "mjolnir/graphtilebuilder.h"

#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace test {

// Return a random number inside [0, 1)
inline float rand01(std::mt19937& gen) {
  std::uniform_real_distribution<float> dis(0, 1);
  return dis(gen);
}

std::string load_binary_file(const std::string& filename);

MATCHER_P2(IsBetween,
           a,
           b,
           std::string(negation ? "isn't" : "is") + " between " + ::testing::PrintToString(a) +
               " and " + ::testing::PrintToString(b)) {
  return a <= arg && arg <= b;
}

template <typename pbf_message_t> bool pbf_equals(const pbf_message_t& a, const pbf_message_t& b) {
  return a.SerializeAsString() == b.SerializeAsString();
}

boost::property_tree::ptree json_to_pt(const std::string& json);

/**
 * Generate a new GraphReader that doesn't re-use a previously
 * statically initizalized tile_extract member variable.
 *
 * Useful if you need to reload a tile extract within the same
 * process
 */
std::shared_ptr<valhalla::baldr::GraphReader>
make_clean_graphreader(const boost::property_tree::ptree& mjolnir_conf);

/*************************************************************/
// Creates an empty traffic file
//
// To actually customize the traffic data, use `customize_live_traffic_data`
//
/*************************************************************/
void build_live_traffic_data(const boost::property_tree::ptree& config,
                             uint32_t traffic_tile_version = valhalla::baldr::TRAFFIC_TILE_VERSION);

/*************************************************************/
// Helper function for customizing traffic data in unit-tests
//
// `setter_cb` is a callback that can modify traffic for each edge
// when building traffic data
/*************************************************************/
using LiveTrafficCustomize = std::function<void(valhalla::baldr::GraphReader&,
                                                valhalla::baldr::TrafficTile&,
                                                int,
                                                valhalla::baldr::TrafficSpeed*)>;

void customize_live_traffic_data(const boost::property_tree::ptree& config,
                                 const LiveTrafficCustomize& setter_cb);

using HistoricalTrafficCustomize = std::function<std::vector<int16_t>(DirectedEdge&)>;
void customize_historical_traffic(const boost::property_tree::ptree& config,
                                  const HistoricalTrafficCustomize& cb);

} // namespace test
} // namespace valhalla
