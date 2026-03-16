// -*- mode: c++ -*-
#pragma once

#include "baldr/directededge.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/predictedspeeds.h"
#include "baldr/traffictile.h"
#include "midgard/encoded.h"
#include "midgard/pointll.h"
#include "midgard/polyline2.h"

#include <boost/property_tree/ptree_fwd.hpp>
#include <gmock/gmock-matchers.h>
#include <gtest/gtest-assertion-result.h>

#include <cmath>
#include <functional>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#ifndef _MSC_VER
#include <sys/mman.h>
#endif

#include <sys/stat.h>

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

boost::property_tree::ptree
make_config(const std::string& path_prefix,
            const std::unordered_map<std::string, std::string>& overrides = {},
            const std::unordered_set<std::string>& removes = {});

template <typename container_t>
testing::AssertionResult shape_equality(const container_t& expected,
                                        const container_t& actual,
                                        typename container_t::value_type::first_type tolerance = 1) {
  auto hd =
      valhalla::midgard::Polyline2<typename container_t::value_type>::HausdorffDistance(expected,
                                                                                        actual);
  if (hd > tolerance)
    return testing::AssertionFailure() << "shape exceeds tolerance by " << hd - tolerance;
  return testing::AssertionSuccess();
}

inline testing::AssertionResult
encoded_shape_equality(const std::string& expected, const std::string& actual, double tolerance = 1) {
  auto expected_shp = valhalla::midgard::decode<std::vector<valhalla::midgard::PointLL>>(expected);
  auto actual_shp = valhalla::midgard::decode<std::vector<valhalla::midgard::PointLL>>(actual);
  return shape_equality(expected_shp, actual_shp, tolerance);
}

/**
 * Deep compares rapidjson::Value, descending into arrays & objects.
 *
 * FAIL()s when the both args aren't equal.
 *
 * @param j1 the actual JSON
 * @param j2 the expected JSON
 */
void json_equality(const rapidjson::Value& j1, const rapidjson::Value& j2);

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

#ifdef DATA_TOOLS
using HistoricalTrafficCustomize =
    std::function<std::optional<std::array<float, valhalla::baldr::kBucketsPerWeek>>(
        valhalla::baldr::DirectedEdge&)>;
void customize_historical_traffic(const boost::property_tree::ptree& config,
                                  const HistoricalTrafficCustomize& cb);

using EdgesCustomize =
    std::function<void(const valhalla::baldr::GraphId&, valhalla::baldr::DirectedEdge&)>;
void customize_edges(const boost::property_tree::ptree& config, const EdgesCustomize& setter_cb);
#endif

} // namespace test
