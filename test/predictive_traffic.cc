#include "test.h"

#include <iostream>
#include <string>
#include <vector>

#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/nodeinfo.h"
#include "midgard/util.h"
#include "mjolnir/graphtilebuilder.h"

#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>

using namespace valhalla::baldr;
using namespace valhalla::mjolnir;
using namespace valhalla::midgard;

namespace {

boost::property_tree::ptree json_to_pt(const std::string& json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  rapidjson::read_json(ss, pt);
  return pt;
}

const auto config = json_to_pt(R"({
    "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1}
  })");

} // namespace

void test_predictive_traffic() {
  boost::property_tree::ptree hierarchy_properties = config.get_child("mjolnir");
  GraphReader reader(hierarchy_properties);

  uint32_t count = 0;
  auto level = TileHierarchy::levels().rbegin();
  for (; level != TileHierarchy::levels().rend(); ++level) {
    // Create a randomized queue of tiles to work from
    auto tile_level = level->second;
    auto level_tiles = reader.GetTileSet(tile_level.level);
    for (const auto& tile_id : level_tiles) {
      const GraphTile* tile = reader.GetGraphTile(tile_id);
      uint32_t nodecount = tile->header()->nodecount();
      GraphId node = tile_id;
      for (uint32_t i = 0; i < nodecount; i++, ++node) {
        // The node we will modify
        auto ni = tile->node(i);
        uint32_t idx = ni->edge_index();
        GraphId edgeid(node.tileid(), node.level(), idx);
        for (uint32_t j = 0, n = ni->edge_count(); j < n; j++, idx++, ++edgeid) {
          auto de = tile->directededge(idx);

          if (edgeid.value == 67134432) {
            if (de->free_flow_speed() != 5)
              throw std::runtime_error("Free flow speed should be 5 for edge 67134432, but it is: " +
                                       std::to_string(de->free_flow_speed()));
            if (de->constrained_flow_speed() != 6)
              throw std::runtime_error(
                  "Constrained flow speed should be 6 for edge 67134432, but it is: " +
                  std::to_string(de->constrained_flow_speed()));
          } else if (edgeid.value == 264348364578) {
            if (de->free_flow_speed() != 13)
              throw std::runtime_error(
                  "Free fow speed should be 13 for edge 264348364578, but it is: " +
                  std::to_string(de->free_flow_speed()));
            if (de->constrained_flow_speed() != 9)
              throw std::runtime_error(
                  "Constrained flow speed should be 9 for edge 264348364578, but it is: " +
                  std::to_string(de->constrained_flow_speed()));
          }
          if (de->constrained_flow_speed() || de->free_flow_speed())
            count++;
        }
      }
    }
  }
  if (count != 23751)
    throw std::runtime_error("Incorrect number of edges updated.  Count: " + std::to_string(count));
}

void test_get_speed() {
  GraphReader reader(config.get_child("mjolnir"));

  // fixture traffic tile 0/003/196.gph
  GraphId id("0/3196/0");
  auto tile = reader.GetGraphTile(id);
  auto de = tile->directededge(id);

  if (!de->has_predicted_speed())
    throw std::runtime_error("No predicted speed on edge " + std::to_string(id));

  if (!de->constrained_flow_speed())
    throw std::runtime_error("No constrained speed on edge " + std::to_string(id));

  if (!de->free_flow_speed())
    throw std::runtime_error("No freeflow speed on edge " + std::to_string(id));

  // Test cases where constrained flow speed is returned
  auto expected = 35;
  std::vector<uint32_t> actuals = {tile->GetSpeed(de), tile->GetSpeed(de, kConstrainedFlowMask),
                                   tile->GetSpeed(de, kConstrainedFlowMask, 25201)};
  for (auto actual : actuals) {
    if (actual != expected) {
      throw std::runtime_error("Expected constrained speed | " + std::to_string(actual) +
                               " != " + std::to_string(expected));
    }
  }

  // Test cases where free flow speed is returned
  expected = 45;
  actuals = {tile->GetSpeed(de, kFreeFlowMask), tile->GetSpeed(de, kFreeFlowMask, kSecondsPerDay),
             tile->GetSpeed(de, kFreeFlowMask, 0)};
  for (auto actual : actuals) {
    if (actual != expected) {
      throw std::runtime_error("Expected freeflow speed | " + std::to_string(actual) +
                               " != " + std::to_string(expected));
    }
  }

  // Test cases where predicted flow speed is returned
  expected = 23;
  auto actual = tile->GetSpeed(de, kPredictedFlowMask, kConstrainedFlowSecondOfDay);
  if (actual != expected) {
    throw std::runtime_error("Expected predicted speed | " + std::to_string(actual) +
                             " != " + std::to_string(expected));
  }

  // Test cases where we has for a time of day that is huge
  expected = 23;
  actual = tile->GetSpeed(de, kPredictedFlowMask, kConstrainedFlowSecondOfDay + kSecondsPerWeek);
  if (actual != expected) {
    throw std::runtime_error("Expected predicted speed | " + std::to_string(actual) +
                             " != " + std::to_string(expected));
  }

  // Test flow_sources
  uint8_t flow_sources;
  tile->GetSpeed(de, kPredictedFlowMask, kConstrainedFlowSecondOfDay, &flow_sources);
  if (!(flow_sources & kPredictedFlowMask)) {
    throw std::runtime_error("Expected flow_sources to include predicted");
  }

  tile->GetSpeed(de, kConstrainedFlowMask, kConstrainedFlowSecondOfDay, &flow_sources);
  if (flow_sources & kPredictedFlowMask) {
    throw std::runtime_error("Expected flow_sources not to include predicted");
  }

  tile->GetSpeed(de, kNoFlowMask, kConstrainedFlowSecondOfDay, &flow_sources);
  if (flow_sources & kPredictedFlowMask) {
    throw std::runtime_error("Expected flow_sources not to include predicted");
  }
}

int main(int argc, char* argv[]) {
  test::suite suite("predictive_traffic");
  // TODO - add this back in when updated data is available!
  //  suite.test(TEST_CASE(test_predictive_traffic));

  suite.test(TEST_CASE(test_get_speed));

  return suite.tear_down();
}
