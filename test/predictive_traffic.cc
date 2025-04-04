#include "test.h"

#include <string>
#include <vector>

#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/nodeinfo.h"
#include "midgard/util.h"
#include "mjolnir/graphtilebuilder.h"

using namespace valhalla::baldr;
using namespace valhalla::mjolnir;
using namespace valhalla::midgard;

namespace {

const auto config = test::json_to_pt(R"({
    "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1}
  })");

} // namespace

// TODO - add this back in when updated data is available!
TEST(PredictiveTraffic, DISABLED_test_predictive_traffic) {
  boost::property_tree::ptree hierarchy_properties = config.get_child("mjolnir");
  GraphReader reader(hierarchy_properties);

  uint32_t count = 0;
  auto level = TileHierarchy::levels().rbegin();
  for (; level != TileHierarchy::levels().rend(); ++level) {
    // Create a randomized queue of tiles to work from
    auto tile_level = *level;
    auto level_tiles = reader.GetTileSet(tile_level.level);
    for (const auto& tile_id : level_tiles) {
      graph_tile_ptr tile = reader.GetGraphTile(tile_id);
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
            EXPECT_EQ(de->free_flow_speed(), 5) << "for edge 67134432";
            EXPECT_EQ(de->constrained_flow_speed(), 6) << "for edge 67134432";
          } else if (edgeid.value == 264348364578) {
            EXPECT_EQ(de->free_flow_speed(), 13) << "for edge 264348364578";
            EXPECT_EQ(de->constrained_flow_speed(), 9) << "for edge 264348364578";
          }
          if (de->constrained_flow_speed() || de->free_flow_speed())
            count++;
        }
      }
    }
  }
  EXPECT_EQ(count, 23751) << "Incorrect number of edges updated";
}

TEST(PredictiveTraffic, test_get_speed) {
  GraphReader reader(config.get_child("mjolnir"));

  // fixture traffic tile 0/003/196.gph
  GraphId id("0/3196/0");
  auto tile = reader.GetGraphTile(id);
  auto de = tile->directededge(id);

  EXPECT_TRUE(de->has_predicted_speed()) << "No predicted speed on edge " + std::to_string(id);

  EXPECT_TRUE(de->constrained_flow_speed()) << "No constrained speed on edge " + std::to_string(id);

  EXPECT_TRUE(de->free_flow_speed()) << "No freeflow speed on edge " + std::to_string(id);

  // Test cases where constrained flow speed is returned
  auto expected = 35;
  std::vector<uint32_t> actuals = {tile->GetSpeed(de), tile->GetSpeed(de, kConstrainedFlowMask),
                                   tile->GetSpeed(de, kConstrainedFlowMask, 25201)};
  for (auto actual : actuals) {
    EXPECT_EQ(actual, expected) << "constrained speeds not equal";
  }

  // Test cases where free flow speed is returned
  expected = 45;
  actuals = {tile->GetSpeed(de, kFreeFlowMask), tile->GetSpeed(de, kFreeFlowMask, kSecondsPerDay),
             tile->GetSpeed(de, kFreeFlowMask, 0)};
  for (auto actual : actuals) {
    EXPECT_EQ(actual, expected) << "freeflow speeds not equal";
  }

  // Test cases where predicted flow speed is returned
  expected = 23;
  auto actual = tile->GetSpeed(de, kPredictedFlowMask, kConstrainedFlowSecondOfDay);
  EXPECT_EQ(actual, expected) << "predicted speed incorrect";

  // Test cases where we has for a time of day that is huge
  expected = 23;
  actual = tile->GetSpeed(de, kPredictedFlowMask, kConstrainedFlowSecondOfDay + kSecondsPerWeek);
  EXPECT_EQ(actual, expected) << "predicted speed incorrect";

  // Test flow_sources
  uint8_t flow_sources;
  tile->GetSpeed(de, kPredictedFlowMask, kConstrainedFlowSecondOfDay, false, &flow_sources);
  EXPECT_TRUE(flow_sources & kPredictedFlowMask) << "Expected flow_sources to include predicted";

  tile->GetSpeed(de, kConstrainedFlowMask, kConstrainedFlowSecondOfDay, false, &flow_sources);
  EXPECT_FALSE(flow_sources & kPredictedFlowMask) << "Expected flow_sources not to include predicted";

  tile->GetSpeed(de, kNoFlowMask, kConstrainedFlowSecondOfDay, false, &flow_sources);

  EXPECT_FALSE(flow_sources & kPredictedFlowMask) << "Expected flow_sources not to include predicted";

  // Test case for truck speed
  auto new_de = *de;
  // Truck speed is not set for the edge initaially. In such case normal speed is used.
  EXPECT_EQ(new_de.truck_speed(), 0);
  EXPECT_EQ(tile->GetSpeed(&new_de, kNoFlowMask, kInvalidSecondsOfWeek, true),
            tile->GetSpeed(&new_de, kNoFlowMask));

  // set truck speed to some value other than normal speed
  uint32_t truck_speed = 10;
  EXPECT_NE(tile->GetSpeed(&new_de, kNoFlowMask), truck_speed);
  new_de.set_truck_speed(truck_speed);

  // Now GetSpeed should return exactly the truck speed
  EXPECT_EQ(tile->GetSpeed(&new_de, kNoFlowMask, kInvalidSecondsOfWeek, true), truck_speed);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
