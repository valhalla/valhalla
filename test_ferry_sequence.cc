#include "gurka.h"
#include "midgard/constants.h"
#include "valhalla/worker.h"

#include <gtest/gtest.h>

using namespace valhalla;
using namespace valhalla::midgard;

uint8_t LEVEL_O = baldr::TileHierarchy::get_level(baldr::RoadClass::kPrimary);

TEST(Standalone, FerrySequenceReclassification) {
  // Test the specific case described in issue #5510:
  // Ferry -> highway -> service -> highway sequence
  // The issue is that the algorithm stops early and doesn't reclassify
  // the service roads in the middle of the sequence

  const std::string ascii_map = R"(
    A---B---C---D---E---F---G
  )";

  const gurka::ways ways = {
      // Ferry connection
      {"AB",
       {{"motor_vehicle", "yes"},
        {"motorcar", "yes"},
        {"bicycle", "yes"},
        {"moped", "yes"},
        {"bus", "yes"},
        {"hov", "yes"},
        {"taxi", "yes"},
        {"motorcycle", "yes"},
        {"route", "ferry"}}},

      // Highway after ferry
      {"BC", {{"highway", "primary"}}},

      // Service road in the middle (this should get reclassified but doesn't with current logic)
      {"CD", {{"highway", "service"}}},
      {"DE", {{"highway", "service"}}},

      // Highway again - high class road
      {"EF", {{"highway", "primary"}}},
      {"FG", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 1000);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_ferry_sequence");
  baldr::GraphReader graph_reader(map.config.get_child("mjolnir"));

  // Check if service roads in the middle got reclassified
  auto CD_edge = gurka::findEdge(graph_reader, layout, "CD", "D");
  auto DE_edge = gurka::findEdge(graph_reader, layout, "DE", "E");

  // These should be reclassified to primary level (level 0) but currently aren't
  // due to the early termination in the algorithm
  std::cout << "CD edge level: " << std::get<0>(CD_edge).level() << std::endl;
  std::cout << "DE edge level: " << std::get<0>(DE_edge).level() << std::endl;

  // With the fix, these should be reclassified to level 0
  EXPECT_EQ(std::get<0>(CD_edge).level(), LEVEL_O) << "CD service road should be reclassified";
  EXPECT_EQ(std::get<0>(DE_edge).level(), LEVEL_O) << "DE service road should be reclassified";
}