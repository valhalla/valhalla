#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;
using namespace valhalla::baldr;

constexpr std::string_view tile_dir = VALHALLA_BUILD_DIR "test/data/bikeshare";

TEST(StandAlone, Basic) {
  const std::string ascii_map = R"(

    A---------B
        b

  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "living_street"}}},
  };

  const gurka::nodes nodes = {{"b", {{"amenity", "bicycle_rental"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {52.142, 7.123});
  auto map = gurka::buildtiles(layout, ways, nodes, {}, std::string(tile_dir),
                               {{"mjolnir.import_bike_share_stations", "1"}});
  std::shared_ptr<baldr::GraphReader> reader =
      test::make_clean_graphreader(map.config.get_child("mjolnir"));

  auto nodeA = gurka::findNode(*reader, layout, "A");
  auto niA = reader->nodeinfo(nodeA);
  EXPECT_EQ(niA->edge_count(), 2);
  auto tile = reader->GetGraphTile(nodeA);
  auto edge = tile->directededge(niA->edge_index() + 1);
  EXPECT_EQ(edge->bss_connection(), true);
  auto nodeBS = edge->endnode();
  auto niBS = tile->node(nodeBS);
  EXPECT_EQ(niBS->edge_count(), 2);
  EXPECT_EQ(niBS->type(), NodeType::kBikeShare);
  EXPECT_TRUE(static_cast<bool>(edge->forwardaccess() & (kBicycleAccess | kPedestrianAccess)));
  EXPECT_TRUE(static_cast<bool>(edge->reverseaccess() & (kBicycleAccess | kPedestrianAccess)));

  auto nodeB = gurka::findNode(*reader, layout, "B");
  auto niB = reader->nodeinfo(nodeB);
  EXPECT_EQ(niB->edge_count(), 2);
  edge = tile->directededge(niB->edge_index() + 1);
  EXPECT_EQ(edge->bss_connection(), true);
  EXPECT_TRUE(static_cast<bool>(edge->forwardaccess() & (kBicycleAccess | kPedestrianAccess)));
  EXPECT_TRUE(static_cast<bool>(edge->reverseaccess() & (kBicycleAccess | kPedestrianAccess)));
}

TEST(StandAlone, DifferentConnectionsPerMode) {
  const std::string ascii_map = R"(
    C---------D
    
    A---------B
        b

  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "living_street"}, {"bicycle", "no"}}},
      {"CD", {{"highway", "living_street"}, {"foot", "no"}}},
  };

  const gurka::nodes nodes = {{"b", {{"amenity", "bicycle_rental"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {52.142, 7.123});
  auto map = gurka::buildtiles(layout, ways, nodes, {}, std::string(tile_dir),
                               {{"mjolnir.import_bike_share_stations", "1"}});
  std::shared_ptr<baldr::GraphReader> reader =
      test::make_clean_graphreader(map.config.get_child("mjolnir"));

  auto nodeA = gurka::findNode(*reader, layout, "A");
  auto niA = reader->nodeinfo(nodeA);
  EXPECT_EQ(niA->edge_count(), 2);
  auto tile = reader->GetGraphTile(nodeA);
  auto edge = tile->directededge(niA->edge_index() + 1);
  EXPECT_EQ(edge->bss_connection(), true);
  auto nodeBS = edge->endnode();
  auto niBS = tile->node(nodeBS);
  EXPECT_EQ(niBS->edge_count(), 4);
  EXPECT_EQ(niBS->type(), NodeType::kBikeShare);

  auto nodeB = gurka::findNode(*reader, layout, "B");
  auto niB = reader->nodeinfo(nodeB);
  EXPECT_EQ(niB->edge_count(), 2);
  edge = tile->directededge(niB->edge_index() + 1);
  EXPECT_EQ(edge->bss_connection(), true);
  EXPECT_TRUE(static_cast<bool>(edge->forwardaccess() & kPedestrianAccess));
  EXPECT_FALSE(static_cast<bool>(edge->forwardaccess() & kBicycleAccess));

  auto nodeC = gurka::findNode(*reader, layout, "C");
  auto niC = reader->nodeinfo(nodeC);
  EXPECT_EQ(niC->edge_count(), 2);
  edge = tile->directededge(niC->edge_index() + 1);
  EXPECT_EQ(edge->bss_connection(), true);
  EXPECT_FALSE(static_cast<bool>(edge->forwardaccess() & kPedestrianAccess));
  EXPECT_TRUE(static_cast<bool>(edge->forwardaccess() & kBicycleAccess));
}
