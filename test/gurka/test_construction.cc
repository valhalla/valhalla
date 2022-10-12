#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

class IncludeConstructionTest : public ::testing::Test {
protected:
  static gurka::nodelayout layout;
  static gurka::ways ways;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
      A---------B--------C
                         |
                         D
    )";

    ways = {
        {"AB", {{"highway", "primary"}}},
        {"BC", {{"highway", "construction"}, {"construction", "primary"}}},
        // road under construction with missing 'construction' tag; should be excluded from the graph
        {"CD", {{"highway", "construction"}}},
    };

    layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  }
};

gurka::nodelayout IncludeConstructionTest::layout = {};
gurka::ways IncludeConstructionTest::ways = {};

TEST_F(IncludeConstructionTest, CheckConstructionsExcluded) {
  // exclude roads under construction
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/construction",
                               {{"mjolnir.include_construction", "false"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  auto AB = gurka::findEdge(reader, layout, "AB", "B");
  ASSERT_NE(std::get<0>(AB), baldr::GraphId{}) << "missing valid road AB";

  auto BC = gurka::findEdge(reader, layout, "BC", "C");
  EXPECT_EQ(std::get<0>(BC), baldr::GraphId{}) << "constructions should be excluded";

  auto CD = gurka::findEdge(reader, layout, "CD", "D");
  EXPECT_EQ(std::get<0>(CD), baldr::GraphId{}) << "constructions should be excluded";
}

TEST_F(IncludeConstructionTest, CheckConstructionsIncluded) {
  // include roads under construction
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/construction",
                               {{"mjolnir.include_construction", "true"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  auto AB = gurka::findEdge(reader, layout, "AB", "B");
  ASSERT_NE(std::get<0>(AB), baldr::GraphId{}) << "missing valid road AB";

  auto BC = gurka::findEdge(reader, layout, "BC", "C");
  ASSERT_NE(std::get<0>(BC), baldr::GraphId{}) << "constructions should be included";
  EXPECT_EQ(std::get<1>(BC)->use(), baldr::Use::kConstruction)
      << "invalid 'use'-type for construction edge";
  // check that access set to zero for constructions
  EXPECT_EQ(std::get<1>(BC)->forwardaccess(), 0) << "access should be turned off for constructions";
  EXPECT_EQ(std::get<1>(BC)->reverseaccess(), 0) << "access should be turned off for constructions";

  auto CD = gurka::findEdge(reader, layout, "CD", "D");
  EXPECT_EQ(std::get<0>(CD), baldr::GraphId{})
      << "constructions with incomplete tags should be excluded";
}

TEST(ConstructionInShortcuts, ExcludeConstructionsFromShortcut) {
  const std::string ascii_map = R"(
                 G                     H
                 |                     |
                 |                     |
      A----------B----------C----------D----------E----------F
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}, {"name", "M1"}}},
      {"BC", {{"highway", "motorway"}, {"name", "M1"}}},
      {"DE", {{"highway", "motorway"}, {"name", "M1"}}},
      {"EF", {{"highway", "motorway"}, {"name", "M1"}}},

      {"CD", {{"highway", "construction"}, {"construction", "motorway"}, {"name", "M1"}}},

      {"BG", {{"highway", "service"}}},
      {"DH", {{"highway", "service"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);

  const auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/construction",
                        {{"mjolnir.include_construction", "true"}, {"mjolnir.shortcuts", "true"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  // check that shortcuts exist but they avoid construction edge
  try {
    auto AC = gurka::findEdgeByNodes(reader, layout, "A", "C");
    ASSERT_TRUE(std::get<1>(AC)->is_shortcut());

    auto DF = gurka::findEdgeByNodes(reader, layout, "D", "F");
    ASSERT_TRUE(std::get<1>(DF)->is_shortcut());
  } catch (const std::exception&) { FAIL() << "expected shortcuts were not built"; }

  // check that road under construction is not a part of any shortcut
  auto CD = gurka::findEdgeByNodes(reader, layout, "C", "D");
  ASSERT_FALSE(std::get<1>(CD)->superseded()) << "construction can't be part of a shortcut";
}

class ConstructionRoutingTest : public ::testing::TestWithParam<std::string> {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
                  E----------F
                  |          |
                  |          |
      A--1--------B----------C--------2--D
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "secondary"}}},
        {"BC", {{"highway", "construction"}, {"construction", "secondary"}}},
        {"CD", {{"highway", "secondary"}}},
        {"BE", {{"highway", "secondary"}}},
        {"EF", {{"highway", "secondary"}}},
        {"FC", {{"highway", "secondary"}}},
    };

    const gurka::nodelayout layout = gurka::detail::map_to_coordinates(ascii_map, 100);

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/construction",
                            {{"mjolnir.include_construction", "true"}});
  }
};

gurka::map ConstructionRoutingTest::map = {};

TEST_P(ConstructionRoutingTest, CheckAvoidRoadsUnderConstruction) {
  const auto costing = GetParam();

  auto result = gurka::do_action(valhalla::Options::route, map, {"1", "2"}, costing);
  // road under construction "BC" should be avoided
  gurka::assert::raw::expect_path(result, {"AB", "BE", "EF", "FC", "CD"});

  result = gurka::do_action(valhalla::Options::route, map, {"1", "2"}, costing,
                            {{"/costing_options/" + costing + "/ignore_access", "1"}});
  // road under construction "BC" should be avoided despite of ignore_access=true
  gurka::assert::raw::expect_path(result, {"AB", "BE", "EF", "FC", "CD"});
}

const std::vector<std::string> costing_types = {"auto",       "taxi",       "bus",
                                                "truck",      "bicycle",    "motor_scooter",
                                                "motorcycle", "pedestrian", "hov"};

INSTANTIATE_TEST_SUITE_P(Test, ConstructionRoutingTest, ::testing::ValuesIn(costing_types));