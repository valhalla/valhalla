#include "gurka.h"
#include "test.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

//=======================================================================================
TEST(HOVTest, mistagged_hov) {
  constexpr double gridsize = 10;

  const std::string ascii_map = R"(
    A---------------------------B
  )";

  // none of these are correctly tagged for HOV routing.
  const std::vector<gurka::ways> all_ways = {
      // specifying nothing should definitely not be considered hov
      {{"AB", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "RT 1"}}}},

      // To be correctly tagged the way must specify both:
      // hov=designated and ( hov:minimum=2 or hov:minimum=3 )
      {{"AB", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "RT 1"}, {"hov", "yes"}}}},
      {{"AB", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "RT 1"}, {"hov", "true"}}}},
      {{"AB", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "RT 1"}, {"hov:minimum", "1"}}}},
      {{"AB", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "RT 1"}, {"hov:minimum", "2"}}}},
      {{"AB", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "RT 1"}, {"hov:minimum", "3"}}}},
      {{"AB", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "RT 1"}, {"hov", "designated"}}}},
      {{"AB",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"name", "RT 1"},
         {"hov", "yes"},
         {"hov:minimum", "3"}}}},
      {{"AB",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"name", "RT 1"},
         {"hov", "true"},
         {"hov:minimum", "3"}}}},
      {{"AB",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"name", "RT 1"},
         {"hov", "designated"},
         {"hov:minimum", "4"}}}},
      {{"AB",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"name", "RT 1"},
         {"hov", "designated"},
         {"hov:minimum", "6"}}}},

      // Also, ways that might change direction or have conditional access will not be considered
      {{"AB",
        {{"highway", "motorway"},
         {"oneway", "reversible"},
         {"name", "RT 1"},
         {"hov", "designated"},
         {"hov:minimum", "2"}}}},
      {{"AB",
        {{"highway", "motorway"},
         {"oneway", "alternating"},
         {"name", "RT 1"},
         {"hov", "designated"},
         {"hov:minimum", "2"}}}},
      {{"AB",
        {{"highway", "motorway"},
         {"oneway", "false"},
         {"name", "RT 1"},
         {"hov", "designated"},
         {"hov:minimum", "2"}}}},
      {{"AB",
        {{"highway", "motorway"},
         {"oneway", "no"},
         {"name", "RT 1"},
         {"hov", "designated"},
         {"hov:minimum", "2"}}}},
      {{"AB",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"name", "RT 1"},
         {"hov", "designated"},
         {"hov:minimum", "2"},
         {"oneway:conditional", "yes"}}}},
      {{"AB",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"name", "RT 1"},
         {"hov", "designated"},
         {"hov:minimum", "2"},
         {"access:conditional", "yes"}}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  for (const auto& ways : all_ways) {
    gurka::map map = gurka::buildtiles(layout, ways, {}, {}, "test/data/mistagged_HOV", {});
    std::shared_ptr<baldr::GraphReader> reader =
        test::make_clean_graphreader(map.config.get_child("mjolnir"));
    auto edge_tuple = gurka::findEdgeByNodes(*reader, layout, "A", "B");
    const baldr::DirectedEdge* edge = std::get<1>(edge_tuple);
    ASSERT_EQ(edge->is_hov_only(), false);
    ASSERT_TRUE(edge->forwardaccess() & kAutoAccess);
  }
}

//=======================================================================================
TEST(HOVTest, correctly_tagged_hov) {
  constexpr double gridsize = 10;

  const std::string ascii_map = R"(
    A---------------------------B
  )";

  // These are correctly tagged for HOV because they specify:
  // hov=designated and ( hov:minimum=2 or hov:minimum=3 )

  // HOV-2
  {
    const gurka::ways hov2_ways = {{"AB",
                                    {{"highway", "motorway"},
                                     {"oneway", "yes"},
                                     {"name", "RT 1"},
                                     {"hov", "designated"},
                                     {"hov:minimum", "2"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    gurka::map map =
        gurka::buildtiles(layout, hov2_ways, {}, {}, "test/data/correctly_tagged_HOV", {});
    std::shared_ptr<baldr::GraphReader> reader =
        test::make_clean_graphreader(map.config.get_child("mjolnir"));
    auto edge_tuple = gurka::findEdgeByNodes(*reader, layout, "A", "B");
    const baldr::DirectedEdge* edge = std::get<1>(edge_tuple);
    ASSERT_EQ(edge->is_hov_only(), true);
    ASSERT_TRUE(edge->hov_type() == baldr::HOVEdgeType::kHOV2);
    ASSERT_TRUE(edge->hov_type() != baldr::HOVEdgeType::kHOV3);
  }

  // HOV-3
  {
    const gurka::ways hov3_ways = {{"AB",
                                    {{"highway", "motorway"},
                                     {"oneway", "yes"},
                                     {"name", "RT 1"},
                                     {"hov", "designated"},
                                     {"hov:minimum", "3"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    gurka::map map =
        gurka::buildtiles(layout, hov3_ways, {}, {}, "test/data/correctly_tagged_HOV", {});
    std::shared_ptr<baldr::GraphReader> reader =
        test::make_clean_graphreader(map.config.get_child("mjolnir"));
    auto edge_tuple = gurka::findEdgeByNodes(*reader, layout, "A", "B");
    const baldr::DirectedEdge* edge = std::get<1>(edge_tuple);
    ASSERT_EQ(edge->is_hov_only(), true);
    ASSERT_TRUE(edge->hov_type() != baldr::HOVEdgeType::kHOV2);
    ASSERT_TRUE(edge->hov_type() == baldr::HOVEdgeType::kHOV3);
  }
}

//=======================================================================================
const std::string include_hov2_true = R"("include_hov2": true)";
const std::string include_hov3_true = R"("include_hov3": true)";
const std::string include_hot_true = R"("include_hot": true)";

const std::string req_hov = R"({
  "locations": [
    {"lat": %s, "lon": %s},
    {"lat": %s, "lon": %s}
  ],
  "costing": "auto",
  "costing_options": {
    "auto": {
      %s
    }
  },
  "date_time": { "type": 3, "value": "current" },
  "format": "osrm",
  "shape_format": "geojson",
  "filters": {
    "attributes": [
      "shape_attributes.closure"
    ],
    "action": "include"
  }
}
)";

//=======================================================================================
class HOV2Test : public ::testing::Test {
protected:
  static gurka::map map;
  static std::shared_ptr<baldr::GraphReader> reader;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
      A-----------------------------------------------------C
      D-----------1--------------------------------2--------F
    )";
    const gurka::ways ways = {{"AC", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "RT 36"}}},
                              {"DF",
                               {{"highway", "motorway"},
                                {"oneway", "yes"},
                                {"name", "HOVExpress2"},
                                {"hov", "designated"},
                                {"hov:minimum", "2"}}}};

    const gurka::nodes nodes;
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/HOV2", {});
    reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  }
};

std::shared_ptr<baldr::GraphReader> HOV2Test::reader;
gurka::map HOV2Test::map = {};

//------------------------------------------------------------------
TEST_F(HOV2Test, default_avoids_hov2) {
  std::string req =
      (boost::format(req_hov) % std::to_string(map.nodes.at("1").lat()) %
       std::to_string(map.nodes.at("1").lng()) % std::to_string(map.nodes.at("2").lat()) %
       std::to_string(map.nodes.at("2").lng()) % "")
          .str();
  auto result = gurka::do_action(Options::route, map, req, reader);

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "RT 36");
}

//------------------------------------------------------------------
TEST_F(HOV2Test, hov2_true_uses_hov2) {
  std::string req =
      (boost::format(req_hov) % std::to_string(map.nodes.at("1").lat()) %
       std::to_string(map.nodes.at("1").lng()) % std::to_string(map.nodes.at("2").lat()) %
       std::to_string(map.nodes.at("2").lng()) % include_hov2_true)
          .str();
  auto result = gurka::do_action(Options::route, map, req, reader);

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "HOVExpress2");
}

//------------------------------------------------------------------
TEST_F(HOV2Test, hot_true_avoids_hov2) {
  std::string req =
      (boost::format(req_hov) % std::to_string(map.nodes.at("1").lat()) %
       std::to_string(map.nodes.at("1").lng()) % std::to_string(map.nodes.at("2").lat()) %
       std::to_string(map.nodes.at("2").lng()) % include_hot_true)
          .str();
  auto result = gurka::do_action(Options::route, map, req, reader);

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "RT 36");
}

//------------------------------------------------------------------
TEST_F(HOV2Test, hov3_true_uses_hov2) {
  std::string req =
      (boost::format(req_hov) % std::to_string(map.nodes.at("1").lat()) %
       std::to_string(map.nodes.at("1").lng()) % std::to_string(map.nodes.at("2").lat()) %
       std::to_string(map.nodes.at("2").lng()) % include_hov3_true)
          .str();
  auto result = gurka::do_action(Options::route, map, req, reader);

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "HOVExpress2");
}

//------------------------------------------------------------------
TEST_F(HOV2Test, hov_costing_uses_auto_hov2) {
  auto replace_all = [](std::string& original, const std::string& find, const std::string& replace) {
    size_t i = 0;
    while ((i = original.find(find, i)) != std::string::npos) {
      original.replace(i, find.size(), replace);
      i += replace.size();
    }
  };

  std::string req =
      (boost::format(req_hov) % std::to_string(map.nodes.at("1").lat()) %
       std::to_string(map.nodes.at("1").lng()) % std::to_string(map.nodes.at("2").lat()) %
       std::to_string(map.nodes.at("2").lng()) % "")
          .str();
  replace_all(req, "auto", "hov");
  auto result = gurka::do_action(Options::route, map, req, reader);

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "HOVExpress2");
}

//=======================================================================================
class HOV3Test : public ::testing::Test {
protected:
  static gurka::map map;
  static std::shared_ptr<baldr::GraphReader> reader;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
      A---------------------------B---------------------------C
      D------1--------------------E------------------2--------F
    )";
    const gurka::ways ways = {{"ABC",
                               {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "RT 36"}}},
                              {"DEF",
                               {{"highway", "motorway"},
                                {"oneway", "yes"},
                                {"name", "HOVExpress3"},
                                {"hov", "designated"},
                                {"hov:minimum", "3"}}}};

    const gurka::nodes nodes;
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/HOV3", {});
    reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  }
};

std::shared_ptr<baldr::GraphReader> HOV3Test::reader;
gurka::map HOV3Test::map = {};

//------------------------------------------------------------------
TEST_F(HOV3Test, default_avoids_hov3) {
  std::string req =
      (boost::format(req_hov) % std::to_string(map.nodes.at("1").lat()) %
       std::to_string(map.nodes.at("1").lng()) % std::to_string(map.nodes.at("2").lat()) %
       std::to_string(map.nodes.at("2").lng()) % "")
          .str();
  auto result = gurka::do_action(Options::route, map, req, reader);

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "RT 36");
}

//------------------------------------------------------------------
TEST_F(HOV3Test, hov2_true_avoids_hov3) {
  std::string req =
      (boost::format(req_hov) % std::to_string(map.nodes.at("1").lat()) %
       std::to_string(map.nodes.at("1").lng()) % std::to_string(map.nodes.at("2").lat()) %
       std::to_string(map.nodes.at("2").lng()) % include_hov2_true)
          .str();
  auto result = gurka::do_action(Options::route, map, req, reader);

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "RT 36");
}

//------------------------------------------------------------------
TEST_F(HOV3Test, hot_true_avoids_hov3) {
  std::string req =
      (boost::format(req_hov) % std::to_string(map.nodes.at("1").lat()) %
       std::to_string(map.nodes.at("1").lng()) % std::to_string(map.nodes.at("2").lat()) %
       std::to_string(map.nodes.at("2").lng()) % include_hot_true)
          .str();
  auto result = gurka::do_action(Options::route, map, req, reader);

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "RT 36");
}

//------------------------------------------------------------------
TEST_F(HOV3Test, hov3_true_uses_hov3) {
  std::string req =
      (boost::format(req_hov) % std::to_string(map.nodes.at("1").lat()) %
       std::to_string(map.nodes.at("1").lng()) % std::to_string(map.nodes.at("2").lat()) %
       std::to_string(map.nodes.at("2").lng()) % include_hov3_true)
          .str();
  auto result = gurka::do_action(Options::route, map, req, reader);

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "HOVExpress3");
}

//=======================================================================================
class HOTTest : public ::testing::Test {
protected:
  static gurka::map map;
  static std::shared_ptr<baldr::GraphReader> reader;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
      A---------------------------B---------------------------C
      D---------1-----------------E--------------------2------F
    )";
    const gurka::ways ways = {{"ABC",
                               {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "RT 36"}}},
                              {"DEF",
                               {{"highway", "motorway"},
                                {"oneway", "yes"},
                                {"name", "HOTExpress3"},
                                {"hov", "designated"},
                                {"hov:minimum", "3"},
                                {"toll", "yes"}}}};

    const gurka::nodes nodes;
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/HOT", {});
    reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  }
};

std::shared_ptr<baldr::GraphReader> HOTTest::reader;
gurka::map HOTTest::map = {};

//------------------------------------------------------------------
TEST_F(HOTTest, default_avoids_hot) {
  std::string req =
      (boost::format(req_hov) % std::to_string(map.nodes.at("1").lat()) %
       std::to_string(map.nodes.at("1").lng()) % std::to_string(map.nodes.at("2").lat()) %
       std::to_string(map.nodes.at("2").lng()) % "")
          .str();
  auto result = gurka::do_action(Options::route, map, req, reader);

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "RT 36");
}

//------------------------------------------------------------------
TEST_F(HOTTest, hov2_true_avoids_hot3) {
  std::string req =
      (boost::format(req_hov) % std::to_string(map.nodes.at("1").lat()) %
       std::to_string(map.nodes.at("1").lng()) % std::to_string(map.nodes.at("2").lat()) %
       std::to_string(map.nodes.at("2").lng()) % include_hov2_true)
          .str();
  auto result = gurka::do_action(Options::route, map, req, reader);

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "RT 36");
}

//------------------------------------------------------------------
TEST_F(HOTTest, hov3_true_uses_hot3) {
  std::string req =
      (boost::format(req_hov) % std::to_string(map.nodes.at("1").lat()) %
       std::to_string(map.nodes.at("1").lng()) % std::to_string(map.nodes.at("2").lat()) %
       std::to_string(map.nodes.at("2").lng()) % include_hov3_true)
          .str();
  auto result = gurka::do_action(Options::route, map, req, reader);

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "HOTExpress3");
}

//------------------------------------------------------------------
TEST_F(HOTTest, hot_true_uses_hot3) {
  std::string req =
      (boost::format(req_hov) % std::to_string(map.nodes.at("1").lat()) %
       std::to_string(map.nodes.at("1").lng()) % std::to_string(map.nodes.at("2").lat()) %
       std::to_string(map.nodes.at("2").lng()) % include_hot_true)
          .str();
  auto result = gurka::do_action(Options::route, map, req, reader);

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "HOTExpress3");
}

//=======================================================================================
class HOVChoices : public ::testing::Test {
protected:
  static gurka::map map;
  static std::shared_ptr<baldr::GraphReader> reader;

  static void SetUpTestSuite() {

    // This gridsize/map is attempting to emulate a real-world hov-lane that parallels
    // a highway. It isn't perfect... the hov-lane would probably be a little less than
    // 5 meters from the highway lane... and AC would be much longer. The transition from
    // E to A and from C to F are about 90 m, which is accurate based on my measurements.
    // Note also that EA is a motorway-link, which emulates what I'm seeing in the real
    // world. However, CF is not a motorway-link.
    constexpr double gridsize = 5;
    const std::string ascii_map = R"(
                                        A------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------C
      D--------------E--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------F----------------G
    )";
    const gurka::ways ways =
        {{"DEFG",
          {{"highway", "motorway"}, {"oneway", "yes"}, {"maxspeed", "40"}, {"name", "RT 36"}}},
         {"EA",
          {{"highway", "motorway_link"},
           {"oneway", "yes"},
           {"name", "US 36 Express Lane"},
           {"hov", "designated"},
           {"hov:minimum", "3"},
           {"maxspeed", "65"},
           {"toll", "yes"}}},
         {"ACF",
          {{"highway", "motorway"},
           {"oneway", "yes"},
           {"name", "US 36 Express Lane"},
           {"hov", "designated"},
           {"hov:minimum", "3"},
           {"maxspeed", "65"},
           {"toll", "yes"}}}};

    const gurka::nodes nodes;
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/HOVChosen", {});
    reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  }
};

std::shared_ptr<baldr::GraphReader> HOVChoices::reader;
gurka::map HOVChoices::map = {};

//------------------------------------------------------------------
TEST_F(HOVChoices, choices) {
  {
    std::string req =
        (boost::format(req_hov) % std::to_string(map.nodes.at("D").lat()) %
         std::to_string(map.nodes.at("D").lng()) % std::to_string(map.nodes.at("G").lat()) %
         std::to_string(map.nodes.at("G").lng()) % "")
            .str();
    auto result = gurka::do_action(Options::route, map, req, reader);

    // The hov lane is fastest, but router cannot choose it because
    // the user has not specified any hov settings.
    EXPECT_EQ(result.directions().routes(0).legs(0).maneuver_size(), 2);
    EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "RT 36");
  }

  {
    std::string req =
        (boost::format(req_hov) % std::to_string(map.nodes.at("D").lat()) %
         std::to_string(map.nodes.at("D").lng()) % std::to_string(map.nodes.at("G").lat()) %
         std::to_string(map.nodes.at("G").lng()) % include_hov2_true)
            .str();
    auto result = gurka::do_action(Options::route, map, req, reader);

    // The hov lane is fastest, but router cannot choose it because
    // the user has allowed hov2 but the highway is hot3.
    EXPECT_EQ(result.directions().routes(0).legs(0).maneuver_size(), 2);
    EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "RT 36");
  }

  {
    std::string req =
        (boost::format(req_hov) % std::to_string(map.nodes.at("D").lat()) %
         std::to_string(map.nodes.at("D").lng()) % std::to_string(map.nodes.at("G").lat()) %
         std::to_string(map.nodes.at("G").lng()) % include_hot_true)
            .str();
    auto result = gurka::do_action(Options::route, map, req, reader);

    // User allows hot-lanes and the router can choose the faster US 36 Express Lane.
    // Note: the return angle from the HOV lane onto the highway is very slight,
    // so we don't announce a maneuver returning onto RT-36.
    EXPECT_EQ(result.directions().routes(0).legs(0).maneuver_size(), 3);
    EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "RT 36");
    EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).street_name(0).value(),
              "US 36 Express Lane");
  }

  {
    std::string req =
        (boost::format(req_hov) % std::to_string(map.nodes.at("D").lat()) %
         std::to_string(map.nodes.at("D").lng()) % std::to_string(map.nodes.at("G").lat()) %
         std::to_string(map.nodes.at("G").lng()) % include_hov3_true)
            .str();
    auto result = gurka::do_action(Options::route, map, req, reader);

    // User allows hov3-lanes and the router can choose the faster US 36 Express Lane.
    // Note: the return angle from the HOV lane onto the highway is very slight,
    // so we don't announce a maneuver returning onto RT-36.
    EXPECT_EQ(result.directions().routes(0).legs(0).maneuver_size(), 3);
    EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "RT 36");
    EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).street_name(0).value(),
              "US 36 Express Lane");
  }
}
