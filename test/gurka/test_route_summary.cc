#include <cmath>
#include <filesystem>
#include <sys/mman.h>
#include <sys/stat.h>

#include "gurka.h"
#include "loki/worker.h"
#include "microtar.h"
#include "mjolnir/adminbuilder.h"
#include "test/test.h"

#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/traffictile.h>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

TEST(TestRouteSummary, GetSummary) {
  const std::string ascii_map = R"(
                                C--------D
                                |        |
      A-------------------------B        E-----------------------------------------F
                                |\      /|
                                | I----J |
                                |        |
                                G--------H
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"name", "RT 1"}}},
      {"BIJE", {{"highway", "motorway"}, {"name", "RT 2"}}},
      {"BCDE", {{"highway", "primary"}, {"name", "RT 3"}, {"foot", "yes"}, {"bicycle", "no"}}},
      {"BGHE", {{"highway", "primary"}, {"name", "RT 4"}, {"foot", "no"}, {"bicycle", "yes"}}},
      {"EF", {{"highway", "primary"}, {"name", "RT 5"}}},
  };

  const auto node_layout = gurka::detail::map_to_coordinates(ascii_map, 100, PointLL{5.108, 52.01});

  std::unordered_map<std::string, std::string> config_map =
      {{"mjolnir.data_processing.use_direction_on_ways", "true"},
       {"mjolnir.admin", VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}};

  std::string workdir = "test/data/gurka_test_route_summary";
  valhalla::gurka::map map = gurka::buildtiles(node_layout, ways, {}, {}, workdir, config_map);

  map.nodes = node_layout;

  // Shortest route chosen: ABIJEF
  valhalla::Api result0 = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");
  EXPECT_EQ(result0.trip().routes_size(), 1);
  EXPECT_EQ(result0.trip().routes(0).legs_size(), 1);
  gurka::assert::osrm::expect_steps(result0, {"RT 1", "RT 2", "RT 5"});
  gurka::assert::osrm::expect_summaries(result0, {"RT 1, RT 5"});

  // Bikes avoid motorways, so the shortest route is not an option.
  // ABCDEF is the next shortest route, but BCDE is marked bicycle=no.
  // The only remaining option is the southernmost route: ABGHEF
  valhalla::Api result1 = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "bicycle");
  EXPECT_EQ(result1.trip().routes_size(), 1);
  EXPECT_EQ(result1.trip().routes(0).legs_size(), 1);
  gurka::assert::osrm::expect_steps(result1, {"RT 1", "RT 4", "RT 5"});
  gurka::assert::osrm::expect_summaries(result1, {"RT 1, RT 5"});

  // Pedestrians avoid motorways, so the shortest route is not an option.
  // BGHE is marked foot=no, so the only option is the northermost route: ABCDEF.
  valhalla::Api result2 = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "pedestrian");
  EXPECT_EQ(result2.trip().routes_size(), 1);
  EXPECT_EQ(result2.trip().routes(0).legs_size(), 1);
  gurka::assert::osrm::expect_steps(result2, {"RT 1", "RT 3", "RT 5"});
  gurka::assert::osrm::expect_summaries(result2, {"RT 1, RT 5"});

  // You can see above that all three results have the same summary: "RT 1, RT 5".
  // Below we mash together the three above results into a single result. When we
  // ask for the summary for the mashed up result below, the summarization logic
  // will be forced generate unique summaries for each route/leg - in this case by
  // adding the middle section of each route/leg.

  // I'm not sure the best way to mash three results together... but for my needs,
  // this seemed to suffice.
  valhalla::Api result;

  // take any options
  *result.mutable_options() = result0.options();

  // add in the three routes
  auto* route = result.mutable_trip()->mutable_routes()->Add();
  *route = result0.trip().routes(0);
  route = result.mutable_trip()->mutable_routes()->Add();
  *route = result1.trip().routes(0);
  route = result.mutable_trip()->mutable_routes()->Add();
  *route = result2.trip().routes(0);

  EXPECT_EQ(result.trip().routes_size(), 3);
  EXPECT_EQ(result.trip().routes(0).legs_size(), 1);
  EXPECT_EQ(result.trip().routes(1).legs_size(), 1);
  EXPECT_EQ(result.trip().routes(2).legs_size(), 1);

  // add in the three directions
  auto* directions = result.mutable_directions()->mutable_routes()->Add();
  *directions = result0.directions().routes(0);
  directions = result.mutable_directions()->mutable_routes()->Add();
  *directions = result1.directions().routes(0);
  directions = result.mutable_directions()->mutable_routes()->Add();
  *directions = result2.directions().routes(0);

  const std::string expected_route_summary0 = "RT 1, RT 2, RT 5";
  const std::string expected_route_summary1 = "RT 1, RT 4, RT 5";
  const std::string expected_route_summary2 = "RT 1, RT 3, RT 5";
  gurka::assert::osrm::expect_summaries(result, {expected_route_summary0, expected_route_summary1,
                                                 expected_route_summary2});

  std::filesystem::remove_all(workdir);
}

TEST(TestRouteSummary, DupSummaryFix) {
  const std::string ascii_map = R"(
                   G-------------------H                I
                  /                     \              / \
      A----------B                       E------------F   J
                 |                       |             \ /
                 |                       |              K
                 |                       |
                 |                       |
                 |                       |
                 C-----------------------D
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"name", "RT 1"}}},
      {"BGHE", {{"highway", "motorway"}, {"name", "RT 1"}}},
      {"EF", {{"highway", "primary"}, {"name", "RT 2"}}},
      {"FIJ", {{"highway", "motorway"}, {"name", "RT 3"}}},
      {"BCDE", {{"highway", "primary"}, {"name", "RT 2"}}},
      {"FKJ", {{"highway", "primary"}, {"name", "RT 4"}}},
  };

  const auto node_layout = gurka::detail::map_to_coordinates(ascii_map, 100, PointLL{5.108, 52.01});

  std::unordered_map<std::string, std::string> config_map =
      {{"mjolnir.data_processing.use_direction_on_ways", "true"},
       {"mjolnir.admin", VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}};

  std::string workdir = "test/data/gurka_test_dup_summary_fix";
  valhalla::gurka::map map = gurka::buildtiles(node_layout, ways, {}, {}, workdir, config_map);

  map.nodes = node_layout;

  // auto will take the short route via the motorway
  valhalla::Api result0 = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, "auto");
  EXPECT_EQ(result0.trip().routes_size(), 1);
  EXPECT_EQ(result0.trip().routes(0).legs_size(), 1);
  gurka::assert::osrm::expect_steps(result0, {"RT 1", "RT 2", "RT 3"});
  gurka::assert::osrm::expect_summaries(result0, {"RT 1, RT 2"});

  // Pedestrians avoid the motorways
  valhalla::Api result1 = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, "pedestrian");
  EXPECT_EQ(result1.trip().routes_size(), 1);
  EXPECT_EQ(result1.trip().routes(0).legs_size(), 1);
  gurka::assert::osrm::expect_steps(result1, {"RT 1", "RT 2", "RT 4"});
  gurka::assert::osrm::expect_summaries(result1, {"RT 1, RT 2"});

  // You can see above that all two results have the same summary: RT 1, RT 2. However, each
  // chose different paths. The auto's longest named segment was RT 1 and the ped's longest
  // named segment was RT 2. Because the longest legs were different, the summary logic
  // stopped searching after the first named segment and returned the minimum number of
  // named segments (2) which resulted in them both having the same summary: RT 1, RT 2.
  // The route's can be made unique by having the auto choose RT 3 and the ped choose RT 4
  // at the end of the route.

  // Below we mash together the three above results into a single result. When we
  // ask for the summary for the mashed up result below, the summarization logic
  // will be forced generate unique summaries for each route/leg - in this case by
  // adding the middle section of each route/leg.

  // I'm not sure the best way to mash three results together... but for my needs,
  // this seemed to suffice.
  valhalla::Api result;

  // take any options
  *result.mutable_options() = result0.options();

  // add in the routes
  auto* route = result.mutable_trip()->mutable_routes()->Add();
  *route = result0.trip().routes(0);
  route = result.mutable_trip()->mutable_routes()->Add();
  *route = result1.trip().routes(0);

  EXPECT_EQ(result.trip().routes_size(), 2);
  EXPECT_EQ(result.trip().routes(0).legs_size(), 1);
  EXPECT_EQ(result.trip().routes(1).legs_size(), 1);

  // add in the directions
  auto* directions = result.mutable_directions()->mutable_routes()->Add();
  *directions = result0.directions().routes(0);
  directions = result.mutable_directions()->mutable_routes()->Add();
  *directions = result1.directions().routes(0);

  const std::string expected_route_summary0 = "RT 1, RT 2, RT 3";
  const std::string expected_route_summary1 = "RT 1, RT 2, RT 4";
  gurka::assert::osrm::expect_summaries(result, {expected_route_summary0, expected_route_summary1});

  std::filesystem::remove_all(workdir);
}

TEST(Standalone, TripLegSummary) {
  const std::string ascii_map = R"(
      A---B---C---D
    )";

  const gurka::ways ways = {{"AB", {{"highway", "motorway"}}},
                            {"BC", {{"highway", "motorway"}, {"toll", "yes"}}},
                            {"CD", {{"route", "ferry"}}}};
  const double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  const std::string workdir = "test/data/gurka_test_route_summary";

  std::string result_json;
  rapidjson::Document result;

  valhalla::gurka::map map = gurka::buildtiles(layout, ways, {}, {}, workdir);

  valhalla::Api result0 = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto",
                                           {{"/directions_type", "none"}}, {}, &result_json);
  EXPECT_TRUE(result0.trip().routes(0).legs(0).summary().has_highway());
  EXPECT_FALSE(result0.trip().routes(0).legs(0).summary().has_toll());
  EXPECT_FALSE(result0.trip().routes(0).legs(0).summary().has_ferry());

  result.Parse(result_json.c_str());
  auto trip = result["trip"].GetObject();
  auto summary = trip["summary"].GetObject();

  EXPECT_TRUE(summary["has_highway"].GetBool());
  EXPECT_FALSE(summary["has_toll"].GetBool());
  EXPECT_FALSE(summary["has_ferry"].GetBool());
  result_json.erase();

  valhalla::Api result1 = gurka::do_action(valhalla::Options::route, map, {"B", "C"}, "auto",
                                           {{"/directions_type", "none"}});
  EXPECT_TRUE(result1.trip().routes(0).legs(0).summary().has_highway());
  EXPECT_TRUE(result1.trip().routes(0).legs(0).summary().has_toll());
  EXPECT_FALSE(result1.trip().routes(0).legs(0).summary().has_ferry());

  valhalla::Api result2 = gurka::do_action(valhalla::Options::route, map, {"C", "D"}, "auto",
                                           {{"/directions_type", "none"}});
  EXPECT_FALSE(result2.trip().routes(0).legs(0).summary().has_highway());
  EXPECT_FALSE(result2.trip().routes(0).legs(0).summary().has_toll());
  EXPECT_TRUE(result2.trip().routes(0).legs(0).summary().has_ferry());

  // Validate that the presence of highway, toll, and ferry tags in route summaries is consistent
  // and does not depend on the `directions_type` value

  valhalla::Api result3 = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto",
                                           {{"/directions_type", "none"}}, {}, &result_json);

  EXPECT_TRUE(result3.trip().routes(0).legs(0).summary().has_highway());
  EXPECT_TRUE(result3.trip().routes(0).legs(0).summary().has_toll());
  EXPECT_TRUE(result3.trip().routes(0).legs(0).summary().has_ferry());

  result.Parse(result_json.c_str());
  trip = result["trip"].GetObject();
  summary = trip["summary"].GetObject();

  EXPECT_TRUE(summary["has_highway"].GetBool());
  EXPECT_TRUE(summary["has_toll"].GetBool());
  EXPECT_TRUE(summary["has_ferry"].GetBool());
  result_json.erase();

  valhalla::Api result4 = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto",
                                           {{"/directions_type", "maneuvers"}}, {}, &result_json);
  EXPECT_TRUE(result4.trip().routes(0).legs(0).summary().has_highway());
  EXPECT_TRUE(result4.trip().routes(0).legs(0).summary().has_toll());
  EXPECT_TRUE(result4.trip().routes(0).legs(0).summary().has_ferry());

  result.Parse(result_json.c_str());
  trip = result["trip"].GetObject();
  summary = trip["summary"].GetObject();

  EXPECT_TRUE(summary["has_highway"].GetBool());
  EXPECT_TRUE(summary["has_toll"].GetBool());
  EXPECT_TRUE(summary["has_ferry"].GetBool());

  std::filesystem::remove_all(workdir);
}
