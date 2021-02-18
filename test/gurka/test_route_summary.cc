#include <cmath>
#include <sstream>
#include <sys/mman.h>
#include <sys/stat.h>

#include "gurka.h"
#include "loki/worker.h"
#include "microtar.h"
#include "mjolnir/adminbuilder.h"
#include "test/test.h"

#include <boost/property_tree/ptree.hpp>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/traffictile.h>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

namespace {

valhalla::gurka::map BuildData(const std::string& workdir) {
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

  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(node_layout, ways, {}, {}, pbf_filename);

  std::unordered_map<std::string, std::string> config_map =
      {{"mjolnir.data_processing.use_direction_on_ways", "true"},
       {"mjolnir.admin", VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}};

  valhalla::gurka::map map = gurka::buildtiles(node_layout, ways, {}, {}, workdir, config_map);

  map.nodes = node_layout;

  return map;
}

} // namespace

TEST(TestRouteSummary, GetSummary) {

  std::string workdir = "test/data/gurka_test_route_summary";

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildData(workdir);

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
  gurka::assert::osrm::expect_summaries(result0, {"RT 1, RT 5"});

  // Pedestrians avoid motorways, so the shortest route is not an option.
  // BGHE is marked foot=no, so the only option is the northermost route: ABCDEF.
  valhalla::Api result2 = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "pedestrian");
  EXPECT_EQ(result2.trip().routes_size(), 1);
  EXPECT_EQ(result2.trip().routes(0).legs_size(), 1);
  gurka::assert::osrm::expect_steps(result2, {"RT 1", "RT 3", "RT 5"});
  gurka::assert::osrm::expect_summaries(result0, {"RT 1, RT 5"});

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

  filesystem::remove_all(workdir);
}
