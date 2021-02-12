#include "gurka.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/multi/geometries/register/multi_polygon.hpp>
#include <gtest/gtest.h>

#include "baldr/graphconstants.h"
#include "baldr/graphreader.h"
#include "loki/polygon_search.h"
#include "midgard/pointll.h"
#include "mjolnir/graphtilebuilder.h"

using namespace valhalla;
namespace bg = boost::geometry;
namespace vm = valhalla::midgard;
namespace vl = valhalla::loki;

// parameterized test class to test all costings
class AvoidTest : public ::testing::TestWithParam<std::string> {
protected:
  static gurka::map avoid_map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(A------B
                                     |      |
                                     |      |
                                     D------E)";
    const gurka::ways ways = {{"AB", {{"highway", "residential"}, {"name", "High"}}},
                              {"DE", {{"highway", "residential"}, {"name", "Low"}}},
                              {"AD", {{"highway", "residential"}, {"name", "1st"}}},
                              {"BE", {{"highway", "residential"}, {"name", "2nd"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
    // Add low length limit for avoid_polygons so it throws an error
    avoid_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_shortcut",
                                  {{"service_limits.max_avoid_polygons_length", "1"}});
  }
};

gurka::map AvoidTest::avoid_map = {};

TEST_F(AvoidTest, TestLength) {
  std::vector<vl::line_bg_t> lines(2);
  // ~ 7.6 km length
  bg::read_wkt(
      "LINESTRING (13.38625361 52.4652558, 13.38625361 52.48000128, 13.4181769 52.48000128,13.38625361 52.4652558)",
      lines[0]);
  // ~ 10.1 km length
  bg::read_wkt(
      "LINESTRING (13.371981 52.439271, 13.371981 52.479088, 13.381839 52.479088, 13.381839 52.439271, 13.371981 52.439271)",
      lines[1]);
  double actual_length = bg::length(lines[0], Haversine());
  actual_length += bg::length(lines[1], Haversine());

  // Add polygons to PBF and calculate total circumference
  Options options;
  double calc_length = 0;
  std::vector<vl::ring_bg_t> e_rings(2);
  bg::read_wkt(
      "POLYGON ((13.38625361 52.4652558, 13.38625361 52.48000128, 13.4181769 52.48000128,13.38625361 52.4652558))",
      e_rings[0]);
  bg::read_wkt(
      "POLYGON ((13.371981 52.439271, 13.371981 52.479088, 13.381839 52.479088, 13.381839 52.439271, 13.371981 52.439271))",
      e_rings[1]);
  auto avoid_polygons = options.mutable_avoid_polygons();
  for (const auto& ring : e_rings) {
    auto* polygon = avoid_polygons->Add();
    for (auto& coord : ring) {
      auto* ll = polygon->add_coords()->mutable_ll();
      ll->set_lng(coord.lng());
      ll->set_lat(coord.lat());
    }
    calc_length += vl::GetRingLength(vl::PBFToRing(*polygon));
  }

  EXPECT_EQ(actual_length, calc_length);
}

TEST_F(AvoidTest, TestConfig) {
  // Add a polygon with longer circumference than the limit
  const std::unordered_map<std::string, std::string>& req_options = {
      {"/avoid_polygons",
       "[[[13.38625361, 52.4652558], [13.38625361, 52.48000128], [13.4181769, 52.48000128], [13.4181769, 52.4652558]]]"}};

  // make sure the right exception is thrown
  try {
    gurka::do_action(Options::route, avoid_map, {"A", "D"}, "auto", req_options);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 167); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_P(AvoidTest, TestAvoidPolygon) {
  auto node_a = avoid_map.nodes.at("A");
  auto node_b = avoid_map.nodes.at("B");
  auto node_d = avoid_map.nodes.at("D");
  auto dx = node_b.lng() - node_a.lng();
  auto dy = node_a.lat() - node_d.lat();

  // create a small polygon intersecting AD ("1st") just below node A
  //      A------B
  //  x---|---x  |
  //  |   |   |  |
  //  x---|---x  |
  //      |      |
  //      D------E
  vl::ring_bg_t ring{{node_a.lng() + 0.1 * dx, node_a.lat() - 0.01 * dy},
                     {node_a.lng() + 0.1 * dx, node_a.lat() - 0.1 * dy},
                     {node_a.lng() - 0.1 * dx, node_a.lat() - 0.1 * dy},
                     {node_a.lng() - 0.1 * dx, node_a.lat() - 0.01 * dy},
                     {node_a.lng() + 0.1 * dx, node_a.lat() - 0.01 * dy}};

  std::ostringstream avoid;
  avoid << "[" << bg::dsv(ring, ", ", "[", "]", ", ", "[", "]") << "]";

  const std::unordered_map<std::string, std::string>& req_options = {
      {"/avoid_polygons", avoid.str()}};

  // will avoid 1st
  auto route = gurka::do_action(Options::route, avoid_map, {"A", "D"}, GetParam(), req_options);
  gurka::assert::raw::expect_path(route, {"High", "2nd", "Low"});
}

TEST_P(AvoidTest, TestAvoid2Polygons) {
  auto node_a = avoid_map.nodes.at("A");
  auto node_b = avoid_map.nodes.at("B");
  auto node_e = avoid_map.nodes.at("E");
  auto dx = std::abs(node_b.lng() - node_a.lng());
  auto dy = std::abs(node_b.lat() - node_e.lat());

  // create 2 small polygons intersecting all connecting roads so it fails to find a path
  //      A------B
  //  x---|-x  y-|---y
  //  |   |    | |   |
  //  x---|-x  y-|---y
  //      |      |
  //      D------E
  vl::ring_bg_t ring1{{node_a.lng() + 0.1 * dx, node_a.lat() - 0.01 * dy},
                      {node_a.lng() + 0.1 * dx, node_a.lat() - 0.1 * dy},
                      {node_a.lng() - 0.1 * dx, node_a.lat() - 0.1 * dy},
                      {node_a.lng() - 0.1 * dx, node_a.lat() - 0.01 * dy},
                      {node_a.lng() + 0.1 * dx, node_a.lat() - 0.01 * dy}};
  vl::ring_bg_t ring2{{node_b.lng() - 0.1 * dx, node_b.lat() - 0.01 * dy},
                      {node_b.lng() + 0.1 * dx, node_b.lat() - 0.01 * dy},
                      {node_b.lng() + 0.1 * dx, node_b.lat() - 0.1 * dy},
                      {node_b.lng() - 0.1 * dx, node_b.lat() - 0.1 * dy},
                      {node_b.lng() - 0.1 * dx, node_b.lat() - 0.01 * dy}};

  std::ostringstream avoid;
  avoid << "[" << bg::dsv(ring1, ", ", "[", "]", ", ", "[", "]") << ","
        << bg::dsv(ring2, ", ", "[", "]", ", ", "[", "]") << "]";

  const std::unordered_map<std::string, std::string>& req_options = {
      {"/avoid_polygons", avoid.str()}};

  // make sure the right exception is thrown
  try {
    gurka::do_action(Options::route, avoid_map, {"A", "D"}, GetParam(), req_options);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

/*
Fails right now in the options parsing in gurka.cc

TEST_P(AvoidTest, TestAvoidLocation) {
  auto node_a = avoid_map.nodes.at("A");
  auto node_b = avoid_map.nodes.at("D");
  auto dx = std::abs(node_b.lng() - node_a.lng());

  auto lon = node_a.lng() - 0.5 * dx;

  // avoid the "High" road ;)
  //      A---x--B
  //      |      |
  //      |      |
  //      |      |
  //      D------E

  const std::unordered_map<std::string, std::string>& req_options = {
      {"/avoid_locations", "{[\"lat\": " + std::to_string(node_a.lat()) + ", \"lon\": " +
std::to_string(lon) + "]}"}};

  // will avoid 1st
  auto route = gurka::do_action(Options::route, avoid_map, {"A", "B"}, GetParam(), req_options);
  gurka::assert::raw::expect_path(route, {"1st", "Low", "2nd"});
}
*/

INSTANTIATE_TEST_SUITE_P(AvoidPolyProfilesTest,
                         AvoidTest,
                         ::testing::Values("auto",
                                           "truck",
                                           "bicycle",
                                           "pedestrian",
                                           "motorcycle",
                                           "motor_scooter",
                                           "hov",
                                           "taxi",
                                           "bus"));
