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
#include "worker.h"

using namespace valhalla;
namespace bg = boost::geometry;
namespace vm = valhalla::midgard;
namespace vl = valhalla::loki;

namespace {
rapidjson::Value get_avoid_locs(const std::vector<vm::PointLL>& locs,
                                rapidjson::MemoryPoolAllocator<>& allocator) {
  rapidjson::Value locs_j(rapidjson::kArrayType);
  for (auto& loc : locs) {
    rapidjson::Value p(rapidjson::kObjectType);
    p.AddMember("lon", loc.lng(), allocator);
    p.AddMember("lat", loc.lat(), allocator);
    locs_j.PushBack(p, allocator);
  }

  return locs_j;
}

rapidjson::Value get_avoid_polys(const std::vector<vl::ring_bg_t>& rings,
                                 rapidjson::MemoryPoolAllocator<>& allocator) {
  rapidjson::Value rings_j(rapidjson::kArrayType);
  for (auto& ring : rings) {
    rapidjson::Value ring_j(rapidjson::kArrayType);
    for (auto& coord : ring) {
      rapidjson::Value coords(rapidjson::kArrayType);
      coords.PushBack(coord.lng(), allocator);
      coords.PushBack(coord.lat(), allocator);
      ring_j.PushBack(coords, allocator);
    }
    rings_j.PushBack(ring_j, allocator);
  }

  return rings_j;
}

// common method can't deal with arrays of floats
std::string build_local_req(rapidjson::Document& doc,
                            rapidjson::MemoryPoolAllocator<>& allocator,
                            const std::vector<midgard::PointLL>& waypoints,
                            const std::string& costing,
                            const rapidjson::Value& geom_obj,
                            const std::string& type) {

  rapidjson::Value locations(rapidjson::kArrayType);
  for (const auto& waypoint : waypoints) {
    rapidjson::Value p(rapidjson::kObjectType);
    p.AddMember("lon", waypoint.lng(), allocator);
    p.AddMember("lat", waypoint.lat(), allocator);
    locations.PushBack(p, allocator);
  }

  doc.AddMember("locations", locations, allocator);
  doc.AddMember("costing", costing, allocator);

  if (type == "avoid_polygons") {
    rapidjson::SetValueByPointer(doc, "/avoid_polygons", geom_obj);
  } else {
    rapidjson::SetValueByPointer(doc, "/avoid_locations", geom_obj);
  }

  rapidjson::StringBuffer sb;
  rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
  doc.Accept(writer);
  return sb.GetString();
}
} // namespace

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
      auto* ll = polygon->add_coords();
      ll->set_lng(coord.lng());
      ll->set_lat(coord.lat());
    }
    calc_length += vl::GetRingLength(vl::PBFToRing(*polygon));
  }

  EXPECT_EQ(actual_length, calc_length);
}

TEST_F(AvoidTest, TestConfig) {
  // Add a polygon with longer perimeter than the limit
  std::vector<vl::ring_bg_t> rings{{{13.38625361, 52.4652558},
                                    {13.38625361, 52.48000128},
                                    {13.4181769, 52.48000128},
                                    {13.4181769, 52.4652558}}};

  auto lls = gurka::detail::to_lls(avoid_map.nodes, {"A", "D"});

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto value = get_avoid_polys(rings, allocator);
  auto type = "avoid_polygons";
  auto req = build_local_req(doc, allocator, lls, "auto", value, type);

  // make sure the right exception is thrown
  try {
    gurka::do_action(Options::route, avoid_map, req);
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
  std::vector<vl::ring_bg_t> rings;
  rings.push_back(ring);

  // build request manually for now
  auto lls = gurka::detail::to_lls(avoid_map.nodes, {"A", "D"});

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto value = get_avoid_polys(rings, allocator);
  auto type = "avoid_polygons";
  auto req = build_local_req(doc, allocator, lls, GetParam(), value, type);

  // will avoid 1st
  auto route = gurka::do_action(Options::route, avoid_map, req);
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
  std::vector<vl::ring_bg_t> rings;
  rings.push_back({{node_a.lng() + 0.1 * dx, node_a.lat() - 0.01 * dy},
                   {node_a.lng() + 0.1 * dx, node_a.lat() - 0.1 * dy},
                   {node_a.lng() - 0.1 * dx, node_a.lat() - 0.1 * dy},
                   {node_a.lng() - 0.1 * dx, node_a.lat() - 0.01 * dy},
                   {node_a.lng() + 0.1 * dx, node_a.lat() - 0.01 * dy}});
  rings.push_back({{node_b.lng() - 0.1 * dx, node_b.lat() - 0.01 * dy},
                   {node_b.lng() + 0.1 * dx, node_b.lat() - 0.01 * dy},
                   {node_b.lng() + 0.1 * dx, node_b.lat() - 0.1 * dy},
                   {node_b.lng() - 0.1 * dx, node_b.lat() - 0.1 * dy},
                   {node_b.lng() - 0.1 * dx, node_b.lat() - 0.01 * dy}});

  // build request manually for now
  auto lls = gurka::detail::to_lls(avoid_map.nodes, {"A", "D"});

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto value = get_avoid_polys(rings, allocator);
  auto type = "avoid_polygons";
  auto req = build_local_req(doc, allocator, lls, GetParam(), value, type);

  // make sure the right exception is thrown
  try {
    gurka::do_action(Options::route, avoid_map, req);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_P(AvoidTest, TestAvoidLocation) {
  auto node_a = avoid_map.nodes.at("A");
  auto node_b = avoid_map.nodes.at("B");
  auto dx = std::abs(node_b.lng() - node_a.lng());

  auto lon = node_a.lng() + 0.5 * dx;

  // avoid the "High" road ;)
  //      A---x--B
  //      |      |
  //      |      |
  //      |      |
  //      D------E
  std::vector<vm::PointLL> points{{lon, node_a.lat()}};

  // build request manually for now
  auto lls = gurka::detail::to_lls(avoid_map.nodes, {"A", "B"});

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto value = get_avoid_locs(points, allocator);
  auto type = "avoid_locations";
  auto req = build_local_req(doc, allocator, lls, GetParam(), value, type);

  // will avoid 1st
  auto route = gurka::do_action(Options::route, avoid_map, req);
  gurka::assert::raw::expect_path(route, {"1st", "Low", "2nd"});
}

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
