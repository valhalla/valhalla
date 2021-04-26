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
// register a few boost.geometry types
using ring_bg_t = std::vector<vm::PointLL>;

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

rapidjson::Value get_avoid_polys(const std::vector<ring_bg_t>& rings,
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
    avoid_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_avoids",
                                  {{"service_limits.max_avoid_polygons_length", "1000"}});
  }
};

gurka::map AvoidTest::avoid_map = {};

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
  ring_bg_t ring{{node_a.lng() + 0.1 * dx, node_a.lat() - 0.01 * dy},
                 {node_a.lng() + 0.1 * dx, node_a.lat() - 0.1 * dy},
                 {node_a.lng() - 0.1 * dx, node_a.lat() - 0.1 * dy},
                 {node_a.lng() - 0.1 * dx, node_a.lat() - 0.01 * dy},
                 {node_a.lng() + 0.1 * dx, node_a.lat() - 0.01 * dy}};
  std::vector<ring_bg_t> rings;
  rings.push_back(ring);

  // build request manually for now
  auto lls = {avoid_map.nodes["A"], avoid_map.nodes["D"]};

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

INSTANTIATE_TEST_SUITE_P(AvoidPolyProfilesTest, AvoidTest, ::testing::Values("auto"));
