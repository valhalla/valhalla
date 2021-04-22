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

rapidjson::Value get_chinese_poly(const std::vector<ring_bg_t>& rings,
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

  if (type == "chinese_polygon") {
    rapidjson::SetValueByPointer(doc, "/chinese_polygon", geom_obj);
  } else {
    rapidjson::SetValueByPointer(doc, "/chinese_polygon", geom_obj);
  }

  rapidjson::StringBuffer sb;
  rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
  doc.Accept(writer);
  return sb.GetString();
}
} // namespace

// parameterized test class to test all costings
class ChinesePostmanTest : public ::testing::TestWithParam<std::string> {
protected:
  static gurka::map chinese_postman_map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
        B------A------C
        |      |    / |
        |      |   /  |
        |      |  /   |
        |      | /    |
        D------E------F
    )";
    const gurka::ways ways = {{"AB", {{"highway", "residential"}, {"name", "High"}}},
                              {"AC", {{"highway", "residential"}, {"name", "Low"}}},
                              {"AE", {{"highway", "residential"}, {"name", "1st"}}},
                              {"BD", {{"highway", "residential"}, {"name", "2nd"}}},
                              {"CE", {{"highway", "residential"}, {"name", "3rd"}}},
                              {"CF", {{"highway", "residential"}, {"name", "4th"}}},
                              {"DE", {{"highway", "residential"}, {"name", "5th"}}},
                              {"EF", {{"highway", "residential"}, {"name", "6th"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
    // Add low length limit for avoid_polygons so it throws an error
    chinese_postman_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_chinese_postman",
                                            {{"service_limits.max_avoid_polygons_length", "1000"}});
  }
};

gurka::map ChinesePostmanTest::chinese_postman_map = {};

TEST_F(ChinesePostmanTest, TestConfig) {
  // Add a polygon with longer perimeter than the limit
  std::vector<ring_bg_t> rings{{{13.38625361, 52.4652558},
                                {13.38625361, 52.48000128},
                                {13.4181769, 52.48000128},
                                {13.4181769, 52.4652558}}};

  auto lls = {chinese_postman_map.nodes["A"], chinese_postman_map.nodes["D"]};

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto value = get_chinese_poly(rings, allocator);
  auto type = "chinese_polygon";
  auto req = build_local_req(doc, allocator, lls, "auto", value, type);

  // make sure the right exception is thrown
  try {
    gurka::do_action(Options::chinese_postman, chinese_postman_map, req);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 167); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_P(ChinesePostmanTest, TestChinesePostmanSimple) {
  auto node_a = chinese_postman_map.nodes.at("A");
  auto node_b = chinese_postman_map.nodes.at("B");
  auto node_d = chinese_postman_map.nodes.at("D");
  auto node_e = chinese_postman_map.nodes.at("E");

  auto dx = node_b.lng() - node_a.lng();
  auto dy = node_a.lat() - node_d.lat();

  // create a polygon covering ABDE
  //   x-------------x
  //   |  B------A---|--C
  //   |  |      |   | /|
  //   |  |      |   |/ |
  //   |  |      |  /|  |
  //   |  |      | / |  |
  //   |  D------E---|--F
  //   x-------------x
  //
  ring_bg_t ring{{node_b.lng() - 0.1 * dx, node_b.lat() + 0.1 * dy},
                 {node_a.lng() + 0.1 * dx, node_a.lat() + 0.1 * dy},
                 {node_e.lng() + 0.1 * dx, node_a.lat() - 0.1 * dy},
                 {node_d.lng() - 0.1 * dx, node_a.lat() - 0.1 * dy},
                 {node_b.lng() - 0.1 * dx, node_b.lat() + 0.1 * dy}};
  std::vector<ring_bg_t> rings;
  rings.push_back(ring);

  // build request manually for now
  auto lls = {chinese_postman_map.nodes["A"], chinese_postman_map.nodes["A"]};

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto value = get_chinese_poly(rings, allocator);
  auto type = "chinese_polygon";
  auto req = build_local_req(doc, allocator, lls, GetParam(), value, type);

  // will avoid 1st
  auto route = gurka::do_action(Options::chinese_postman, chinese_postman_map, req);
  gurka::assert::raw::expect_path(route, {"High", "2nd", "Low"});
}

INSTANTIATE_TEST_SUITE_P(ChinesePostmanProfilesTest, ChinesePostmanTest, ::testing::Values("auto"));
