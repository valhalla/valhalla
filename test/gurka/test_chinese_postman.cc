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

rapidjson::Value get_chinese_polygon(ring_bg_t ring, rapidjson::MemoryPoolAllocator<>& allocator) {
  rapidjson::Value ring_j(rapidjson::kArrayType);
  for (auto& coord : ring) {
    rapidjson::Value coords(rapidjson::kArrayType);
    coords.PushBack(coord.lng(), allocator);
    coords.PushBack(coord.lat(), allocator);
    ring_j.PushBack(coords, allocator);
  }

  return ring_j;
}

// common method can't deal with arrays of floats
std::string build_local_req(rapidjson::Document& doc,
                            rapidjson::MemoryPoolAllocator<>& allocator,
                            const std::vector<midgard::PointLL>& waypoints,
                            const std::string& costing,
                            const rapidjson::Value& chinese_polygon,
                            const rapidjson::Value& avoid_polygons) {

  rapidjson::Value locations(rapidjson::kArrayType);
  for (const auto& waypoint : waypoints) {
    rapidjson::Value p(rapidjson::kObjectType);
    p.AddMember("lon", waypoint.lng(), allocator);
    p.AddMember("lat", waypoint.lat(), allocator);
    locations.PushBack(p, allocator);
  }

  doc.AddMember("locations", locations, allocator);
  doc.AddMember("costing", costing, allocator);

  rapidjson::SetValueByPointer(doc, "/chinese_postman_polygon", chinese_polygon);
  rapidjson::SetValueByPointer(doc, "/avoid_polygons", avoid_polygons);

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
        A------B------C
        |      |      |
        |      |      |
        |      |      |
        |      |      |
        D------E------F
    )";
    const gurka::ways ways = {{"AB", {{"highway", "residential"}, {"name", "High"}}},
                              {"AD", {{"highway", "residential"}, {"name", "Low"}}},
                              {"BC", {{"highway", "residential"}, {"name", "1st"}}},
                              {"BE", {{"highway", "residential"}, {"name", "2nd"}}},
                              {"CF", {{"highway", "residential"}, {"name", "3rd"}}},
                              {"DE", {{"highway", "residential"}, {"name", "4th"}}},
                              {"EF", {{"highway", "residential"}, {"name", "5th"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
    // Add low length limit for avoid_polygons so it throws an error
    chinese_postman_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_chinese_postman",
                                            {{"service_limits.max_avoid_polygons_length", "1000"}});
  }
};

gurka::map ChinesePostmanTest::chinese_postman_map = {};

TEST_P(ChinesePostmanTest, TestChinesePostmanSimple) {
  auto node_a = chinese_postman_map.nodes.at("A");
  auto node_b = chinese_postman_map.nodes.at("B");
  auto node_c = chinese_postman_map.nodes.at("C");
  auto node_d = chinese_postman_map.nodes.at("D");
  auto node_e = chinese_postman_map.nodes.at("E");
  auto node_f = chinese_postman_map.nodes.at("F");

  auto c_b = node_c.lng() - node_b.lng();
  auto b_a = node_b.lng() - node_a.lng();
  auto a_d = node_a.lat() - node_d.lat();

  // create a chinese polygon covering ABDE and avoid polygon covering AD
  //   c---------------c
  //   |    A------B---|--C
  //   | a--|--a   |   |  |
  //   | |  |  |   |   |  |
  //   | a--|--a   |   |  |
  //   |    |      |   |  |
  //   |    D------E---|--F
  //   c---------------c

  auto ratio = 0.2;
  ring_bg_t chinese_ring{{node_b.lng() + ratio * c_b, node_b.lat() + ratio * a_d},
                         {node_e.lng() + ratio * c_b, node_e.lat() - ratio * a_d},
                         {node_d.lng() - ratio * c_b, node_d.lat() - ratio * a_d},
                         {node_a.lng() - ratio * c_b, node_a.lat() + ratio * a_d},
                         {node_b.lng() + ratio * c_b, node_b.lat() + ratio * a_d}};

  auto avoid_ratio = 0.1;
  auto small_avoid_ratio = 0.01;
  ring_bg_t avoid_ring{
      {node_a.lng() + avoid_ratio * b_a, node_a.lat() - small_avoid_ratio * a_d},
      {node_a.lng() + avoid_ratio * b_a, node_a.lat() - avoid_ratio * a_d},
      {node_a.lng() - avoid_ratio * b_a, node_a.lat() - avoid_ratio * a_d},
      {node_a.lng() - avoid_ratio * b_a, node_a.lat() - small_avoid_ratio * a_d},
      {node_a.lng() + avoid_ratio * b_a, node_a.lat() - small_avoid_ratio * a_d},
  };

  std::vector<ring_bg_t> avoid_rings;
  avoid_rings.push_back(avoid_ring);

  // build request manually for now
  auto lls = {chinese_postman_map.nodes["A"], chinese_postman_map.nodes["A"]};

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto chinese_polygon = get_chinese_polygon(chinese_ring, allocator);
  auto avoid_polygons = get_avoid_polys(avoid_rings, allocator);
  auto req = build_local_req(doc, allocator, lls, GetParam(), chinese_polygon, avoid_polygons);

  auto route = gurka::do_action(Options::chinese_postman, chinese_postman_map, req);
  // gurka::assert::raw::expect_path(route, {"High", "Low", "5th", "2nd"});
}

TEST_P(ChinesePostmanTest, TestChinesePostmanNotConnected) {
  std::cout << ">>>>>> TestChinesePostmanNotConnected" << std::endl;
  auto node_a = chinese_postman_map.nodes.at("A");
  auto node_b = chinese_postman_map.nodes.at("B");
  auto node_c = chinese_postman_map.nodes.at("C");
  auto node_d = chinese_postman_map.nodes.at("D");
  auto node_e = chinese_postman_map.nodes.at("E");
  auto node_f = chinese_postman_map.nodes.at("F");

  auto c_b = node_c.lng() - node_b.lng();
  auto b_a = node_b.lng() - node_a.lng();
  auto a_d = node_a.lat() - node_d.lat();

  // create a chinese polygon covering ABDE and avoid polygon covering AD, BE
  //   c---------------c
  //   |    A------B---|--C
  //   | a--|------|-a |  |
  //   | |  |      | | |  |
  //   | a--|------|-a |  |
  //   |    |      |   |  |
  //   |    D------E---|--F
  //   c---------------c

  auto ratio = 0.2;
  ring_bg_t chinese_ring{{node_b.lng() + ratio * c_b, node_b.lat() + ratio * a_d},
                         {node_e.lng() + ratio * c_b, node_e.lat() - ratio * a_d},
                         {node_d.lng() - ratio * c_b, node_d.lat() - ratio * a_d},
                         {node_a.lng() - ratio * c_b, node_a.lat() + ratio * a_d},
                         {node_b.lng() + ratio * c_b, node_b.lat() + ratio * a_d}};

  auto avoid_ratio = 0.1;
  auto small_avoid_ratio = 0.01;
  ring_bg_t avoid_ring{
      {node_b.lng() + avoid_ratio * c_b, node_b.lat() - small_avoid_ratio * a_d},
      {node_b.lng() + avoid_ratio * c_b, node_b.lat() - avoid_ratio * a_d},
      {node_a.lng() - avoid_ratio * b_a, node_a.lat() - avoid_ratio * a_d},
      {node_a.lng() - avoid_ratio * b_a, node_a.lat() - small_avoid_ratio * a_d},
      {node_b.lng() + avoid_ratio * c_b, node_b.lat() - small_avoid_ratio * a_d},
  };

  std::vector<ring_bg_t> avoid_rings;
  avoid_rings.push_back(avoid_ring);

  // build request manually for now
  auto lls = {chinese_postman_map.nodes["A"], chinese_postman_map.nodes["A"]};

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto chinese_polygon = get_chinese_polygon(chinese_ring, allocator);
  auto avoid_polygons = get_avoid_polys(avoid_rings, allocator);
  auto req = build_local_req(doc, allocator, lls, GetParam(), chinese_polygon, avoid_polygons);

  // make sure the right exception is thrown
  try {
    gurka::do_action(Options::chinese_postman, chinese_postman_map, req);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 450); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

INSTANTIATE_TEST_SUITE_P(ChinesePostmanProfilesTest, ChinesePostmanTest, ::testing::Values("auto"));
