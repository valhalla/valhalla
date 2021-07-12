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
  static gurka::map complex_chinese_postman_map;

  //    A------B---<--C--->--G
  //    |      |      |      |
  //    |      |      ^      v
  //    |      |      |      |
  //    |      |      |      |
  //    D------E------F--<---H

  static void SetUpTestSuite() {
    // Setup chinese_postman_map
    const std::string ascii_map = R"(
    p---------q-------r------s-----t
    |    A----|--B----|--C---|--G  |
    | i--|--j-|--|--k |  |   |  |  |
    | |  |  | |  |  | |  |   |  |  |
    | l--|--m-|--|--n |  |   |  |  |
    |    |    |  |    |  |   |  |  |
    |    D----|--E----|--F---|--H  |
    u---------v-------w------x-----y
    )";
    const gurka::ways ways = {
        {"AB", {{"highway", "residential"}, {"name", "AB"}}},
        {"BA", {{"highway", "residential"}, {"name", "BA"}}},

        {"AD", {{"highway", "residential"}, {"name", "AD"}}},
        {"DA", {{"highway", "residential"}, {"name", "DA"}}},

        {"CB", {{"highway", "residential"}, {"name", "CB"}, {"oneway", "yes"}}},

        {"BE", {{"highway", "residential"}, {"name", "BE"}}},
        {"EB", {{"highway", "residential"}, {"name", "EB"}}},

        {"DE", {{"highway", "residential"}, {"name", "DE"}}},
        {"ED", {{"highway", "residential"}, {"name", "ED"}}},

        {"EF", {{"highway", "residential"}, {"name", "EF"}}},
        {"FE", {{"highway", "residential"}, {"name", "FE"}}},

        {"FC", {{"highway", "residential"}, {"name", "FC"}, {"oneway", "yes"}}},
        {"CG", {{"highway", "residential"}, {"name", "CG"}, {"oneway", "yes"}}},
        {"GH", {{"highway", "residential"}, {"name", "GH"}, {"oneway", "yes"}}},
        {"HF", {{"highway", "residential"}, {"name", "HF"}, {"oneway", "yes"}}},
    };
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
    // Add low length limit for avoid_polygons so it throws an error
    chinese_postman_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_chinese_postman",
                                            {{"service_limits.max_exclude_polygons_length", "1000"}});

    // Setup complex_chinese_postman_map
    // B----<---A--->----F
    //  \       | \     /|
    //   \      |  ^   v |
    //    \     v   \ /  |
    //     v    |    E   v
    //      \   |     \  |
    //       \  |      ^ |
    //        \ |       \|
    //          C---->---D

    const std::string complex_ascii_map = R"(
            p--------------------------q
            |    B--------A--------F   |
            |     \       | \     /|   |
            |      \      |  \   / |   |
            |       \     |   \ /  |   |
            |        \    |    E   |   |
            |         \   |     \  |   |
            |          \  |      \ |   |
            |           \ |       \|   |
            |             C--------D   |
            r--------------------------s
    )";
    const gurka::ways complex_ways =
        {{"AB", {{"highway", "residential"}, {"name", "AB"}, {"oneway", "yes"}}},
         {"AC", {{"highway", "residential"}, {"name", "AC"}, {"oneway", "yes"}}},
         {"AF", {{"highway", "residential"}, {"name", "AF"}, {"oneway", "yes"}}},

         {"BC", {{"highway", "residential"}, {"name", "BC"}, {"oneway", "yes"}}},

         {"CD", {{"highway", "residential"}, {"name", "CD"}, {"oneway", "yes"}}},

         {"DE", {{"highway", "residential"}, {"name", "DE"}, {"oneway", "yes"}}},

         {"EA", {{"highway", "residential"}, {"name", "EA"}, {"oneway", "yes"}}},

         {"FD", {{"highway", "residential"}, {"name", "FD"}, {"oneway", "yes"}}},
         {"FE", {{"highway", "residential"}, {"name", "FE"}, {"oneway", "yes"}}}};
    const auto complex_layout = gurka::detail::map_to_coordinates(complex_ascii_map, 10);
    // Add low length limit for avoid_polygons so it throws an error
    complex_chinese_postman_map =
        gurka::buildtiles(complex_layout, complex_ways, {}, {},
                          "test/data/gurka_complex_chinese_postman",
                          {{"service_limits.max_exclude_polygons_length", "1000"}});
  }
};

gurka::map ChinesePostmanTest::chinese_postman_map = {};
gurka::map ChinesePostmanTest::complex_chinese_postman_map = {};

TEST_P(ChinesePostmanTest, DISABLED_TestChinesePostmanSimple) {
  // create a chinese polygon (prwu) and avoid polygon (ijml)

  ring_bg_t chinese_ring{chinese_postman_map.nodes.at("p"), chinese_postman_map.nodes.at("r"),
                         chinese_postman_map.nodes.at("w"), chinese_postman_map.nodes.at("u")};

  ring_bg_t avoid_ring{chinese_postman_map.nodes.at("i"), chinese_postman_map.nodes.at("j"),
                       chinese_postman_map.nodes.at("m"), chinese_postman_map.nodes.at("l")};

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
  gurka::assert::raw::expect_path(route, {"AB", "BE", "ED", "DE", "EB", "BA"});

  // build request manually for now
  auto lls2 = {chinese_postman_map.nodes["D"], chinese_postman_map.nodes["D"]};

  rapidjson::Document doc2;
  doc2.SetObject();
  auto& allocator2 = doc2.GetAllocator();
  auto chinese_polygon2 = get_chinese_polygon(chinese_ring, allocator2);
  auto avoid_polygons2 = get_avoid_polys(avoid_rings, allocator2);
  auto req2 = build_local_req(doc2, allocator2, lls2, GetParam(), chinese_polygon2, avoid_polygons2);

  auto route2 = gurka::do_action(Options::chinese_postman, chinese_postman_map, req2);
  gurka::assert::raw::expect_path(route2, {"DE", "EB", "BA", "AB", "BE", "ED"});
}

TEST_P(ChinesePostmanTest, DISABLED_TestChinesePostmanNotConnected) {
  // create a chinese polygon (prwu) and avoid polygon (iknl)
  // the exclude polygon is dividing the map into two, that makes it not connected.

  ring_bg_t chinese_ring{chinese_postman_map.nodes.at("p"), chinese_postman_map.nodes.at("r"),
                         chinese_postman_map.nodes.at("w"), chinese_postman_map.nodes.at("u")};

  ring_bg_t avoid_ring{chinese_postman_map.nodes.at("i"), chinese_postman_map.nodes.at("k"),
                       chinese_postman_map.nodes.at("n"), chinese_postman_map.nodes.at("l")};

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

TEST_P(ChinesePostmanTest, DISABLED_TestChinesePostmanOneWayIdealGraph) {
  // create a chinese polygon (rtyw)

  ring_bg_t chinese_ring{chinese_postman_map.nodes.at("r"), chinese_postman_map.nodes.at("t"),
                         chinese_postman_map.nodes.at("y"), chinese_postman_map.nodes.at("w")};

  std::vector<ring_bg_t> avoid_rings;

  // build request manually for now
  auto lls = {chinese_postman_map.nodes["C"], chinese_postman_map.nodes["C"]};

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto chinese_polygon = get_chinese_polygon(chinese_ring, allocator);
  auto avoid_polygons = get_avoid_polys(avoid_rings, allocator);
  auto req = build_local_req(doc, allocator, lls, GetParam(), chinese_polygon, avoid_polygons);

  auto route = gurka::do_action(Options::chinese_postman, chinese_postman_map, req);
  gurka::assert::raw::expect_path(route, {"CG", "GH", "HF", "FC"});

  // // build request manually for now
  auto lls2 = {chinese_postman_map.nodes["G"], chinese_postman_map.nodes["G"]};

  rapidjson::Document doc2;
  doc2.SetObject();
  auto& allocator2 = doc2.GetAllocator();
  auto chinese_polygon2 = get_chinese_polygon(chinese_ring, allocator2);
  auto avoid_polygons2 = get_avoid_polys(avoid_rings, allocator2);
  auto req2 = build_local_req(doc2, allocator2, lls2, GetParam(), chinese_polygon2, avoid_polygons2);

  auto route2 = gurka::do_action(Options::chinese_postman, chinese_postman_map, req2);
  gurka::assert::raw::expect_path(route2, {"GH", "HF", "FC", "CG"});
}

TEST_P(ChinesePostmanTest, DISABLED_TestChinesePostmanUnbalancedNodes) {
  // create a chinese polygon (qsxv)

  ring_bg_t chinese_ring{chinese_postman_map.nodes.at("q"), chinese_postman_map.nodes.at("s"),
                         chinese_postman_map.nodes.at("x"), chinese_postman_map.nodes.at("v")};

  std::vector<ring_bg_t> avoid_rings;

  // build request manually for now (only works for C and E)
  // B and F got this error: Could not find candidate edge used for destination label
  auto lls = {chinese_postman_map.nodes["C"], chinese_postman_map.nodes["C"]};

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto chinese_polygon = get_chinese_polygon(chinese_ring, allocator);
  auto avoid_polygons = get_avoid_polys(avoid_rings, allocator);
  auto req = build_local_req(doc, allocator, lls, GetParam(), chinese_polygon, avoid_polygons);

  auto route = gurka::do_action(Options::chinese_postman, chinese_postman_map, req);
  gurka::assert::raw::expect_path(route, {"CB", "BE", "EF", "FE", "EB", "BE", "EF", "FC"});
}

TEST_P(ChinesePostmanTest, TestChinesePostmanUnbalancedNodesComplex) {
  ring_bg_t chinese_ring{complex_chinese_postman_map.nodes.at("p"),
                         complex_chinese_postman_map.nodes.at("q"),
                         complex_chinese_postman_map.nodes.at("s"),
                         complex_chinese_postman_map.nodes.at("r")};

  std::vector<ring_bg_t> avoid_rings;

  // build request manually for now
  auto lls = {complex_chinese_postman_map.nodes["B"], complex_chinese_postman_map.nodes["B"]};

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto chinese_polygon = get_chinese_polygon(chinese_ring, allocator);
  auto avoid_polygons = get_avoid_polys(avoid_rings, allocator);
  auto req = build_local_req(doc, allocator, lls, GetParam(), chinese_polygon, avoid_polygons);

  auto route = gurka::do_action(Options::chinese_postman, complex_chinese_postman_map, req);
  gurka::assert::raw::expect_path(route, {"BC", "CD", "DE", "EA", "AF", "FD", "DE", "EA", "AC", "CD",
                                          "DE", "EA", "AF", "FE", "EA", "AB"});

  // build request manually for now
  auto lls2 = {complex_chinese_postman_map.nodes["C"], complex_chinese_postman_map.nodes["C"]};

  rapidjson::Document doc2;
  doc2.SetObject();
  auto& allocator2 = doc2.GetAllocator();
  auto chinese_polygon2 = get_chinese_polygon(chinese_ring, allocator2);
  auto avoid_polygons2 = get_avoid_polys(avoid_rings, allocator2);
  auto req2 = build_local_req(doc2, allocator2, lls2, GetParam(), chinese_polygon2, avoid_polygons2);

  auto route2 = gurka::do_action(Options::chinese_postman, complex_chinese_postman_map, req2);
  gurka::assert::raw::expect_path(route2, {"CD", "DE", "EA", "AF", "FD", "DE", "EA", "AC", "CD", "DE",
                                           "EA", "AF", "FE", "EA", "AB", "BC"});
}

INSTANTIATE_TEST_SUITE_P(ChinesePostmanProfilesTest, ChinesePostmanTest, ::testing::Values("auto"));
