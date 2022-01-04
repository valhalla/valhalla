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
#include <valhalla/proto/options.pb.h>

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

std::unordered_set<valhalla::baldr::GraphId> get_edges(gurka::map map, std::string nodes) {
  valhalla::Options options;
  options.set_costing(valhalla::Costing::auto_);
  auto* co = options.add_costing_options();
  co->set_costing(valhalla::Costing::auto_);

  const auto costing = valhalla::sif::CostFactory{}.Create(*co);
  GraphReader reader(map.config.get_child("mjolnir"));

  auto* exclude_polygons = options.mutable_exclude_polygons();

  google::protobuf::RepeatedPtrField<valhalla::Options::Ring> rings;

  auto* ring = rings.Add();

  // create the chinese postman polygon
  // auto* ring = options.mutable_chinese_polygon();
  std::list<valhalla::midgard::PointLL> coords;
  for (auto& c : nodes) {
    coords.push_back(map.nodes[std::string(1, c)]);
  }

  for (const auto& coord : coords) {
    auto* ll = ring->add_coords();
    ll->set_lat(coord.lat());
    ll->set_lng(coord.lng());
  }
  return vl::edges_in_rings(rings, reader, costing, 10000, "chinese_postman");
  // return vl::edges_in_ring(*ring, reader, costing, 10000);
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
  // Skip if empty to avoid segmentation fault
  if (avoid_polygons.Size() > 0) {
    if (avoid_polygons[0].Size() > 0) {
      rapidjson::SetValueByPointer(doc, "/avoid_polygons", avoid_polygons);
    }
  }

  rapidjson::StringBuffer sb;
  rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
  doc.Accept(writer);
  return sb.GetString();
}

// common method can't deal with arrays of floats
std::string build_local_req_route(rapidjson::Document& doc,
                                  rapidjson::MemoryPoolAllocator<>& allocator,
                                  const std::vector<midgard::PointLL>& waypoints,
                                  const std::string& costing,
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

  // Skip if empty to avoid segmentation fault
  if (avoid_polygons.Size() > 0) {
    if (avoid_polygons[0].Size() > 0) {
      rapidjson::SetValueByPointer(doc, "/avoid_polygons", avoid_polygons);
    }
  }

  rapidjson::StringBuffer sb;
  rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
  doc.Accept(writer);
  return sb.GetString();
}

std::string build_local_req_matrix(rapidjson::Document& doc,
                                   rapidjson::MemoryPoolAllocator<>& allocator,
                                   const std::vector<midgard::PointLL>& sources,
                                   const std::vector<midgard::PointLL>& targets,
                                   const std::string& costing) {

  rapidjson::Value source_locations(rapidjson::kArrayType);
  for (const auto& source : sources) {
    rapidjson::Value p(rapidjson::kObjectType);
    p.AddMember("lon", source.lng(), allocator);
    p.AddMember("lat", source.lat(), allocator);
    source_locations.PushBack(p, allocator);
  }

  rapidjson::Value target_locations(rapidjson::kArrayType);
  for (const auto& target : targets) {
    rapidjson::Value p(rapidjson::kObjectType);
    p.AddMember("lon", target.lng(), allocator);
    p.AddMember("lat", target.lat(), allocator);
    target_locations.PushBack(p, allocator);
  }

  doc.AddMember("sources", source_locations, allocator);
  doc.AddMember("targets", target_locations, allocator);
  doc.AddMember("costing", costing, allocator);

  rapidjson::StringBuffer sb;
  rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
  doc.Accept(writer);
  return sb.GetString();
}

void test_request_matrix(const gurka::map& map,
                         const std::string& costing,
                         const std::string& sources_string,
                         const std::string& target_string) {

  std::vector<valhalla::midgard::PointLL> sources;
  for (auto& c : sources_string) {
    sources.push_back(map.nodes.at(std::string(1, c)));
  }

  std::vector<valhalla::midgard::PointLL> targets;
  for (auto& c : target_string) {
    targets.push_back(map.nodes.at(std::string(1, c)));
  }

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto req = build_local_req_matrix(doc, allocator, sources, targets, costing);
  auto matrix_result = gurka::do_action(Options::sources_to_targets, map, req);
}
ring_bg_t create_ring_from_string(gurka::map map, const std::string& polygon_string) {
  ring_bg_t ring{};
  for (auto& c : polygon_string) {
    ring.push_back(map.nodes[std::string(1, c)]);
  }
  return ring;
}

void test_request_route(const gurka::map& map,
                        const std::string& costing,
                        const std::string& exclude_polygon_string,
                        const std::string& start_node,
                        const std::string& end_node,
                        const std::vector<std::string>& expected_names) {

  // exclude ring can have more than one, but for now, use one only
  ring_bg_t exclude_ring = create_ring_from_string(map, exclude_polygon_string);

  std::vector<ring_bg_t> exclude_rings;
  exclude_rings.push_back(exclude_ring);

  // build request manually for now
  auto lls = {map.nodes.at(start_node), map.nodes.at(end_node)};

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto exclude_polygons = get_avoid_polys(exclude_rings, allocator);
  auto req = build_local_req_route(doc, allocator, lls, costing, exclude_polygons);

  auto route = gurka::do_action(Options::route, map, req);
  gurka::assert::raw::expect_path(route, expected_names);
}

valhalla::Api request_cp(const gurka::map& map,
                         const std::string& costing,
                         const std::string& chinese_polygon_string,
                         const std::string& exclude_polygon_string,
                         const std::string& start_node,
                         const std::string& end_node) {

  ring_bg_t chinese_ring = create_ring_from_string(map, chinese_polygon_string);
  // exclude ring can have more than one, but for now, use one only
  ring_bg_t exclude_ring = create_ring_from_string(map, exclude_polygon_string);

  std::vector<ring_bg_t> exclude_rings;
  exclude_rings.push_back(exclude_ring);

  // build request manually for now
  // If statements below can be put as a function
  std::vector<midgard::PointLL> lls;
  if (start_node.size() == 1) {
    lls.push_back(map.nodes.at(start_node));
  } else if (start_node.size() == 2) {
    auto sn0 = map.nodes.at(std::string(1, start_node.at(0)));
    auto sn1 = map.nodes.at(std::string(1, start_node.at(1)));
    auto sn_lat = sn0.lat() + (sn1.lat() - sn0.lat()) / 2.0;
    auto sn_lng = sn0.lng() + (sn1.lng() - sn0.lng()) / 2.0;
    auto sn = PointLL(sn_lng, sn_lat);

    lls.push_back(sn);
  }
  if (end_node.size() == 1) {
    lls.push_back(map.nodes.at(end_node));
  } else if (end_node.size() == 2) {
    auto en0 = map.nodes.at(std::string(1, end_node.at(0)));
    auto en1 = map.nodes.at(std::string(1, end_node.at(1)));
    auto en_lat = en0.lat() + (en1.lat() - en0.lat()) / 2.0;
    auto en_lng = en0.lng() + (en1.lng() - en0.lng()) / 2.0;
    auto en = PointLL(en_lng, en_lat);

    lls.push_back(en);
  }

  if (lls.size() != 2) {
    throw std::logic_error("There must be 2 elements.");
  }

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto chinese_polygon = get_chinese_polygon(chinese_ring, allocator);
  auto exclude_polygons = get_avoid_polys(exclude_rings, allocator);
  auto req = build_local_req(doc, allocator, lls, costing, chinese_polygon, exclude_polygons);

  return gurka::do_action(Options::chinese_postman, map, req);
}

void test_request(const gurka::map& map,
                  const std::string& costing,
                  const std::string& chinese_polygon_string,
                  const std::string& exclude_polygon_string,
                  const std::string& start_node,
                  const std::string& end_node,
                  const std::vector<std::string>& expected_names) {
  auto route =
      request_cp(map, costing, chinese_polygon_string, exclude_polygon_string, start_node, end_node);
  gurka::assert::raw::expect_path(route, expected_names);
}

void test_request_multi_results(const gurka::map& map,
                                const std::string& costing,
                                const std::string& chinese_polygon_string,
                                const std::string& exclude_polygon_string,
                                const std::string& start_node,
                                const std::string& end_node,
                                const std::vector<std::vector<std::string>>& expected_names) {
  auto route =
      request_cp(map, costing, chinese_polygon_string, exclude_polygon_string, start_node, end_node);
  gurka::assert::raw::expect_path_optional(route, expected_names);
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
    // AB_2 means AB road is a two way road.
    const gurka::ways ways = {// two ways
                              {"AB", {{"highway", "residential"}, {"name", "AB_2"}}},
                              {"AD", {{"highway", "residential"}, {"name", "AD_2"}}},
                              {"BE", {{"highway", "residential"}, {"name", "BE_2"}}},
                              {"DE", {{"highway", "residential"}, {"name", "DE_2"}}},
                              {"EF", {{"highway", "residential"}, {"name", "EF_2"}}},
                              // one way
                              {"CB", {{"highway", "residential"}, {"name", "CB"}, {"oneway", "yes"}}},
                              {"FC", {{"highway", "residential"}, {"name", "FC"}, {"oneway", "yes"}}},
                              {"CG", {{"highway", "residential"}, {"name", "CG"}, {"oneway", "yes"}}},
                              {"GH", {{"highway", "residential"}, {"name", "GH"}, {"oneway", "yes"}}},
                              {"HF",
                               {{"highway", "residential"}, {"name", "HF"}, {"oneway", "yes"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
    // Add low length limit for avoid_polygons so it throws an error
    chinese_postman_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_chinese_postman",
                                            {{"service_limits.max_exclude_polygons_length", "1000"},
                                             {"service_limits.max_chinese_polygon_length", "1000"}});

    // Setup complex_chinese_postman_map "AB", "BC", "CD", "DE", "EA"
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
            p----------------x----------q
            |    B--------A--|------F   |
            |     \       | \|     /|   |
            |      \      |  |\   / |   |
            |       \     |  | \ /  |   |
            |        \    |  |  E   |   |
            |         \   |  |   \  |   |
            |          \  |  |    \ |   |
            |           \ |  |     \|   |
            |             C--|------D   |
            r----------------y----------s
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
                          {{"service_limits.max_exclude_polygons_length", "1000"},
                           {"service_limits.max_chinese_polygon_length", "1000"}});
  }
};

gurka::map ChinesePostmanTest::chinese_postman_map = {};
gurka::map ChinesePostmanTest::complex_chinese_postman_map = {};

TEST_P(ChinesePostmanTest, TestMaxChinesePolygonPerimeter) {
  // Add a polygon with longer perimeter than the limit
  ring_bg_t chinese_ring{{13.38625361, 52.4652558},
                         {13.38625361, 52.48000128},
                         {13.4181769, 52.48000128},
                         {13.4181769, 52.4652558}};

  ring_bg_t exclude_ring = create_ring_from_string(chinese_postman_map, "");
  std::vector<ring_bg_t> exclude_rings;
  exclude_rings.push_back(exclude_ring);

  auto lls = {chinese_postman_map.nodes["A"], chinese_postman_map.nodes["C"]};

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto chinese_polygon = get_chinese_polygon(chinese_ring, allocator);
  auto exclude_polygons = get_avoid_polys(exclude_rings, allocator);
  auto req = build_local_req(doc, allocator, lls, GetParam(), chinese_polygon, exclude_polygons);

  // make sure the right exception is thrown
  try {
    gurka::do_action(Options::route, chinese_postman_map, req);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 173); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_F(ChinesePostmanTest, TestChinesePostmanEdges) {
  ASSERT_EQ(get_edges(chinese_postman_map, "styx").size(), 1);  // a one-way
  ASSERT_EQ(get_edges(chinese_postman_map, "rsxw").size(), 1);  // a one-way
  ASSERT_EQ(get_edges(chinese_postman_map, "rtyw").size(), 4);  // 4 one-ways
  ASSERT_EQ(get_edges(chinese_postman_map, "pqvu").size(), 2);  // a two-ways
  ASSERT_EQ(get_edges(chinese_postman_map, "prwu").size(), 8);  // 4 two-ways
  ASSERT_EQ(get_edges(chinese_postman_map, "qsxv").size(), 6);  // 2 two-ways and 2 one-ways
  ASSERT_EQ(get_edges(chinese_postman_map, "ptyu").size(), 15); // 5 two-ways and 5 one-ways
}

TEST_P(ChinesePostmanTest, TestChinesePostmanSimple) {
  // create a chinese polygon (prwu)
  test_request(chinese_postman_map, GetParam(), "prwu", "ijml", "A", "A",
               {"AB_2", "BE_2", "DE_2", "DE_2", "BE_2", "AB_2"});
  test_request(chinese_postman_map, GetParam(), "prwu", "ijml", "B", "B",
               {"AB_2", "AB_2", "BE_2", "DE_2", "DE_2", "BE_2"});
}

TEST_P(ChinesePostmanTest, TestChinesePostmanOneWayIdealGraph) {
  // create a chinese polygon (rtyw)
  test_request(chinese_postman_map, GetParam(), "rtyw", "", "C", "C", {"CG", "GH", "HF", "FC"});
  test_request(chinese_postman_map, GetParam(), "rtyw", "", "G", "G", {"GH", "HF", "FC", "CG"});
}

TEST_P(ChinesePostmanTest, TestChinesePostmanUnbalancedNodes) {
  // create a chinese polygon (qsxv)
  test_request_multi_results(chinese_postman_map, GetParam(), "qsxv", "", "B", "B",
                             {
                                 {"BE_2", "EF_2", "FC", "CB", "BE_2", "EF_2", "EF_2",
                                  "BE_2"}, // ubuntu
                                 {"BE_2", "EF_2", "EF_2", "BE_2", "BE_2", "EF_2", "FC", "CB"} // osx
                             });
  test_request_multi_results(chinese_postman_map, GetParam(), "qsxv", "", "F", "F",
                             {{"FC", "CB", "BE_2", "EF_2", "EF_2", "BE_2", "BE_2", "EF_2"}, // ubuntu
                              {"EF_2", "BE_2", "BE_2", "EF_2", "FC", "CB", "BE_2", "EF_2"}} // osx
  );
}

TEST_P(ChinesePostmanTest, TestChinesePostmanUnbalancedNodesComplex) {
  // create a chinese polygon (pqsr)
  test_request(complex_chinese_postman_map, GetParam(), "pqsr", "", "B", "B",
               {"BC", "CD", "DE", "EA", "AC", "CD", "DE", "EA", "AF", "FD", "DE", "EA", "AF", "FE",
                "EA", "AB"});
  test_request(complex_chinese_postman_map, GetParam(), "pqsr", "", "C", "C",
               {"CD", "DE", "EA", "AB", "BC", "CD", "DE", "EA", "AF", "FD", "DE", "EA", "AF", "FE",
                "EA", "AC"});
}

TEST_P(ChinesePostmanTest, TestChinesePostmanOriginOutside) {
  // create a chinese polygon (qsxv)
  try {
    test_request(chinese_postman_map, GetParam(), "qsxv", "", "A", "A",
                 {"BE_2", "EF_2", "FC", "CB", "BE_2", "EF_2", "EF_2", "BE_2"});
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 451); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
  try {
    test_request(chinese_postman_map, GetParam(), "qsxv", "", "q", "q",
                 {"BE_2", "EF_2", "FC", "CB", "BE_2", "EF_2", "EF_2", "BE_2"});
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 451); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_P(ChinesePostmanTest, TestChinesePostmanDifferentOriginDestination) {
  // A very simple example, only a one-way road is possible, ideal graph
  test_request(chinese_postman_map, GetParam(), "styx", "", "G", "H", {"GH"});

  // A little more complex example, only a two-way road is possible, non-ideal graph
  test_request(chinese_postman_map, GetParam(), "pqvu", "", "A", "D", {"AD_2", "AD_2", "AD_2"});

  // A little more complex example, 4 two-way roads are possible, non-ideal graph
  test_request(chinese_postman_map, GetParam(), "rtyw", "", "C", "H",
               {"CG", "GH", "HF", "FC", "CG", "GH"});

  // A more complex example, non-ideal graph
  test_request(complex_chinese_postman_map, GetParam(), "pqsr", "", "F", "E",
               {"FD", "DE", "EA", "AB", "BC", "CD", "DE", "EA", "AC", "CD", "DE", "EA", "AF", "FE"});
}

TEST_P(ChinesePostmanTest, TestChinesePostmanOutsidePolygon) {
  // test_request(chinese_postman_map, GetParam(), "prwu", "iknl", "D", "A", {"GH"});
  test_request(complex_chinese_postman_map, GetParam(), "xqsy", "", "F", "E",
               {"FD", "DE", "EA", "AF", "FE"});
  test_request(complex_chinese_postman_map, GetParam(), "xqsy", "", "E", "F",
               {"EA", "AF", "FD", "DE", "EA", "AF", "FE", "EA", "AF"});
  test_request(complex_chinese_postman_map, GetParam(), "pxyr", "", "A", "C",
               {"AB", "BC", "CD", "DE", "EA", "AC"});
}

TEST_P(ChinesePostmanTest, TestChinesePostmanMiddleEdge) {

  // create a chinese polygon (prwu)
  test_request(chinese_postman_map, GetParam(), "prwu", "ijml", "AB", "AB",
               {"AB_2", "BE_2", "DE_2", "DE_2", "BE_2", "AB_2"});
}

TEST_P(ChinesePostmanTest, TestRoute) {
  test_request_route(complex_chinese_postman_map, GetParam(), "", "E", "D",
                     {
                         "EA",
                         "AC",
                         "CD",
                     });
  test_request_route(complex_chinese_postman_map, GetParam(), "", "C", "A", {"CD", "DE", "EA"});
}

TEST_P(ChinesePostmanTest, TestChinesePostmanMatrix) {
  // Merely testing that the cost matrix is running properly
  test_request_matrix(chinese_postman_map, GetParam(), "GHFEDCBA", "GHFEDCBA");
}

INSTANTIATE_TEST_SUITE_P(
    ChinesePostmanProfilesTest,
    ChinesePostmanTest,
    ::testing::Values("auto", "truck", "motorcycle", "motor_scooter", "hov", "taxi", "bus"));
