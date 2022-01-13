#include "gurka.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/multi/geometries/register/multi_polygon.hpp>
#include <gtest/gtest.h>
#include <valhalla/proto/options.pb.h>

#include "baldr/graphconstants.h"
#include "baldr/graphreader.h"
#include "loki/polygon_search.h"
#include "midgard/pointll.h"
#include "mjolnir/graphtilebuilder.h"
#include "sif/costfactory.h"
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

  rapidjson::Pointer(type).Set(doc, geom_obj);

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
    // several polygons and one location to avoid to reference in the tests
    const std::string ascii_map = R"(
                                     A---x--B
                                     |      |
                                   h---i  l---m
                                   | | |  | | |
                                   k---j  o---n
                                     |      |   p-q
                                     C------D---|-|---E---F
                                                s-r
                                     )";

    const gurka::ways ways = {{"AB", {{"highway", "tertiary"}, {"name", "High"}}},
                              {"CD", {{"highway", "tertiary"}, {"name", "Low"}}},
                              {"AC", {{"highway", "tertiary"}, {"name", "1st"}}},
                              {"BD", {{"highway", "tertiary"}, {"name", "2nd"}}},
                              {"DE", {{"highway", "tertiary"}, {"name", "2nd"}}},
                              {"EF", {{"highway", "tertiary"}, {"name", "2nd"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10, {1.0, 1.0});
    // Add low length limit for exclude_polygons so it throws an error
    avoid_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_avoids",
                                  {{"service_limits.max_exclude_polygons_length", "1000"}});
  }
};

gurka::map AvoidTest::avoid_map = {};

TEST_F(AvoidTest, TestMaxPolygonPerimeter) {
  // Add a polygon with longer perimeter than the limit
  std::vector<ring_bg_t> rings{{{13.38625361, 52.4652558},
                                {13.38625361, 52.48000128},
                                {13.4181769, 52.48000128},
                                {13.4181769, 52.4652558}}};

  auto lls = {avoid_map.nodes["A"], avoid_map.nodes["C"]};

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto value = get_avoid_polys(rings, allocator);
  auto req = build_local_req(doc, allocator, lls, "auto", value, "/exclude_polygons");

  // make sure the right exception is thrown
  try {
    gurka::do_action(Options::route, avoid_map, req);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 167); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_P(AvoidTest, TestAvoidPolygonWithDeprecatedParam) {
  // avoid the first polygon;
  // use deprecated "avoid_polygons" instead of "exclude_polygons"

  std::vector<ring_bg_t> rings{
      {avoid_map.nodes["h"], avoid_map.nodes["i"], avoid_map.nodes["j"], avoid_map.nodes["k"]}};

  // build request manually for now
  auto lls = {avoid_map.nodes["A"], avoid_map.nodes["C"]};

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto value = get_avoid_polys(rings, allocator);
  auto req = build_local_req(doc, allocator, lls, GetParam(), value, "/avoid_polygons");

  // will avoid 1st
  auto route = gurka::do_action(Options::route, avoid_map, req);
  gurka::assert::raw::expect_path(route, {"High", "2nd", "Low"});
}

TEST_P(AvoidTest, TestAvoid2Polygons) {
  // create 2 small polygons intersecting all connecting roads so it fails to find a path
  // one clockwise ring, one counter-clockwise ring
  std::vector<ring_bg_t> rings{{avoid_map.nodes["h"], avoid_map.nodes["i"], avoid_map.nodes["j"],
                                avoid_map.nodes["k"]},
                               {avoid_map.nodes["o"], avoid_map.nodes["n"], avoid_map.nodes["m"],
                                avoid_map.nodes["l"]}};

  // build request manually for now
  auto lls = {avoid_map.nodes["A"], avoid_map.nodes["C"]};

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto value = get_avoid_polys(rings, allocator);
  auto req = build_local_req(doc, allocator, lls, GetParam(), value, "/exclude_polygons");

  // make sure the right exception is thrown
  try {
    gurka::do_action(Options::route, avoid_map, req);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_F(AvoidTest, TestAvoidShortcutsTruck) {
  valhalla::Options options;
  options.set_costing_type(valhalla::Costing::truck);
  auto& co = (*options.mutable_costings())[Costing::truck];
  co.set_type(valhalla::Costing::truck);

  // create the polygon intersecting a shortcut
  auto* rings = options.mutable_exclude_polygons();
  auto* ring = rings->Add();
  for (const auto& coord :
       {avoid_map.nodes["p"], avoid_map.nodes["q"], avoid_map.nodes["r"], avoid_map.nodes["s"]}) {
    auto* ll = ring->add_coords();
    ll->set_lat(coord.lat());
    ll->set_lng(coord.lng());
  }

  const auto costing = valhalla::sif::CostFactory{}.Create(co);
  GraphReader reader(avoid_map.config.get_child("mjolnir"));

  // should return the shortcut edge ID as well
  size_t found_shortcuts = 0;
  auto avoid_edges = vl::edges_in_rings(*rings, reader, costing, 10000);
  for (const auto& edge_id : avoid_edges) {
    if (reader.GetGraphTile(edge_id)->directededge(edge_id)->is_shortcut()) {
      found_shortcuts++;
    }
  }

  // 2 shortcuts + 2 edges
  ASSERT_EQ(avoid_edges.size(), 4);
  ASSERT_EQ(found_shortcuts, 2);
}

TEST_P(AvoidTest, TestAvoidLocation) {
  // avoid the location on "High road"
  std::vector<vm::PointLL> avoid_locs{avoid_map.nodes["x"]};

  // build request manually for now
  auto lls = {avoid_map.nodes["A"], avoid_map.nodes["B"]};

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto value = get_avoid_locs(avoid_locs, allocator);
  auto req = build_local_req(doc, allocator, lls, GetParam(), value, "/exclude_locations");

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
                                           "taxi",
                                           "bus"));
