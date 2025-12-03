#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "exceptions.h"
#include "gurka.h"
#include "loki/polygon_search.h"
#include "loki/worker.h"
#include "midgard/pointll.h"
#include "proto/options.pb.h"
#include "sif/costfactory.h"

#include <boost/format.hpp>
#include <gtest/gtest.h>
#include <test.h>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::loki;

class LokiWorkerTest : public loki_worker_t {
public:
  using loki_worker_t::loki_worker_t;
  using loki_worker_t::parse_costing;
};

namespace {
// register a few boost.geometry types
using ring_bg_t = std::vector<PointLL>;

rapidjson::Value get_avoid_locs(const std::vector<PointLL>& locs,
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

rapidjson::Value build_exclude_level_polygons(const std::vector<ring_bg_t>& rings,
                                              const std::vector<std::vector<float>>& levels,
                                              rapidjson::MemoryPoolAllocator<>& alloc) {
  assert(rings.size() == levels.size());
  rapidjson::Value featurecollection_j(rapidjson::kObjectType);
  featurecollection_j.AddMember("type", "FeatureCollection", alloc);
  rapidjson::Value features_j(rapidjson::kArrayType);
  for (size_t i = 0; i < rings.size(); ++i) {
    // one GeoJSON feature per polygon
    rapidjson::Value feature_j(rapidjson::kObjectType);
    feature_j.AddMember("type", "Feature", alloc);

    rapidjson::Value properties_j(rapidjson::kObjectType);
    rapidjson::Value geometry_j(rapidjson::kObjectType);

    // one ring per feature
    rapidjson::Value ring_j(rapidjson::kArrayType);
    for (const auto& coord : rings.at(i)) {
      rapidjson::Value coords(rapidjson::kArrayType);
      coords.PushBack(coord.lng(), alloc);
      coords.PushBack(coord.lat(), alloc);
      ring_j.PushBack(coords, alloc);
    }

    // one level array per polygon
    rapidjson::Value level_j(rapidjson::kArrayType);
    for (const auto& level : levels.at(i)) {
      level_j.PushBack(level, alloc);
    }

    geometry_j.AddMember("type", "Polygon", alloc);
    geometry_j.AddMember("coordinates", ring_j, alloc);
    properties_j.AddMember("levels", level_j, alloc);
    feature_j.AddMember("properties", properties_j, alloc);
    feature_j.AddMember("geometry", geometry_j, alloc);
    features_j.PushBack(feature_j, alloc);
  }

  featurecollection_j.AddMember("features", features_j, alloc);
  return featurecollection_j;
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
                                                        t----w
                                                  a-----+----+----b
                                     A---x--B---G-+--H--+-I--+-J  |
                                     |      |     |  |  u----v |  |
                                   h---i  l---m   |  K----L----M  |
                                   | | |  | | |   d-------+----+--c
                                   k---j  o---n           |    |
                                     |      |   p-q       |    |
                                     C------D---|-|---E---F----N
                                                s-r
                                     )";

    const gurka::ways ways = {
        {"AB", {{"highway", "tertiary"}, {"name", "High"}}},
        {"CD", {{"highway", "tertiary"}, {"name", "Low"}}},
        {"AC", {{"highway", "tertiary"}, {"name", "1st"}}},
        {"BD", {{"highway", "tertiary"}, {"name", "2nd"}}},
        {"DE", {{"highway", "tertiary"}, {"name", "2nd"}}},
        {"EF", {{"highway", "tertiary"}, {"name", "2nd"}}},
        {"GB", {{"highway", "tertiary"}}},
        {"GH", {{"highway", "corridor"}, {"level", "5"}}},
        {"HI", {{"highway", "corridor"}, {"level", "2"}}},
        {"HK", {{"highway", "corridor"}, {"level", "3"}}},
        {"KL", {{"highway", "corridor"}, {"level", "3"}}},
        {"LM", {{"highway", "corridor"}, {"level", "3"}}},
        {"IJ", {{"highway", "corridor"}, {"level", "2"}}},
        {"JM", {{"highway", "corridor"}}},
        {"LF", {{"highway", "corridor"}}},
        {"FN", {{"highway", "corridor"}}},
        {"MN", {{"highway", "corridor"}}},
        {"JM", {{"highway", "corridor"}}},
    };
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {1.01, 1.01});
    // Add low length limit for exclude_polygons so it throws an error
    avoid_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_avoids",
                                  {{"service_limits.max_exclude_polygons_length", "10000"},
                                   {"mjolnir.shortcut_caching", "true"}});
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

TEST_F(AvoidTest, ExcludeLevels) {
  auto options_factory = [](const std::vector<float>& levels)
      -> std::pair<valhalla::Options, valhalla::sif::cost_ptr_t> {
    valhalla::Options options;
    options.set_costing_type(valhalla::Costing::pedestrian);
    auto& co = (*options.mutable_costings())[Costing::pedestrian];
    co.set_type(valhalla::Costing::pedestrian);

    // create the polygon intersecting a shortcut
    auto* rings = options.mutable_exclude_polygons();
    auto* ring = rings->Add();
    auto* option_levels = options.mutable_exclude_levels();
    auto* level = option_levels->Add();
    for (const auto& lvl : levels) {
      level->add_levels(lvl);
    }
    for (const auto& coord :
         {avoid_map.nodes.at("a"), avoid_map.nodes.at("b"), avoid_map.nodes.at("c"),
          avoid_map.nodes.at("d"), avoid_map.nodes.at("a")}) {
      auto* ll = ring->add_coords();
      ll->set_lat(coord.lat());
      ll->set_lng(coord.lng());
    }
    auto costing = valhalla::sif::CostFactory{}.Create(co);

    return std::make_pair(options, costing);
  };

  using test_params = std::pair<std::vector<float>, size_t>;

  // make sure the correct amounts of edges are excluded given a list of levels per ring
  std::vector<test_params> params = {{{2.f}, 4}, {{3.f}, 6},       {{5.f}, 2},     {{}, 18},
                                     {{0.f}, 0}, {{2.f, 3.f}, 10}, {{0.f, 5.f}, 2}};

  auto reader = test::make_clean_graphreader(avoid_map.config.get_child("mjolnir"));
  for (const auto& param : params) {
    auto options = options_factory(param.first);

    auto edge_names = [&reader](const std::unordered_set<GraphId>& edge_ids) -> std::string {
      std::stringstream ss;

      std::string sep = "";
      for (const auto& edge_id : edge_ids) {
        ss << sep;
        sep = ", ";
        auto ei = reader->edgeinfo(edge_id);
        ss << ei.GetNames()[0];
      }
      return ss.str();
    };

    // should return the shortcut edge ID as well
    auto avoid_edges = edges_in_rings(options.first, *reader, options.second, 10000);
    ASSERT_EQ(avoid_edges.size(), param.second) << edge_names(avoid_edges);
  }
}

TEST_F(AvoidTest, ExcludeLevelsJSON) {
  std::vector<std::vector<PointLL>> exclude_ring = {{avoid_map.nodes.at("a"), avoid_map.nodes.at("b"),
                                                     avoid_map.nodes.at("c"), avoid_map.nodes.at("d"),
                                                     avoid_map.nodes.at("a")}};
  std::vector<PointLL> waypoints = {avoid_map.nodes.at("M"), avoid_map.nodes.at("L")};

  std::vector<std::vector<float>> levels = {{3.f}};
  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto excludes_j = build_exclude_level_polygons(exclude_ring, levels, allocator);
  auto req =
      build_local_req(doc, allocator, waypoints, "pedestrian", excludes_j, "/exclude_polygons");

  auto res = gurka::do_action(Options::route, avoid_map, req);
  gurka::assert::raw::expect_path(res, {"MN", "FN", "LF"});
}

TEST_F(AvoidTest, ExcludeLevelsJSON_Multiple) {
  std::vector<std::vector<PointLL>> exclude_ring = {{avoid_map.nodes.at("a"), avoid_map.nodes.at("b"),
                                                     avoid_map.nodes.at("c"), avoid_map.nodes.at("d"),
                                                     avoid_map.nodes.at("a")}};
  std::vector<PointLL> waypoints = {avoid_map.nodes.at("G"), avoid_map.nodes.at("L")};

  std::vector<std::vector<float>> levels = {{3.f, 2.f}};
  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto excludes_j = build_exclude_level_polygons(exclude_ring, levels, allocator);
  auto req =
      build_local_req(doc, allocator, waypoints, "pedestrian", excludes_j, "/exclude_polygons");

  auto res = gurka::do_action(Options::route, avoid_map, req);
  gurka::assert::raw::expect_path(res, {"GB", "2nd", "2nd", "2nd", "LF"});
}

TEST_F(AvoidTest, ExcludeLevelsJSON_MultiRing) {
  std::vector<std::vector<PointLL>> exclude_rings = {
      {avoid_map.nodes.at("a"), avoid_map.nodes.at("b"), avoid_map.nodes.at("c"),
       avoid_map.nodes.at("d"), avoid_map.nodes.at("a")},
      {avoid_map.nodes.at("t"), avoid_map.nodes.at("u"), avoid_map.nodes.at("v"),
       avoid_map.nodes.at("w"), avoid_map.nodes.at("t")},
  };
  std::vector<PointLL> waypoints = {avoid_map.nodes.at("G"), avoid_map.nodes.at("L")};

  std::vector<std::vector<float>> levels = {{3.f}, {2.f}};
  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto excludes_j = build_exclude_level_polygons(exclude_rings, levels, allocator);
  auto req =
      build_local_req(doc, allocator, waypoints, "pedestrian", excludes_j, "/exclude_polygons");

  auto res = gurka::do_action(Options::route, avoid_map, req);
  gurka::assert::raw::expect_path(res, {"GB", "2nd", "2nd", "2nd", "LF"});
}

TEST_F(AvoidTest, ExcludeLevelsJSON_OnlyOneLevelExclude) {
  std::vector<std::vector<PointLL>> exclude_rings = {
      {avoid_map.nodes.at("a"), avoid_map.nodes.at("b"), avoid_map.nodes.at("c"),
       avoid_map.nodes.at("d"), avoid_map.nodes.at("a")},
      {avoid_map.nodes.at("t"), avoid_map.nodes.at("u"), avoid_map.nodes.at("v"),
       avoid_map.nodes.at("w"), avoid_map.nodes.at("t")},
  };
  std::vector<PointLL> waypoints = {avoid_map.nodes.at("G"), avoid_map.nodes.at("L")};

  std::vector<std::vector<float>> levels = {{3.f}, {}}; // second one is empty levels
  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  auto excludes_j = build_exclude_level_polygons(exclude_rings, levels, allocator);
  auto req =
      build_local_req(doc, allocator, waypoints, "pedestrian", excludes_j, "/exclude_polygons");

  auto res = gurka::do_action(Options::route, avoid_map, req);
  gurka::assert::raw::expect_path(res, {"GB", "2nd", "2nd", "2nd", "LF"});
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

TEST_F(AvoidTest, TestInvalidAvoidPolygons) {
  // https://github.com/valhalla/valhalla/issues/3905
  std::string req =
      R"({
          "locations": [
            {"lat": %s, "lon": %s},
            {"lat": %s, "lon": %s}
          ],
          "costing":"auto",
        )";
  std::string req_base =
      (boost::format(req) % std::to_string(avoid_map.nodes.at("A").lat()) %
       std::to_string(avoid_map.nodes.at("A").lng()) % std::to_string(avoid_map.nodes.at("D").lat()) %
       std::to_string(avoid_map.nodes.at("D").lng()))
          .str();
  Api request;

  // empty polygon
  auto req_str = req_base + R"("avoid_polygons": [[]]})";
  std::cerr << req_str << std::endl;
  ParseApi(req_str, Options::route, request);
  // loki would previously segfault on exclude_polygons=[[]]
  gurka::do_action(Options::route, avoid_map, req_str);
  EXPECT_TRUE(request.options().exclude_polygons_size() == 0);

  // an object!
  req_str = req_base + R"("avoid_polygons": {}})";
  std::cerr << req_str << std::endl;
  try {
    ParseApi(req_str, Options::route, request);
    auto res = gurka::do_action(Options::route, avoid_map, req_str);
    FAIL() << "Expected to throw";
  } catch (const valhalla_exception_t& e) { EXPECT_EQ(e.code, 137); } catch (...) {
    FAIL() << "Expected different error";
  }

  // array of empty array and empty object
  req_str = req_base + R"("avoid_polygons": [[]]})";
  ParseApi(req_str, Options::route, request);
  gurka::do_action(Options::route, avoid_map, req_str);
  EXPECT_TRUE(request.options().exclude_polygons_size() == 0);

  // a valid polygon and an empty object!
  req_str = req_base +
            R"("avoid_polygons": [[[1.0, 1.0], [1.00001, 1.00001], [1.00002, 1.00002], [1.0, 1.0]],
      {}]})";
  ParseApi(req_str, Options::route, request);
  auto res = gurka::do_action(Options::route, avoid_map, req_str);
  EXPECT_TRUE(request.options().exclude_polygons_size() == 1);

  // protect the public API too
  baldr::GraphReader reader(avoid_map.config.get_child("mjolnir"));
  LokiWorkerTest loki_worker(avoid_map.config);
  valhalla::Api vanilla_request;
  vanilla_request.mutable_options()->set_costing_type(valhalla::Costing_Type_auto_);
  vanilla_request.mutable_options()->mutable_exclude_polygons()->Add();
  (*vanilla_request.mutable_options()->mutable_costings())[valhalla::Costing::auto_];

  // adding an empty polygon was previously causing a segfault
  loki_worker.parse_costing(vanilla_request);
  EXPECT_TRUE(vanilla_request.options().exclude_polygons_size() == 1);
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
  auto reader = test::make_clean_graphreader(avoid_map.config.get_child("mjolnir"));

  // should return the shortcut edge ID as well
  size_t found_shortcuts = 0;
  auto avoid_edges = edges_in_rings(options, *reader, costing, 10000);
  for (const auto& edge_id : avoid_edges) {
    if (reader->GetGraphTile(edge_id)->directededge(edge_id)->is_shortcut()) {
      found_shortcuts++;
    }
  }

  // 2 shortcuts + 2 edges
  ASSERT_EQ(avoid_edges.size(), 4);
  ASSERT_EQ(found_shortcuts, 2);
}

TEST_P(AvoidTest, TestAvoidLocation) {
  // avoid the location on "High road"
  std::vector<PointLL> avoid_locs{avoid_map.nodes["x"]};

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
