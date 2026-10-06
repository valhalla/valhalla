#include "baldr/graphconstants.h"
#include "baldr/graphreader.h"
#include "baldr/json.h"
#include "baldr/rapidjson_utils.h"
#include "gurka.h"
#include "midgard/pointll.h"
#include "mjolnir/util.h"
#include "test.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

using namespace valhalla;
using namespace valhalla::gurka;

const std::string workdir = "test/data/gurka_elevation";

std::string json_escape(const std::string& unescaped) {
  std::stringstream ss;
  baldr::json::OstreamVisitor v(ss);
  v(unescaped);
  std::string escaped = ss.str().substr(1);
  escaped.pop_back();
  return escaped;
}

// we need to prove that what we store in the graph is roughly equivalent to pulling it out of the
// elevation tile directly, we dont even care what the values are, just that they are
// approximately similar, the only other thing to do is validate the response format looks
// good but in order to pull the data out to compare you're already doing that
TEST(Standalone, ElevationCompareToSkadi) {
  // SKETCH OF THE TEST BELOW:
  // call gurka do_action with route action and elevation requested
  // pull the route shape out of the route response
  // send the route shape to gurka do_action with the height action
  // pull the elevation out of the route response
  // compare it to the height response

  const std::string ascii_map = R"(
                 A---B
                 |   |
                 |   C
                 |
              D--E-----F------G
              |        |
              |        |
              |        1
              H--I-----J
                       2
                       |
                       |
             K---L-----M
             |         |
             |         |
             N---O-----P
             |   |     |
             |   |     |
             Q---R-----S

    T--3--U        separate components, far from the cluster and not to scale
    V-4-W-5-X-6-Y                     motorway, contracts to one shortcut
    Z--0=b=8~~9--7                    plain, bridge, tunnel, ferry
  )";

  const gurka::ways ways = {
      {"KNQ", {{"highway", "service"}, {"service", "alley"}}},
      {"NOP", {{"highway", "residential"}, {"name", "East Chestnut Street"}}},
      {"QRS", {{"highway", "service"}, {"service", "alley"}}},
      {"OR", {{"highway", "service"}, {"service", "alley"}}},
      {"SPM2J1F", {{"highway", "service"}, {"service", "alley"}}},
      {"KLM", {{"highway", "service"}, {"service", "alley"}, {"name", "East Center Alley"}}},
      {"DEFG", {{"highway", "service"}, {"service", "alley"}, {"name", "North Alley"}}},
      {"DH", {{"highway", "service"}, {"service", "alley"}}},
      {"HIJ", {{"highway", "secondary"}, {"name", "East Main Street"}}},
      {"EABC", {{"highway", "service"}, {"service", "driveway"}}},
      {"T3U", {{"highway", "service"}}},
      {"V4W", {{"highway", "motorway"}, {"name", "Ridge Freeway"}}},
      {"W5X", {{"highway", "motorway"}, {"name", "Ridge Freeway"}}},
      {"X6Y", {{"highway", "motorway"}, {"name", "Ridge Freeway"}}},
      {"Z0", {{"highway", "secondary"}, {"name", "Harbour Road"}}},
      {"0b8", {{"highway", "secondary"}, {"name", "Harbour Road"}, {"bridge", "yes"}}},
      {"89", {{"highway", "secondary"}, {"name", "Harbour Road"}, {"tunnel", "yes"}}},
      {"97", {{"route", "ferry"}, {"motor_vehicle", "yes"}, {"name", "The Crossing"}}},
  };

  // Create our layout based on real world data.
  // Really the only one we care about is SPM2J1F or omsid 326371867
  using nodelayout = std::map<std::string, midgard::PointLL>;
  nodelayout layout;

  layout.insert({"1", {-76.4945823, 40.6517478}});
  layout.insert({"2", {-76.4945322, 40.6516377}});
  layout.insert({"A", {-76.4951627, 40.6526995}});
  layout.insert({"B", {-76.4948476, 40.6527545}});
  layout.insert({"C", {-76.4948142, 40.6526154}});
  layout.insert({"D", {-76.4956555, 40.6521481}});
  layout.insert({"E", {-76.4951254, 40.6521625}});
  layout.insert({"F", {-76.4946354, 40.6521889}});
  layout.insert({"G", {-76.494053, 40.6522276}});
  layout.insert({"H", {-76.4956165, 40.6516304}});
  layout.insert({"I", {-76.4950803, 40.6516647}});
  layout.insert({"J", {-76.4945597, 40.651698}});
  layout.insert({"K", {-76.4958143, 40.6511348}});
  layout.insert({"L", {-76.4951349, 40.6506908}});
  layout.insert({"M", {-76.4944987, 40.6512172}});
  layout.insert({"N", {-76.4957653, 40.6506541}});
  layout.insert({"O", {-76.4951349, 40.6506908}});
  layout.insert({"P", {-76.4944599, 40.6507301}});
  layout.insert({"Q", {-76.4957235, 40.6502434}});
  layout.insert({"R", {-76.4950865, 40.6501919}});
  layout.insert({"S", {-76.4944069, 40.6502916}});
  layout.insert({"T", {-76.8, 40.2}});
  layout.insert({"3", {-76.79, 40.21}});
  layout.insert({"U", {-76.78, 40.22}});
  // a separate component, clear of the bicycle routes asserted above
  layout.insert({"V", {-76.6500, 40.40}});
  layout.insert({"4", {-76.6375, 40.40}});
  layout.insert({"W", {-76.6250, 40.40}});
  layout.insert({"5", {-76.6125, 40.40}});
  layout.insert({"X", {-76.6000, 40.40}});
  layout.insert({"6", {-76.5875, 40.40}});
  layout.insert({"Y", {-76.5750, 40.40}});
  layout.insert({"Z", {-76.70, 40.30}});
  layout.insert({"0", {-76.68, 40.30}});
  layout.insert({"b", {-76.67, 40.30}});
  layout.insert({"8", {-76.66, 40.30}});
  layout.insert({"9", {-76.64, 40.30}});
  layout.insert({"7", {-76.62, 40.30}});

  // create a fake elevation tile over the gurka map area
  midgard::PointLL bottom_left(-77, 40), upper_right(-76, 41);
  auto corner_to_corner_dist = bottom_left.Distance(upper_right);
  // just a randomly chosen max height that will create reasonable changes in local elevation
  double max_height = 9000;
  std::vector<int16_t> tile(3601 * 3601, 0);
  for (size_t i = 0; i < 3601; ++i) {   // latitude pixels
    for (size_t j = 0; j < 3601; ++j) { // longitude pixels
      // we set the height at each pixel of the srtm tile based on its lat lon. srtm tiles have their
      // origin in the south west corner of the tile. our tile is 40, -77, that means the top right
      // corner is 41, -76. so that our values dont get too crazy we'll just use the distance a given
      // pixel is, from the bottom left corner to pick a height for a given pixel. this will give us a
      // full tile of pixels even if the heights are not real they will vary by realistic amounts. the
      // bottom left will have a height of 0 meters and the top right will have max_height. height
      // will smoothly vary from corner to corner

      // convert pixel to ll
      double lon = (static_cast<double>(j) / 3601) - 77;
      double lat = (static_cast<double>(i) / 3601) + 40;
      // measure distance and use it to scale a max height range
      auto dist_ratio = bottom_left.Distance(midgard::PointLL(lon, lat)) / corner_to_corner_dist;
      int16_t height = std::round(dist_ratio * max_height);
      // and set the height in the tile data (flipping to big endian to match the srtm spec)
      tile[i * 3601 + j] = ((height & 0xFF) << 8) | ((height >> 8) & 0xFF);
    }
  }

  if (!std::filesystem::exists(workdir)) {
    bool created = std::filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  // actually store it
  std::ofstream file(workdir + "/N40W077.hgt", std::ios::binary | std::ios::trunc);
  file.write(static_cast<const char*>(static_cast<void*>(tile.data())),
             sizeof(int16_t) * tile.size());
  ASSERT_TRUE(file.good()) << "File stream is not good";
  file.close();

  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map map;
  map.nodes = layout;
  map.config = test::make_config(workdir, {});
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("additional_data.elevation", workdir);

  std::vector<std::string> input_files = {pbf_filename};
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate);

  for (const auto& waypoints : std::vector<std::vector<std::string>>{
           {"T", "3", "U"},
           {"U", "3", "T"},
       }) {
    SCOPED_TRACE(waypoints.front() + " through " + waypoints[1] + " to " + waypoints.back());

    std::string through_json;
    gurka::do_action(valhalla::Options::route, map, waypoints, "bicycle",
                     {
                         {"/locations/0/type", "break"},
                         {"/locations/1/type", "through"},
                         {"/locations/2/type", "break"},
                         {"/elevation_interval", "30"},
                     },
                     {}, &through_json);

    rapidjson::Document through_result;
    through_result.Parse(through_json.c_str());
    auto through_elevation = rapidjson::get_child_optional(through_result, "/trip/legs/0/elevation");
    auto through_shape = rapidjson::get_child_optional(through_result, "/trip/legs/0/shape");

    ASSERT_TRUE(through_elevation && through_elevation->IsArray());
    ASSERT_TRUE(through_shape && through_shape->IsString());

    std::string height_json;
    std::string request = R"({"height_precision":1,"resample_distance":30,"encoded_polyline":")" +
                          json_escape(through_shape->GetString()) + R"("})";
    gurka::do_action(valhalla::Options::height, map, request, {}, &height_json);

    rapidjson::Document height_result;
    height_result.Parse(height_json.c_str());
    auto height_elevation = rapidjson::get_child_optional(height_result, "/height");
    ASSERT_TRUE(height_elevation && height_elevation->IsArray());

    ASSERT_EQ(through_elevation->Size(), height_elevation->Size());

    for (rapidjson::SizeType i = 0; i < through_elevation->Size(); ++i) {
      EXPECT_NEAR((*through_elevation)[i].GetFloat(), (*height_elevation)[i].GetFloat(), 0.5f);
    }
  }

  // try a bunch of routes
  for (const auto& waypoints : std::vector<std::vector<std::string>>{
           {"S", "F"},
           {"C", "N"},
       }) {

    // get a route with elevation included
    std::string route_json;
    [[maybe_unused]] auto route =
        gurka::do_action(valhalla::Options::route, map, waypoints, "bicycle",
                         {{"/elevation_interval", "30"}}, {}, &route_json);
    rapidjson::Document result;
    result.Parse(route_json.c_str());

    // for each leg
    for (size_t leg_index = 0; leg_index < waypoints.size() - 1; ++leg_index) {
      // pull out the shape from the leg
      auto s =
          rapidjson::get_child_optional(result, ("/trip/legs/" + std::to_string(leg_index) + "/shape")
                                                    .c_str());
      EXPECT_TRUE(s && s->IsString());
      auto shape = json_escape(s->GetString());

      std::string height_json;
      std::string request =
          R"({"height_precision":1,"resample_distance":30,"encoded_polyline":")" + shape + R"("})";
      [[maybe_unused]] auto height =
          gurka::do_action(valhalla::Options::height, map, request, {}, &height_json);

      // pull out the elevation from the route result leg
      auto elevation =
          rapidjson::get_child_optional(result,
                                        ("/trip/legs/" + std::to_string(leg_index) + "/elevation")
                                            .c_str());
      EXPECT_TRUE(elevation && elevation->IsArray());
      std::vector<float> elevation_along_edges;
      for (const auto& e : elevation->GetArray()) {
        elevation_along_edges.push_back(std::round(e.GetFloat() * 10) / 10);
        std::cout << std::round(e.GetFloat() * 10) / 10 << std::endl;
      }

      result.Parse(height_json.c_str());
      // pull out the elevation from the height result
      elevation = rapidjson::get_child_optional(result, "/height");
      EXPECT_TRUE(elevation && elevation->IsArray());
      std::vector<float> elevation_from_skadi;
      for (const auto& e : elevation->GetArray()) {
        elevation_from_skadi.push_back(std::round(e.GetFloat() * 10) / 10);
        std::cout << std::round(e.GetFloat() * 10) / 10 << std::endl;
      }

      EXPECT_EQ(elevation_along_edges.size(), elevation_from_skadi.size());
      for (size_t i = 0; i < elevation_along_edges.size(); ++i) {
        EXPECT_NEAR(elevation_along_edges[i], elevation_from_skadi[i], 0.5f);
      }
    }
  }

  // a quick test for when you request a route without elevation that its not there
  // get a route without elevation included
  for (const auto& waypoints : std::vector<std::vector<std::string>>{
           {"S", "F"},
           {"C", "N"},
       }) {
    std::string route_json;
    [[maybe_unused]] auto route =
        gurka::do_action(valhalla::Options::route, map, {"S", "F"}, "bicycle", {}, {}, &route_json);
    rapidjson::Document result;
    result.Parse(route_json.c_str());

    for (size_t leg_index = 0; leg_index < waypoints.size() - 1; ++leg_index) {
      [[maybe_unused]] auto s =
          rapidjson::get_child_optional(result, ("/trip/legs/" + std::to_string(leg_index) + "/shape")
                                                    .c_str());

      auto elevation =
          rapidjson::get_child_optional(result,
                                        ("/trip/legs/" + std::to_string(leg_index) + "/elevation")
                                            .c_str());
      EXPECT_FALSE(elevation && elevation->IsArray());
    }
  }

  // shortcuts keep mean elevation and grade, but not the postings
  {
    baldr::GraphReader reader(map.config.get_child("mjolnir"));
    auto postings = [&reader](const auto& found) {
      const auto* edge = std::get<1>(found);
      auto info = reader.GetGraphTile(std::get<0>(found))->edgeinfo(edge);
      double interval = 0.0;
      auto encoded = info.encoded_elevation(edge->length(), interval);
      EXPECT_EQ(info.has_elevation(), !encoded.empty());
      EXPECT_NE(info.mean_elevation(), baldr::kNoElevationData);
      return encoded.size();
    };

    size_t shortcuts = 0;
    for (auto tile_id : reader.GetTileSet()) {
      auto tile = reader.GetGraphTile(tile_id);
      for (const auto& edge : tile->GetDirectedEdges()) {
        if (!edge.is_shortcut())
          continue;
        ++shortcuts;
        EXPECT_FALSE(tile->edgeinfo(&edge).has_elevation())
            << "shortcut postings duplicate its base edges and are never read";
        EXPECT_NE(tile->edgeinfo(&edge).mean_elevation(), baldr::kNoElevationData);
        EXPECT_NE(edge.weighted_grade(), 0u) << "costing reads grade on shortcuts";
      }
    }
    ASSERT_GT(shortcuts, 0u) << "no shortcuts were built, so the check above proves nothing";

    // an ordinary edge keeps its postings; anything the runtime can interpolate stores none
    ASSERT_GT(postings(gurka::findEdgeByNodes(reader, layout, "V", "W")), 0u) << "plain motorway";
    ASSERT_GT(postings(gurka::findEdgeByNodes(reader, layout, "Z", "0")), 0u) << "plain secondary";

    const auto bridge = gurka::findEdgeByNodes(reader, layout, "0", "8");
    ASSERT_TRUE(std::get<1>(bridge)->bridge()) << "way is not tagged as a bridge";
    EXPECT_EQ(postings(bridge), 0u) << "bridge";

    const auto tunnel = gurka::findEdgeByNodes(reader, layout, "8", "9");
    ASSERT_TRUE(std::get<1>(tunnel)->tunnel()) << "way is not tagged as a tunnel";
    EXPECT_EQ(postings(tunnel), 0u) << "tunnel";

    const auto ferry = gurka::findEdgeByNodes(reader, layout, "9", "7");
    ASSERT_EQ(std::get<1>(ferry)->use(), baldr::Use::kFerry) << "way is not a ferry";
    EXPECT_EQ(postings(ferry), 0u) << "ferry";
  }

  // routing over the contracted chain still returns a profile, because FormPath recovers the
  // shortcut into base edges that do carry postings
  {
    std::string route_json;
    gurka::do_action(valhalla::Options::route, map, {"V", "Y"}, "auto",
                     {{"/elevation_interval", "30"},
                      {"/locations/0/minimum_reachability", "0"},
                      {"/locations/1/minimum_reachability", "0"}},
                     {}, &route_json);

    rapidjson::Document result;
    result.Parse(route_json.c_str());
    ASSERT_FALSE(result.HasParseError());
    auto elevation = rapidjson::get_child_optional(result, "/trip/legs/0/elevation");
    auto shape = rapidjson::get_child_optional(result, "/trip/legs/0/shape");
    ASSERT_TRUE(elevation && elevation->IsArray());
    ASSERT_TRUE(shape && shape->IsString());
    ASSERT_GT(elevation->Size(), 150u) << "the whole ~6km chain should be covered at 30m";

    // same skadi comparison the bicycle legs above make, over a path built from a shortcut
    std::string height_json;
    std::string request = R"({"height_precision":1,"resample_distance":30,"encoded_polyline":")" +
                          json_escape(shape->GetString()) + R"("})";
    gurka::do_action(valhalla::Options::height, map, request, {}, &height_json);

    rapidjson::Document height_result;
    height_result.Parse(height_json.c_str());
    auto heights = rapidjson::get_child_optional(height_result, "/height");
    ASSERT_TRUE(heights && heights->IsArray());
    ASSERT_EQ(elevation->Size(), heights->Size());
    for (rapidjson::SizeType i = 0; i < elevation->Size(); ++i) {
      EXPECT_NEAR((*elevation)[i].GetFloat(), (*heights)[i].GetFloat(), 0.5f) << "posting " << i;
    }
  }

  // a route across the bridge, tunnel and ferry still gets a complete profile
  {
    std::string route_json;
    gurka::do_action(valhalla::Options::route, map, {"Z", "7"}, "auto",
                     {{"/elevation_interval", "30"},
                      {"/locations/0/minimum_reachability", "0"},
                      {"/locations/1/minimum_reachability", "0"}},
                     {}, &route_json);

    rapidjson::Document result;
    result.Parse(route_json.c_str());
    ASSERT_FALSE(result.HasParseError());
    auto elevation = rapidjson::get_child_optional(result, "/trip/legs/0/elevation");
    ASSERT_TRUE(elevation && elevation->IsArray());
    ASSERT_GT(elevation->Size(), 150u) << "the whole ~6km chain should be covered at 30m";
    for (const auto& e : elevation->GetArray()) {
      EXPECT_NE(e.GetFloat(), baldr::kNoElevationData);
    }
  }

  // starting mid-bridge takes SetElevation's trimmed branch, which interpolates the stored
  // array rather than copying it
  {
    std::string route_json;
    gurka::do_action(valhalla::Options::route, map, {"b", "7"}, "auto",
                     {{"/elevation_interval", "30"},
                      {"/locations/0/minimum_reachability", "0"},
                      {"/locations/1/minimum_reachability", "0"}},
                     {}, &route_json);

    rapidjson::Document result;
    result.Parse(route_json.c_str());
    ASSERT_FALSE(result.HasParseError());
    auto elevation = rapidjson::get_child_optional(result, "/trip/legs/0/elevation");
    ASSERT_TRUE(elevation && elevation->IsArray());
    // proves the origin really snapped mid-bridge: node 8 would give 3.4km, node 0 would give 5.1km
    auto length = rapidjson::get_child_optional(result, "/trip/legs/0/summary/length");
    ASSERT_TRUE(length && length->IsNumber());
    EXPECT_NEAR(length->GetDouble(), 4.24, 0.15) << "origin did not snap mid-edge";
    ASSERT_GT(elevation->Size(), 100u) << "~4km from mid-bridge to the far side of the ferry";
    for (const auto& e : elevation->GetArray()) {
      EXPECT_NE(e.GetFloat(), baldr::kNoElevationData);
    }
  }

  // trace_attributes serves the same profile and still reports mean elevation per edge
  {
    std::string trace_json;
    gurka::do_action(valhalla::Options::trace_attributes, map, {"V", "4", "W", "5", "X", "6", "Y"},
                     "auto", {{"/elevation_interval", "30"}}, {}, &trace_json);

    rapidjson::Document result;
    result.Parse(trace_json.c_str());
    ASSERT_FALSE(result.HasParseError());
    auto elevation = rapidjson::get_child_optional(result, "/elevation");
    ASSERT_TRUE(elevation && elevation->IsArray());
    ASSERT_GT(elevation->Size(), 150u);

    auto edges = rapidjson::get_child_optional(result, "/edges");
    ASSERT_TRUE(edges && edges->IsArray());
    ASSERT_GT(edges->GetArray().Size(), 0u);
    for (const auto& edge : edges->GetArray()) {
      EXPECT_TRUE(edge.HasMember("mean_elevation")) << "omitted only when it is kNoElevationData";
    }
  }
}
