#include "gurka.h"
#include "test.h"

#include "baldr/json.h"
#include "loki/worker.h"
#include "midgard/pointll.h"

#include <filesystem>

#include <gtest/gtest.h>

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

  // create a fake elevation tile over the gurka map area
  PointLL bottom_left(-77, 40), upper_right(-76, 41);
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
      auto dist_ratio = bottom_left.Distance(PointLL(lon, lat)) / corner_to_corner_dist;
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
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  // try a bunch of routes
  for (const auto& waypoints : std::vector<std::vector<std::string>>{
           {"S", "F"},
           {"C", "N"},
       }) {

    // get a route with elevation included
    std::string route_json;
    auto route = gurka::do_action(valhalla::Options::route, map, waypoints, "bicycle",
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
      auto height = gurka::do_action(valhalla::Options::height, map, request, {}, &height_json);

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
    auto route =
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
}
