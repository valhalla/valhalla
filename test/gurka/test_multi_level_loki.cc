#include "gurka.h"
#include "rapidjson/document.h"
#include <boost/format.hpp>
#include <boost/optional.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{{}};

namespace {
struct Waypoint {
  midgard::PointLL ll;
  boost::optional<int8_t> preferred_z_level;
};

std::string ToString(const rapidjson::Document& doc) {
  rapidjson::StringBuffer sb;
  rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
  doc.Accept(writer);
  return sb.GetString();
}

std::vector<std::string> GetStepNames(const rapidjson::Document& doc) {
  std::vector<std::string> names;
  auto steps = doc["routes"][0]["legs"][0]["steps"].GetArray();
  names.reserve(steps.Size());
  for (const auto& step : steps) {
    names.emplace_back(step["name"].GetString());
  }
  return names;
}

std::string BuildRequest(const std::vector<Waypoint>& waypoints) {
  rapidjson::Document requestDoc;
  requestDoc.SetObject();
  auto& allocator = requestDoc.GetAllocator();
  rapidjson::Value locations(rapidjson::kArrayType);
  for (const auto& waypoint : waypoints) {
    rapidjson::Value p(rapidjson::kObjectType);
    p.AddMember("lon", waypoint.ll.lng(), allocator);
    p.AddMember("lat", waypoint.ll.lat(), allocator);
    if (waypoint.preferred_z_level) {
      p.AddMember("preferred_z_level", *waypoint.preferred_z_level, allocator);
      // `preferred_z_level` is only working correctly if radius is provided,
      // because Z-level filtering is performed as last step in loki and if we don't provide
      // radius we would have only single candidate on the last step in this case
      p.AddMember("radius", 1, allocator);
    }

    locations.PushBack(p, allocator);
  }
  requestDoc.AddMember("locations", locations, allocator);
  requestDoc.AddMember("costing", "auto", allocator);
  requestDoc.AddMember("verbose", true, allocator);
  requestDoc.AddMember("shape_match", "map_snap", allocator);

  return ToString(requestDoc);
}

} // namespace

class MultiLevelLoki : public ::testing::Test {
protected:
  static gurka::map map;
  static std::string ascii_map;
  static gurka::nodelayout layout;
  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    ascii_map = R"(
                          A
                          H
                          |
                          |
                          B-------E        
                          |       |
                          |       |
                          I       |
                          C       |
                          |       |
                          |       |
                          D-------F
                          |
                          |
                          G
    )";

    const gurka::ways ways = {{"AB", {{"highway", "motorway"}}},
                              {"BE", {{"highway", "motorway"}}},
                              {"EF", {{"highway", "motorway"}}},
                              {"FD", {{"highway", "motorway"}}},
                              {"HI", {{"highway", "motorway"}, {"layer", "-1"}}},
                              {"ID", {{"highway", "motorway"}, {"layer", "-1"}}},
                              {"DG", {{"highway", "motorway"}}}

    };

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_multi_level_loki", build_config);
  }

  rapidjson::Document Route(const std::vector<Waypoint>& waypoints) {
    auto result = gurka::do_action(valhalla::Options::route, map, BuildRequest(waypoints));
    return gurka::convert_to_json(result, valhalla::Options_Format_osrm);
  }
};
gurka::map MultiLevelLoki::map = {};
std::string MultiLevelLoki::ascii_map = {};
gurka::nodelayout MultiLevelLoki::layout = {};

TEST_F(MultiLevelLoki, test_multilevel_loki) {
  auto start = map.nodes["B"];
  auto end = map.nodes["G"];

  EXPECT_EQ(GetStepNames(Route({{start, -1}, {end}})), std::vector<std::string>({"HI", "HI"}));
  EXPECT_EQ(GetStepNames(Route({{start, 0}, {end}})),
            std::vector<std::string>({"BE", "EF", "FD", "DG", "DG"}));
  EXPECT_EQ(GetStepNames(Route({{start}, {end}})), std::vector<std::string>({"HI", "HI"}));
}

TEST_F(MultiLevelLoki, test_no_matching_z_level) {
  // we don't have Z-level equal to "-1" near the `E` node,
  // we should return something anyway
  auto start = map.nodes["E"];
  auto end = map.nodes["G"];

  EXPECT_EQ(GetStepNames(Route({{start, -1}, {end}})),
            std::vector<std::string>({"EF", "FD", "DG", "DG"}));
}
