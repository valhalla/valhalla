#include <optional>
#include <unordered_map>

#include <gtest/gtest.h>

#include "baldr/rapidjson_utils.h"
#include "gurka.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{{}};

namespace {
struct Waypoint {
  std::string node;
  std::optional<int8_t> preferred_layer;
};

} // namespace

class MultiLayerLoki : public ::testing::Test {
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
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_multi_layer_loki", build_config);
  }

  valhalla::Api Route(const std::vector<Waypoint>& waypoints) {
    std::vector<std::string> nodes;
    std::unordered_map<std::string, std::string> options;
    for (size_t index = 0; index < waypoints.size(); ++index) {
      const auto& wp = waypoints[index];
      nodes.emplace_back(wp.node);
      if (wp.preferred_layer) {
        options["/locations/" + std::to_string(index) + "/preferred_layer"] =
            std::to_string(*wp.preferred_layer);
        options["/locations/" + std::to_string(index) + "/radius"] = "1";
      }
    }
    return gurka::do_action(valhalla::Options::route, map, nodes, "auto", options);
  }
};
gurka::map MultiLayerLoki::map = {};
std::string MultiLayerLoki::ascii_map = {};
gurka::nodelayout MultiLayerLoki::layout = {};

TEST_F(MultiLayerLoki, test_multilevel_loki) {
  auto result = Route({{"B", -1}, {"G", std::nullopt}});
  gurka::assert::osrm::expect_steps(result, std::vector<std::string>({"HI"}));

  result = Route({{"B", 0}, {"G", std::nullopt}});
  gurka::assert::osrm::expect_steps(result, std::vector<std::string>({"BE", "EF", "FD", "DG"}));
  result = Route({{"B", std::nullopt}, {"G", std::nullopt}});
  gurka::assert::osrm::expect_steps(result, std::vector<std::string>({"HI"}));
}

TEST_F(MultiLayerLoki, test_no_matching_layer) {
  auto result = Route({{"E", -1}, {"G", std::nullopt}});
  gurka::assert::osrm::expect_steps(result, std::vector<std::string>({"EF", "FD", "DG"}));
}