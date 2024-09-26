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
  std::optional<int16_t> preferred_level;
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
  auto result = Route({{"B", -1}, {"G"}});
  gurka::assert::osrm::expect_steps(result, std::vector<std::string>({"HI"}));

  result = Route({{"B", 0}, {"G"}});
  gurka::assert::osrm::expect_steps(result, std::vector<std::string>({"BE", "EF", "FD", "DG"}));
  result = Route({{"B"}, {"G"}});
  gurka::assert::osrm::expect_steps(result, std::vector<std::string>({"HI"}));
}

TEST_F(MultiLayerLoki, test_no_matching_layer) {
  auto result = Route({{"E", -1}, {"G"}});
  gurka::assert::osrm::expect_steps(result, std::vector<std::string>({"EF", "FD", "DG"}));
}

/**************************************************************************************/

class MultiLevelLoki : public ::testing::Test {
protected:
  static gurka::map map;
  static std::string ascii_map;
  static gurka::nodelayout layout;
  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 50;

    /**
     * Difficult to represent visually, so here is a stacked view:
     *
     * ground level:
     *  C-----------------D
     *  |                 |
     *  |                 |
     *  |                 |
     * (A)---------------(B)
     *
     * first floor:
     *   G---------------H
     *   |               |
     *   |               |
     *   |               |
     *  (E)-------------(F)
     *
     * () = connected via stairs
     */
    ascii_map = R"(
      C-G-----------H-D
      | |  x    y   | |
      | |           | |
      | |           | |                     z
      | |           | |
      | |           | |
      A~E-----------F~B
    )";

    const gurka::ways ways = {
        // ground floor
        {"AB", {{"highway", "corridor"}, {"level", "0"}}},
        {"AC", {{"highway", "corridor"}, {"level", "0"}}},
        {"CD", {{"highway", "corridor"}, {"level", "0"}}},
        {"DB", {{"highway", "corridor"}, {"level", "0"}}},
        // level 1
        {"EG", {{"highway", "corridor"}, {"level", "1"}}},
        {"EF", {{"highway", "corridor"}, {"level", "1"}}},
        {"GH", {{"highway", "corridor"}, {"level", "1"}}},
        {"HF", {{"highway", "corridor"}, {"level", "1"}}},
        // stairs
        {"AE", {{"highway", "steps"}, {"level", "0;1"}}},
        {"FB", {{"highway", "steps"}, {"level", "0;1"}}},

    };

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_multi_level_loki", build_config);
  }

  valhalla::Api Route(const std::vector<Waypoint>& waypoints) {
    std::vector<std::string> nodes;
    std::unordered_map<std::string, std::string> options;
    for (size_t index = 0; index < waypoints.size(); ++index) {
      const auto& wp = waypoints[index];
      nodes.emplace_back(wp.node);
      if (wp.preferred_level) {
        options["/locations/" + std::to_string(index) + "/search_filter/level"] =
            std::to_string(*wp.preferred_level);
        // options["/locations/" + std::to_string(index) + "/radius"] = "200";
      }
    }
    return gurka::do_action(valhalla::Options::route, map, nodes, "pedestrian", options);
  }
};
gurka::map MultiLevelLoki::map = {};
std::string MultiLevelLoki::ascii_map = {};
gurka::nodelayout MultiLevelLoki::layout = {};

TEST_F(MultiLevelLoki, TraverseLevels) {
  auto result = Route({{"x", 0, 0}, {"y", 0, 1}});
  ASSERT_EQ(result.info().warnings().size(), 2);
  EXPECT_EQ(result.info().warnings().Get(0).code(), 401);
  EXPECT_EQ(result.info().warnings().Get(1).code(), 401);
  gurka::assert::raw::expect_path(result, {"CD", "AC", "AE", "EG", "GH"});
}

TEST_F(MultiLevelLoki, NonExistentLevel) {
  try {
    auto result = Route({{"x", 0, 0}, {"y", 0, 6}});
    FAIL() << "We should not get to here";
  } catch (const valhalla_exception_t& e) {
    EXPECT_EQ(e.code, 171);
    EXPECT_STREQ(e.what(), "No suitable edges near location");
  } catch (...) { FAIL() << "Failed with unexpected exception type"; }
}

TEST_F(MultiLevelLoki, Cutoff) {
  try {
    auto result = Route({{"x", 0, 0}, {"z", 0, 1}});
    FAIL() << "We should not get to here";
  } catch (const valhalla_exception_t& e) {
    EXPECT_EQ(e.code, 171);
    EXPECT_STREQ(e.what(), "No suitable edges near location");
  } catch (...) { FAIL() << "Failed with unexpected exception type"; }
}