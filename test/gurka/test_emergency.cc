#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {

using options_t = std::unordered_map<std::string, std::string>;

const std::string kDateTime = "2026-11-10T08:00";

// Each route is computed with all three searches, as each has its own access checks
const std::vector<options_t> kSearchVariants = {
    {},                                                          // bidirectional A*
    {{"/date_time/type", "1"}, {"/date_time/value", kDateTime}}, // forward A*
    {{"/date_time/type", "2"}, {"/date_time/value", kDateTime}}, // reverse A*
};

options_t with(options_t options, const options_t& more) {
  options.insert(more.begin(), more.end());
  return options;
}

} // namespace

/*************************************************************/
// ways and barriers only open to emergency vehicles

TEST(EmergencyAccess, EmergencyOnlyWayAndBarrier) {
  constexpr double gridsize_metres = 50;

  const std::string ascii_map = R"(
        A---B---C---D
            |   |
            E---F

        G---H---I
        |       |
        J-------K
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "service"}, {"access", "no"}, {"emergency", "yes"}}},
      {"CD", {{"highway", "primary"}}},
      {"BE", {{"highway", "primary"}}},
      {"EF", {{"highway", "primary"}}},
      {"FC", {{"highway", "primary"}}},
      {"GHI", {{"highway", "residential"}}},
      {"GJ", {{"highway", "residential"}}},
      {"JK", {{"highway", "residential"}}},
      {"KI", {{"highway", "residential"}}},
  };
  const gurka::nodes nodes = {
      {"H", {{"barrier", "bollard"}, {"emergency", "yes"}}},
  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/emergency_access");

  auto result = gurka::do_action(Options::route, map, {"A", "D"}, "auto");
  gurka::assert::raw::expect_path(result, {"AB", "BE", "EF", "FC", "CD"});
  result = gurka::do_action(Options::route, map, {"A", "D"}, "emergency");
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD"});

  result = gurka::do_action(Options::route, map, {"G", "I"}, "auto");
  gurka::assert::raw::expect_path(result, {"GJ", "JK", "KI"});
  result = gurka::do_action(Options::route, map, {"G", "I"}, "emergency");
  gurka::assert::raw::expect_path(result, {"GHI", "GHI"});
}

/*************************************************************/
// oneways behave like auto: closed by default, open with ignore_oneways

class EmergencyOnewayTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    // CB is a oneway from C to B, so A -> D has to take the detour BE, EF, FD
    const std::string ascii_map = R"(
      A--B--C--D
         |     |
         |     |
         |     |
         E-----F
    )";

    const gurka::ways ways = {{"AB", {{"highway", "residential"}}},
                              {"CB", {{"highway", "residential"}, {"oneway", "yes"}}},
                              {"CD", {{"highway", "residential"}}},
                              {"BE", {{"highway", "residential"}}},
                              {"EF", {{"highway", "residential"}}},
                              {"FD", {{"highway", "residential"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 20);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/emergency_oneway");
  }

  static const std::vector<std::string> kDetour;
  static const std::vector<std::string> kWrongWay;
  static const options_t kIgnoreOneways;
};

gurka::map EmergencyOnewayTest::map = {};
const std::vector<std::string> EmergencyOnewayTest::kDetour = {"AB", "BE", "EF", "FD"};
const std::vector<std::string> EmergencyOnewayTest::kWrongWay = {"AB", "CB", "CD"};
const options_t EmergencyOnewayTest::kIgnoreOneways = {{"/costing_options/emergency/ignore_oneways",
                                                        "1"}};

TEST_F(EmergencyOnewayTest, ClosedByDefault) {
  for (const auto& search : kSearchVariants) {
    auto result = gurka::do_action(Options::route, map, {"A", "D"}, "emergency", search);
    gurka::assert::raw::expect_path(result, kDetour);
  }
}

TEST_F(EmergencyOnewayTest, IgnoreOneways) {
  for (const auto& search : kSearchVariants) {
    auto result =
        gurka::do_action(Options::route, map, {"A", "D"}, "emergency", with(search, kIgnoreOneways));
    gurka::assert::raw::expect_path(result, kWrongWay);
  }
}

TEST_F(EmergencyOnewayTest, Matrix) {
  // CostMatrix expands from both ends, so both trees have to accept the wrong way
  auto closed = gurka::do_action(Options::sources_to_targets, map, {"A"}, {"D"}, "emergency");
  auto open =
      gurka::do_action(Options::sources_to_targets, map, {"A"}, {"D"}, "emergency", kIgnoreOneways);
  EXPECT_EQ(open.matrix().algorithm(), Matrix::CostMatrix);
  EXPECT_NEAR(closed.matrix().distances(0), 340, 10);
  EXPECT_NEAR(open.matrix().distances(0), 180, 10);
}
