#include "gurka.h"
#include "test.h"

using namespace valhalla;

class DeadendBarrier : public ::testing::Test {
protected:
  static gurka::nodelayout layout;
  static gurka::ways ways;
  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
       A-----1----B
    )";

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);

    ways = {
        {"A1", {{"highway", "primary"}}},
        {"1B", {{"highway", "primary"}}},
    };
  }
};

gurka::nodelayout DeadendBarrier::layout = {};
gurka::ways DeadendBarrier::ways = {};

TEST_F(DeadendBarrier, DeniedAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"access", "no"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/deadend_barrier_no_access");
  try {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }
}

TEST_F(DeadendBarrier, AllowedAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"access", "yes"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/deadend_barrier_allowed_access");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
  gurka::assert::raw::expect_path(result, {"A1", "1B"});
}

TEST_F(DeadendBarrier, NoInfoBarrierAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/deadend_barrier_no_info_access");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
  gurka::assert::raw::expect_path(result, {"A1", "1B"});
}

TEST_F(DeadendBarrier, BikeRestricted) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"bicycle", "no"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/deadend_barrier_bike_restricted_barrier");

  // auto
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
  gurka::assert::raw::expect_path(result, {"A1", "1B"});

  // bicycle
  try {
    result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "bicycle");
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }
}

TEST_F(DeadendBarrier, BikeAllowedNoOtherInformation) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"bicycle", "yes"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/deadend_barrier_bike_allowed_barrier");

  // auto
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
  gurka::assert::raw::expect_path(result, {"A1", "1B"});

  // bicycle
  result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "bicycle");
  gurka::assert::raw::expect_path(result, {"A1", "1B"});
}
