#include "gurka.h"
#include "test.h"

using namespace valhalla;

class DeadendBarrier : public testing::TestWithParam<std::string> {
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

TEST_P(DeadendBarrier, DeniedAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", GetParam()}, {"access", "no"}}},
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

TEST_P(DeadendBarrier, AllowedAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", GetParam()}, {"access", "yes"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/deadend_barrier_allowed_access");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
  gurka::assert::raw::expect_path(result, {"A1", "1B"});
}

TEST_P(DeadendBarrier, PrivateAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", GetParam()}, {"access", "private"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/deadend_barrier_private_access");

  // auto
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
  gurka::assert::raw::expect_path(result, {"A1", "1B"});

  // pedestrian
  result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "pedestrian");
  gurka::assert::raw::expect_path(result, {"A1", "1B"});
}

class GateBarrier : public DeadendBarrier {};

TEST_P(GateBarrier, NoInfoBarrierAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", GetParam()}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/deadend_gate_no_info_access");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
  gurka::assert::raw::expect_path(result, {"A1", "1B"});
}

TEST_P(GateBarrier, BikeRestricted) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", GetParam()}, {"bicycle", "no"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/deadend_gate_bike_restricted_barrier");

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

TEST_P(GateBarrier, BikeAllowedNoOtherInformation) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", GetParam()}, {"bicycle", "yes"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/deadend_gate_bike_allowed_barrier");

  // auto
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
  gurka::assert::raw::expect_path(result, {"A1", "1B"});

  // bicycle
  result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "bicycle");
  gurka::assert::raw::expect_path(result, {"A1", "1B"});
}

class BollardBarrier : public DeadendBarrier {};

TEST_P(BollardBarrier, NoInfoBarrierAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", GetParam()}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/deadend_bollard_no_info_access");
  try {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "pedestrian");
  gurka::assert::raw::expect_path(result, {"A1", "1B"});
}

TEST_P(BollardBarrier, BikeRestricted) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", GetParam()}, {"bicycle", "no"}}},
  };
  const gurka::map map = gurka::buildtiles(layout, ways, nodes, {},
                                           "test/data/deadend_bollarrd_bike_restricted_barrier");

  // auto
  try {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }

  // bicycle
  try {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "bicycle");
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }
}

TEST_P(BollardBarrier, BikeAllowedNoOtherInformation) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", GetParam()}, {"bicycle", "yes"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/deadend_bollard_bike_allowed_barrier");

  // auto
  try {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }

  // bicycle
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "bicycle");
  gurka::assert::raw::expect_path(result, {"A1", "1B"});
}

const std::vector<std::string> gates = {"gate", "yes", "lift_gate", "swing_gate"};
const std::vector<std::string> bollards = {"bollard", "block", "jersey_barrier"};

INSTANTIATE_TEST_SUITE_P(GateBasicAccess, DeadendBarrier, testing::ValuesIn(gates));
INSTANTIATE_TEST_SUITE_P(BollardBasicAccess, DeadendBarrier, testing::ValuesIn(bollards));

INSTANTIATE_TEST_SUITE_P(GateAccess, GateBarrier, testing::ValuesIn(gates));
INSTANTIATE_TEST_SUITE_P(BollardAccess, BollardBarrier, testing::ValuesIn(bollards));
