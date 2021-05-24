#include "gurka.h"
#include "test.h"

using namespace valhalla;

class MultipleBarriers : public ::testing::Test {
protected:
  static gurka::nodelayout layout;
  static gurka::ways ways;
  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
       C-----2----D
       |          |
       A-----1----B
    )";

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);

    ways = {
        {"AC2", {{"highway", "primary"}}},
        {"2DB", {{"highway", "primary"}}},
        {"A1", {{"highway", "primary"}}},
        {"1B", {{"highway", "primary"}}},
    };
  }

  void check_auto_path(const gurka::map& map, const std::vector<std::string>& expected_path) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
    gurka::assert::raw::expect_path(result, expected_path);
  }
};

gurka::nodelayout MultipleBarriers::layout = {};
gurka::ways MultipleBarriers::ways = {};

TEST_F(MultipleBarriers, DeniedBarrierAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"access", "no"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_no_access");
  check_auto_path(map, {"AC2", "2DB"});
}

TEST_F(MultipleBarriers, AllowedBarrierAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"access", "yes"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_allowed_access");
  check_auto_path(map, {"A1", "1B"});
}

TEST_F(MultipleBarriers, AllowedVehicleBarrierAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"motor_vehicle", "yes"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_vehicle_allowed");
  check_auto_path(map, {"A1", "1B"});
}

TEST_F(MultipleBarriers, NoInfoBarrierAccess) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_no_access_info");
  check_auto_path(map, {"AC2", "2DB"});
}

TEST_F(MultipleBarriers, TwoBarriers) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}}},
      {"2", {{"barrier", "gate"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_two_gates");
  check_auto_path(map, {"A1", "1B"});
}

TEST_F(MultipleBarriers, ClosedLongRoute) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"motor_vehicle", "no"}}},
      {"2", {{"barrier", "gate"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_closed_long");
  check_auto_path(map, {"AC2", "2DB"});
}

TEST_F(MultipleBarriers, ClosedShortRoute) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}}},
      {"2", {{"barrier", "gate"}, {"motor_vehicle", "no"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_closed_short");
  check_auto_path(map, {"A1", "1B"});
}

TEST_F(MultipleBarriers, BothClosed) {
  const gurka::nodes nodes = {
      {"1", {{"barrier", "gate"}, {"motor_vehicle", "no"}}},
      {"2", {{"barrier", "gate"}, {"access", "no"}}},
  };
  const gurka::map map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/multiple_barrier_both_closed");
  try {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }
}

class AccessibleBarriers : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
             B-1--C
            /      \
           /        \
          /          \
         /            \
        A----------2---D
         \            /
          F------3---E
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}}}, {"B1C", {{"highway", "primary"}}},
        {"CD", {{"highway", "primary"}}}, {"A2D", {{"highway", "primary"}}},
        {"AF", {{"highway", "primary"}}}, {"F3", {{"highway", "primary"}}},
        {"3E", {{"highway", "primary"}}}, {"ED", {{"highway", "primary"}}},
    };

    const gurka::nodes nodes = {
        // gate is opened
        {"1", {{"barrier", "gate"}, {"access", "yes"}}},
        // access is not specified, huge penalty is added.
        {"2", {{"barrier", "gate"}}},
        // gate is opened for bicycle only.
        {"3", {{"barrier", "lift_gate"}, {"access", "no"}, {"bicycle", "yes"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/accessible_barriers");
  }
};

gurka::map AccessibleBarriers::map = {};

TEST_F(AccessibleBarriers, Auto) {
  const std::string cost = "auto";
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, cost);
  gurka::assert::raw::expect_path(result, {"AB", "B1C", "B1C", "CD"});
}

TEST_F(AccessibleBarriers, Bicycle) {
  const std::string cost = "bicycle";
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, cost);
  gurka::assert::raw::expect_path(result, {"AF", "F3", "3E", "ED"});
}
