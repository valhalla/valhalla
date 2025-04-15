#include "gurka.h"
#include "proto/api.pb.h"

#include <gtest/gtest.h>

using namespace valhalla;

class TimeDependentRestrictions : public testing::TestWithParam<int> {
public:
  static gurka::map map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
          D----B----A
          |    |
          |    |
          E----C----F
     )";

    const gurka::ways ways = {
        {"AB", {{"highway", "motorway"}}}, {"BC", {{"highway", "motorway"}}},
        {"BD", {{"highway", "motorway"}}}, {"CF", {{"highway", "motorway"}}},
        {"DE", {{"highway", "motorway"}}}, {"EC", {{"highway", "motorway"}}},
    };
    const gurka::nodelayout layout = gurka::detail::map_to_coordinates(ascii_map, 10);
    const gurka::relations relations = {{{
                                             {gurka::way_member, "AB", "from"},
                                             {gurka::way_member, "BC", "to"},
                                             {gurka::node_member, "B", "via"},
                                         },
                                         {{"type", "restriction"},
                                          {"restriction", "no_left_turn"},
                                          {"hour_on", "07:00"},
                                          {"hour_off", "19:00"}}}};
    map =
        gurka::buildtiles(layout, ways, {}, relations, "test/data/gurka_time_dependent_restrictions",
                          {{"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"}});
  }
};
gurka::map TimeDependentRestrictions::map = {};

TEST_P(TimeDependentRestrictions, NoRestrictionEarly) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto",
                                 {{"/date_time/type", std::to_string(GetParam())},
                                  {"/date_time/value", "2021-04-02T06:30"}});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CF"});
}

TEST_P(TimeDependentRestrictions, ActiveHourRestriction) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto",
                                 {{"/date_time/type", std::to_string(GetParam())},
                                  {"/date_time/value", "2021-04-02T14:00"}});
  gurka::assert::raw::expect_path(result, {"AB", "BD", "DE", "EC", "CF"});
}

TEST_P(TimeDependentRestrictions, NoRestrictionsLate) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto",
                                 {{"/date_time/type", std::to_string(GetParam())},
                                  {"/date_time/value", "2021-04-02T19:20"}});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CF"});
}

// Note: do not run these tests with date_time_type=0.
// It overwrites specified in the query time by local time.
INSTANTIATE_TEST_SUITE_P(Restrictions, TimeDependentRestrictions, testing::Range(1, 4));

class TimeDependentRestrictionsInDifferentTimezone : public testing::TestWithParam<int> {
public:
  static gurka::map map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
          A----B----C----D
               |         |
               |         |
               E----F----G
     )";

    const gurka::ways ways = {
        {"AB", {{"highway", "motorway"}}}, {"BC", {{"highway", "motorway"}}},
        {"CD", {{"highway", "motorway"}}}, {"BE", {{"highway", "motorway"}}},
        {"DG", {{"highway", "motorway"}}}, {"EF", {{"highway", "motorway"}}},
        {"FG", {{"highway", "motorway"}}},
    };
    const gurka::nodelayout layout =
        gurka::detail::map_to_coordinates(ascii_map, 100, {-114.0556, 39.057120});

    const gurka::relations relations = {{{
                                             {gurka::way_member, "BC", "from"},
                                             {gurka::way_member, "CD", "to"},
                                             {gurka::node_member, "C", "via"},
                                         },
                                         {{"type", "restriction"},
                                          {"restriction", "no_straight_on"},
                                          {"hour_on", "08:00"},
                                          {"hour_off", "09:00"}}}};
    map =
        gurka::buildtiles(layout, ways, {}, relations, "test/data/gurka_time_dependent_restrictions",
                          {{"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"}});
  }
};
gurka::map TimeDependentRestrictionsInDifferentTimezone::map = {};

TEST_P(TimeDependentRestrictionsInDifferentTimezone, NoRestrictionEarly) {
  auto timeValue = GetParam() == 1
                       ? "2021-01-02T06:30"
                       : "2021-01-02T07:30"; // Earlier time at the neighbor tz or at the restriction
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto",
                                 {{"/date_time/type", std::to_string(GetParam())},
                                  {"/date_time/value", timeValue}});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD"});
}

TEST_P(TimeDependentRestrictionsInDifferentTimezone, ActiveHourRestriction) {
  auto timeValue = GetParam() == 1
                       ? "2021-01-02T07:30"
                       : "2021-01-02T08:30"; // Active time at the neighbor tz or at the restriction
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto",
                                 {{"/date_time/type", std::to_string(GetParam())},
                                  {"/date_time/value", timeValue}});
  gurka::assert::raw::expect_path(result, {"AB", "BE", "EF", "FG", "DG"});
}

TEST_P(TimeDependentRestrictionsInDifferentTimezone, NoRestrictionsLate) {
  auto timeValue = GetParam() == 1
                       ? "2021-01-02T08:30"
                       : "2021-01-02T09:30"; // Later time at the neighbor tz or at the restriction
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto",
                                 {{"/date_time/type", std::to_string(GetParam())},
                                  {"/date_time/value", timeValue}});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD"});
}

INSTANTIATE_TEST_SUITE_P(Restrictions,
                         TimeDependentRestrictionsInDifferentTimezone,
                         testing::Range(1, 3));
