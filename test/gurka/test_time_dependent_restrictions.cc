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
