#include "gurka.h"
#include "proto/api.pb.h"

#include <gtest/gtest.h>

using namespace valhalla;

using tags = std::map<std::string, std::string>;

class TimeDependentTags : public ::testing::Test {
public:
  static gurka::ways ways;
  static gurka::nodelayout layout;
  static std::vector<gurka::relation_member> restriction_members;
  static std::vector<std::string> working_hours;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
          D----B----A
          |    |
          |    |
          E----C----F
     )";

    ways = {
        {"AB", {{"highway", "motorway"}}}, {"BC", {{"highway", "motorway"}}},
        {"BD", {{"highway", "motorway"}}}, {"CF", {{"highway", "motorway"}}},
        {"DE", {{"highway", "motorway"}}}, {"EC", {{"highway", "motorway"}}},
    };
    layout = gurka::detail::map_to_coordinates(ascii_map, 10);
    restriction_members = {
        {gurka::way_member, "AB", "from"},
        {gurka::way_member, "BC", "to"},
        {gurka::node_member, "B", "via"},
    };

    working_hours = {
        "2021-04-22T06:30", "2021-04-22T14:00", "2021-04-22T23:00",
        "2021-04-25T14:00", "2021-04-25T23:00",
    };
  }
};
gurka::ways TimeDependentTags::ways = {};
gurka::nodelayout TimeDependentTags::layout = {};
std::vector<gurka::relation_member> TimeDependentTags::restriction_members = {};
std::vector<std::string> TimeDependentTags::working_hours = {};

TEST_F(TimeDependentTags, HourRestrictions) {
  tags hour_only = {
      {"type", "restriction"},
      {"restriction", "no_left_turn"},
      {"hour_on", "07:00"},
      {"hour_off", "19:00"},
  };
  const gurka::relations restrictions = {{restriction_members, hour_only}};
  const gurka::map map =
      gurka::buildtiles(layout, ways, {}, restrictions,
                        "test/data/gurka_time_dependent_restrictions_hour",
                        {{"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"}});

  for (size_t x = 0; x < working_hours.size(); ++x) {
    auto result =
        gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto",
                         {{"/date_time/type", "1"}, {"/date_time/value", working_hours.at(x)}});
    if (x == 1 || x == 3) { //  Restriction is on.
      gurka::assert::raw::expect_path(result, {"AB", "BD", "DE", "EC", "CF"});
    } else { //  Restriction is off.
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CF"});
    }
  }
}

TEST_F(TimeDependentTags, DayRestrictions) {
  tags day_hour = {{"type", "restriction"},
                   {"restriction", "no_left_turn"},
                   {"day_on", "Monday"},
                   {"day_off", "Friday"}};
  const gurka::relations restrictions = {{restriction_members, day_hour}};
  const gurka::map map =
      gurka::buildtiles(layout, ways, {}, restrictions,
                        "test/data/gurka_time_dependent_restrictions_day",
                        {{"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"}});

  for (size_t x = 0; x < working_hours.size(); ++x) {
    auto result =
        gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto",
                         {{"/date_time/type", "1"}, {"/date_time/value", working_hours.at(x)}});
    if (x == 0 || x == 1 || x == 2) { //  Restriction is on.
      gurka::assert::raw::expect_path(result, {"AB", "BD", "DE", "EC", "CF"});
    } else { //  Restriction is off.
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CF"});
    }
  }
}

TEST_F(TimeDependentTags, DayAndHourRestrictions) {
  tags day_hour = {{"type", "restriction"}, {"restriction", "no_left_turn"},
                   {"day_on", "Monday"},    {"day_off", "Friday"},
                   {"hour_on", "07:00"},    {"hour_off", "19:00"}};
  const gurka::relations restrictions = {{restriction_members, day_hour}};
  const gurka::map map =
      gurka::buildtiles(layout, ways, {}, restrictions,
                        "test/data/gurka_time_dependent_restrictions_day_and_hour",
                        {{"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"}});

  for (size_t x = 0; x < working_hours.size(); ++x) {
    auto result =
        gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto",
                         {{"/date_time/type", "1"}, {"/date_time/value", working_hours.at(x)}});
    if (x == 1) { //  Restriction is on.
      gurka::assert::raw::expect_path(result, {"AB", "BD", "DE", "EC", "CF"});
    } else { //  Restriction is off.
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CF"});
    }
  }
}

TEST_F(TimeDependentTags, ConditionalEdgeRestriction) {
  // almost a year.
  tags day_hour = {{"type", "restriction"},
                   {"restriction:conditional", "no_left_turn @ (Apr 24-Apr 21 7:00-19:00)"}};
  const gurka::relations restrictions = {{restriction_members, day_hour}};
  const gurka::map map =
      gurka::buildtiles(layout, ways, {}, restrictions,
                        "test/data/gurka_time_dependent_restrictions_day_and_hour",
                        {{"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"}});

  for (size_t x = 0; x < working_hours.size(); ++x) {
    auto result =
        gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto",
                         {{"/date_time/type", "1"}, {"/date_time/value", working_hours.at(x)}});
    if (x == 3) { // Restriction is on.
      gurka::assert::raw::expect_path(result, {"AB", "BD", "DE", "EC", "CF"});
    } else { // Restriction is off.
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CF"});
    }
  }
}

TEST_F(TimeDependentTags, IgnorePartialRestriction) {
  std::vector<std::pair<std::string, std::string>> partial_restrictions{{"day_on", "Thursday"},
                                                                        {"day_off", "Thursday"},
                                                                        {"hour_on", "07:00"},
                                                                        {"hour_off", "19:00"}};

  for (const auto& partial_restriction : partial_restrictions) {
    tags partial_restriction_tags = {
        {"type", "restriction"},
        {"restriction", "no_left_turn"},
        partial_restriction,
    };
    const gurka::relations restrictions = {{restriction_members, partial_restriction_tags}};
    const gurka::map map =
        gurka::buildtiles(layout, ways, {}, restrictions,
                          "test/data/gurka_time_dependent_restrictions_partial",
                          {{"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"}});

    for (std::string& working_hour : working_hours) { //  Restriction is always off.
      auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto",
                                     {{"/date_time/type", "1"}, {"/date_time/value", working_hour}});
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CF"});
    }
  }
}
