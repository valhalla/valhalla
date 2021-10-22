#include "gurka.h"
#include "proto/api.pb.h"

#include <gtest/gtest.h>

using namespace valhalla;

class ProbableRestrictions : public testing::TestWithParam<int> {
public:
  static gurka::map map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
     I----G----D----B----A
     |    |    |    |
     |    |    |    |
     J----H----E----C----F
     )";

    const gurka::ways ways = {
        {"AB", {{"highway", "motorway"}}}, {"BC", {{"highway", "motorway"}}},
        {"BD", {{"highway", "motorway"}}}, {"CF", {{"highway", "motorway"}}},
        {"DE", {{"highway", "motorway"}}}, {"EC", {{"highway", "motorway"}}},
        {"DG", {{"highway", "motorway"}}}, {"GH", {{"highway", "motorway"}}},
        {"HE", {{"highway", "motorway"}}}, {"GI", {{"highway", "motorway"}}},
        {"IJ", {{"highway", "motorway"}}}, {"JH", {{"highway", "motorway"}}},
    };
    const gurka::nodelayout layout = gurka::detail::map_to_coordinates(ascii_map, 10);
    const gurka::relations relations =
        {{{
              {gurka::way_member, "AB", "from"},
              {gurka::way_member, "BC", "to"},
              {gurka::node_member, "B", "via"},
          },
          {{"type", "restriction"},
           // probable restriction will get tossed due to a regular restriction
           {"restriction:probable", "no_left_turn @ probability=100"},
           {"restriction", "no_left_turn"},
           {"hour_on", "07:00"},
           {"hour_off", "19:00"}}},
         {{
              {gurka::way_member, "BD", "from"},
              {gurka::way_member, "DE", "to"},
              {gurka::node_member, "D", "via"},
          },
          {{"type", "restriction"},
           {"restriction:conditional", "no_left_turn @ (07:00-19:00)"},
           // probable restriction will get tossed due to a conditional restriction
           {"restriction:probable", "no_left_turn @ probability=100"}}},
         {{
              {gurka::way_member, "DG", "from"},
              {gurka::way_member, "GH", "to"},
              {gurka::node_member, "G", "via"},
          },
          {{"type", "restriction"},
           // restriction will be ignored because the probability is less than 100
           {"restriction:probable", "no_left_turn @ probability=75"}}},
         {{
              {gurka::way_member, "HE", "from"},
              {gurka::way_member, "EC", "to"},
              {gurka::node_member, "E", "via"},
          },
          {{"type", "restriction"}, {"restriction:probable", "only_straight_on @ probability=100"}}},
         {{
              {gurka::way_member, "GI", "from"},
              {gurka::way_member, "IJ", "to"},
              {gurka::node_member, "I", "via"},
          },
          {{"type", "restriction"}, {"restriction:probable", "no_left_turn @ probability=0"}}}};
    map = gurka::buildtiles(layout, ways, {}, relations, "test/data/gurka_time_probable_restrictions",
                            {{"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"}});
  }
};
gurka::map ProbableRestrictions::map = {};

TEST_P(ProbableRestrictions, NoRestrictionEarly) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto",
                                 {{"/date_time/type", std::to_string(GetParam())},
                                  {"/date_time/value", "2021-04-02T06:30"}});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CF"});
}

TEST_P(ProbableRestrictions, ActiveHourRestriction) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto",
                                 {{"/date_time/type", std::to_string(GetParam())},
                                  {"/date_time/value", "2021-04-02T14:00"}});
  gurka::assert::raw::expect_path(result, {"AB", "BD", "DG", "GH", "HE", "EC", "CF"});
}

TEST_P(ProbableRestrictions, ActiveHourRestrictionWithProbability) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto",
                                 {{"/date_time/type", std::to_string(GetParam())},
                                  {"/date_time/value", "2021-04-02T14:00"},
                                  {"/costing_options/auto/restriction_probability", "40"}});
  gurka::assert::raw::expect_path(result, {"AB", "BD", "DG", "GI", "IJ", "JH", "HE", "EC", "CF"});
}

TEST_P(ProbableRestrictions, ActiveHourRestrictionWithHighProbability) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto",
                                 {{"/date_time/type", std::to_string(GetParam())},
                                  {"/date_time/value", "2021-04-02T14:00"},
                                  {"/costing_options/auto/restriction_probability", "1040"}});
  gurka::assert::raw::expect_path(result, {"AB", "BD", "DG", "GH", "HE", "EC", "CF"});
}

TEST_P(ProbableRestrictions, NoRestrictionEarlyWithLowProbability) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto",
                                 {{"/date_time/type", std::to_string(GetParam())},
                                  {"/date_time/value", "2021-04-02T06:30"},
                                  {"/costing_options/auto/restriction_probability", "0"}});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CF"});
}

TEST_P(ProbableRestrictions, NoRestrictionsLate) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto",
                                 {{"/date_time/type", std::to_string(GetParam())},
                                  {"/date_time/value", "2021-04-02T19:20"}});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CF"});
}

// Note: do not run these tests with date_time_type=0.
// It overwrites specified in the query time by local time.
INSTANTIATE_TEST_SUITE_P(Restrictions, ProbableRestrictions, testing::Range(1, 4));
