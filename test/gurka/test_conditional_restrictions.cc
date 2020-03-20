#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

const gurka::relations relations_auto = {
    {{{gurka::way_member, "AB", "from"}, {gurka::way_member, "ED", "to"}},
     {{"type", "restriction"},
      {"restriction:conditional",
       "yes @ Mar 00:00-07:00;Mar 17:30-24:00;Apr 00:00-07:00;Apr 19:00-24:00;Aug 00:00-07:00;Aug 19:00-24:00;Sep 00:00-07:00;Sep 18:00-24:00;Oct 00:00-07:00;Oct 17:00-24:00;Jan-Feb 00:00-07:30;Jan-Feb 17:00-24:00;May-Jul 00:00-07:00;May-Jul 20:00-24:00"}}}};
const gurka::relations relations_bicycle = {
    {{{gurka::way_member, "AB", "from"}, {gurka::way_member, "ED", "to"}},
     {{"type", "restriction"},
      {"restriction:conditional",
       "no @ Mar 00:00-07:00;Mar 17:30-24:00;Apr 00:00-07:00;Apr 19:00-24:00;Aug 00:00-07:00;Aug 19:00-24:00;Sep 00:00-07:00;Sep 18:00-24:00;Oct 00:00-07:00;Oct 17:00-24:00;Jan-Feb 00:00-07:30;Jan-Feb 17:00-24:00;May-Jul 00:00-07:00;May-Jul 20:00-24:00"}}}};
const gurka::relations relations_pedestrian = {
    {{{gurka::way_member, "AB", "from"}, {gurka::way_member, "ED", "to"}},
     {{"type", "restriction"},
      {"restriction:conditional",
       "yes @ Mar 00:00-07:00;Mar 17:30-24:00;Apr 00:00-07:00;Apr 19:00-24:00;Aug 00:00-07:00;Aug 19:00-24:00;Sep 00:00-07:00;Sep 18:00-24:00;Oct 00:00-07:00;Oct 17:00-24:00;Jan-Feb 00:00-07:30;Jan-Feb 17:00-24:00;May-Jul 00:00-07:00;May-Jul 20:00-24:00"}}}};

class ConditionalRestrictions : public ::testing::Test {
protected:
  static gurka::map map_auto;
  static gurka::map map_bicycle;
  static gurka::map map_pedestrian;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
    F----A----B
    |    |    \
    E----D----C)";

    const gurka::ways ways =
        {{"AB", {{"highway", "service"}}},
         {"BC", {{"highway", "service"}}},
         {"DC", {{"highway", "service"}}},
         {"ED", {{"highway", "service"}}},
         {"AD", {{"highway", "service"}, {"car", "no"}, {"bicycle", "yes"}, {"pedestrian", "yes"}}},
         {"FA", {{"highway", "service"}}},
         {"EF", {{"highway", "service"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map_auto =
        gurka::buildtiles(layout, ways, {}, relations_auto, "test/data/conditional_restrictions_1");
    map_bicycle = gurka::buildtiles(layout, ways, {}, relations_bicycle,
                                    "test/data/conditional_restrictions_2");
    map_pedestrian = gurka::buildtiles(layout, ways, {}, relations_pedestrian,
                                       "test/data/conditional_restrictions_3");
  }
};

gurka::map ConditionalRestrictions::map_auto = {};
gurka::map ConditionalRestrictions::map_bicycle = {};
gurka::map ConditionalRestrictions::map_pedestrian = {};

/*************************************************************/
TEST_F(ConditionalRestrictions, NoRestrictionAuto) {
  auto result = gurka::route(map_auto, "A", "D", "auto", "2020-04-02T19:00");
  gurka::assert::osrm::expect_route(result, {"AD"});
}

TEST_F(ConditionalRestrictions, RestrictionAuto) {
  auto result = gurka::route(map_auto, "A", "D", "auto", "2020-04-02T12:00");
  gurka::assert::osrm::expect_no_route(result);
}

TEST_F(ConditionalRestrictions, NoRestrictionBike) {
  auto result = gurka::route(map_bicycle, "A", "D", "bicycle", "2020-04-02T12:00");
  gurka::assert::osrm::expect_route(result, {"AD"});
}

TEST_F(ConditionalRestrictions, RestrictionBike) {
  auto result = gurka::route(map_bicycle, "A", "D", "bicycle", "2020-04-02T19:00");
  gurka::assert::osrm::expect_no_route(result);
}

TEST_F(ConditionalRestrictions, NoRestrictionPedestrian) {
  auto result = gurka::route(map_pedestrian, "A", "D", "pedestrian", "2020-04-02T19:00");
  gurka::assert::osrm::expect_route(result, {"AD"});
}

TEST_F(ConditionalRestrictions, RestrictionPedestrian) {
  auto result = gurka::route(map_pedestrian, "A", "D", "pedestrian", "2020-04-02T12:00");
  gurka::assert::osrm::expect_no_route(result);
}
