#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

/*
 * Tests the timeAllowed and timeDenied conditional restrictions for auto, bicycle and pedestrian
 * costing. NOTE: To test restrictions, the map must contain more than 1 edge.  If you only have 1
 * edge, then it will never set the date_time info because its traversing a trivial path (see
 * https://github.com/valhalla/valhalla/blob/master/src/thor/triplegbuilder.cc#L1216)
 */
class ConditionalRestrictions : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
         A----B
         |    |
         D    C
          \  /
           E    
    )";

    const std::string timeDenied =
        "no @ (Mar 00:00-07:00;Mar 17:30-24:00;Apr 00:00-07:00;Apr 19:00-24:00;Aug 00:00-07:00;Aug 19:00-24:00)";
    const std::string timeAllowed =
        "yes @ (Mar 00:00-07:00;Mar 17:30-24:00;Apr 00:00-07:00;Apr 19:00-24:00;Aug 00:00-07:00;Aug 19:00-24:00)";
    const gurka::ways ways = {
        {"AD",
         {{"highway", "service"},
          {"motorcar", "no"},
          {"bicycle", "yes"},
          {"foot", "yes"},
          {"bicycle:conditional", timeDenied},
          {"foot:conditional", timeAllowed}}},
        {"AB",
         {{"highway", "service"},
          {"motorcar:conditional", timeAllowed},
          {"bicycle:conditional", timeDenied},
          {"foot:conditional", timeAllowed}}},
        {"BC", {{"highway", "service"}, {"motorcar:conditional", timeAllowed}}},
        {"CD",
         {{"highway", "service"},
          {"motorcar:conditional", timeAllowed},
          {"bicycle:conditional", timeDenied},
          {"foot:conditional", timeAllowed}}},
        {"DE",
         {{"highway", "service"},
          {"motorcar:conditional", timeAllowed},
          {"bicycle:conditional", timeDenied},
          {"foot:conditional", timeAllowed}}},
        {"CE",
         {{"highway", "service"},
          {"motorcar:conditional", timeAllowed},
          {"bicycle:conditional", timeDenied},
          {"foot:conditional", timeAllowed}}},

    };

    const gurka::relations relations = {
        {{{gurka::node_member, "A", "from"}, {gurka::node_member, "D", "to"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map =
        gurka::buildtiles(layout, ways, {}, relations, "test/data/conditional_restrictions",
                          {{"mjolnir.timezone", {VALHALLA_SOURCE_DIR "build/test/data/tz.sqlite"}}});
  }
};

gurka::map ConditionalRestrictions::map = {};

/*************************************************************/

TEST_F(ConditionalRestrictions, NoRestrictionAutoNoDate) {
  auto result = gurka::route(map, "A", "E", "auto");
  gurka::assert::osrm::expect_steps(result, {"AB", "BC", "CE"});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CE"});
}

TEST_F(ConditionalRestrictions, NoRestrictionAuto) {
  auto result = gurka::route(map, "A", "E", "auto", "2020-04-15T06:00");
  gurka::assert::osrm::expect_steps(result, {"AB", "BC", "CE"});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CE"});
}

TEST_F(ConditionalRestrictions, RestrictionAuto) {
  // this tests that the expected exception is thrown
  EXPECT_THROW(
      {
        try {
          auto result = gurka::route(map, "A", "E", "auto", "2020-04-02T12:00");
        } catch (const std::exception& e) {
          // and this tests that it has the correct message
          EXPECT_STREQ("No path could be found for input", e.what());
          throw;
        }
      },
      std::exception);
}

TEST_F(ConditionalRestrictions, NoRestrictionBikeNoDate) {
  auto result = gurka::route(map, "A", "E", "bicycle");
  gurka::assert::osrm::expect_steps(result, {"AD", "DE"});
  gurka::assert::raw::expect_path(result, {"AD", "DE"});
}

TEST_F(ConditionalRestrictions, NoRestrictionBike) {
  auto result = gurka::route(map, "A", "E", "bicycle", "2020-04-02T12:00");
  gurka::assert::osrm::expect_steps(result, {"AD", "DE"});
  gurka::assert::raw::expect_path(result, {"AD", "DE"});
}

TEST_F(ConditionalRestrictions, RestrictionBike) {
  // this tests that the expected exception is thrown
  EXPECT_THROW(
      {
        try {
          auto result = gurka::route(map, "A", "E", "bicycle", "2020-04-02T20:00");
        } catch (const std::exception& e) {
          // and this tests that it has the correct message
          EXPECT_STREQ("No path could be found for input", e.what());
          throw;
        }
      },
      std::exception);
}

TEST_F(ConditionalRestrictions, NoRestrictionPedestrianNoDate) {
  auto result = gurka::route(map, "A", "E", "pedestrian");
  gurka::assert::osrm::expect_steps(result, {"AD", "DE"});
  gurka::assert::raw::expect_path(result, {"AD", "DE"});
}

TEST_F(ConditionalRestrictions, NoRestrictionPedestrian) {
  auto result = gurka::route(map, "A", "E", "pedestrian", "2020-04-02T20:00");
  gurka::assert::osrm::expect_steps(result, {"AD", "DE"});
  gurka::assert::raw::expect_path(result, {"AD", "DE"});
}

TEST_F(ConditionalRestrictions, RestrictionPedestrian) {
  // this tests that the expected exception is thrown
  EXPECT_THROW(
      {
        try {
          auto result = gurka::route(map, "A", "E", "pedestrian", "2020-04-02T12:00");
        } catch (const std::exception& e) {
          // and this tests that it has the correct message
          EXPECT_STREQ("No path could be found for input", e.what());
          throw;
        }
      },
      std::exception);
}
