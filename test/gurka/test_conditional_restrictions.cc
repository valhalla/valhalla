#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

class ConditionalRestrictions : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
         A--B
         |  \
         D--C)";

    const gurka::ways ways =
        {{"AD", {{"highway", "service"}, {"motorcar", "no"}, {"motor_vehicle", "no"}}},
         {"AB", {{"highway", "service"},
             {"motorcar:conditional","yes @ (Mar 00:00-07:00);Mar 17:30-24:00;Apr 00:00-07:00;Apr 19:00-24:00;Aug 00:00-07:00;Aug 19:00-24:00;Sep 00:00-07:00;Sep 18:00-24:00;Oct 00:00-07:00;Oct 17:00-24:00;Jan-Feb 00:00-07:30;Jan-Feb 17:00-24:00;May-Jul 00:00-07:00;May-Jul 20:00-24:00)"},
             {"motor_vehicle:conditional","yes @ (Mar 00:00-07:00;Mar 17:30-24:00;Apr 00:00-07:00;Apr 19:00-24:00;Aug 00:00-07:00;Aug 19:00-24:00;Sep 00:00-07:00;Sep 18:00-24:00;Oct 00:00-07:00;Oct 17:00-24:00;Jan-Feb 00:00-07:30;Jan-Feb 17:00-24:00;May-Jul 00:00-07:00;May-Jul 20:00-24:00)"}
             }},
         {"BC", {{"highway", "service"},
             {"motorcar:conditional","yes @ (Mar 00:00-07:00;Mar 17:30-24:00;Apr 00:00-07:00;Apr 19:00-24:00;Aug 00:00-07:00;Aug 19:00-24:00;Sep 00:00-07:00;Sep 18:00-24:00;Oct 00:00-07:00;Oct 17:00-24:00;Jan-Feb 00:00-07:30;Jan-Feb 17:00-24:00;May-Jul 00:00-07:00;May-Jul 20:00-24:00)"},
             {"motor_vehicle:conditional","yes @ (Mar 00:00-07:00;Mar 17:30-24:00;Apr 00:00-07:00;Apr 19:00-24:00;Aug 00:00-07:00;Aug 19:00-24:00;Sep 00:00-07:00;Sep 18:00-24:00;Oct 00:00-07:00;Oct 17:00-24:00;Jan-Feb 00:00-07:30;Jan-Feb 17:00-24:00;May-Jul 00:00-07:00;May-Jul 20:00-24:00)"}
             }},
         {"CD", {{"highway", "service"},
             {"motorcar:conditional","yes @ (Mar 00:00-07:00;Mar 17:30-24:00;Apr 00:00-07:00;Apr 19:00-24:00;Aug 00:00-07:00;Aug 19:00-24:00;Sep 00:00-07:00;Sep 18:00-24:00;Oct 00:00-07:00;Oct 17:00-24:00;Jan-Feb 00:00-07:30;Jan-Feb 17:00-24:00;May-Jul 00:00-07:00;May-Jul 20:00-24:00)"},
             {"motor_vehicle:conditional","yes @ (Mar 00:00-07:00;Mar 17:30-24:00;Apr 00:00-07:00;Apr 19:00-24:00;Aug 00:00-07:00;Aug 19:00-24:00;Sep 00:00-07:00;Sep 18:00-24:00;Oct 00:00-07:00;Oct 17:00-24:00;Jan-Feb 00:00-07:30;Jan-Feb 17:00-24:00;May-Jul 00:00-07:00;May-Jul 20:00-24:00)"}
             }}};


    const gurka::relations relations = {{{{gurka::node_member, "A", "from"}, {gurka::node_member, "D", "to"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map = gurka::buildtiles(layout, ways, {}, relations, "test/data/conditional_restrictions");
  }
};

gurka::map ConditionalRestrictions::map = {};

/*************************************************************/
TEST_F(ConditionalRestrictions, NoRestrictionAutoNoDate) {
  auto result = gurka::route(map, "A", "D", "auto");
  gurka::assert::osrm::expect_route(result, {"AB", "BC", "CD"});
}

TEST_F(ConditionalRestrictions, NoRestrictionAuto) {
  auto result = gurka::route(map, "A", "D", "auto", "2020-04-02T20:00");
  gurka::assert::osrm::expect_route(result, {"AB", "BC", "CD"});
}

TEST_F(ConditionalRestrictions, RestrictionAuto) {
// this tests that the expected exception is thrown
    EXPECT_THROW({
        try
        {
          auto result = gurka::route(map, "A", "D", "auto", "2020-04-02T12:00");
        }
        catch( const std::exception& e )
        {
            // and this tests that it has the correct message
            EXPECT_STREQ( "No path could be found for input", e.what() );
            throw;
        }
    }, std::exception );
}

