#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class RouteWithPronunciation : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
                       J
                       |
                       |
                       |
                       I
                      /|\
                    /  |  \
                  /    |    \
           L----K-------------H----G
           A----B-------------E----F
                  \    |    /
                    \  |  /
                      \|/
                       C
                       |
                       |
                       |
                       D


    )";

    const gurka::ways ways =
        {{"ABEF", {{"highway", "motorway"}, {"name", ""}, {"ref", "I 70"}, {"oneway", "yes"}}},
         {"GHKL", {{"highway", "motorway"}, {"name", ""}, {"ref", "I 70"}, {"oneway", "yes"}}},
         {"JICD",
          {{"highway", "primary"},
           {"name", "Lancaster Road"},
           {"name:pronunciation", "ˈlæŋkəstər ˈɹoʊd"},
           {"ref", "SR 37"}}},
         {"BC",
          {{"highway", "motorway_link"},
           {"name", ""},
           {"oneway", "yes"},
           {"junction:ref", "126"},
           {"destination", "Granville;Lancaster"},
           {"destination:pronunciation", "ˈgɹænvɪl;ˈlæŋkəstər"},
           {"destination:ref", "SR 37"}}},
         {"CE",
          {{"highway", "motorway_link"},
           {"name", ""},
           {"oneway", "yes"},
           {"destination:ref", "I 70 East"}}},
         {"HI",
          {{"highway", "motorway_link"},
           {"name", ""},
           {"oneway", "yes"},
           {"junction:ref", "126"},
           {"destination", "Granville;Lancaster"},
           {"destination:pronunciation", "ˈgɹænvɪl;ˈlæŋkəstər"},
           {"destination:ref", "SR 37"}}},
         {"IK",
          {{"highway", "motorway_link"},
           {"name", ""},
           {"oneway", "yes"},
           {"destination:ref", "I 70 West"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_route_with_pronunciation",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map RouteWithPronunciation::map = {};

/*************************************************************/

TEST_F(RouteWithPronunciation, CheckStreetNamesAndSigns) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");
  gurka::assert::raw::expect_path(result, {"I 70", "", "Lancaster Road/SR 37"});

  // Verify starting on I 70
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).street_name(0).value(), "I 70");

  // Verify sign pronunciations - alphabet & value
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).sign().exit_onto_streets_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).sign().exit_onto_streets(0).text(),
            "SR 37");
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).sign().exit_toward_locations_size(), 2);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).sign().exit_toward_locations(0).text(),
            "Granville");
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(1)
                .sign()
                .exit_toward_locations(0)
                .pronunciation()
                .alphabet(),
            Pronunciation_Alphabet_kIpa);
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(1)
                .sign()
                .exit_toward_locations(0)
                .pronunciation()
                .value(),
            "ˈgɹænvɪl");
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).sign().exit_toward_locations(1).text(),
            "Lancaster");
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(1)
                .sign()
                .exit_toward_locations(1)
                .pronunciation()
                .alphabet(),
            Pronunciation_Alphabet_kIpa);
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(1)
                .sign()
                .exit_toward_locations(1)
                .pronunciation()
                .value(),
            "ˈlæŋkəstər");

  // Verify street name pronunciation - alphabet & value
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(2).street_name_size(), 2);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(2).street_name(0).value(),
            "Lancaster Road");
  EXPECT_EQ(result.directions()
                .routes(0)
                .legs(0)
                .maneuver(2)
                .street_name(0)
                .pronunciation()
                .alphabet(),
            Pronunciation_Alphabet_kIpa);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(2).street_name(0).pronunciation().value(),
            "ˈlæŋkəstər ˈɹoʊd");
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(2).street_name(1).value(), "SR 37");
}
