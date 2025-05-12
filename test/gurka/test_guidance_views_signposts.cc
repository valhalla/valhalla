#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class GuidanceViews_Signboards : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {

    // A--B-C-SIGNBOARD-D-E--F
    const std::string ascii_map = R"(
      A----B----C----X
                 \
                  D
                   \
               E----F----G
    )";
    const gurka::ways ways =
        {{"AB", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "National Route 1"}}},
         {"BC", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "National Route 1"}}},
         {"CX", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "National Route 1"}}},
         {"CD",
          {{"highway", "motorway_link"},
           {"oneway", "yes"},
           {"name", "xyz ramp"},
           {"guidance_view:signboard:base", "SI_53271604;A1"}}},
         {"DF", {{"highway", "motorway_link"}, {"oneway", "yes"}, {"name", "xyz ramp"}}},
         {"EFG", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "National Route 2"}}}};

    const gurka::nodes nodes = {{"C", {{"highway", "motorway_junction"}, {"ref", "A1"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/guidance_views_signboards", {});
  }
};

gurka::map GuidanceViews_Signboards::map = {};

/*************************************************************/

TEST_F(GuidanceViews_Signboards, CheckGuidanceViews) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto");

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).guidance_views_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).guidance_views(0).type(),
            DirectionsLeg_GuidanceView_Type_kSignboard);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).guidance_views(0).base_id(),
            "SI_53271604A1");
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).guidance_views(0).data_id(), "1001");

  result = gurka::do_action(valhalla::Options::route, map, {"A", "X"}, "auto");
  // should be no guidance views
  for (int i = 0; i < result.directions().routes(0).legs(0).maneuver_size(); ++i) {
    EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(i).guidance_views_size(), 0);
  }
}
