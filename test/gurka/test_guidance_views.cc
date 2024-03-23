#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class GuidanceViews : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {

    // A--B-BASE-C--D-OVERLAY-E--F
    const std::string ascii_map = R"(
      A----B----C----X
                 \
                  D
                   \
               E----F----G
    )";
    const gurka::ways ways =
        {{"AB", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "National Route 1"}}},
         {"BC",
          {{"highway", "motorway"},
           {"oneway", "yes"},
           {"name", "National Route 1"},
           {"guidance_view:jct:base", "PA717;1"}}},
         {"CX", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "National Route 1"}}},
         {"CD", {{"highway", "motorway_link"}, {"oneway", "yes"}, {"name", "xyz ramp"}}},
         {"DF",
          {{"highway", "motorway_link"},
           {"oneway", "yes"},
           {"name", "xyz ramp"},
           {"guidance_view:jct:overlay", "PA717;E"}}},

         {"EFG", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "National Route 2"}}}};

    const gurka::nodes nodes = {{"C", {{"highway", "motorway_junction"}, {"ref", "10"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/guidance_views", {});
  }
};

gurka::map GuidanceViews::map = {};

/*************************************************************/

TEST_F(GuidanceViews, CheckGuidanceViews) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto");

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).guidance_views_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).guidance_views(0).type(),
            DirectionsLeg_GuidanceView_Type_kJunction);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).guidance_views(0).base_id(), "PA7171");
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).guidance_views(0).overlay_ids(0),
            "PA717E");
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).guidance_views(0).data_id(), "1001");
}
