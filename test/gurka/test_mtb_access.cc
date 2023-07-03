#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

// Check that MTB tags override SAC scale and allow bicycle access
class MtbAccess : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    // A--B
    const std::string ascii_map = R"(A----B----C)";
    const gurka::ways ways = {{"AB",
                               {{"highway", "cycleway"},
                                {"sac_scale", "mountain_hiking"},
                                {"mtb:scale:uphill", "2"},
                                {"foot", "designated"}}},
                              {"BC",
                               {{"highway", "cycleway"},
                                {"sac_scale", "mountain_hiking"},
                                {"mtb:scale:uphill", "2"},
                                {"foot", "designated"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/mtb_access");
  }
};

gurka::map MtbAccess::map = {};

/*************************************************************/

TEST_F(MtbAccess, CheckMtbAccess) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "bicycle");
  gurka::assert::osrm::expect_steps(result, {"AB"});
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}
