#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

// Check that golf cart tags override others to allow access
class GolfCartAccess : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 500;

    // A network with varying levels of golf cart access
    const std::string ascii_map = R"(
        A----B----C----D----E----F----G----H
                                      |    |
                                      I----J
    )";
    const gurka::ways ways = {{"AB",
                               {{"highway", "residential"},
                                {"golf_cart", "designated"}}},
                              {"BC",
                               {{"highway", "living_street"},
                               }},
                              {"CD",
                               {{"highway", "tertiary"},
                               }},
                              {"DE",
                               {{"highway", "service"},
                               }},
                              {"EF",
                               {{"highway", "path"},
                                   {"surface", "paved"},
                               }},
                              {"FG",
                               {{"highway", "path"},
                                   {"surface", "paved"},
                               }},
                              {"GH",
                               {{"highway", "path"},  // not paved
                               }},
                              {"GI",
                               {{"highway", "path"},
                                   {"surface", "paved"},
                               }},
                              {"IJ",
                               {{"highway", "path"},
                                   {"surface", "paved"},
                               }},
                              {"HJ",
                               {{"highway", "path"},
                                   {"surface", "paved"},
                               }}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/mtb_access");
  }
};

gurka::map GolfCartAccess::map = {};

/*************************************************************/

TEST_F(GolfCartAccess, CheckGolfCartAccess) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "golf_cart");
  gurka::assert::osrm::expect_steps(result, {"AB", "GI", "IJ", "HJ"});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GI", "IJ", "HJ"});
}
