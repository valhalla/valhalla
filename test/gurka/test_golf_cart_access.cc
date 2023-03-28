#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

// Check that golf carts have access to the correct roads
class GolfCartAccess : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 500;

    // A network with varying levels of golf cart access
    const std::string ascii_map = R"(
        A--B--C--D--E--F--G--H--I--J
                          |  |  |  |
                          K--L--M--N
                                |
                                O---P
                                |   |
                                Q---R
                                |\
                                S-T
                                |  \
                                U---V
    )";
    const gurka::ways ways = {{"AB",
                               {{"highway", "residential"}
                               }},
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
                               {{"highway", "service"},
                                   {"service", "parking_aisle"},  // Should be allowed
                               }},
                              {"FG",
                               {{"highway", "path"},
                                   {"surface", "paved"},
                               }},
                              {"GH",
                               {{"highway", "path"},  // not paved!
                               }},
                              {"HI",
                               {{"highway", "secondary"},
                               }},
                              {"IJ",
                               {{"highway", "path"},
                                   {"surface", "paved"},
                                   {"motor_vehicle", "no"},  // Tricky; motor_vehicle=no...
                                   {"golf_cart", "yes"},     // ... but golf_cart=yes
                               }},

                              // N/S cross streets
                              {"GK",
                               {{"highway", "residential"},
                               }},
                              {"HL",
                               {{"highway", "residential"},
                               }},
                              {"IM",
                               {{"highway", "primary"},
                               }},
                              {"JN",
                               {{"highway", "residential"},
                               }},

                              // Second set of N/S connectors
                              {"MO",
                               {{"highway", "residential"},
                               }},
                              {"OQ",
                               {{"highway", "residential"},
                                   {"maxspeed", "40mph"}  // Exceeds allowable speed of 60kph
                               }},
                              {"PR",
                               {{"highway", "residential"},
                               }},

                              // Middle E/W highway segments
                              {"KL",
                               {{"highway", "residential"},
                               }},
                              {"LM",
                               {{"highway", "primary"},  // No access for golf carts
                               }},
                              {"MN",
                               {{"highway", "residential"},
                               }},

                              // Lower
                              {"OP",
                               {{"highway", "residential"},
                               }},
                              {"QR",
                               {{"highway", "residential"},
                               }},

                              // Triangle
                              {"QS",
                               {{"highway", "residential"},
                                   {"golf_cart", "private"},
                               }},
                              {"QT",
                               {{"highway", "secondary"},
                                   {"maxspeed", "30mph"}
                               }},
                              {"ST",
                               {{"highway", "residential"},
                               }},
                              {"SU",
                               {{"highway", "residential"},
                                   {"access", "private"},
                               }},
                              {"TV",
                               {{"highway", "secondary"},
                               }},
                              {"UV",
                               {{"highway", "secondary"},
                               }}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/golf_cart_access");
  }
};

gurka::map GolfCartAccess::map = {};

/*************************************************************/

TEST_F(GolfCartAccess, CheckGolfCartAccess) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "U"}, "golf_cart");
  gurka::assert::osrm::expect_steps(result, {"AB", "GK", "KL", "HL", "HI", "JN", "MN", "MO", "OP", "PR", "QR", "QT", "UV"});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GK", "KL", "HL", "HI", "IJ", "JN", "MN", "MO", "OP", "PR", "QR", "QT", "TV", "UV"});
}

TEST_F(GolfCartAccess, CheckGolfCartAccessReducedSpeedLimit) {
  try {
    std::unordered_map<std::string, std::string> options = {
      {"/costing_options/golf_cart/max_allowed_speed_limit", "47"}};
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "U"}, "golf_cart", options);
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }
}
