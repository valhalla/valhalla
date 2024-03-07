#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

// Check that golf carts prefer lower classes of roads
class GolfCartPreference : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    // A network with a mixture of road classes, all of which are legally traversable
    const std::string ascii_map = R"(
        A--B--------------C
           |              |
           D--E-----------F
              |           |
              G-----------H
    )";
    const gurka::ways ways = {{"AB",
                               {{"highway", "secondary"}
                               }},
                              {"BC",
                               {{"highway", "secondary"},
                               }},

                              // N/S cross streets between secondary and res
                              {"BD",
                               {{"highway", "tertiary"},
                               }},
                              {"CF",
                               {{"highway", "tertiary"},
                               }},

                              // Middle residential road
                              {"DE",
                               {{"highway", "residential"},
                               }},
                              {"EF",
                               {{"highway", "residential"},
                               {"golf_cart", "destination"},
                               }},

                              // Cart paths (tagged with several variations)
                              {"EG",
                               {{"highway", "path"},
                                   {"surface", "paved"},
                               }},
                              {"GH",
                               {{"highway", "path"},
                                   {"surface", "paved"},
                                   {"golf_cart", "designated"},
                               }},
                              {"FH",
                               {{"highway", "path"},
                                   {"golf", "cartpath"},
                               }}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/golf_cart_preference");
  }
};

gurka::map GolfCartPreference::map = {};

/*************************************************************/

TEST_F(GolfCartPreference, CheckGolfCartPreference) {
  std::unordered_map<std::string, std::string> options = {
      {"/costing_options/low_speed_vehicle/vehicle_type", "golf_cart"}};
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "low_speed_vehicle", options);
  gurka::assert::osrm::expect_steps(result, {"AB", "BD", "DE", "EG", "GH", "FH"});
  gurka::assert::raw::expect_path(result, {"AB", "BD", "DE", "EG", "GH", "FH", "CF"});
}
