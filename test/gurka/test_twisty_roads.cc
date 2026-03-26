#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

namespace {

// Two alternative paths from A to B with the same road class and speed:
//
//   Straight: A --------- B   (1000m, curvature=0)
//   Serpentine: A-c-d-e-f-g-h-i-j-k-B  (1414m, curvature=15)
//
// The serpentine zigzags between rows 1 and 2 with 1-column spacing, creating
// tight 45-degree turns at each intermediate node. With curvature=15, maxspeed=80,
// and use_twisty_roads=1.0 the 75% cost discount (factor 0.25) makes the serpentine
// effectively ~301m (vs 850m for straight).
//
// ASCII grid (each cell = 100m, no leading indent so positions are exact):
//
//   A---------B         row 0: A(0,0) and B(10,0)
//    c e g i k          row 1: c(1,1) e(3,1) g(5,1) i(7,1) k(9,1)
//     d f h j           row 2: d(2,2) f(4,2) h(6,2) j(8,2)
//
// Serpentine: A(0,0)->c(1,1)->d(2,2)->e(3,1)->f(4,2)->g(5,1)->h(6,2)->i(7,1)->j(8,2)->k(9,1)->B(10,0)
// Each segment = sqrt(2)*100 ≈ 141m; 10 segments = 1414m total.
// Consecutive triplets c,d,e / d,e,f / ... / i,j,k each have circumradius = 100m
// (isoceles triangle, base=2 units, height=1 unit: R = a*b*c/(4*Area) = 2*sqrt(2)*sqrt(2)/(4*1) = 1)
// -> score = 1500/100 = 15 per triplet -> curvature = 15 (maximum).
//
// Cost analysis (motorcycle, base_factor=0.85 for primary density=0):
//   Default (use_twisty_roads=0.5): twisty_factor_=0.0, no modification
//     Serpentine: 1414*0.85 = 1202 > Straight: 1000*0.85 = 850 -> takes straight ✓
//   Prefer (use_twisty_roads=1.0): factor *= (1 - 1.0*1.0*0.75) = 0.25
//     Serpentine: 1414*0.85*0.25 = 301 < Straight: 850 -> takes twisty ✓
//   Avoid (use_twisty_roads=0.0): factor *= (1 + 1.0*1.0*0.75) = 1.75
//     Serpentine: 1414*0.85*1.75 = 2102 > Straight: 850 -> takes straight ✓

const std::string kTwistyMap = R"(
A---------B
 c e g i k
  d f h j
)";

class TwistyRoads : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const gurka::ways ways = {
        // Straight: A directly to B, 1000m, curvature=0
        {"AB", {{"highway", "primary"}, {"maxspeed", "80"}}},
        // Serpentine: same endpoints, zigzag shape points create curvature=15
        // Order: A, then alternating row1/row2 nodes left-to-right, then B
        {"AcdefghijkB", {{"highway", "primary"}, {"maxspeed", "80"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(kTwistyMap, 100);
    map_ = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_twisty_roads",
                             {{"mjolnir.concurrency", "1"}});
  }

  static gurka::map map_;
};

gurka::map TwistyRoads::map_ = {};

// Test 1: Default (use_twisty_roads=0.5) is neutral — takes the shorter straight path
TEST_F(TwistyRoads, DefaultTakesStraightPath) {
  auto result = gurka::do_action(Options::route, map_, {"A", "B"}, "motorcycle");
  gurka::assert::raw::expect_path(result, {"AB"});
}

// Test 2: use_twisty_roads=1.0 prefers twisty — should take the serpentine
TEST_F(TwistyRoads, PreferTwistyTakesCurvyPath) {
  auto result =
      gurka::do_action(Options::route, map_, {"A", "B"}, "motorcycle",
                       {{"/costing_options/motorcycle/use_twisty_roads", "1.0"}});
  const auto path = gurka::detail::get_paths(result).front();
  const std::vector<std::string> straight_path = {"AB"};
  EXPECT_NE(path, straight_path) << "Expected twisty path but got straight path";
}

// Test 3: use_twisty_roads=0.0 avoids twisty roads — takes the straight path
TEST_F(TwistyRoads, AvoidTwistyTakesStraightPath) {
  auto result =
      gurka::do_action(Options::route, map_, {"A", "B"}, "motorcycle",
                       {{"/costing_options/motorcycle/use_twisty_roads", "0.0"}});
  gurka::assert::raw::expect_path(result, {"AB"});
}

// Test 4: use_twisty_roads works for auto costing too
TEST_F(TwistyRoads, AutoCostingPrefersTwisty) {
  auto result =
      gurka::do_action(Options::route, map_, {"A", "B"}, "auto",
                       {{"/costing_options/auto/use_twisty_roads", "1.0"}});
  const auto path = gurka::detail::get_paths(result).front();
  const std::vector<std::string> straight_path = {"AB"};
  EXPECT_NE(path, straight_path) << "Expected twisty path but got straight path";
}

} // namespace

// Speed floor test — same geometry but with 30 km/h roads (below the 50 km/h floor).
// The twisty bonus must NOT apply, so the straight path is always preferred.
namespace {

const std::string kSlowTwistyMap = R"(
A---------B
 c e g i k
  d f h j
)";

class TwistyRoadsSpeedFloor : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const gurka::ways ways = {
        {"AB", {{"highway", "residential"}, {"maxspeed", "30"}}},
        {"AcdefghijkB", {{"highway", "residential"}, {"maxspeed", "30"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(kSlowTwistyMap, 100);
    map_ = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_twisty_roads_slow",
                             {{"mjolnir.concurrency", "1"}});
  }

  static gurka::map map_;
};

gurka::map TwistyRoadsSpeedFloor::map_ = {};

// Test 5: Speed floor — even at use_twisty_roads=1.0, no twisty bonus on <50 km/h roads.
// Router takes the straight (shorter) path.
TEST_F(TwistyRoadsSpeedFloor, SpeedFloorPreventsTwistyBonus) {
  auto result =
      gurka::do_action(Options::route, map_, {"A", "B"}, "motorcycle",
                       {{"/costing_options/motorcycle/use_twisty_roads", "1.0"}});
  gurka::assert::raw::expect_path(result, {"AB"});
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
