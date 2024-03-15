#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;

TEST(StandAlone, CostMatrixTrivialRoutes) {
  const std::string ascii_map = R"(
    A-2---1-B
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}, {"maxspeed:forward", "100"}, {"maxspeed:backward", "10"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/costmatrix_trivial");

  {
    auto matrix = gurka::do_action(valhalla::Options::sources_to_targets, map, {"1"}, {"2"}, "auto");
    EXPECT_EQ(matrix.matrix().distances(0), 400);
    // this fails because it's getting the wrong directed edge:
    // https://github.com/valhalla/valhalla/issues/4433
    EXPECT_NEAR(matrix.matrix().times(0), 144.f, 0.1);
  }
}
