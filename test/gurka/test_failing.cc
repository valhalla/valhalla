#include "gurka.h"
#include "midgard/encoded.h"
#include "midgard/polyline2.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;

TEST(StandAlone, CostMatrixTrivialRoutes) {
  const std::string ascii_map = R"(
    A-1---2-B
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}, {"maxspeed:forward", "10"}, {"maxspeed:backward", "100"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/costmatrix_trivial");

  {
    auto matrix =
        gurka::do_action(valhalla::Options::sources_to_targets, map, {"1"}, {"2"}, "auto",
                         {{"/prioritize_bidirectional", "true"}, {"/shape_format", "polyline6"}});
    EXPECT_EQ(matrix.matrix().distances(0), 400);
    std::cerr << "shape is " << matrix.matrix().shapes(0) << "\n";
    // this fails because it's getting the wrong directed edge with 100 kmh:
    // https://github.com/valhalla/valhalla/issues/4433
    EXPECT_NEAR(matrix.matrix().times(0), 144.f, 0.1);
  }
}

TEST(StandAlone, CostMatrixOneWayRoute) {
  const std::string ascii_map = R"(
    C-------D
    |       |
    |       |
    A-1---2-B
  )";
  const gurka::ways ways = {{"AB", {{"highway", "residential"}, {"oneway", "yes"}}},
                            {"AC", {{"highway", "residential"}}},
                            {"CD", {{"highway", "residential"}}},
                            {"DB", {{"highway", "residential"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {},
                               VALHALLA_BUILD_DIR "test/data/costmatrix_trivial_oneway");

  {
    auto matrix =
        gurka::do_action(valhalla::Options::sources_to_targets, map, {"1"}, {"2"}, "auto",
                         {{"/shape_format", "polyline6"}, {"/prioritize_bidirectional", "true"}});
    EXPECT_LT(matrix.matrix().distances(0), 2100);
    std::vector<midgard::PointLL> coords =
        midgard::decode<std::vector<midgard::PointLL>>(matrix.matrix().shapes(0));
    auto length = midgard::Polyline2<midgard::PointLL>::Length(coords);
    EXPECT_NE(matrix.matrix().distances(0), length);
    std::cerr << "shape is " << matrix.matrix().shapes(0) << "\n";
    // this fails because it's getting the wrong directed edge with 100 kmh:
    // https://github.com/valhalla/valhalla/issues/4433
    // EXPECT_NEAR(matrix.matrix().times(0), 144.f, 0.1);
  }
}