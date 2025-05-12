#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

class Precision : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {

    const std::string ascii_map = R"(
    A-1--B-C----\
                 D------\
                         -----E--2---F)";

    const gurka::ways ways = {{"ABCDEF", {{"highway", "primary"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/precision",
                            {{"mjolnir.hierarchy", "false"}, {"mjolnir.concurrency", "1"}});
  }
};

gurka::map Precision::map = {};

/*************************************************************/
TEST_F(Precision, WaypointsOnNodes) {
  auto result = gurka::do_action(Options::route, map, {"A", "F"}, "auto");
  auto shape =
      midgard::decode<std::vector<midgard::PointLL>>(result.trip().routes(0).legs(0).shape());
  auto expected_shape = decltype(shape){map.nodes["A"], map.nodes["B"], map.nodes["C"],
                                        map.nodes["D"], map.nodes["E"], map.nodes["F"]};

  EXPECT_EQ(shape.size(), expected_shape.size());
  for (size_t i = 0; i < shape.size(); ++i) {
    EXPECT_NEAR(shape[i].lat(), expected_shape[i].lat(), 0.000001);
    EXPECT_NEAR(shape[i].lng(), expected_shape[i].lng(), 0.000001);
  }
}

TEST_F(Precision, PartialOffsetCheckOne) {
  auto result = gurka::do_action(Options::route, map, {"1", "2"}, "auto");
  auto shape =
      midgard::decode<std::vector<midgard::PointLL>>(result.trip().routes(0).legs(0).shape());
  auto expected_shape = decltype(shape){map.nodes["1"], map.nodes["B"], map.nodes["C"],
                                        map.nodes["D"], map.nodes["E"], map.nodes["2"]};

  EXPECT_EQ(shape.size(), expected_shape.size());
  for (size_t i = 0; i < shape.size(); ++i) {
    EXPECT_NEAR(shape[i].lat(), expected_shape[i].lat(), 0.000001);
    EXPECT_NEAR(shape[i].lng(), expected_shape[i].lng(), 0.000001);
  }
}

TEST_F(Precision, PartialOffsetCheckTwo) {
  auto result = gurka::do_action(Options::route, map, {"A", "2"}, "auto");
  auto shape =
      midgard::decode<std::vector<midgard::PointLL>>(result.trip().routes(0).legs(0).shape());
  auto expected_shape = decltype(shape){map.nodes["A"], map.nodes["B"], map.nodes["C"],
                                        map.nodes["D"], map.nodes["E"], map.nodes["2"]};

  EXPECT_EQ(shape.size(), expected_shape.size());
  for (size_t i = 0; i < shape.size(); ++i) {
    EXPECT_NEAR(shape[i].lat(), expected_shape[i].lat(), 0.000001);
    EXPECT_NEAR(shape[i].lng(), expected_shape[i].lng(), 0.000001);
  }
}

TEST_F(Precision, PartialOffsetCheckThree) {
  auto result = gurka::do_action(Options::route, map, {"1", "F"}, "auto");
  auto shape =
      midgard::decode<std::vector<midgard::PointLL>>(result.trip().routes(0).legs(0).shape());
  auto expected_shape = decltype(shape){map.nodes["1"], map.nodes["B"], map.nodes["C"],
                                        map.nodes["D"], map.nodes["E"], map.nodes["F"]};

  EXPECT_EQ(shape.size(), expected_shape.size());
  for (size_t i = 0; i < shape.size(); ++i) {
    EXPECT_NEAR(shape[i].lat(), expected_shape[i].lat(), 0.000001);
    EXPECT_NEAR(shape[i].lng(), expected_shape[i].lng(), 0.000001);
  }
}
