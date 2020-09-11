#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {

class IntersectionTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
          X
          |
      A---B---C---D
              |
              Y
    )";
    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}}},  {"BC", {{"highway", "primary"}, {"access", "private"}}},
        {"CD", {{"highway", "primary"}}},  {"BX", {{"highway", "motorway"}}},
        {"CY", {{"highway", "motorway"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_intersections_class_restricted");
  }
};

gurka::map IntersectionTest::map = {};

TEST_F(IntersectionTest, RestrictedClass) {
  std::string from = "A", to = "Y";
  valhalla::Api raw_result = gurka::route(map, "A", "Y", "auto");
  // Convert raw api response OSRM formatted JSON
  rapidjson::Document response = gurka::convert_to_json(raw_result, valhalla::Options_Format_osrm);

  auto steps = response["routes"][0]["legs"][0]["steps"].GetArray();
  // First step, should have 2 intersections, with the second onehaving a
  // single "restricted" class
  // restricted
  ASSERT_EQ(steps[0]["intersections"].Size(), 2);
  EXPECT_EQ(steps[0]["intersections"][1]["classes"][0].GetString(), std::string("restricted"));

  // Intersection on last step should have the motoroway class
  ASSERT_EQ(steps[1]["intersections"].Size(), 1);
  EXPECT_EQ(steps[1]["intersections"][0]["classes"][0].GetString(), std::string("motorway"));
}

} // namespace
