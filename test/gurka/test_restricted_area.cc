#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {

class RestrictedTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
          H       I       J       O
          |       |       |       |
      A---B---C---D---E---F---G---N---P
              |       |       |
              K       L       M
    )";
    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}}},
        {"BC", {{"highway", "primary"}, {"access", "private"}}},
        {"CD", {{"highway", "primary"}}},
        {"DE", {{"highway", "primary"}}},
        {"EF", {{"highway", "primary"}}},
        {"FG", {{"highway", "primary"}}},
        {"GN", {{"highway", "primary"}}},
        {"NP", {{"highway", "primary"}}},
        {"CK", {{"highway", "motorway"}}},
        {"DI", {{"highway", "primary"}, {"access", "delivery"}}},
        {"EL", {{"highway", "primary"}, {"access", "destination"}}},
        {"FJ", {{"highway", "primary"}, {"access", "customers"}}},
        {"GM", {{"highway", "primary"}, {"access", "permit"}}},
        {"NO", {{"highway", "primary"}, {"access", "residents"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_intersections_class_restricted");
  }
};

gurka::map RestrictedTest::map = {};

TEST_F(RestrictedTest, AccessPrivate) {
  valhalla::Api raw_result = gurka::do_action(valhalla::Options::route, map, {"A", "K"}, "auto");
  // Convert raw api response OSRM formatted JSON
  rapidjson::Document response = gurka::convert_to_json(raw_result, valhalla::Options_Format_osrm);

  auto steps = response["routes"][0]["legs"][0]["steps"].GetArray();
  // First step, should have 2 intersections, with the second one having a
  // single "restricted" class
  ASSERT_EQ(steps[0]["intersections"].Size(), 2);
  EXPECT_EQ(steps[0]["intersections"][1]["classes"][0].GetString(), std::string("restricted"));

  // Intersection on last step should have the motoroway class
  ASSERT_EQ(steps[1]["intersections"].Size(), 1);
  EXPECT_EQ(steps[1]["intersections"][0]["classes"][0].GetString(), std::string("motorway"));
}

TEST_F(RestrictedTest, AccessDelivery) {
  valhalla::Api raw_result = gurka::do_action(valhalla::Options::route, map, {"C", "I"}, "auto");
  // Convert raw api response OSRM formatted JSON
  rapidjson::Document response = gurka::convert_to_json(raw_result, valhalla::Options_Format_osrm);

  auto steps = response["routes"][0]["legs"][0]["steps"].GetArray();
  ASSERT_EQ(steps[0]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[0]["intersections"][0].HasMember("classes"));

  ASSERT_EQ(steps[1]["intersections"].Size(), 1);
  EXPECT_EQ(steps[1]["intersections"][0]["classes"][0].GetString(), std::string("restricted"));
}

TEST_F(RestrictedTest, AccessDestination) {
  valhalla::Api raw_result = gurka::do_action(valhalla::Options::route, map, {"D", "L"}, "auto");
  // Convert raw api response OSRM formatted JSON
  rapidjson::Document response = gurka::convert_to_json(raw_result, valhalla::Options_Format_osrm);

  auto steps = response["routes"][0]["legs"][0]["steps"].GetArray();
  ASSERT_EQ(steps[0]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[0]["intersections"][0].HasMember("classes"));

  ASSERT_EQ(steps[1]["intersections"].Size(), 1);
  EXPECT_EQ(steps[1]["intersections"][0]["classes"][0].GetString(), std::string("restricted"));
}

TEST_F(RestrictedTest, AccessCustomers) {
  valhalla::Api raw_result = gurka::do_action(valhalla::Options::route, map, {"E", "J"}, "auto");
  // Convert raw api response OSRM formatted JSON
  rapidjson::Document response = gurka::convert_to_json(raw_result, valhalla::Options_Format_osrm);

  auto steps = response["routes"][0]["legs"][0]["steps"].GetArray();
  ASSERT_EQ(steps[0]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[0]["intersections"][0].HasMember("classes"));

  ASSERT_EQ(steps[1]["intersections"].Size(), 1);
  EXPECT_EQ(steps[1]["intersections"][0]["classes"][0].GetString(), std::string("restricted"));
}

TEST_F(RestrictedTest, AccessPermit) {
  valhalla::Api raw_result = gurka::do_action(valhalla::Options::route, map, {"F", "M"}, "auto");
  // Convert raw api response OSRM formatted JSON
  rapidjson::Document response = gurka::convert_to_json(raw_result, valhalla::Options_Format_osrm);

  auto steps = response["routes"][0]["legs"][0]["steps"].GetArray();
  ASSERT_EQ(steps[0]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[0]["intersections"][0].HasMember("classes"));

  ASSERT_EQ(steps[1]["intersections"].Size(), 1);
  EXPECT_EQ(steps[1]["intersections"][0]["classes"][0].GetString(), std::string("restricted"));
}

TEST_F(RestrictedTest, AccessResidents) {
  valhalla::Api raw_result = gurka::do_action(valhalla::Options::route, map, {"G", "O"}, "auto");
  // Convert raw api response OSRM formatted JSON
  rapidjson::Document response = gurka::convert_to_json(raw_result, valhalla::Options_Format_osrm);

  auto steps = response["routes"][0]["legs"][0]["steps"].GetArray();
  ASSERT_EQ(steps[0]["intersections"].Size(), 1);
  EXPECT_FALSE(steps[0]["intersections"][0].HasMember("classes"));

  ASSERT_EQ(steps[1]["intersections"].Size(), 1);
  EXPECT_EQ(steps[1]["intersections"][0]["classes"][0].GetString(), std::string("restricted"));
}

} // namespace
