#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.data_processing.use_admin_db", "false"}};

class Admin : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
                          A
                          |
                          |
                          B
                          |
                          |
                          C
  )";

    const gurka::ways ways = {{"AB", {{"highway", "motorway"}, {"driving_side", "right"}}},
                              {"BC", {{"highway", "motorway"}, {"driving_side", "left"}}}};

    const gurka::nodes nodes =
        {{"A", {{"iso:3166_1", "US"}, {"iso:3166_2", "US-PA"}}},
         {"B", {{"iso:3166_1", "US"}, {"iso:3166_2", "US-PA"}, {"iso:3166_2", "US-MD"}}},
         {"C", {{"iso:3166_1", "US"}, {"iso:3166_2", "US-MD"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_admin", build_config);
  }
};
gurka::map Admin::map = {};
Api api;
rapidjson::Document d;

/*************************************************************/

TEST_F(Admin, Iso) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "auto");

  // rest_area
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.admin(0).country_code(), "US"); // AB
  EXPECT_EQ(leg.admin(0).state_code(), "PA");   // AB
  EXPECT_EQ(leg.admin(1).country_code(), "US"); // BC
  EXPECT_EQ(leg.admin(1).state_code(), "MD");   // BC
  EXPECT_FALSE(leg.node(0).edge().drive_on_left());
  EXPECT_TRUE(leg.node(1).edge().drive_on_left());
}

TEST_F(Admin, test_admin_response) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "auto");
  auto d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  ASSERT_EQ(d["routes"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"][0]["steps"].Size(), 3);

  // Expect admin list at leg level
  auto leg = d["routes"][0]["legs"][0].GetObject();
  EXPECT_TRUE(leg.HasMember("admins"));
  EXPECT_STREQ(leg["admins"][0]["iso_3166_1"].GetString(), "US");
  EXPECT_STREQ(leg["admins"][0]["iso_3166_1_alpha3"].GetString(), "USA");
  EXPECT_EQ(leg["admins"].Size(), 2);

  auto steps = leg["steps"].GetArray();

  // First step has admin_index=0
  int step_index = 0;
  EXPECT_EQ(steps[step_index]["intersections"].Size(), 1);
  EXPECT_TRUE(steps[step_index]["intersections"][0].HasMember("admin_index"));
  EXPECT_EQ(steps[step_index]["intersections"][0]["admin_index"].GetInt(), 0);

  // Second step has admin_index=0
  step_index++;
  EXPECT_EQ(steps[step_index]["intersections"].Size(), 1);
  EXPECT_TRUE(steps[step_index]["intersections"][0].HasMember("admin_index"));
  EXPECT_EQ(steps[step_index]["intersections"][0]["admin_index"].GetInt(), 0);

  // Third step has admin_index=1
  step_index++;
  EXPECT_EQ(steps[step_index]["intersections"].Size(), 1);
  EXPECT_TRUE(steps[step_index]["intersections"][0].HasMember("admin_index"));
  EXPECT_EQ(steps[step_index]["intersections"][0]["admin_index"].GetInt(), 1);
}
