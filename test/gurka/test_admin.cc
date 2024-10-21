#include "gurka.h"
#include "test.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.data_processing.use_admin_db", "false"}};

class AdminTest : public ::testing::Test {
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
gurka::map AdminTest::map = {};
Api api;
rapidjson::Document d;

/*************************************************************/

TEST_F(AdminTest, Iso) {
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

TEST_F(AdminTest, test_osrm_response) {
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

/**************************************************************************** */

TEST(Standalone, AdminCrossingsCountry) {
  constexpr double gridsize_metres = 100;

  // Border between C and D
  const std::string ascii_map = R"(
   A
   |
   B
   |
   C
  --\--
     D
      \
       E
       |
       F
       |
       G
  )";

  const gurka::ways ways = {{"AB", {{"highway", "motorway"}}}, {"BC", {{"highway", "motorway"}}},
                            {"CD", {{"highway", "motorway"}}}, {"DE", {{"highway", "motorway"}}},
                            {"EF", {{"highway", "motorway"}}}, {"FG", {{"highway", "motorway"}}}};

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {-111.962238354, 49.003392362});
  const auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_admin_country",
                        {{"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}}});
  std::string result_json;
  rapidjson::Document result;
  auto api = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto",
                              {{"/admin_crossings", "1"}}, {}, &result_json);
  result.Parse(result_json.c_str());

  ASSERT_EQ(result["trip"]["legs"].Size(), 1);

  // Expect admin list at leg level
  auto leg = result["trip"]["legs"][0].GetObject();
  ASSERT_EQ(leg["admins"].Size(), 2);
  EXPECT_TRUE(leg.HasMember("admins"));
  EXPECT_STREQ(leg["admins"][0]["country_code"].GetString(), "CA");
  EXPECT_STREQ(leg["admins"][0]["state_code"].GetString(), "");
  EXPECT_STREQ(leg["admins"][0]["country_text"].GetString(), "Canada");
  EXPECT_STREQ(leg["admins"][0]["state_text"].GetString(), "");
  EXPECT_STREQ(leg["admins"][1]["country_code"].GetString(), "US");
  EXPECT_STREQ(leg["admins"][1]["state_code"].GetString(), "");
  EXPECT_STREQ(leg["admins"][1]["country_text"].GetString(), "United States");
  EXPECT_STREQ(leg["admins"][1]["state_text"].GetString(), "");
  auto crossings = leg["admin_crossings"].GetArray();

  // Total of 1 crossing
  EXPECT_EQ(crossings.Size(), 1);
  EXPECT_EQ(crossings[0]["from_admin_index"], 0);
  EXPECT_EQ(crossings[0]["to_admin_index"], 1);
  EXPECT_EQ(crossings[0]["begin_shape_index"].GetUint64(), 2);
  EXPECT_EQ(crossings[0]["end_shape_index"].GetUint64(), 3);
}

TEST(Standalone, AdminCrossingsState) {
  constexpr double gridsize_metres = 100;

  // border between E and F
  const std::string ascii_map = R"(
            | 
   A-B-C-D-E+F-G
            | 
  )";

  const gurka::ways ways = {{"AB", {{"highway", "motorway"}}}, {"BC", {{"highway", "motorway"}}},
                            {"CD", {{"highway", "motorway"}}}, {"DE", {{"highway", "motorway"}}},
                            {"EF", {{"highway", "motorway"}}}, {"FG", {{"highway", "motorway"}}}};

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {-80.527768076, 41.934220471});
  const auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_admin_state",
                        {{"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}}});
  std::string result_json;
  rapidjson::Document result;
  auto api = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto",
                              {{"/admin_crossings", "1"}}, {}, &result_json);
  result.Parse(result_json.c_str());

  ASSERT_EQ(result["trip"]["legs"].Size(), 1);

  // Expect admin list at leg level
  auto leg = result["trip"]["legs"][0].GetObject();
  ASSERT_EQ(leg["admins"].Size(), 2);
  EXPECT_TRUE(leg.HasMember("admins"));
  EXPECT_STREQ(leg["admins"][0]["country_code"].GetString(), "US");
  EXPECT_STREQ(leg["admins"][0]["state_code"].GetString(), "OH");
  EXPECT_STREQ(leg["admins"][0]["country_text"].GetString(), "United States");
  EXPECT_STREQ(leg["admins"][0]["state_text"].GetString(), "Ohio");
  EXPECT_STREQ(leg["admins"][1]["country_code"].GetString(), "US");
  EXPECT_STREQ(leg["admins"][1]["state_code"].GetString(), "PA");
  EXPECT_STREQ(leg["admins"][1]["country_text"].GetString(), "United States");
  EXPECT_STREQ(leg["admins"][1]["state_text"].GetString(), "Pennsylvania");
  auto crossings = leg["admin_crossings"].GetArray();

  // Total of 1 crossing
  EXPECT_EQ(crossings.Size(), 1);
  EXPECT_EQ(crossings[0]["from_admin_index"], 0);
  EXPECT_EQ(crossings[0]["to_admin_index"], 1);
  EXPECT_EQ(crossings[0]["begin_shape_index"].GetUint64(), 4);
  EXPECT_EQ(crossings[0]["end_shape_index"].GetUint64(), 5);
}

TEST(Standalone, AdminCrossingsMultiple) {
  constexpr double gridsize_metres = 100;

  // Borders between AB, DE and FG
  const std::string ascii_map = R"(
  A       G
  |       |
  |  ---+-+
  | /   | F
 -+-    | |
  B     | |
  C---D-+-E
        |
  )";

  const gurka::ways ways = {{"AB", {{"highway", "motorway"}}}, {"BC", {{"highway", "motorway"}}},
                            {"CD", {{"highway", "motorway"}}}, {"DE", {{"highway", "motorway"}}},
                            {"EF", {{"highway", "motorway"}}}, {"FG", {{"highway", "motorway"}}}};

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {-80.528267838, 42.3255});
  const auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_admin_state_country",
                        {{"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}}});
  std::string result_json;
  rapidjson::Document result;
  auto api = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto",
                              {{"/admin_crossings", "1"}}, {}, &result_json);
  result.Parse(result_json.c_str());

  ASSERT_EQ(result["trip"]["legs"].Size(), 1);

  // Expect admin list at leg level
  auto leg = result["trip"]["legs"][0].GetObject();
  ASSERT_EQ(leg["admins"].Size(), 3);
  EXPECT_TRUE(leg.HasMember("admins"));
  EXPECT_STREQ(leg["admins"][0]["country_code"].GetString(), "CA");
  EXPECT_STREQ(leg["admins"][0]["state_code"].GetString(), "ON");
  EXPECT_STREQ(leg["admins"][0]["country_text"].GetString(), "Canada");
  EXPECT_STREQ(leg["admins"][0]["state_text"].GetString(), "Ontario");
  EXPECT_STREQ(leg["admins"][1]["country_code"].GetString(), "US");
  EXPECT_STREQ(leg["admins"][1]["state_code"].GetString(), "OH");
  EXPECT_STREQ(leg["admins"][1]["country_text"].GetString(), "United States");
  EXPECT_STREQ(leg["admins"][1]["state_text"].GetString(), "Ohio");
  EXPECT_STREQ(leg["admins"][2]["country_code"].GetString(), "US");
  EXPECT_STREQ(leg["admins"][2]["state_code"].GetString(), "PA");
  EXPECT_STREQ(leg["admins"][2]["country_text"].GetString(), "United States");
  EXPECT_STREQ(leg["admins"][2]["state_text"].GetString(), "Pennsylvania");
  auto crossings = leg["admin_crossings"].GetArray();

  // Total of 3 crossings
  EXPECT_EQ(crossings.Size(), 3);
  EXPECT_EQ(crossings[0]["from_admin_index"], 0);
  EXPECT_EQ(crossings[0]["to_admin_index"], 1);
  EXPECT_EQ(crossings[0]["begin_shape_index"].GetUint64(), 0);
  EXPECT_EQ(crossings[0]["end_shape_index"].GetUint64(), 1);
  EXPECT_EQ(crossings[1]["from_admin_index"], 1);
  EXPECT_EQ(crossings[1]["to_admin_index"], 2);
  EXPECT_EQ(crossings[1]["begin_shape_index"].GetUint64(), 3);
  EXPECT_EQ(crossings[1]["end_shape_index"].GetUint64(), 4);
  EXPECT_EQ(crossings[2]["from_admin_index"], 2);
  EXPECT_EQ(crossings[2]["to_admin_index"], 0);
  EXPECT_EQ(crossings[2]["begin_shape_index"].GetUint64(), 5);
  EXPECT_EQ(crossings[2]["end_shape_index"].GetUint64(), 6);
}
