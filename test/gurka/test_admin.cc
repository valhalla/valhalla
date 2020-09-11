#include "gurka.h"
#include <boost/format.hpp>
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

    const gurka::ways ways = {{"AB", {{"highway", "motorway"}}}, {"BC", {{"highway", "motorway"}}}};

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
  auto result = gurka::route(map, "A", "C", "auto");

  // rest_area
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.admin(0).country_code(), "US"); // AB
  EXPECT_EQ(leg.admin(0).state_code(), "PA");   // AB
  EXPECT_EQ(leg.admin(1).country_code(), "US"); // BC
  EXPECT_EQ(leg.admin(1).state_code(), "MD");   // BC
}
