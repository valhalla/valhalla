#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

TEST(StandAlone, ZeroLength) {
  const std::string ascii_map = R"(A---1----B)";
  const gurka::ways ways = {{"AB", {{"highway", "primary"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 1000);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_zero_length");
  auto raw_result = gurka::do_action(valhalla::Options::route, map, {"1", "1"}, "auto");

  rapidjson::Document result = gurka::convert_to_json(raw_result, valhalla::Options_Format_osrm);

  ASSERT_EQ(result["routes"][0]["distance"], 0);
  auto steps = result["routes"][0]["legs"][0]["steps"].GetArray();
  ASSERT_EQ(steps.Size(), 2);
  for (auto& step : steps) {
    ASSERT_EQ(step["distance"], 0);
  }
}
