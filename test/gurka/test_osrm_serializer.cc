#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, OsrmSerializerShape) {
  const std::string ascii_map = R"(
    B---C---D
    |
    A
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10, {40.7351162, -73.985719});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/osrm_serializer_shape");

  // Test that full shape is returned by default
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto",
                                 {{"/shape_format", "geojson"}});
  auto json = gurka::convert_to_json(result, Options::Format::Options_Format_osrm);
  EXPECT_EQ(json["routes"][0]["geometry"]["coordinates"].GetArray().Size(), 4);

  // Test that shape is simplified (should simplify out C but not B)
  result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto",
                            {{"/generalize", "0"}, {"/shape_format", "geojson"}});
  json = gurka::convert_to_json(result, Options::Format::Options_Format_osrm);
  EXPECT_EQ(json["routes"][0]["geometry"]["coordinates"].GetArray().Size(), 3);
}
