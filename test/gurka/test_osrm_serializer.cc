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

TEST(Standalone, HeadingNumberTurnLeft) {
  const std::string ascii_map = R"(
    B---D
    |
    A
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BD", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {40.7351162, -73.985719});
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/osrm_serializer_route_turnleft_heading");

  // Test that full shape is returned by default
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "pedestrian",
                                 {{"/shape_format", "geojson"}});
  //
  auto json = gurka::convert_to_json(result, Options::Format::Options_Format_osrm);
  auto heading_number{1};
  for (int i = 0; i < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++i)
    ASSERT_EQ(heading_number, json["routes"][0]["legs"][0]["steps"][0]["intersections"][0]["bearings"]
                                  .GetArray()
                                  .Size());
}

TEST(Standalone, HeadingNumberTRoute) {
  const std::string ascii_map = R"(
    C----B---F
         |
         A
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
      {"BF", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {40.7351162, -73.985719});
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/osrm_serializer_route_troute_heading");

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "pedestrian");

  auto json = gurka::convert_to_json(result, Options::Format::Options_Format_osrm);
  std::vector<int> expected_headings{1, 3, 1};
  for (int i = 0; i < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++i) {
    for (int j = 0; j < json["routes"][0]["legs"][0]["steps"][i]["intersections"].GetArray().Size();
         ++j) {
      ASSERT_EQ(expected_headings[i],
                json["routes"][0]["legs"][0]["steps"][i]["intersections"][j]["bearings"]
                    .GetArray()
                    .Size());
    }
  }
}

TEST(Standalone, HeadingNumberForkRoute) {

  const std::string ascii_map = R"(A---B----------C-----D
                                          \        /
                                           E------F)";

  const gurka::ways ways = {{"ABCD", {{"highway", "primary"}}},
                            {"BE", {{"highway", "primary"}}},
                            {"FC", {{"highway", "primary"}}},
                            {"EF", {{"highway", "primary"}}}};

  const double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/osrm_serializer_route_forkroad_heading");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "pedestrian");

  auto json = gurka::convert_to_json(result, valhalla::Options_Format_osrm);
  std::vector<int> expected_headings{1, 3, 3, 1};
  for (int i = 0; i < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++i) {
    for (int j = 0; j < json["routes"][0]["legs"][0]["steps"][i]["intersections"].GetArray().Size();
         ++j) {
      ASSERT_EQ(expected_headings[i],
                json["routes"][0]["legs"][0]["steps"][i]["intersections"][j]["bearings"]
                    .GetArray()
                    .Size());
    }
  }
}

TEST(Standalone, HeadingNumberCrossRoad) {
  const std::string ascii_map = R"(
         E
         |
    C----B---F
         |
         A
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
      {"BF", {{"highway", "primary"}}},
      {"BE", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {40.7351162, -73.985719});
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/osrm_serializer_route_crossroad_heading");

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "pedestrian");

  auto json = gurka::convert_to_json(result, Options::Format::Options_Format_osrm);
  std::vector<int> expected_headings{1, 4, 1};
  for (int i = 0; i < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++i) {
    for (int j = 0; j < json["routes"][0]["legs"][0]["steps"][i]["intersections"].GetArray().Size();
         ++j) {
      ASSERT_EQ(expected_headings[i],
                json["routes"][0]["legs"][0]["steps"][i]["intersections"][j]["bearings"]
                    .GetArray()
                    .Size());
    }
  }
}

TEST(Standalone, HeadingNumber2CrossRoad) {
  const std::string ascii_map = R"(
         E   I
         |   |
    C----B---F----G
         |   |
         A   D
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "secondary"}}}, {"BE", {{"highway", "secondary"}}},
      {"BC", {{"highway", "primary"}}},   {"BF", {{"highway", "primary"}}},
      {"FG", {{"highway", "primary"}}},   {"FI", {{"highway", "service"}}},
      {"FD", {{"highway", "service"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {40.7351162, -73.985719});
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/osrm_serializer_route_2crossroad_heading");

  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "G"}, "pedestrian");

  auto json = gurka::convert_to_json(result, Options::Format::Options_Format_osrm);
  std::vector<int> expected_headings{1, 4, 4, 1};
  int index{0};
  for (int i = 0; i < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++i) {
    for (int j = 0; j < json["routes"][0]["legs"][0]["steps"][i]["intersections"].GetArray().Size();
         ++j) {
      ASSERT_EQ(expected_headings[index++],
                json["routes"][0]["legs"][0]["steps"][i]["intersections"][j]["bearings"]
                    .GetArray()
                    .Size());
    }
  }

  result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "pedestrian");
  json = gurka::convert_to_json(result, Options::Format::Options_Format_osrm);
  index = 0;
  for (int i = 0; i < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++i) {
    for (int j = 0; j < json["routes"][0]["legs"][0]["steps"][i]["intersections"].GetArray().Size();
         ++j) {
      std::cout << "heading --> "
                << json["routes"][0]["legs"][0]["steps"][i]["intersections"][j]["bearings"]
                       .GetArray()
                       .Size()
                << std::endl;
      ASSERT_EQ(expected_headings[index++],
                json["routes"][0]["legs"][0]["steps"][i]["intersections"][j]["bearings"]
                    .GetArray()
                    .Size());
    }
  }
}
