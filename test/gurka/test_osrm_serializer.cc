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

TEST(Standalone, HeadingNumber2CrossRoadPedestrian) {
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
  auto map = gurka::buildtiles(layout, ways, {}, {},
                               "test/data/osrm_serializer_route_2crossroad_heading_pedestrian");

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
      ASSERT_EQ(expected_headings[index++],
                json["routes"][0]["legs"][0]["steps"][i]["intersections"][j]["bearings"]
                    .GetArray()
                    .Size());
    }
  }
}

TEST(Standalone, HeadingNumber2CrossRoadAuto) {
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
  auto map = gurka::buildtiles(layout, ways, {}, {},
                               "test/data/osrm_serializer_route_2crossroad_heading_auto");

  // Expected number of headings at each intersection of a step.
  std::vector<int> expected_headings{1, 4, 4, 1};
  // route C-G
  {
    auto result = gurka::do_action(valhalla::Options::route, map, {"C", "G"}, "auto");
    auto json = gurka::convert_to_json(result, Options::Format::Options_Format_osrm);
    int index{0};

    std::vector<int> out_indexes{0, 1, 1}; // `out` field expected values
    int out_cnt{0};
    std::vector<int> in_indexes{3, 3, 0}; // `in` field expected values
    int in_cnt{0};
    for (int steps = 0; steps < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++steps) {
      int intersection_num =
          json["routes"][0]["legs"][0]["steps"][steps]["intersections"].GetArray().Size();
      for (int intersection = 0; intersection < intersection_num; ++intersection) {
        // Validate `out` edge indexes and headings.
        if (json["routes"][0]["legs"][0]["steps"][steps]["intersections"][intersection].HasMember(
                "out")) {
          int out_index =
              json["routes"][0]["legs"][0]["steps"][steps]["intersections"][intersection]["out"]
                  .GetInt();
          EXPECT_EQ(out_index, out_indexes[out_cnt++])
              << "FAILED at step " << steps << ", intersection " << intersection;

          if (json["routes"][0]["legs"][0]["steps"][steps]["intersections"][intersection].HasMember(
                  "bearing"))
            // All bearings have the same direction which is 90
            EXPECT_EQ(json["routes"][0]["legs"][0]["steps"][steps]["intersections"][intersection]
                          ["bearing"][out_index]
                              .GetInt(),
                      90);
        }

        // Validate `in` edge indexes and headings.
        if (json["routes"][0]["legs"][0]["steps"][steps]["intersections"][intersection].HasMember(
                "in")) {
          int in_index =
              json["routes"][0]["legs"][0]["steps"][steps]["intersections"][intersection]["in"]
                  .GetInt();
          EXPECT_EQ(in_index, in_indexes[in_cnt++])
              << "FAILED at step " << steps << ", intersection " << intersection;

          if (json["routes"][0]["legs"][0]["steps"][steps]["intersections"][intersection].HasMember(
                  "bearings"))
            // All bearings have the same direction which is 270
            EXPECT_EQ(json["routes"][0]["legs"][0]["steps"][steps]["intersections"][intersection]
                          ["bearings"][in_index]
                              .GetInt(),
                      270);
        }

        EXPECT_EQ(expected_headings[index++], json["routes"][0]["legs"][0]["steps"][steps]
                                                  ["intersections"][intersection]["bearings"]
                                                      .GetArray()
                                                      .Size());
      }
    }
  }

  // route A-G
  {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto");
    auto json = gurka::convert_to_json(result, Options::Format::Options_Format_osrm);
    int index{0};

    // Expected `out` edge indexes.
    std::vector<int> out_indexes{0, 1, 1};
    int out_cnt{0};

    // Expected `in` edge indexes.
    std::vector<int> in_indexes{2, 3, 0};
    int in_cnt{0};

    // clang-format off
    for (int step = 0; step < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++step) {

      int intersection_num = json["routes"][0]["legs"][0]["steps"][step]["intersections"].GetArray().Size();
      for (int intersection = 0; intersection < intersection_num; ++intersection) {
        // Validate `out` edge indexes and headings.
        if (json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection].HasMember("out")) {
          int out_index = json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["out"].GetInt();
          EXPECT_EQ(out_index, out_indexes[out_cnt++]) << "FAILED at step " << step
          << ", intersection " << intersection;

          if (json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection].HasMember("bearings")) {
            if (!step) {
              EXPECT_EQ(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["bearings"]
                        [out_index].GetInt(),0) << "FAILED at step " << step << ", intersection " << intersection;
            } else if (step == 1) {
                // All bearings have the same angle, 90.
                EXPECT_EQ(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["bearings"]
                [out_index].GetInt(),90) << "FAILED at step " << step << ", intersection " << intersection;
            } else { // step 3
              // All bearings have the same angle, 270.
              EXPECT_EQ(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["bearings"]
              [out_index].GetInt(), 270) << "FAILED at step " << step << ", intersection " << intersection;
            }
          }
        }

        // Validate `in` edge indexes and headings.
        if (json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection].HasMember("in")) {
          int in_index = json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["in"].GetInt();
          EXPECT_EQ(in_index, in_indexes[in_cnt++]) << "FAILED at step " << step << ", intersection " << intersection;

          if (json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection].HasMember("bearings")) {
            if (!step) {
              // All bearings have the same angle, 0.
              EXPECT_EQ(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["bearings"][in_index].GetInt(),
                        0)
                  << "FAILED at step " << step << ", intersection " << intersection;
            } else if (step == 1) {
              if (!intersection) {
              // All bearings have the same angle, 180.
              EXPECT_EQ(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["bearings"]
                            [in_index].GetInt(), 180)
                  << "FAILED at step " << step << ", intersection " << intersection;
              } else {
                // All bearings have the same angle, 270.
                EXPECT_EQ(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["bearings"]
                              [in_index].GetInt(), 270)
                    << "FAILED at step " << step << ", intersection " << intersection;
              }
            } else { // step 3
              // All bearings have the same angle, 270.
              EXPECT_EQ(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["bearings"]
                            [in_index].GetInt(),270)
                  << "FAILED at step " << step << ", intersection " << intersection;
            }
          }
        }
        // clang-format on

        EXPECT_EQ(expected_headings[index++], json["routes"][0]["legs"][0]["steps"][step]
                                                  ["intersections"][intersection]["bearings"]
                                                      .GetArray()
                                                      .Size());
      }
    }
  }
}

TEST(Standalone, HeadingNumberAutoRoute) {
  const std::string ascii_map = R"(
         I                    L
         |                   /
         |                  /
   E-----F-----------------G-----K
         |                /
         |               /
         |              /
    A----B-------------C
    |    |
    J    D
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},       {"BC", {{"highway", "primary"}}},
      {"EF", {{"highway", "secondary"}}},     {"FG", {{"highway", "secondary"}}},
      {"GK", {{"highway", "secondary"}}},     {"JA", {{"highway", "secondary"}}},
      {"DB", {{"highway", "secondary"}}},     {"BF", {{"highway", "secondary"}}},
      {"FI", {{"highway", "secondary"}}},     {"CG", {{"highway", "motorway_link"}}},
      {"GL", {{"highway", "motorway_link"}}},
  };

  const gurka::relations relations = {
      {{
           {gurka::way_member, "AB", "from"},
           {gurka::way_member, "BC", "to"},
           {gurka::node_member, "C", "via"},
       },
       {
           {"type", "restriction"},
           {"restriction", "only_straight_on"},
       }},
      {{
           {gurka::way_member, "CG", "from"},
           {gurka::way_member, "GK", "to"},
           {gurka::node_member, "G", "via"},
       },
       {
           {"type", "restriction"},
           {"restriction", "only_right_turn"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {0, 0});
  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/osrm_serializer_auto_route");

  // route J-A-C-G-K
  auto result = gurka::do_action(valhalla::Options::route, map, {"J", "K"}, "auto");

  gurka::assert::osrm::expect_steps(result, {"JA", "AB", "CG", "GK"});
  gurka::assert::raw::expect_path(result, {"JA", "AB", "BC", "CG", "GK"});

  auto json = gurka::convert_to_json(result, Options::Format::Options_Format_osrm);

  // 5 steps/maneuvers expected for route `J-A-C-G-K`.
  ASSERT_EQ(json["routes"][0]["legs"][0]["steps"].GetArray().Size(), 5);

  std::vector<int> expected_headings{1, 2, 4, 2, 4, 1};
  int index{0};
  // Validate number of bearings at each intersection of each step.
  for (int step = 0; step < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++step) {
    int intersection_num =
        json["routes"][0]["legs"][0]["steps"][step]["intersections"].GetArray().Size();
    for (int intersection = 0; intersection < intersection_num; ++intersection) {
      EXPECT_EQ(expected_headings[index++],
                json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["bearings"]
                    .GetArray()
                    .Size());
    }
  }

  // Expected `out` edge indexes.
  std::vector<int> out_indexes{0, 0, 1, 0, 1};
  // Expected `in` edge indexes.
  // No `in` for the 1st step
  std::vector<int> in_indexes{-1, 1, 3, 1, 2, 0};

  // Expected `out` edge heading.
  std::vector<int> out_headings{360, 90, 90, 45, 90};

  // Expected `in` edge heading.
  // No `in` for the 1st step
  std::vector<int> in_headings{-1, 180, 270, 270, 225, 270};
  index = 0;

  // Validating `in` and `out` edge indexes and bearings at each intersection of each step.
  // clang-format off
  for (int step = 0; step < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++step) {
    int intersection_num = json["routes"][0]["legs"][0]["steps"][step]["intersections"].GetArray().Size();
    for (int intersection = 0; intersection < intersection_num; ++intersection) {
      if (step == 0) {
        EXPECT_FALSE(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection].HasMember("in"))
            << "FAILED at step " << step << ", intersection " << intersection;
        ASSERT_TRUE(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection].HasMember("out"));

        int out_index = json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["out"].GetInt();
        EXPECT_EQ(out_index, out_indexes[index]) << "FAILED at step " << step << ", intersection " << intersection;

        ASSERT_TRUE(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection].HasMember("bearings"));

        int out_heading = json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["bearings"]
                          [out_index].GetInt();
        EXPECT_EQ(out_heading, out_headings[index++]) << "FAILED at step " << step << ", intersection "
                                                           << intersection;
      } else if (step < 4) {
        ASSERT_TRUE(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection].HasMember("out"));

        int out_index = json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["out"].GetInt();
        EXPECT_EQ(out_index, out_indexes[index])
            << "FAILED at step " << step << ", intersection " << intersection;

        ASSERT_TRUE(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection].HasMember("bearings"));

        int out_heading = json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["bearings"][out_index]
                                  .GetInt();
        EXPECT_EQ(out_heading, out_headings[index])
            << "FAILED at step " << step << ", intersection " << intersection;

        ASSERT_TRUE(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection].HasMember("in"));

        int in_index = json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["in"].GetInt();
        EXPECT_EQ(in_index, in_indexes[index])
            << "FAILED at step " << step << ", intersection " << intersection;

        int in_heading = json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["bearings"][in_index]
                                 .GetInt();
        EXPECT_EQ(in_heading, in_headings[index++])
            << "FAILED at step " << step << ", intersection " << intersection;
      } else {
        ASSERT_TRUE(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection].HasMember("in"));
        ASSERT_FALSE(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection].HasMember("out"));

        int in_index = json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["in"].GetInt();
        EXPECT_EQ(in_index, in_indexes[index]) << "FAILED at step " << step << ", intersection " << intersection;

        ASSERT_TRUE(json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection].HasMember("bearings"));

        int in_heading = json["routes"][0]["legs"][0]["steps"][step]["intersections"][intersection]["bearings"][in_index]
                                 .GetInt();
        EXPECT_EQ(in_heading, in_headings[index++]) << "FAILED at step " << step << ", intersection " << intersection;
      }
    }
  }
  // clang-format on
}
