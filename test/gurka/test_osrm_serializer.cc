#include "baldr/rapidjson_utils.h"
#include "gurka.h"
#include <boost/format.hpp>
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
  std::vector<int> expected_headings{1, 2, 1};
  for (rapidjson::SizeType i = 0; i < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++i)
    ASSERT_EQ(expected_headings[i],
              json["routes"][0]["legs"][0]["steps"][i]["intersections"][0]["bearings"]
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
  for (rapidjson::SizeType i = 0; i < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++i) {
    for (rapidjson::SizeType j = 0;
         j < json["routes"][0]["legs"][0]["steps"][i]["intersections"].GetArray().Size(); ++j) {
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
  for (rapidjson::SizeType i = 0; i < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++i) {
    for (rapidjson::SizeType j = 0;
         j < json["routes"][0]["legs"][0]["steps"][i]["intersections"].GetArray().Size(); ++j) {
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
  for (rapidjson::SizeType i = 0; i < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++i) {
    for (rapidjson::SizeType j = 0;
         j < json["routes"][0]["legs"][0]["steps"][i]["intersections"].GetArray().Size(); ++j) {
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
  for (rapidjson::SizeType i = 0; i < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++i) {
    for (size_t j = 0;
         j < json["routes"][0]["legs"][0]["steps"][i]["intersections"].GetArray().Size(); ++j) {
      ASSERT_EQ(expected_headings[index++],
                json["routes"][0]["legs"][0]["steps"][i]["intersections"][j]["bearings"]
                    .GetArray()
                    .Size());
    }
  }

  result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "pedestrian");
  json = gurka::convert_to_json(result, Options::Format::Options_Format_osrm);
  index = 0;
  for (rapidjson::SizeType i = 0; i < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++i) {
    for (rapidjson::SizeType j = 0;
         j < json["routes"][0]["legs"][0]["steps"][i]["intersections"].GetArray().Size(); ++j) {
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
    for (rapidjson::SizeType steps = 0;
         steps < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++steps) {
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
    for (rapidjson::SizeType step = 0; step < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++step) {

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
  for (rapidjson::SizeType step = 0; step < json["routes"][0]["legs"][0]["steps"].GetArray().Size();
       ++step) {
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
  for (rapidjson::SizeType step = 0; step < json["routes"][0]["legs"][0]["steps"].GetArray().Size(); ++step) {
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

class VoiceInstructions : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 50;

    const std::string ascii_map = R"(
             X   Y
              \  |           --M--N
               \ |  __ -- ¯¯
    A----------BCD<
                 |  ¯¯ -- __
                 |           --E--F
                 Z
    )";

    const gurka::ways ways =
        {{"AB", {{"highway", "primary"}, {"maxspeed", "80"}, {"name", "10th Avenue SE"}}},
         {"BC", {{"highway", "primary"}, {"maxspeed", "50"}, {"name", "10th Avenue SE"}}},
         {"CD", {{"highway", "primary"}, {"maxspeed", "30"}, {"name", "10th Avenue SE"}}},
         {"CX", {{"highway", "primary"}, {"maxspeed", "30"}, {"name", "Sidestreet"}}},
         {"DMN", {{"highway", "primary"}, {"maxspeed", "30"}, {"name", "Heinrich Street"}}},
         {"DEF", {{"highway", "primary"}, {"maxspeed", "30"}, {"name", "Alfred Street"}}},
         {"YDZ", {{"highway", "primary"}, {"name", "Market Street"}, {"oneway", "yes"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {0.0, 0.0});

    const std::unordered_map<std::string, std::string> build_config{
        {"mjolnir.data_processing.use_admin_db", "false"}};

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/osrm_serializer_voice", build_config);
  }
  rapidjson::Document json_request(const std::string& from,
                                   const std::string& to,
                                   const std::string& language = "en-US") {
    const std::string& request =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto","voice_instructions":true,"language":"%s"})") %
         std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
         std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()) % language)
            .str();
    auto result = gurka::do_action(valhalla::Options::route, map, request);
    return gurka::convert_to_json(result, Options::Format::Options_Format_osrm);
  }
};

gurka::map VoiceInstructions::map = {};

TEST_F(VoiceInstructions, VoiceInstructionsPresent) {
  auto json = json_request("A", "F");
  auto steps = json["routes"][0]["legs"][0]["steps"].GetArray();
  // Validate that each step (except the last one) has voiceInstructions with announcement,
  // ssmlAnnouncement and distanceAlongGeometry
  for (int step = 0; step < steps.Size() - 1; ++step) {
    ASSERT_TRUE(steps[step].HasMember("voiceInstructions"));
    ASSERT_TRUE(steps[step]["voiceInstructions"].IsArray());

    EXPECT_GT(steps[step]["voiceInstructions"].Size(), 0);
    for (int instr = 0; instr < steps[step]["voiceInstructions"].GetArray().Size(); ++instr) {
      ASSERT_TRUE(steps[step]["voiceInstructions"][instr].HasMember("announcement"));
      ASSERT_TRUE(steps[step]["voiceInstructions"][instr].HasMember("ssmlAnnouncement"));
      ASSERT_TRUE(steps[step]["voiceInstructions"][instr].HasMember("distanceAlongGeometry"));
    }
  }

  // Validate the last step as empty voiceInstructions
  ASSERT_TRUE(steps[steps.Size() - 1].HasMember("voiceInstructions"));
  ASSERT_TRUE(steps[steps.Size() - 1]["voiceInstructions"].IsArray());
  EXPECT_EQ(steps[steps.Size() - 1]["voiceInstructions"].Size(), 0);
}

// depart_instruction
//
// 13 grids * 50m/grid = 650m
// => distanceAlongGeometry = 650m
//
// verbal_transition_alert_instruction
//
// The idea is that the instructions come a fixed amount of seconds before the maneuver takes place.
// For whatever reasons, a distance in meters from the end of the maneuver needs to be provided
// though. When different speeds are used on the road, they all need to be taken into account.
//
// CD: 50m / 30km/h = 50m * 3,600s / 30,000m = 50m * 0.12s/m = 6s
// BC: 50m / 50km/h = 50m * 3,600s / 50,000m = 50m * 0.072s/m = 3.6s
// SECONDS_BEFORE_VERBAL_TRANSITION_ALERT_INSTRUCTION = 35s
// AB: 35s - 6s - 3.6s = 25.4s
//     25.4s * 80 km/h = 25.4s * 80,000m / 3600s ~= 564m
// => distanceAlongGeometry = 564,45m + 50m + 50m = 664m
// => larger then depart_instruction/the maneuver -> won't be played
//
// verbal_pre_transition_instruction
//
// SECONDS_BEFORE_VERBAL_PRE_TRANSITION_INSTRUCTION = 10s
// AB: 10s - 6s - 3.6s = 0.4s
//     0.4s * 80 km/h = 0.4s * 80,000m / 3600s ~= 9m
// => distanceAlongGeometry = 9m + 50m + 50m = 109m
TEST_F(VoiceInstructions, DistanceAlongGeometryVoiceInstructions) {
  auto json = json_request("A", "D");
  auto steps = json["routes"][0]["legs"][0]["steps"].GetArray();

  auto depart_instruction = steps[0]["voiceInstructions"][0].GetObject();
  EXPECT_STREQ(
      depart_instruction["announcement"].GetString(),
      "Drive east on 10th Avenue SE. Then, in 700 meters, You will arrive at your destination.");
  EXPECT_EQ(depart_instruction["distanceAlongGeometry"].GetFloat(), 650.0);
  auto verbal_pre_transition_instruction = steps[0]["voiceInstructions"][1].GetObject();
  EXPECT_STREQ(verbal_pre_transition_instruction["announcement"].GetString(),
               "You have arrived at your destination.");
  EXPECT_EQ(round(verbal_pre_transition_instruction["distanceAlongGeometry"].GetFloat()), 109);
}

TEST_F(VoiceInstructions, ShortDepartVoiceInstructions) {
  auto json = json_request("C", "F");
  auto steps = json["routes"][0]["legs"][0]["steps"].GetArray();

  EXPECT_EQ(steps[0]["voiceInstructions"].Size(), 2);

  auto depart_instruction = steps[0]["voiceInstructions"][0].GetObject();
  EXPECT_STREQ(depart_instruction["announcement"].GetString(),
               "Drive east on 10th Avenue SE. Then Bear right onto Alfred Street.");
  EXPECT_EQ(depart_instruction["distanceAlongGeometry"].GetFloat(), 50.0);
  auto verbal_transition_alert_instruction = steps[0]["voiceInstructions"][1].GetObject();
  EXPECT_STREQ(verbal_transition_alert_instruction["announcement"].GetString(),
               "Bear right onto Alfred Street.");
  EXPECT_EQ(verbal_transition_alert_instruction["distanceAlongGeometry"].GetFloat(), 12.5);
}

TEST_F(VoiceInstructions, ShortIntermediateStepVoiceInstructions) {
  auto json = json_request("X", "Z");
  auto steps = json["routes"][0]["legs"][0]["steps"].GetArray();

  EXPECT_EQ(steps[1]["voiceInstructions"].Size(), 1); // just pre tansition instruction

  auto verbal_pre_transition_instruction = steps[1]["voiceInstructions"][0].GetObject();
  EXPECT_STREQ(verbal_pre_transition_instruction["announcement"].GetString(),
               "Turn right onto Market Street. Then You will arrive at your destination.");
  EXPECT_EQ(verbal_pre_transition_instruction["distanceAlongGeometry"].GetFloat(), 50.0);
}

TEST_F(VoiceInstructions, AllVoiceInstructions) {
  auto json = json_request("A", "F");
  auto steps = json["routes"][0]["legs"][0]["steps"].GetArray();

  auto depart_instruction = steps[0]["voiceInstructions"][0].GetObject();
  EXPECT_STREQ(depart_instruction["announcement"].GetString(),
               "Drive east on 10th Avenue SE. Then Bear right onto Alfred Street.");
  EXPECT_EQ(depart_instruction["distanceAlongGeometry"].GetFloat(), 650.0);

  auto bear_right_instruction = steps[0]["voiceInstructions"][1].GetObject();
  EXPECT_STREQ(bear_right_instruction["announcement"].GetString(), "Bear right onto Alfred Street.");
  EXPECT_EQ(round(bear_right_instruction["distanceAlongGeometry"].GetFloat()), 109);

  auto continue_instruction = steps[1]["voiceInstructions"][0].GetObject();
  EXPECT_STREQ(continue_instruction["announcement"].GetString(), "Continue for 900 meters.");
  EXPECT_EQ(continue_instruction["distanceAlongGeometry"].GetFloat(), 847.0);

  auto arrive_instruction = steps[1]["voiceInstructions"][1].GetObject();
  EXPECT_STREQ(arrive_instruction["announcement"].GetString(),
               "In 300 meters, You will arrive at your destination.");
  // ~= 291.6
  EXPECT_GT(arrive_instruction["distanceAlongGeometry"].GetFloat(), 291);
  EXPECT_LT(arrive_instruction["distanceAlongGeometry"].GetFloat(), 292);

  auto final_arrive_instruction = steps[1]["voiceInstructions"][2].GetObject();
  EXPECT_STREQ(final_arrive_instruction["announcement"].GetString(),
               "You have arrived at your destination.");
  // ~= 83.3
  EXPECT_GT(final_arrive_instruction["distanceAlongGeometry"].GetFloat(), 83);
  EXPECT_LT(final_arrive_instruction["distanceAlongGeometry"].GetFloat(), 84);

  auto last_instruction = steps[2]["voiceInstructions"].GetArray();
  EXPECT_EQ(last_instruction.Size(), 0);
}

TEST_F(VoiceInstructions, DefaultVoiceLocalePresent) {
  auto json = json_request("A", "F");
  auto routes = json["routes"].GetArray();
  // Validate that each route has the default voiceLocale
  for (int route = 0; route < routes.Size(); ++route) {
    ASSERT_TRUE(routes[route].HasMember("voiceLocale"));
    EXPECT_STREQ(routes[route]["voiceLocale"].GetString(), "en-US");
  }
}

TEST_F(VoiceInstructions, VoiceLocalePresent) {
  auto json = json_request("A", "F", "de-DE");
  auto routes = json["routes"].GetArray();
  // Validate that each route has the voiceLocale from the options
  for (int route = 0; route < routes.Size(); ++route) {
    ASSERT_TRUE(routes[route].HasMember("voiceLocale"));
    EXPECT_STREQ(routes[route]["voiceLocale"].GetString(), "de-DE");
  }
}

TEST(Standalone, BannerInstructions) {
  const std::string ascii_map = R"(
    A-------------1-B---X
                     \
                      \-2
                        |
    Y-------------------C---D
                        Z
  )";

  const gurka::ways ways = {{"A1", {{"name", "Alley Broadway"}, {"highway", "motorway"}}},
                            {"1B",
                             {{"name", "Alley Broadway"},
                              {"highway", "motorway"},
                              {"lanes", "2"},
                              {"turn:lanes", "through|through;slight_right"}}},
                            {"B2",
                             {{"name", "Broadway Course"},
                              {"highway", "motorway_link"},
                              {"destination", "Destiny Town"}}},
                            {"2C",
                             {{"name", "Broadway Course"},
                              {"highway", "motorway_link"},
                              {"destination", "Destiny Town"},
                              {"lanes", "2"},
                              {"turn:lanes", "left|right"}}},
                            {"CD", {{"name", "Course Drive"}, {"highway", "primary"}, {"ref", "R2"}}},
                            {"BX", {{"highway", "motorway"}}},
                            {"CY", {{"highway", "primary"}}},
                            {"CZ", {{"highway", "primary"}}}};
  const gurka::nodes nodes = {{"B", {{"highway", "motorway_junction"}, {"ref", "10"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 50, {0, 0});
  auto map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/osrm_serializer_banner_instructions");

  auto from = "A";
  auto to = "D";
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto","banner_instructions":true})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  auto json = gurka::convert_to_json(result, Options::Format::Options_Format_osrm);

  auto steps = json["routes"][0]["legs"][0]["steps"].GetArray();

  // Validate that each step (except the last one) has bannerInstructions with primary
  for (int step = 0; step < steps.Size() - 1; ++step) {
    ASSERT_TRUE(steps[step].HasMember("bannerInstructions"));
    ASSERT_TRUE(steps[step]["bannerInstructions"].IsArray());
    EXPECT_GT(steps[step]["bannerInstructions"].GetArray().Size(), 0);
    for (int instr = 0; instr < steps[step]["bannerInstructions"].GetArray().Size(); ++instr) {
      ASSERT_TRUE(steps[step]["bannerInstructions"][instr].HasMember("distanceAlongGeometry"));
      ASSERT_TRUE(steps[step]["bannerInstructions"][instr].HasMember("primary"));
      ASSERT_TRUE(steps[step]["bannerInstructions"][instr]["primary"].HasMember("type"));
      ASSERT_TRUE(steps[step]["bannerInstructions"][instr]["primary"].HasMember("text"));
      ASSERT_TRUE(steps[step]["bannerInstructions"][instr]["primary"].HasMember("components"));
      ASSERT_TRUE(steps[step]["bannerInstructions"][instr]["primary"]["components"].IsArray());
    }

    // Validate the last step has empty bannerInstructions
    ASSERT_TRUE(steps[steps.Size() - 1].HasMember("bannerInstructions"));
    ASSERT_TRUE(steps[steps.Size() - 1]["bannerInstructions"].IsArray());
    EXPECT_EQ(steps[steps.Size() - 1]["bannerInstructions"].GetArray().Size(), 0);
  }

  // validate first step has two bannerInstruction
  // validate first step's distance of first bannerInstruction
  EXPECT_EQ(steps[0]["bannerInstructions"][0]["distanceAlongGeometry"].GetFloat(), 800.0);
  // validate first step's distance of scond bannerInstruction
  // EXPECT_EQ(steps[0]["bannerInstructions"][1]["distanceAlongGeometry"].GetFloat(), 400.0);

  // validate first step's first bannerInstructions' primary instructions
  // "primary": {
  //   "type": "off ramp",
  //   "modifier": "slight_right",
  //   "text": "Broadway Course",
  //   "components": [
  //     {"text": "Exit","type": "text"},
  //     {"text": "10","type": "exit-number"},
  //     {"text": "Broadway Course","type": "text"}
  //   ]
  // },
  auto primary_0 = steps[0]["bannerInstructions"][0]["primary"].GetObject();
  EXPECT_STREQ(primary_0["type"].GetString(), "off ramp");
  EXPECT_STREQ(primary_0["modifier"].GetString(), "slight right");
  EXPECT_STREQ(primary_0["text"].GetString(), "Broadway Course");
  EXPECT_STREQ(primary_0["text"].GetString(), "Broadway Course");
  EXPECT_EQ(primary_0["components"].GetArray().Size(), 3);
  EXPECT_STREQ(primary_0["components"][0]["type"].GetString(), "exit");
  EXPECT_STREQ(primary_0["components"][0]["text"].GetString(), "Exit");
  EXPECT_STREQ(primary_0["components"][1]["type"].GetString(), "exit-number");
  EXPECT_STREQ(primary_0["components"][1]["text"].GetString(), "10");
  EXPECT_STREQ(primary_0["components"][2]["type"].GetString(), "text");
  EXPECT_STREQ(primary_0["components"][2]["text"].GetString(), "Broadway Course");

  // validate the first step's secondary instructions
  // "secondary": {
  //   "text": "Destiny Town",
  //   "components": [{"text": "Destiny Town","type": "text"}]
  // },
  ASSERT_TRUE(steps[0]["bannerInstructions"][0].HasMember("secondary"));
  ASSERT_TRUE(steps[0]["bannerInstructions"][0]["secondary"].HasMember("text"));
  ASSERT_TRUE(steps[0]["bannerInstructions"][1].HasMember("secondary"));
  ASSERT_TRUE(steps[0]["bannerInstructions"][1]["secondary"].HasMember("text"));
  auto secondary_0 = steps[0]["bannerInstructions"][0]["secondary"].GetObject();
  ASSERT_TRUE(secondary_0.HasMember("text"));
  EXPECT_STREQ(secondary_0["text"].GetString(), "Destiny Town");
  ASSERT_TRUE(secondary_0.HasMember("components"));
  ASSERT_TRUE(secondary_0["components"].IsArray());
  EXPECT_EQ(secondary_0["components"].GetArray().Size(), 1);
  ASSERT_TRUE(secondary_0["components"][0].IsObject());
  EXPECT_STREQ(secondary_0["components"][0]["type"].GetString(), "text");
  EXPECT_STREQ(secondary_0["components"][0]["text"].GetString(), "Destiny Town");

  // validate the first step's second bannerInstructions' sub instructions
  // "sub": {
  //   "text": "",
  //   "components": [
  //     {
  //       "active": false,
  //       "text": "",
  //       "directions": ["straight"],
  //       "type": "lane"
  //     },
  //     {
  //       "active_direction": "slight right",
  //       "active": true,
  //       "text": "",
  //       "directions": ["straight","slight right"],
  //       "type": "lane"
  //     }
  //   ]
  // }
  ASSERT_FALSE(steps[0]["bannerInstructions"][0].HasMember("sub"));
  ASSERT_TRUE(steps[0]["bannerInstructions"][1].HasMember("sub"));
  ASSERT_TRUE(steps[0]["bannerInstructions"][1].HasMember("primary"));
  ASSERT_TRUE(steps[0]["bannerInstructions"][1].HasMember("secondary"));
  ASSERT_EQ(steps[0]["bannerInstructions"][1]["distanceAlongGeometry"].GetFloat(), 400);
  auto sub_0 = steps[0]["bannerInstructions"][1]["sub"].GetObject();
  ASSERT_TRUE(sub_0.HasMember("components"));
  ASSERT_TRUE(sub_0["components"].IsArray());
  EXPECT_EQ(sub_0["components"].GetArray().Size(), 2);
  for (int component = 0; component < sub_0["components"].GetArray().Size(); ++component) {
    EXPECT_STREQ(sub_0["components"][component]["type"].GetString(), "lane");
    ASSERT_TRUE(sub_0["components"][component]["directions"].IsArray());
  }
  EXPECT_STREQ(sub_0["components"][0]["directions"][0].GetString(), "straight");
  ASSERT_FALSE(sub_0["components"][0]["active"].GetBool());
  EXPECT_STREQ(sub_0["components"][1]["directions"][0].GetString(), "straight");
  EXPECT_STREQ(sub_0["components"][1]["directions"][1].GetString(), "slight right");
  EXPECT_STREQ(sub_0["components"][1]["active_direction"].GetString(), "slight right");
  ASSERT_TRUE(sub_0["components"][1]["active"].GetBool());

  // validate second step's primary instructions
  EXPECT_EQ(steps[1]["bannerInstructions"].GetArray().Size(), 1);
  EXPECT_GT(steps[1]["bannerInstructions"][0]["distanceAlongGeometry"].GetFloat(), 310);
  EXPECT_LT(steps[1]["bannerInstructions"][0]["distanceAlongGeometry"].GetFloat(), 340);
  auto primary_1 = steps[1]["bannerInstructions"][0]["primary"].GetObject();
  EXPECT_STREQ(primary_1["type"].GetString(), "turn");
  EXPECT_STREQ(primary_1["modifier"].GetString(), "left");
  EXPECT_STREQ(primary_1["text"].GetString(), "Course Drive");
  ASSERT_TRUE(primary_1.HasMember("components"));
  ASSERT_TRUE(primary_1["components"].IsArray());
  EXPECT_EQ(primary_1["components"].GetArray().Size(), 3);
  ASSERT_TRUE(primary_1["components"][0].IsObject());
  EXPECT_STREQ(primary_1["components"][0]["type"].GetString(), "text");
  EXPECT_STREQ(primary_1["components"][0]["text"].GetString(), "Course Drive");
  EXPECT_STREQ(primary_1["components"][1]["type"].GetString(), "delimiter");
  EXPECT_STREQ(primary_1["components"][1]["text"].GetString(), "/");
  EXPECT_STREQ(primary_1["components"][2]["type"].GetString(), "text");
  EXPECT_STREQ(primary_1["components"][2]["text"].GetString(), "R2");

  // validate the second step's second bannerInstructions' sub instructions
  // "sub": {
  //   "text": "",
  //   "components": [
  //     {
  //       "active_direction": "left",
  //       "active": true,
  //       "text": "",
  //       "directions": ["left"],
  //       "type": "lane"
  //     },
  //     {
  //       "active": false,
  //       "text": "",
  //       "directions": ["right"],
  //       "type": "lane"
  //     }
  //   ]
  // },
  ASSERT_TRUE(steps[1]["bannerInstructions"][0].HasMember("sub"));
  auto sub_1 = steps[1]["bannerInstructions"][0]["sub"].GetObject();
  ASSERT_TRUE(sub_1.HasMember("components"));
  ASSERT_TRUE(sub_1["components"].IsArray());
  EXPECT_EQ(sub_1["components"].GetArray().Size(), 2);
  for (int component = 0; component < sub_1["components"].GetArray().Size(); ++component) {
    EXPECT_STREQ(sub_1["components"][component]["type"].GetString(), "lane");
    ASSERT_TRUE(sub_1["components"][component]["directions"].IsArray());
  }
  EXPECT_STREQ(sub_1["components"][0]["directions"][0].GetString(), "left");
  ASSERT_TRUE(sub_1["components"][0]["active"].GetBool());
  EXPECT_STREQ(sub_1["components"][0]["active_direction"].GetString(), "left");
  EXPECT_STREQ(sub_1["components"][1]["directions"][0].GetString(), "right");
  ASSERT_FALSE(sub_1["components"][1]["active"].GetBool());

  // validate third step's primary instructions
  auto primary_2 = steps[2]["bannerInstructions"][0]["primary"].GetObject();
  EXPECT_STREQ(primary_2["type"].GetString(), "arrive");
  EXPECT_STREQ(primary_2["text"].GetString(), "You have arrived at your destination.");
}

TEST(Standalone, BannerInstructionsRoundabout) {
  const std::string ascii_map = R"(
           B
          3|
         J-I-H
        /K   |
    A---1D   |
         |   |
         E-F-G
          \2
           L
  )";

  const gurka::ways ways =
      {{"A1D", {{"name", "Road Name"}, {"highway", "primary"}, {"oneway", "yes"}}},
       {"DEFGHIJKD",
        {{"name", "Small Roundabout"}, {"highway", "primary"}, {"junction", "roundabout"}}},
       {"B3J", {{"name", "Other"}, {"highway", "primary"}, {"oneway", "yes"}}},
       {"IB", {{"name", "Other"}, {"highway", "primary"}, {"oneway", "yes"}}},
       {"L2F", {{"name", "Other"}, {"highway", "primary"}, {"oneway", "yes"}}},
       {"EL", {{"name", "Other"}, {"highway", "primary"}, {"oneway", "yes"}}}};
  const gurka::nodes nodes = {{"1", {{"highway", "give_way"}, {"direction", "forward"}}},
                              {"2", {{"highway", "give_way"}, {"direction", "forward"}}},
                              {"3", {{"highway", "give_way"}, {"direction", "forward"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 20, {0, 0});
  const std::unordered_map<std::string, std::string> build_config{
      {"mjolnir.data_processing.use_admin_db", "false"}};

  auto map =
      gurka::buildtiles(layout, ways, nodes, {},
                        "test/data/osrm_serializer_banner_instructions_roundabout", build_config);

  auto from = "A";
  auto to = "B";
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto","banner_instructions":true})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  auto json = gurka::convert_to_json(result, Options::Format::Options_Format_osrm);

  auto steps = json["routes"][0]["legs"][0]["steps"].GetArray();

  // Validate that each step (except the last one) has bannerInstructions with primary
  for (int step = 0; step < steps.Size() - 1; ++step) {
    ASSERT_TRUE(steps[step].HasMember("bannerInstructions"));
    ASSERT_TRUE(steps[step]["bannerInstructions"].IsArray());
    EXPECT_GT(steps[step]["bannerInstructions"].GetArray().Size(), 0);
    for (int instr = 0; instr < steps[step]["bannerInstructions"].GetArray().Size(); ++instr) {
      ASSERT_TRUE(steps[step]["bannerInstructions"][instr].HasMember("distanceAlongGeometry"));
      ASSERT_TRUE(steps[step]["bannerInstructions"][instr].HasMember("primary"));
      ASSERT_TRUE(steps[step]["bannerInstructions"][instr]["primary"].HasMember("type"));
      ASSERT_TRUE(steps[step]["bannerInstructions"][instr]["primary"].HasMember("text"));
      ASSERT_TRUE(steps[step]["bannerInstructions"][instr]["primary"].HasMember("components"));
      ASSERT_TRUE(steps[step]["bannerInstructions"][instr]["primary"]["components"].IsArray());
    }
  }

  // Validate the last step has empty bannerInstructions
  ASSERT_TRUE(steps[steps.Size() - 1].HasMember("bannerInstructions"));
  ASSERT_TRUE(steps[steps.Size() - 1]["bannerInstructions"].IsArray());
  EXPECT_EQ(steps[steps.Size() - 1]["bannerInstructions"].GetArray().Size(), 0);

  // validate first step's primary instructions
  auto primary_0 = steps[0]["bannerInstructions"][0]["primary"].GetObject();
  EXPECT_STREQ(primary_0["type"].GetString(), "rotary");
  ASSERT_TRUE(primary_0.HasMember("degrees"));
  EXPECT_EQ(primary_0["degrees"].GetFloat(), 270);
  EXPECT_STREQ(primary_0["text"].GetString(), "Other");
}

class Rotary : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
                              V
                              |
                              |
                              |
                              U
                             / \
                            3   |
                            K---J
                       L---/     \---I
                    --/               \--
                   /                     \
                  M                       H
                  |                       |
            /----N                         G-2--\
      Q----R     |                         |     S------T
            \--1-A                         F----/
                  |                       |
                  B                       E
                   \                     /
                    --\               /--
                       C---\     /---D
                            Y---Z


    )";

    const gurka::ways ways =
        {{"ABCYZDEFG", {{"name", "Grand Rotary"}, {"highway", "primary"}, {"junction", "circular"}}},
         {"GHIJK", {{"name", "Grand Rotary"}, {"highway", "primary"}, {"junction", "circular"}}},
         {"KLMNA", {{"name", "Grand Rotary"}, {"highway", "primary"}, {"junction", "circular"}}},

         {"QR", {{"name", "Western Road"}, {"highway", "primary"}}},
         {"R1A", {{"name", "Western Road"}, {"highway", "primary"}, {"oneway", "yes"}}},
         {"NR", {{"name", "Western Road"}, {"highway", "primary"}, {"oneway", "yes"}}},

         {"TS", {{"name", "Eastern Road"}, {"highway", "primary"}}},
         {"S2G", {{"name", "Eastern Road"}, {"highway", "primary"}, {"oneway", "yes"}}},
         {"FS", {{"name", "Eastern Road"}, {"highway", "primary"}, {"oneway", "yes"}}},

         {"UV", {{"name", "Northern Road"}, {"highway", "primary"}}},
         {"U3K", {{"name", "Northern Road"}, {"highway", "primary"}, {"oneway", "yes"}}},
         {"JU", {{"name", "Northern Road"}, {"highway", "primary"}, {"oneway", "yes"}}}};
    const gurka::nodes nodes = {{"1", {{"highway", "give_way"}, {"direction", "forward"}}},
                                {"2", {{"highway", "give_way"}, {"direction", "forward"}}},
                                {"3", {{"highway", "give_way"}, {"direction", "forward"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10, {13.34792, 52.51585});
    const std::unordered_map<std::string, std::string> build_config{
        {"mjolnir.data_processing.use_admin_db", "false"}};

    map = gurka::buildtiles(layout, ways, nodes, {},
                            "test/data/osrm_serializer_banner_instructions_rotary", build_config);
  }

  rapidjson::Document json_request(const std::string& from, const std::string& to) {
    const std::string& request =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto","banner_instructions":true})") %
         std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
         std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
            .str();
    auto result = gurka::do_action(valhalla::Options::route, map, request);
    return gurka::convert_to_json(result, Options::Format::Options_Format_osrm);
  }
};

gurka::map Rotary::map = {};

TEST_F(Rotary, BannerInstructionsRotaryWestNorth) {
  auto json = json_request("Q", "V");
  auto steps = json["routes"][0]["legs"][0]["steps"].GetArray();

  auto primary_0 = steps[0]["bannerInstructions"][0]["primary"].GetObject();

  EXPECT_STREQ(primary_0["type"].GetString(), "rotary");
  ASSERT_TRUE(primary_0.HasMember("degrees"));
  EXPECT_GT(primary_0["degrees"].GetFloat(), 225);
  EXPECT_LT(primary_0["degrees"].GetFloat(), 315);
  ASSERT_TRUE(primary_0.HasMember("driving_side"));
  EXPECT_STREQ(primary_0["driving_side"].GetString(), "right");
  EXPECT_STREQ(primary_0["text"].GetString(), "Northern Road");

  auto primary_1 = steps[1]["bannerInstructions"][0]["primary"].GetObject();

  EXPECT_STREQ(primary_1["type"].GetString(), "exit rotary");
  ASSERT_TRUE(primary_1.HasMember("degrees"));
  EXPECT_GT(primary_1["degrees"].GetFloat(), 225);
  EXPECT_LT(primary_1["degrees"].GetFloat(), 315);
  ASSERT_TRUE(primary_1.HasMember("driving_side"));
  EXPECT_STREQ(primary_1["driving_side"].GetString(), "right");
  EXPECT_STREQ(primary_1["text"].GetString(), "Northern Road");
}

TEST_F(Rotary, BannerInstructionsOnRotary) {
  auto json = json_request("A", "V");
  auto steps = json["routes"][0]["legs"][0]["steps"].GetArray();

  auto primary_0 = steps[0]["bannerInstructions"][0]["primary"].GetObject();

  EXPECT_STREQ(primary_0["type"].GetString(), "exit roundabout");
  ASSERT_TRUE(primary_0.HasMember("degrees"));
  ASSERT_TRUE(primary_0.HasMember("driving_side"));
  EXPECT_STREQ(primary_0["driving_side"].GetString(), "right");
  EXPECT_STREQ(primary_0["text"].GetString(), "Northern Road");
}

TEST_F(Rotary, BannerInstructionsRotaryEastNorth) {
  auto json = json_request("T", "V");
  auto steps = json["routes"][0]["legs"][0]["steps"].GetArray();

  auto primary_0 = steps[0]["bannerInstructions"][0]["primary"].GetObject();

  EXPECT_STREQ(primary_0["type"].GetString(), "rotary");
  ASSERT_TRUE(primary_0.HasMember("degrees"));
  EXPECT_GT(primary_0["degrees"].GetFloat(), 45);
  EXPECT_LT(primary_0["degrees"].GetFloat(), 135);
  ASSERT_TRUE(primary_0.HasMember("driving_side"));
  EXPECT_STREQ(primary_0["driving_side"].GetString(), "right");
  EXPECT_STREQ(primary_0["text"].GetString(), "Northern Road");

  auto primary_1 = steps[1]["bannerInstructions"][0]["primary"].GetObject();

  EXPECT_STREQ(primary_1["type"].GetString(), "exit rotary");
  ASSERT_TRUE(primary_1.HasMember("degrees"));
  EXPECT_GT(primary_1["degrees"].GetFloat(), 45);
  EXPECT_LT(primary_1["degrees"].GetFloat(), 135);
  ASSERT_TRUE(primary_1.HasMember("driving_side"));
  EXPECT_STREQ(primary_1["driving_side"].GetString(), "right");
  EXPECT_STREQ(primary_1["text"].GetString(), "Northern Road");
}

TEST_F(Rotary, BannerInstructionsRotaryEastWest) {
  auto json = json_request("Q", "T");
  auto steps = json["routes"][0]["legs"][0]["steps"].GetArray();

  auto primary_0 = steps[0]["bannerInstructions"][0]["primary"].GetObject();

  EXPECT_STREQ(primary_0["type"].GetString(), "rotary");
  ASSERT_TRUE(primary_0.HasMember("degrees"));
  EXPECT_GT(primary_0["degrees"].GetFloat(), 135);
  EXPECT_LT(primary_0["degrees"].GetFloat(), 225);
  ASSERT_TRUE(primary_0.HasMember("driving_side"));
  EXPECT_STREQ(primary_0["driving_side"].GetString(), "right");
  EXPECT_STREQ(primary_0["text"].GetString(), "Eastern Road");

  auto primary_1 = steps[1]["bannerInstructions"][0]["primary"].GetObject();

  EXPECT_STREQ(primary_1["type"].GetString(), "exit rotary");
  ASSERT_TRUE(primary_1.HasMember("degrees"));
  EXPECT_GT(primary_1["degrees"].GetFloat(), 135);
  EXPECT_LT(primary_1["degrees"].GetFloat(), 225);
  ASSERT_TRUE(primary_1.HasMember("driving_side"));
  EXPECT_STREQ(primary_1["driving_side"].GetString(), "right");
  EXPECT_STREQ(primary_1["text"].GetString(), "Eastern Road");
}

TEST_F(Rotary, EndOnRotary) {
  // This is just a test that ending on the rotary doesn't cause a segfault :D
  auto json = json_request("Q", "F");
  auto steps = json["routes"][0]["legs"][0]["steps"].GetArray();

  auto primary_0 = steps[0]["bannerInstructions"][0]["primary"].GetObject();

  EXPECT_STREQ(primary_0["type"].GetString(), "rotary");
  ASSERT_TRUE(primary_0.HasMember("degrees"));
}
