#include "gurka.h"
#include "src/mjolnir/legal_speed.h"
#include "src/mjolnir/speed_assigner.h"
#include <gtest/gtest.h>

#include <filesystem>
#include <utility>

using namespace valhalla;

struct expected_edge_speed {
  std::string start_node;
  std::string end_node;
  uint32_t expected;
  uint32_t truck_expected;
  expected_edge_speed() = delete;
  expected_edge_speed(std::string sn, std::string en, uint32_t e, uint32_t t)
      : start_node(std::move(sn)), end_node(std::move(en)), expected(e), truck_expected(t) {
  }
};

struct testable_legal_speed_assigner : public SimpleLegalSpeedAssigner {
public:
  testable_legal_speed_assigner(const std::string& file_path, const bool update_speeds)
      : SimpleLegalSpeedAssigner(file_path, update_speeds) {
  }
  using SimpleLegalSpeedAssigner::legal_speeds_map_;
};

TEST(Standalone, DefaultSpeedConfig) {
  const std::string urban = R"(
    a  j  i
     ABCDE
     FGHIJ
    bKLMNOh
     PQRST
    cUVWXYg
     Z xyz
    d  e  f
  )";

  auto layout = gurka::detail::map_to_coordinates(urban, 190);

  const std::string rural = R"(
            0 klmnopqrstuvw
           9 1
          8   2
          7   3
           6 4
            5
  )";

  const auto rural_layout = gurka::detail::map_to_coordinates(rural, 100, {7, 58});
  layout.insert(rural_layout.begin(), rural_layout.end());

  std::pair<std::string, std::string> oneway{"oneway", "yes"};
  std::pair<std::string, std::string> dest{"destination", "north pole"};

  gurka::ways ways = {
      // way (we throw out speeds lower than 10kph so we had to start higher)
      {"AK", {{"maxspeed", "66"}, {"highway", "motorway"}}},
      {"AL", {{"maxspeed", "67"}, {"highway", "trunk"}}},
      {"AM", {{"maxspeed", "68"}, {"highway", "primary"}}},
      {"AN", {{"maxspeed", "69"}, {"highway", "secondary"}}},
      {"DK", {{"maxspeed", "70"}, {"highway", "tertiary"}}},
      {"DL", {{"maxspeed", "71"}, {"highway", "unclassified"}}},
      {"DM", {{"maxspeed", "72"}, {"highway", "residential"}}},
      {"DN", {{"maxspeed", "73"}, {"highway", "service"}}},
      // link exiting
      {"OP", {{"maxspeed", "10"}, oneway, dest, {"highway", "motorway_link"}}},
      {"QR", {{"maxspeed", "11"}, oneway, dest, {"highway", "trunk_link"}}},
      {"ST", {{"maxspeed", "12"}, oneway, dest, {"highway", "primary_link"}}},
      {"UV", {{"maxspeed", "13"}, oneway, dest, {"highway", "secondary_link"}}},
      {"WX", {{"maxspeed", "14"}, oneway, dest, {"highway", "tertiary_link"}}},
      // link turning
      {"AB", {{"maxspeed", "16"}, oneway, {"highway", "motorway_link"}}},
      {"CD", {{"maxspeed", "17"}, oneway, {"highway", "trunk_link"}}},
      {"AG", {{"maxspeed", "18"}, oneway, {"highway", "primary_link"}}},
      {"HI", {{"maxspeed", "19"}, oneway, {"highway", "secondary_link"}}},
      {"KL", {{"maxspeed", "20"}, oneway, {"highway", "tertiary_link"}}},
      // roundabout
      {"BC", {{"maxspeed", "22"}, {"junction", "roundabout"}, {"highway", "motorway"}}},
      {"BD", {{"maxspeed", "23"}, {"junction", "roundabout"}, {"highway", "trunk"}}},
      {"BI", {{"maxspeed", "24"}, {"junction", "roundabout"}, {"highway", "primary"}}},
      {"BH", {{"maxspeed", "25"}, {"junction", "roundabout"}, {"highway", "secondary"}}},
      {"BK", {{"maxspeed", "26"}, {"junction", "roundabout"}, {"highway", "tertiary"}}},
      {"BE", {{"maxspeed", "27"}, {"junction", "roundabout"}, {"highway", "unclassified"}}},
      {"BF", {{"maxspeed", "28"}, {"junction", "roundabout"}, {"highway", "residential"}}},
      {"BJ", {{"maxspeed", "29"}, {"junction", "roundabout"}, {"highway", "service"}}},
      // service
      {"BW", {{"maxspeed", "30"}, {"highway", "service"}, {"service", "driveway"}}},
      {"BX", {{"maxspeed", "31"}, {"highway", "service"}, {"service", "alley"}}},
      {"BY", {{"maxspeed", "32"}, {"highway", "service"}, {"service", "parking_aisle"}}},
      {"BZ", {{"maxspeed", "33"}, {"highway", "service"}, {"service", "drive-through"}}},
      // ferry stuff is untouched
      {"xy", {{"maxspeed", "1"}, {"route", "ferry"}, {"motor_vehicle", "yes"}}},
      {"yz", {{"maxspeed", "1"}, {"route", "shuttle_train"}, {"motor_vehicle", "yes"}}},

      // way
      {"01", {{"maxspeed", "34"}, {"highway", "motorway"}}},
      {"02", {{"maxspeed", "35"}, {"highway", "trunk"}}},
      {"03", {{"maxspeed", "36"}, {"highway", "primary"}}},
      {"04", {{"maxspeed", "37"}, {"highway", "secondary"}}},
      {"05", {{"maxspeed", "38"}, {"highway", "tertiary"}}},
      {"06", {{"maxspeed", "39"}, {"highway", "unclassified"}}},
      {"07", {{"maxspeed", "40"}, {"highway", "residential"}}},
      {"08", {{"maxspeed", "41"}, {"highway", "service"}}},
      // link exiting
      {"36", {{"maxspeed", "42"}, oneway, dest, {"highway", "motorway_link"}}},
      {"37", {{"maxspeed", "43"}, oneway, dest, {"highway", "trunk_link"}}},
      {"13", {{"maxspeed", "44"}, oneway, dest, {"highway", "primary_link"}}},
      {"14", {{"maxspeed", "45"}, oneway, dest, {"highway", "secondary_link"}}},
      {"15", {{"maxspeed", "46"}, oneway, dest, {"highway", "tertiary_link"}}},
      // link turning
      {"kl", {{"maxspeed", "48"}, oneway, {"highway", "motorway_link"}}},
      {"mn", {{"maxspeed", "49"}, oneway, {"highway", "trunk_link"}}},
      {"op", {{"maxspeed", "50"}, oneway, {"highway", "primary_link"}}},
      {"qr", {{"maxspeed", "51"}, oneway, {"highway", "secondary_link"}}},
      {"st", {{"maxspeed", "52"}, oneway, {"highway", "tertiary_link"}}},
      // roundabouts
      {"24", {{"maxspeed", "54"}, {"junction", "roundabout"}, {"highway", "motorway"}}},
      {"25", {{"maxspeed", "55"}, {"junction", "roundabout"}, {"highway", "trunk"}}},
      {"26", {{"maxspeed", "56"}, {"junction", "roundabout"}, {"highway", "primary"}}},
      {"27", {{"maxspeed", "57"}, {"junction", "roundabout"}, {"highway", "secondary"}}},
      {"28", {{"maxspeed", "58"}, {"junction", "roundabout"}, {"highway", "tertiary"}}},
      {"29", {{"maxspeed", "59"}, {"junction", "roundabout"}, {"highway", "unclassified"}}},
      {"34", {{"maxspeed", "60"}, {"junction", "roundabout"}, {"highway", "residential"}}},
      {"35", {{"maxspeed", "61"}, {"junction", "roundabout"}, {"highway", "service"}}},
      // service
      {"17", {{"maxspeed", "62"}, {"highway", "service"}, {"service", "driveway"}}},
      {"18", {{"maxspeed", "63"}, {"highway", "service"}, {"service", "alley"}}},
      {"19", {{"maxspeed", "64"}, {"highway", "service"}, {"service", "parking_aisle"}}},
      {"23", {{"maxspeed", "65"}, {"highway", "service"}, {"service", "drive-through"}}},
      // ferry stuff is untouched
      {"uv", {{"maxspeed", "1"}, {"route", "ferry"}, {"motor_vehicle", "yes"}}},
      {"vw", {{"maxspeed", "1"}, {"route", "shuttle_train"}, {"motor_vehicle", "yes"}}},
  };

  // we need to add a bunch of edges to pump up the road density
  for (char from = 'a'; from <= 'j'; ++from) {
    for (char to = from + 1; to <= 'j'; ++to) {
      for (char too = to + 1; too <= 'j'; ++too) {
        std::string nodes = "";
        nodes.push_back(from);
        nodes.push_back(to);
        nodes.push_back(too);
        ways.emplace(nodes, std::map<std::string, std::string>{
                                {"maxspeed", "99"},
                                {"highway", "residential"},
                            });
      }
    }
  }

  if (!std::filesystem::exists("test/data") && !std::filesystem::create_directories("test/data"))
    throw std::runtime_error("couldn't create directories");
  std::filesystem::remove("test/data/speed_config.json");

  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"(
      [{
        "urban": {
          "way": [65,66,67,68,69,70,71,72],
          "link_exiting": [9,10,11,12,13],
          "link_turning": [15,16,17,18,19],
          "roundabout": [21,22,23,24,25,26,27,28],
          "driveway": 29,
          "alley": 30,
          "parking_aisle": 31,
          "drive-through": 32
        },
        "suburban": {
          "way": [88,88,88,88,88,88,88,88],
          "link_exiting": [88,88,88,88,88],
          "link_turning": [88,88,88,88,88],
          "roundabout": [88,88,88,88,88,88,88,88],
          "driveway": 88,
          "alley": 88,
          "parking_aisle": 88,
          "drive-through": 88
        },
        "rural": {
          "way": [33,34,35,36,37,38,39,40],
          "link_exiting": [41,42,43,44,45],
          "link_turning": [47,48,49,50,51],
          "roundabout": [53,54,55,56,57,58,59,60],
          "driveway": 61,
          "alley": 62,
          "parking_aisle": 63,
          "drive-through": 64
        }
      }]
    )";
  }

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/speed_config",
                               {
                                   {"mjolnir.default_speeds_config", "test/data/speed_config.json"},
                                   {"mjolnir.reclassify_links", "false"},
                               });

  // NOTE: So the ways above are specified in the order of the speed config below. Notice that the
  // configs speeds are in ascending order so too are the maxspeed (speed limit) tags on all of the
  // ways except that they are 1 kph higher than the default speeds we want to assign. This makes it
  // so that our test cases are also built into the data so that to verify the assignment happened
  // properly all we have to do is subtract one from the speed limit and check that the default speed
  // has that value EXCEPT the ferries because they should remain untouched by the config speeds
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  for (auto tile_id : reader.GetTileSet()) {
    auto tile = reader.GetGraphTile(tile_id);
    for (const auto& edge : tile->GetDirectedEdges()) {
      auto info = tile->edgeinfo(&edge);
      auto name = info.GetNames().front();
      // skip anything that looks like the extra length we added to make it urban
      if (name.front() >= 'a' && name.front() <= 'j')
        continue;
      // skip links that aren't drivable in the forward direction (since they arent exits or turns)
      if (edge.link() && !(edge.forwardaccess() & baldr::kVehicularAccess))
        continue;

      int diff = info.speed_limit() - edge.speed();
      if (edge.use() == baldr::Use::kFerry || edge.use() == baldr::Use::kRailFerry)
        EXPECT_LT(diff, 0)
            << name << " should have had a negative difference in speed limit and default speed";
      else
        EXPECT_EQ(diff, 1)
            << name << " should have had a default speed that was one less than the speed limit";
    }
  }
}

TEST(Standalone, SuburbanSpeedConfig) {
  const std::string suburban = R"(
    a  j  i
     ABCDE
     FGHIJ
    bKLMNOh
     PQRST
    cUVWXYg
     Z xyz
    d  e  f
  )";

  auto layout = gurka::detail::map_to_coordinates(suburban, 100);

  std::pair<std::string, std::string> oneway{"oneway", "yes"};
  std::pair<std::string, std::string> dest{"destination", "north pole"};

  gurka::ways ways = {
      // way (we throw out speeds lower than 10kph so we had to start higher)
      {"AK", {{"maxspeed", "66"}, {"highway", "motorway"}}},
      {"AL", {{"maxspeed", "67"}, {"highway", "trunk"}}},
      {"AM", {{"maxspeed", "68"}, {"highway", "primary"}}},
      {"AN", {{"maxspeed", "69"}, {"highway", "secondary"}}},
      {"DK", {{"maxspeed", "70"}, {"highway", "tertiary"}}},
      {"DL", {{"maxspeed", "71"}, {"highway", "unclassified"}}},
      {"DM", {{"maxspeed", "72"}, {"highway", "residential"}}},
      {"DN", {{"maxspeed", "73"}, {"highway", "service"}}},
      // link exiting
      {"OP", {{"maxspeed", "10"}, oneway, dest, {"highway", "motorway_link"}}},
      {"QR", {{"maxspeed", "11"}, oneway, dest, {"highway", "trunk_link"}}},
      {"ST", {{"maxspeed", "12"}, oneway, dest, {"highway", "primary_link"}}},
      {"UV", {{"maxspeed", "13"}, oneway, dest, {"highway", "secondary_link"}}},
      {"WX", {{"maxspeed", "14"}, oneway, dest, {"highway", "tertiary_link"}}},
      // link turning
      {"AB", {{"maxspeed", "16"}, oneway, {"highway", "motorway_link"}}},
      {"CD", {{"maxspeed", "17"}, oneway, {"highway", "trunk_link"}}},
      {"AG", {{"maxspeed", "18"}, oneway, {"highway", "primary_link"}}},
      {"HI", {{"maxspeed", "19"}, oneway, {"highway", "secondary_link"}}},
      {"KL", {{"maxspeed", "20"}, oneway, {"highway", "tertiary_link"}}},
      // roundabout
      {"BC", {{"maxspeed", "22"}, {"junction", "roundabout"}, {"highway", "motorway"}}},
      {"BD", {{"maxspeed", "23"}, {"junction", "roundabout"}, {"highway", "trunk"}}},
      {"BI", {{"maxspeed", "24"}, {"junction", "roundabout"}, {"highway", "primary"}}},
      {"BH", {{"maxspeed", "25"}, {"junction", "roundabout"}, {"highway", "secondary"}}},
      {"BK", {{"maxspeed", "26"}, {"junction", "roundabout"}, {"highway", "tertiary"}}},
      {"BE", {{"maxspeed", "27"}, {"junction", "roundabout"}, {"highway", "unclassified"}}},
      {"BF", {{"maxspeed", "28"}, {"junction", "roundabout"}, {"highway", "residential"}}},
      {"BJ", {{"maxspeed", "29"}, {"junction", "roundabout"}, {"highway", "service"}}},
      // service
      {"BW", {{"maxspeed", "30"}, {"highway", "service"}, {"service", "driveway"}}},
      {"BX", {{"maxspeed", "31"}, {"highway", "service"}, {"service", "alley"}}},
      {"BY", {{"maxspeed", "32"}, {"highway", "service"}, {"service", "parking_aisle"}}},
      {"BZ", {{"maxspeed", "33"}, {"highway", "service"}, {"service", "drive-through"}}},
      // ferry stuff is untouched
      {"xy", {{"maxspeed", "1"}, {"route", "ferry"}, {"motor_vehicle", "yes"}}},
      {"yz", {{"maxspeed", "1"}, {"route", "shuttle_train"}, {"motor_vehicle", "yes"}}},
  };

  // we need to add a bunch of edges to pump up the road density
  for (char from = 'a'; from <= 'j'; ++from) {
    for (char to = from + 1; to <= 'j'; ++to) {
      for (char too = to + 1; too <= 'j'; ++too) {
        std::string nodes = "";
        nodes.push_back(from);
        nodes.push_back(to);
        nodes.push_back(too);
        ways.emplace(nodes, std::map<std::string, std::string>{
                                {"maxspeed", "99"},
                                {"highway", "residential"},
                            });
      }
    }
  }

  if (!std::filesystem::exists("test/data") && !std::filesystem::create_directories("test/data"))
    throw std::runtime_error("couldn't create directories");
  std::filesystem::remove("test/data/speed_config_suburban.json");

  {
    std::ofstream speed_config("test/data/speed_config_suburban.json");
    speed_config << R"(
      [{
        "suburban": {
          "way": [65,66,67,68,69,70,71,72],
          "link_exiting": [9,10,11,12,13],
          "link_turning": [15,16,17,18,19],
          "roundabout": [21,22,23,24,25,26,27,28],
          "driveway": 29,
          "alley": 30,
          "parking_aisle": 31,
          "drive-through": 32
        },
        "urban": {
          "way": [88,88,88,88,88,88,88,88],
          "link_exiting": [88,88,88,88,88],
          "link_turning": [88,88,88,88,88],
          "roundabout": [88,88,88,88,88,88,88,88],
          "driveway": 88,
          "alley": 88,
          "parking_aisle": 88,
          "drive-through": 88
        },
        "rural": {
          "way": [33,34,35,36,37,38,39,40],
          "link_exiting": [41,42,43,44,45],
          "link_turning": [47,48,49,50,51],
          "roundabout": [53,54,55,56,57,58,59,60],
          "driveway": 61,
          "alley": 62,
          "parking_aisle": 63,
          "drive-through": 64
        }
      }]
    )";
  }

  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/speed_config_suburban",
                        {
                            {"mjolnir.default_speeds_config", "test/data/speed_config_suburban.json"},
                            {"mjolnir.reclassify_links", "false"},
                        });

  // NOTE: So the ways above are specified in the order of the speed config below. Notice that the
  // configs speeds are in ascending order so too are the maxspeed (speed limit) tags on all of the
  // ways except that they are 1 kph higher than the default speeds we want to assign. This makes it
  // so that our test cases are also built into the data so that to verify the assignment happened
  // properly all we have to do is subtract one from the speed limit and check that the default speed
  // has that value EXCEPT the ferries because they should remain untouched by the config speeds
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  for (auto tile_id : reader.GetTileSet()) {
    auto tile = reader.GetGraphTile(tile_id);
    for (const auto& edge : tile->GetDirectedEdges()) {
      auto info = tile->edgeinfo(&edge);
      auto name = info.GetNames().front();
      // skip anything that looks like the extra length we added to make it urban
      if (name.front() >= 'a' && name.front() <= 'j')
        continue;
      // skip links that aren't drivable in the forward direction (since they arent exits or turns)
      if (edge.link() && !(edge.forwardaccess() & baldr::kVehicularAccess))
        continue;

      int diff = info.speed_limit() - edge.speed();
      if (edge.use() == baldr::Use::kFerry || edge.use() == baldr::Use::kRailFerry)
        EXPECT_LT(diff, 0)
            << name << " should have had a negative difference in speed limit and default speed";
      else
        EXPECT_EQ(diff, 1)
            << name << " should have had a default speed that was one less than the speed limit";
    }
  }
}

struct testable_assigner : public SpeedAssigner {
public:
  testable_assigner(const std::string& file_path) : SpeedAssigner(file_path) {
  }
  using SpeedAssigner::tables;
};

TEST(Standalone, AdminFallback) {
  DirectedEdge edge{};
  edge.set_all_forward_access();

  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"(
      [{
        "urban": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11, "alley": 11,
          "parking_aisle": 11, "drive-through": 11
        },
        "suburban": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11, "alley": 11,
          "parking_aisle": 11, "drive-through": 11
        },
        "rural": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11, "alley": 11,
          "parking_aisle": 11, "drive-through": 11
        }
      }]
    )";
    speed_config.close();
    testable_assigner assigner("test/data/speed_config.json");
    ASSERT_TRUE(assigner.UpdateSpeed(edge, 6, true, "foo", "bar"));
    ASSERT_TRUE(assigner.UpdateSpeed(edge, 6, true, "foo", ""));
    ASSERT_TRUE(assigner.UpdateSpeed(edge, 6, true, "", "bar"));
    ASSERT_TRUE(assigner.UpdateSpeed(edge, 6, true, "", ""));
  }

  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"(
      [{
        "iso3166-1": "US", "iso3166-2": "PA",
        "urban": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11, "alley": 11,
          "parking_aisle": 11, "drive-through": 11
        },
        "suburban": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11, "alley": 11,
          "parking_aisle": 11, "drive-through": 11
        },
        "rural": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11, "alley": 11,
          "parking_aisle": 11, "drive-through": 11
        }
      }]
    )";
    speed_config.close();
    testable_assigner assigner("test/data/speed_config.json");
    ASSERT_FALSE(assigner.UpdateSpeed(edge, 6, true, "foo", "bar"));
    ASSERT_FALSE(assigner.UpdateSpeed(edge, 6, true, "foo", ""));
    ASSERT_FALSE(assigner.UpdateSpeed(edge, 6, true, "", "bar"));
    ASSERT_FALSE(assigner.UpdateSpeed(edge, 6, true, "", ""));
    edge.set_link(true);
    edge.set_classification(baldr::RoadClass::kServiceOther);
    ASSERT_FALSE(assigner.UpdateSpeed(edge, 6, true, "US", "PA"));
    ASSERT_FALSE(assigner.UpdateSpeed(edge, 6, true, "US", "PA"));
    edge.set_classification(baldr::RoadClass::kMotorway);
    edge.set_link(false);
  }

  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"(
      [{
        "iso3166-1": "US", "iso3166-2": "PA",
        "urban": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11, "alley": 11,
          "parking_aisle": 11, "drive-through": 11
        },
        "suburban": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11, "alley": 11,
          "parking_aisle": 11, "drive-through": 11
        },
        "rural": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11, "alley": 11,
          "parking_aisle": 11, "drive-through": 11
        }
      },
      {
        "iso3166-1": "US",
        "urban": {
          "way": [12,12,12,12,12,12,12,12], "link_exiting": [12,12,12,12,12], "link_turning":
          [12,12,12,12,12], "roundabout": [12,12,12,12,12,12,12,12], "driveway": 12, "alley": 12,
          "parking_aisle": 12, "drive-through": 12
        },
        "suburban": {
          "way": [12,12,12,12,12,12,12,12], "link_exiting": [12,12,12,12,12], "link_turning":
          [12,12,12,12,12], "roundabout": [12,12,12,12,12,12,12,12], "driveway": 12, "alley": 12,
          "parking_aisle": 12, "drive-through": 12
        },
        "rural": {
          "way": [12,12,12,12,12,12,12,12], "link_exiting": [12,12,12,12,12], "link_turning":
          [12,12,12,12,12], "roundabout": [12,12,12,12,12,12,12,12], "driveway": 12, "alley": 12,
          "parking_aisle": 12, "drive-through": 12
        }
      },
      {
        "urban": {
          "way": [13,13,13,13,13,13,13,13], "link_exiting": [13,13,13,13,13], "link_turning":
          [13,13,13,13,13], "roundabout": [13,13,13,13,13,13,13,13], "driveway": 13, "alley": 13,
          "parking_aisle": 13, "drive-through": 13
        },
        "suburban": {
          "way": [13,13,13,13,13,13,13,13], "link_exiting": [13,13,13,13,13], "link_turning":
          [13,13,13,13,13], "roundabout": [13,13,13,13,13,13,13,13], "driveway": 13, "alley": 13,
          "parking_aisle": 13, "drive-through": 13
        },
        "rural": {
          "way": [13,13,13,13,13,13,13,13], "link_exiting": [13,13,13,13,13], "link_turning":
          [13,13,13,13,13], "roundabout": [13,13,13,13,13,13,13,13], "driveway": 13, "alley": 13,
          "parking_aisle": 13, "drive-through": 13
        }
      }]
    )";
    speed_config.close();
    testable_assigner assigner("test/data/speed_config.json");
    ASSERT_TRUE(assigner.UpdateSpeed(edge, 6, true, "foo", "bar"));
    ASSERT_EQ(edge.speed(), 13);
    ASSERT_TRUE(assigner.UpdateSpeed(edge, 6, true, "US", "bar"));
    ASSERT_EQ(edge.speed(), 12);
    ASSERT_TRUE(assigner.UpdateSpeed(edge, 6, true, "PA", "PA"));
    ASSERT_EQ(edge.speed(), 13);
    ASSERT_TRUE(assigner.UpdateSpeed(edge, 6, true, "US", "PA"));
    ASSERT_EQ(edge.speed(), 11);
  }
}

TEST(Standalone, Malformed) {
  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"(foo)";
    speed_config.close();
    testable_assigner assigner("test/data/speed_config.json");
    ASSERT_TRUE(assigner.tables.empty());
  }
  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"({})";
    speed_config.close();
    testable_assigner assigner("test/data/speed_config.json");
    ASSERT_TRUE(assigner.tables.empty());
  }
  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"(
      [{
        "iso3166-1": ""
      }]
    )";
    speed_config.close();
    testable_assigner assigner("test/data/speed_config.json");
    ASSERT_TRUE(assigner.tables.empty());
  }
  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"(
      [{
        "iso3166-1": "US", "iso3166-2": ""
      }]
    )";
    speed_config.close();
    testable_assigner assigner("test/data/speed_config.json");
    ASSERT_TRUE(assigner.tables.empty());
  }
  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"(
      [{
        "rural": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11, "alley": 11,
          "parking_aisle": 11, "drive-through": 11
        }
      }]
    )";
    speed_config.close();
    testable_assigner assigner("test/data/speed_config.json");
    ASSERT_TRUE(assigner.tables.empty());
  }
  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"(
      [{
        "rural": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11, "alley": 11,
          "parking_aisle": 11, "drive-through": 11
        },
        "suburban": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11, "alley": 11,
          "parking_aisle": 11, "drive-through": 11
        }
      }]
    )";
    speed_config.close();
    testable_assigner assigner("test/data/speed_config.json");
    ASSERT_TRUE(assigner.tables.empty());
  }
  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"([{"urban": {"way": [11]}}])";
    speed_config.close();
    testable_assigner assigner("test/data/speed_config.json");
    ASSERT_TRUE(assigner.tables.empty());
  }
  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"(
      [{
        "rural": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11]
        }
      }]
    )";
    speed_config.close();
    testable_assigner assigner("test/data/speed_config.json");
    ASSERT_TRUE(assigner.tables.empty());
  }
  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"(
      [{
        "rural": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11
        }
      }]
    )";
    speed_config.close();
    testable_assigner assigner("test/data/speed_config.json");
    ASSERT_TRUE(assigner.tables.empty());
  }
  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"(
      [{
        "rural": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11, "alley": 11
        }
      }]
    )";
    speed_config.close();
    testable_assigner assigner("test/data/speed_config.json");
    ASSERT_TRUE(assigner.tables.empty());
  }
  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"(
      [{
        "rural": {
          "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
          [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11, "alley": 11,
          "parking_aisle": 11
        }
      }]
    )";
    speed_config.close();
    testable_assigner assigner("test/data/speed_config.json");
    ASSERT_TRUE(assigner.tables.empty());
  }
}

TEST(Standalone, HandleNulls) {
  DirectedEdge edge{};
  edge.set_all_forward_access();

  std::ofstream speed_config("test/data/speed_config.json");
  speed_config << R"(
    [{
      "urban": {
        "way": [null,null,null,null,null,null,null,null], "link_exiting":
        [null,null,null,null,null], "link_turning": [null,null,null,null,null], "roundabout":
        [null,null,null,null,null,null,null,null], "driveway": null, "alley": null,
        "parking_aisle": null, "drive-through": null
      },
      "suburban": {
        "way": [null,null,null,null,null,null,null,null], "link_exiting":
        [null,null,null,null,null], "link_turning": [null,null,null,null,null], "roundabout":
        [null,null,null,null,null,null,null,null], "driveway": null, "alley": null,
        "parking_aisle": null, "drive-through": null
      },
      "rural": {
        "way": [11,11,11,11,11,11,11,11], "link_exiting": [11,11,11,11,11], "link_turning":
        [11,11,11,11,11], "roundabout": [11,11,11,11,11,11,11,11], "driveway": 11, "alley": 11,
        "parking_aisle": 11, "drive-through": 11
      }
    }]
  )";
  speed_config.close();
  testable_assigner assigner("test/data/speed_config.json");
  ASSERT_FALSE(assigner.tables.empty());
  ASSERT_FALSE(assigner.UpdateSpeed(edge, 6, true, "foo", "bar"));
  ASSERT_TRUE(assigner.UpdateSpeed(edge, 1, true, "foo", "bar"));
  ASSERT_EQ(edge.speed(), 11);
}

TEST(Standalone, LegalSpeedsDensity) {
  const std::string urban = R"(
    a  j  i

    b K-L h
    x | | y
    c M-N g

    d  e  f
  )";

  auto layout = gurka::detail::map_to_coordinates(urban, 190, {5.121517, 52.092902});

  const std::string rural = R"(
    0--1--2--3--4
  )";

  const auto rural_layout = gurka::detail::map_to_coordinates(rural, 100, {5.305367, 51.692884});
  layout.insert(rural_layout.begin(), rural_layout.end());

  gurka::ways ways = {
      {"KL", {{"highway", "tertiary"}}},
      {"KM", {{"highway", "tertiary"}, {"maxspeed", "12"}}},
      {"LN", {{"highway", "tertiary"}, {"maxspeed:hgv", "12"}}},
      {"MN", {{"highway", "tertiary"}, {"maxspeed", "11"}, {"maxspeed:hgv", "10"}}},
      {"xy", {{"route", "ferry"}, {"maxspeed", "1"}, {"motor_vehicle", "yes"}}},
      {"01", {{"highway", "tertiary"}}},
      {"12", {{"highway", "tertiary"}, {"maxspeed", "12"}}},
      {"23", {{"highway", "tertiary"}, {"maxspeed:hgv", "10"}}},
      {"34", {{"highway", "tertiary"}, {"maxspeed", "11"}, {"maxspeed:hgv", "13"}}},

  };

  // we need to add a bunch of edges to pump up the road density
  for (char from = 'a'; from <= 'j'; ++from) {
    for (char to = from + 1; to <= 'j'; ++to) {
      for (char too = to + 1; too <= 'j'; ++too) {
        std::string nodes = "";
        nodes.push_back(from);
        nodes.push_back(to);
        nodes.push_back(too);
        ways.emplace(nodes, std::map<std::string, std::string>{
                                {"maxspeed", "99"},
                                {"highway", "residential"},
                            });
      }
    }
  }

  // build a small legal speed JSON file
  std::ofstream speed_config("test/data/legal_speeds_density.json");
  speed_config << R"(
    { "speedLimitsByCountryCode": {
      "NL": [
      {
        "name": "urban",
        "tags": {
          "maxspeed": "30",
          "maxspeed:hgv": "19"
        }
      },
      {
        "name": "rural",
        "tags": {
          "maxspeed": "21",
          "maxspeed:hgv": "18"
        }
      }
    ]
  }}
  )";
  speed_config.close();

  gurka::map map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/legalspeeddensity",
                        {{"mjolnir.admin",
                          {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}},
                         {"mjolnir.use_legal_speed_as_edge_speed", "true"},
                         {"mjolnir.legal_speeds_config", "test/data/legal_speeds_density.json"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  std::vector<expected_edge_speed> expected_speeds;
  expected_speeds.emplace_back("K", "L", 30, 19);
  expected_speeds.emplace_back("K", "M", 12, 19);
  expected_speeds.emplace_back("L", "N", 30, 12);
  expected_speeds.emplace_back("M", "N", 11, 10);
  expected_speeds.emplace_back("0", "1", 21, 18);
  expected_speeds.emplace_back("1", "2", 12, 18);
  expected_speeds.emplace_back("2", "3", 21, 10);
  expected_speeds.emplace_back("3", "4", 11, 13);

  for (const auto& speeds : expected_speeds) {
    auto found = gurka::findEdgeByNodes(reader, layout, speeds.start_node, speeds.end_node);
    auto edge = std::get<1>(found);
    EXPECT_EQ(edge->speed(), speeds.expected);

    if (speeds.truck_expected) {
      EXPECT_EQ(edge->truck_speed(), speeds.truck_expected);
    }
  }
}

TEST(Standalone, LegalSpeedsRoadClass) {

  constexpr double gridsize = 500;

  const std::string ascii_map = R"(
      A--B--C--D
      |  |  |  |
      E--F--G--H
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}}},
      {"BC", {{"highway", "service"}}},
      {"CD", {{"highway", "trunk"}}},
      {"DH", {{"highway", "living_street"}}},
      {"EF", {{"highway", "motorway"}, {"maxspeed", "99"}}},
      {"AE", {{"highway", "motorway"}, {"maxspeed", "99"}, {"maxspeed:hgv", "96"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.167640, 52.088865});

  // build a small legal speed JSON file
  std::ofstream speed_config("test/data/legal_speeds.json");
  speed_config << R"(
    { "speedLimitsByCountryCode": {
      "NL": [
      {
        "name": "urban",
        "tags": {
          "maxspeed": "30",
          "maxspeed:hgv": "20"
        }
      },
      {
        "name": "trunk",
        "tags": {
          "maxspeed": "21",
          "maxspeed:hgv": "18 mph"
        }
      },
      {
        "name": "service road",
        "tags": {
          "maxspeed": "27",
          "maxspeed:hgv": "17"
        }
      },
      {
        "tags": {
          "maxspeed": "90",
          "maxspeed:hgv": "80"
        }
      },
      {
        "name": "rural",
        "tags": {
          "maxspeed": "92",
          "maxspeed:hgv": "84"
        }
      },
      {
        "name": "living street",
        "tags": {
          "maxspeed": "walk"
        }
      },
      {
        "name": "motorway",
        "tags": {
          "maxspeed": "117",
          "maxspeed:hgv": "58"
        }
      }
    ]
  }}
  )";
  speed_config.close();

  gurka::map map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/legalspeed",
                        {{"mjolnir.admin",
                          {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}},
                         {"mjolnir.use_legal_speed_as_edge_speed", "true"},
                         {"mjolnir.legal_speeds_config", "test/data/legal_speeds.json"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  std::vector<expected_edge_speed> expected_speeds;

  expected_speeds.emplace_back("A", "B", 117, 58);
  expected_speeds.emplace_back("B", "C", 27, 17);
  expected_speeds.emplace_back("C", "D", 21, static_cast<uint32_t>(std::round(18 * kMPHtoKPH)));
  expected_speeds.emplace_back("D", "H", 10, 0);
  expected_speeds.emplace_back("E", "F", 99, 58);
  expected_speeds.emplace_back("A", "E", 99, 96);

  for (const auto& speeds : expected_speeds) {
    auto found = gurka::findEdgeByNodes(reader, layout, speeds.start_node, speeds.end_node);
    auto edge = std::get<1>(found);
    EXPECT_EQ(edge->speed(), speeds.expected);
    auto sl = reader.edgeinfo(std::get<0>(found)).speed_limit();
    EXPECT_EQ(speeds.expected, sl);
    if (speeds.truck_expected) {
      EXPECT_EQ(edge->truck_speed(), speeds.truck_expected);
    }
  }
}

TEST(Standalone, SpeedLimitOnly) {

  constexpr double gridsize = 500;

  const std::string ascii_map = R"(
      A--B--C--D
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}}},
      {"BC", {{"highway", "service"}}},
      {"CD", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.167640, 52.088865});

  // build a small legal speed JSON file
  std::ofstream speed_config("test/data/legal_speeds.json");
  speed_config << R"(
    { "speedLimitsByCountryCode": {
      "NL": [
      {
        "name": "urban",
        "tags": {
          "maxspeed": "11"
        }
      },
      {
        "name": "service road",
        "tags": {
          "maxspeed": "12"
        }
      },
      {
        "name": "rural",
        "tags": {
          "maxspeed": "13"
        }
      },
      {
        "name": "motorway",
        "tags": {
          "maxspeed": "14"
        }
      }
    ]
  }}
  )";
  speed_config.close();

  gurka::map map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/legalspeed",
                        {{"mjolnir.admin",
                          {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}},
                         {"mjolnir.use_legal_speed_as_edge_speed", "false"},
                         {"mjolnir.legal_speeds_config", "test/data/legal_speeds.json"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  auto AB = gurka::findEdgeByNodes(reader, layout, "A", "B");
  auto AB_edge = std::get<1>(AB);
  EXPECT_NE(AB_edge->speed(), 14);
  auto AB_sl = reader.edgeinfo(std::get<0>(AB)).speed_limit();
  EXPECT_EQ(AB_sl, 14);
}

TEST(Standalone, TruckSpeedFromAuto) {
  constexpr double gridsize = 500;

  const std::string ascii_map = R"(
      A--B--C--D--E--F
    )";
  // clang-format off
  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "service"}}},
      {"CD", {{"highway", "motorway"}}},
      {"DE", {{"highway", "living_street"}}},
      {"EF", {{"highway", "trunk"}}},
  };
  // clang-format on

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.167640, 52.088865});

  std::ofstream speed_config("test/data/legal_speeds_no_truck.json");
  speed_config << R"(
    { "speedLimitsByCountryCode": {
      "NL": [
      {
        "name": "rural",
        "tags": {
          "maxspeed": "11"
        }
      },
      {
        "name": "service road",
        "tags": {
          "maxspeed": "12"
        }
      },
      {
        "name": "motorway",
        "tags": {
          "maxspeed": "13"
        }
      },
      {
        "name": "living street",
        "tags": {
          "maxspeed": "14"
        }
      },
      {
        "name": "trunk",
        "tags": {
          "maxspeed": "15"
        }
      }
    ]
  }}
  )";
  speed_config.close();

  gurka::map map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/legalspeed_no_truck",
                        {{"mjolnir.admin",
                          {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}},
                         {"mjolnir.use_legal_speed_as_edge_speed", "true"},
                         {"mjolnir.legal_speeds_config", "test/data/legal_speeds_no_truck.json"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  std::vector<expected_edge_speed> expected_speeds;

  expected_speeds.emplace_back("A", "B", 11, 11);
  expected_speeds.emplace_back("B", "C", 12, 12);
  expected_speeds.emplace_back("C", "D", 13, 13);
  expected_speeds.emplace_back("D", "E", 14, 14);
  expected_speeds.emplace_back("E", "F", 15, 15);

  for (const auto& speeds : expected_speeds) {
    auto found = gurka::findEdgeByNodes(reader, layout, speeds.start_node, speeds.end_node);
    auto edge = std::get<1>(found);
    EXPECT_EQ(edge->speed(), speeds.expected);
    EXPECT_EQ(edge->truck_speed(), speeds.truck_expected);
  }
}

TEST(Standalone, MalformedJson) {
  constexpr double gridsize = 500;

  const std::string ascii_map = R"(
      A--B
    )";
  // clang-format off
  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}}},
  };
  // clang-format on

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.167640, 52.088865});

  std::ofstream speed_config("test/data/legal_speeds_malformed.json");
  speed_config << R"(
    { "foo":  }
  )";
  speed_config.close();

  gurka::map map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/legalspeed_no_truck",
                        {{"mjolnir.admin",
                          {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}},
                         {"mjolnir.use_legal_speed_as_edge_speed", "true"},
                         {"mjolnir.legal_speeds_config", "test/data/legal_speeds_malformed.json"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  std::vector<expected_edge_speed> expected_speeds;

  expected_speeds.emplace_back("A", "B", 105, 0);

  for (const auto& speeds : expected_speeds) {
    auto found = gurka::findEdgeByNodes(reader, layout, speeds.start_node, speeds.end_node);
    auto edge = std::get<1>(found);
    EXPECT_EQ(edge->speed(), speeds.expected);
    EXPECT_EQ(edge->truck_speed(), speeds.truck_expected);
  }
}

TEST(Standalone, InvalidJson) {
  constexpr double gridsize = 500;

  const std::string ascii_map = R"(
      A--B
    )";
  // clang-format off
  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}}},
  };
  // clang-format on

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.167640, 52.088865});

  std::ofstream speed_config("test/data/legal_speeds_invalid.json");
  speed_config << R"(
    { "unsupportedKey": {
      "NL": [
        { "name": "rural", "tags": {"maxspeed": "30"}}
      ]
    }  }
  )";
  speed_config.close();

  gurka::map map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/legalspeed_no_truck",
                        {{"mjolnir.admin",
                          {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}},
                         {"mjolnir.use_legal_speed_as_edge_speed", "true"},
                         {"mjolnir.legal_speeds_config", "test/data/legal_speeds_invalid.json"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  std::vector<expected_edge_speed> expected_speeds;

  expected_speeds.emplace_back("A", "B", 105, 0);

  for (const auto& speeds : expected_speeds) {
    auto found = gurka::findEdgeByNodes(reader, layout, speeds.start_node, speeds.end_node);
    auto edge = std::get<1>(found);
    EXPECT_EQ(edge->speed(), speeds.expected);
    EXPECT_EQ(edge->truck_speed(), speeds.truck_expected);
  }
}

TEST(Standalone, Fallback) {
  constexpr double gridsize = 500;

  const std::string ascii_map = R"(
      A--B
    )";
  // clang-format off
  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}}},
  };
  // clang-format on

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.167640, 52.088865});

  std::ofstream speed_config("test/data/legal_speeds_invalid.json");
  speed_config << R"(
    { "speedLimitsByCountryCode": {
      "NL": [
        { "tags": {"maxspeed": "30", "maxspeed:hgv": "29"}}
      ]
    }  }
  )";
  speed_config.close();

  gurka::map map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/legalspeed_no_truck",
                        {{"mjolnir.admin",
                          {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}},
                         {"mjolnir.use_legal_speed_as_edge_speed", "true"},
                         {"mjolnir.legal_speeds_config", "test/data/legal_speeds_invalid.json"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  std::vector<expected_edge_speed> expected_speeds;

  expected_speeds.emplace_back("A", "B", 30, 29);

  for (const auto& speeds : expected_speeds) {
    auto found = gurka::findEdgeByNodes(reader, layout, speeds.start_node, speeds.end_node);
    auto edge = std::get<1>(found);
    EXPECT_EQ(edge->speed(), speeds.expected);
    EXPECT_EQ(edge->truck_speed(), speeds.truck_expected);
  }
}

TEST(Standalone, SpeedMap) {

  std::ofstream speed_config("test/data/legal_speeds_unit.json");
  speed_config << R"(
    { "speedLimitsByCountryCode": {
      "NL": [
      {
        "tags": {
          "maxspeed": "31",
          "maxspeed:hgv": "29"
        }
      },
      {
        "name": "rural",
        "tags": {
          "maxspeed": "11"
        }
      },
      {
        "name": "service road",
        "tags": {
          "maxspeed": "12",
          "maxspeed:hgv": "11"
        }
      },
      {
        "name": "motorway",
        "tags": {
          "maxspeed": "13",
          "maxspeed:hgv": "12"
        }
      },
      {
        "name": "living street",
        "tags": {
          "maxspeed": "14",
          "maxspeed:hgv": "13"
        }
      },
      {
        "name": "trunk",
        "tags": {
          "maxspeed": "15",
          "maxspeed:hgv": "14"
        }
      },
      {
        "name": "urban",
        "tags": {
          "maxspeed": "16",
          "maxspeed:hgv": "15"
        }
      }
    ]
  }}
  )";
  speed_config.close();
  testable_legal_speed_assigner assigner("test/data/legal_speeds_unit.json", true);
  auto found = assigner.legal_speeds_map_.find("NL");
  ASSERT_NE(found, assigner.legal_speeds_map_.end());
  auto map = found->second;

  EXPECT_EQ(map.fallback.auto_, 31);
  EXPECT_EQ(map.fallback.truck_, 29);
  EXPECT_EQ(map.rural.auto_, 11);
  EXPECT_EQ(map.rural.truck_, 0);
  EXPECT_EQ(map.service.auto_, 12);
  EXPECT_EQ(map.service.truck_, 11);
  EXPECT_EQ(map.motorway.auto_, 13);
  EXPECT_EQ(map.motorway.truck_, 12);
  EXPECT_EQ(map.living_street.auto_, 14);
  EXPECT_EQ(map.living_street.truck_, 13);
  EXPECT_EQ(map.trunk.auto_, 15);
  EXPECT_EQ(map.trunk.truck_, 14);
  EXPECT_EQ(map.urban.auto_, 16);
  EXPECT_EQ(map.urban.truck_, 15);
}