#include "gurka.h"
#include "src/mjolnir/legal_speed.h"
#include "src/mjolnir/speed_assigner.h"
#include <gtest/gtest.h>

#include <filesystem>

using namespace valhalla;

struct testable_legal_speed_assigner : public SimpleLegalSpeedAssigner {
public:
  testable_legal_speed_assigner(const std::string& file_path) : SimpleLegalSpeedAssigner(file_path) {
  }
  using SimpleLegalSpeedAssigner::legal_speeds_map_;
};

struct expected_edge_speed {
  std::string start_node;
  std::string end_node;
  uint32_t expected;
  uint32_t truck_expected;

  expected_edge_speed() = delete;
  expected_edge_speed(std::string sn, std::string en, uint32_t e, uint32_t t)
      : start_node(sn), end_node(en), expected(e), truck_expected(t) {
  }
};

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
                         {"mjolnir.legal_speeds_config", "test/data/legal_speeds.json"}});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  std::vector<expected_edge_speed> expected_speeds;

  expected_speeds.emplace_back("A", "B", 117, 0);
  expected_speeds.emplace_back("B", "C", 27, 17);
  expected_speeds.emplace_back("C", "D", 21, static_cast<uint32_t>(std::round(18 * kMPHtoKPH)));
  expected_speeds.emplace_back("D", "H", 10, 0);
  expected_speeds.emplace_back("E", "F", 99, 58);
  expected_speeds.emplace_back("A", "E", 99, 96);

  for (const auto& speeds : expected_speeds) {
    auto found = gurka::findEdgeByNodes(reader, layout, speeds.start_node, speeds.end_node);
    auto edge = std::get<1>(found);
    EXPECT_EQ(edge->speed(), speeds.expected);

    if (speeds.truck_expected) {
      EXPECT_EQ(edge->truck_speed(), speeds.truck_expected);
    }
  }
}