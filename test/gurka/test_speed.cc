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

TEST(Standalone, LegalSpeeds) {

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
      {"BF", {{"highway", "motorway"}, {"maxspeed", "17 mph"}}},
      {"CG", {{"highway", "living_street"}, {"maxspeed", "walk"}}},
      {"FG", {{"highway", "motorway"}}},
      {"GH", {{"highway", "motorway"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.167640, 52.088865});

  // build a small legal speed JSON file
  std::ofstream speed_config("test/data/legal_speeds.json");
  speed_config << R"(
    {
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
          "maxspeed:hgv": "18"
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
          "maxspeed": "17",
          "maxspeed:hgv": "15"
        }
      },
      {
        "name": "motorway",
        "tags": {
          "maxspeed": "117",
          "maxspeed:hgv": "58"
        }}]}
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
  expected_speeds.emplace_back("C", "D", 21, 18);
  expected_speeds.emplace_back("D", "H", 17, 15);

  for (const auto& speeds : expected_speeds) {
    auto found = gurka::findEdgeByNodes(reader, layout, speeds.start_node, speeds.end_node);
    auto edge = std::get<1>(found);
    EXPECT_EQ(edge->speed(), speeds.expected);

    if (speeds.truck_expected) {
      EXPECT_EQ(edge->truck_speed(), speeds.truck_expected);
    }
  }
}