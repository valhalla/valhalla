#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, DefaultSpeedConfig) {
  const std::string urban = R"(
    ABCDE
    FGHIJ
    KLMNO
    PQRST
    UVWXY
    Z
  )";

  auto layout = gurka::detail::map_to_coordinates(urban, 190);

  const std::string rural = R"(
            0tuvwxyz
           9 1
          8   2
          7   3
           6 4
            5
  )";

  const auto rural_layout = gurka::detail::map_to_coordinates(rural, 10, {7, 58});
  layout.insert(rural_layout.begin(), rural_layout.end());

  std::pair<std::string, std::string> oneway{"oneway", "yes"};
  std::pair<std::string, std::string> dest{"destination", "north pole"};

  const gurka::ways ways = {
      // way
      {"AK", {{"maxspeed", "2"}, {"highway", "motorway"}}},
      {"AL", {{"maxspeed", "3"}, {"highway", "trunk"}}},
      {"AM", {{"maxspeed", "4"}, {"highway", "primary"}}},
      {"AN", {{"maxspeed", "5"}, {"highway", "secondary"}}},
      {"AO", {{"maxspeed", "6"}, {"highway", "tertiary"}}},
      {"AP", {{"maxspeed", "7"}, {"highway", "unclassified"}}},
      {"AQ", {{"maxspeed", "8"}, {"highway", "residential"}}},
      {"AR", {{"maxspeed", "9"}, {"highway", "service"}}},
      // link exiting
      {"AS", {{"maxspeed", "10"}, oneway, dest, {"highway", "motorway_link"}}},
      {"AT", {{"maxspeed", "11"}, oneway, dest, {"highway", "trunk_link"}}},
      {"AU", {{"maxspeed", "12"}, oneway, dest, {"highway", "primary_link"}}},
      {"AV", {{"maxspeed", "13"}, oneway, dest, {"highway", "secondary_link"}}},
      {"AW", {{"maxspeed", "14"}, oneway, dest, {"highway", "tertiary_link"}}},
      {"AX", {{"maxspeed", "15"}, oneway, dest, {"highway", "residential_link"}}},
      // link turning
      {"AB", {{"maxspeed", "16"}, oneway, {"highway", "motorway_link"}}},
      {"BC", {{"maxspeed", "17"}, oneway, {"highway", "trunk_link"}}},
      {"CD", {{"maxspeed", "18"}, oneway, {"highway", "primary_link"}}},
      {"DE", {{"maxspeed", "19"}, oneway, {"highway", "secondary_link"}}},
      {"FG", {{"maxspeed", "20"}, oneway, {"highway", "tertiary_link"}}},
      {"GH", {{"maxspeed", "21"}, oneway, {"highway", "residential_link"}}},
      // roundabout
      {"BO", {{"maxspeed", "22"}, {"junction", "roundabout"}, {"highway", "motorway"}}},
      {"BP", {{"maxspeed", "23"}, {"junction", "roundabout"}, {"highway", "trunk"}}},
      {"BQ", {{"maxspeed", "24"}, {"junction", "roundabout"}, {"highway", "primary"}}},
      {"BR", {{"maxspeed", "25"}, {"junction", "roundabout"}, {"highway", "secondary"}}},
      {"BS", {{"maxspeed", "26"}, {"junction", "roundabout"}, {"highway", "tertiary"}}},
      {"BT", {{"maxspeed", "27"}, {"junction", "roundabout"}, {"highway", "unclassified"}}},
      {"BU", {{"maxspeed", "28"}, {"junction", "roundabout"}, {"highway", "residential"}}},
      {"BV", {{"maxspeed", "29"}, {"junction", "roundabout"}, {"highway", "service"}}},
      // service
      {"BW", {{"maxspeed", "30"}, {"highway", "service"}, {"service", "driveway"}}},
      {"BX", {{"maxspeed", "31"}, {"highway", "service"}, {"service", "alley"}}},
      {"BY", {{"maxspeed", "32"}, {"highway", "service"}, {"service", "parking_aisle"}}},
      {"BZ", {{"maxspeed", "33"}, {"highway", "service"}, {"service", "drive-through"}}},
      // ferry stuff is untouched
      {"CK", {{"maxspeed", "1"}, {"route", "ferry"}, {"motor_vehicle", "yes"}}},
      {"CL", {{"maxspeed", "1"}, {"route", "shuttle_train"}, {"motor_vehicle", "yes"}}},

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
      {"09", {{"maxspeed", "42"}, oneway, dest, {"highway", "motorway_link"}}},
      {"12", {{"maxspeed", "43"}, oneway, dest, {"highway", "trunk_link"}}},
      {"13", {{"maxspeed", "44"}, oneway, dest, {"highway", "primary_link"}}},
      {"14", {{"maxspeed", "45"}, oneway, dest, {"highway", "secondary_link"}}},
      {"15", {{"maxspeed", "46"}, oneway, dest, {"highway", "tertiary_link"}}},
      {"16", {{"maxspeed", "47"}, oneway, dest, {"highway", "residential_link"}}},
      // link turning
      {"tu", {{"maxspeed", "48"}, oneway, {"highway", "motorway_link"}}},
      {"uv", {{"maxspeed", "49"}, oneway, {"highway", "trunk_link"}}},
      {"vw", {{"maxspeed", "50"}, oneway, {"highway", "primary_link"}}},
      {"wx", {{"maxspeed", "51"}, oneway, {"highway", "secondary_link"}}},
      {"xy", {{"maxspeed", "52"}, oneway, {"highway", "tertiary_link"}}},
      {"yz", {{"maxspeed", "53"}, oneway, {"highway", "residential_link"}}},
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
      {"36", {{"maxspeed", "1"}, {"route", "ferry"}, {"motor_vehicle", "yes"}}},
      {"37", {{"maxspeed", "1"}, {"route", "shuttle_train"}, {"motor_vehicle", "yes"}}},
  };

  if (!filesystem::create_directories("test/data"))
    throw std::runtime_error("couldn't create directories");
  filesystem::remove("test/data/speed_config.json");

  {
    std::ofstream speed_config("test/data/speed_config.json");
    speed_config << R"(
      [{
        "iso3166-1": "",
        "iso3166-2": "",
        "urban": {
          "way": [1,2,3,4,5,6,7,8],
          "link_exiting": [9,10,11,12,13,14],
          "link_turning": [15,16,17,18,19,20],
          "roundabout": [21,22,23,24,25,26,27,28],
          "driveway": 29,
          "alley": 30,
          "parking_aisle": 31,
          "drive-through": 32
        },
        "rural": {
          "way": [33,34,35,36,37,38,39,40],
          "link_exiting": [41,42,43,44,45,46],
          "link_turning": [47,48,49,50,51,52],
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
                               {{"mjolnir.default_speeds", "test/data/speed_config.json"}});

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
      auto info = tile->edgeinfo(edge.edgeinfo_offset());
      auto name = info.GetNames().front();
      int diff = info.speed_limit() - edge.speed();
      if (edge.use() == baldr::Use::kFerry || edge.use() == baldr::Use::kRailFerry)
        ASSERT_LT(diff, 0)
            << name << " should have had a negative difference in speed limit and default speed";
      else
        ASSERT_EQ(diff, 1)
            << name << " should have had a default speed that was one less than the speed limit";
    }
  }
}
