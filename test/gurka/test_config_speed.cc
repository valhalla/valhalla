#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, DefaultSpeedConfig) {
  const std::string urban = R"(
    a  j  i
     ABCDE
     FGHIJ
    bKLMNOh
     PQRST
    cUVWXYg
     Z
    d  e  f
  )";

  auto layout = gurka::detail::map_to_coordinates(urban, 190);

  const std::string rural = R"(
            0 klmnopqrstuvwxyz
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
      {"YZ", {{"maxspeed", "15"}, oneway, dest, {"highway", "residential_link"}}},
      // link turning
      {"AB", {{"maxspeed", "16"}, oneway, {"highway", "motorway_link"}}},
      {"CD", {{"maxspeed", "17"}, oneway, {"highway", "trunk_link"}}},
      {"AG", {{"maxspeed", "18"}, oneway, {"highway", "primary_link"}}},
      {"HI", {{"maxspeed", "19"}, oneway, {"highway", "secondary_link"}}},
      {"KL", {{"maxspeed", "20"}, oneway, {"highway", "tertiary_link"}}},
      {"MN", {{"maxspeed", "21"}, oneway, {"highway", "residential_link"}}},
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
      {"wx", {{"maxspeed", "42"}, oneway, dest, {"highway", "motorway_link"}}},
      {"yz", {{"maxspeed", "43"}, oneway, dest, {"highway", "trunk_link"}}},
      {"13", {{"maxspeed", "44"}, oneway, dest, {"highway", "primary_link"}}},
      {"14", {{"maxspeed", "45"}, oneway, dest, {"highway", "secondary_link"}}},
      {"15", {{"maxspeed", "46"}, oneway, dest, {"highway", "tertiary_link"}}},
      {"16", {{"maxspeed", "47"}, oneway, dest, {"highway", "residential_link"}}},
      // link turning
      {"kl", {{"maxspeed", "48"}, oneway, {"highway", "motorway_link"}}},
      {"mn", {{"maxspeed", "49"}, oneway, {"highway", "trunk_link"}}},
      {"op", {{"maxspeed", "50"}, oneway, {"highway", "primary_link"}}},
      {"qr", {{"maxspeed", "51"}, oneway, {"highway", "secondary_link"}}},
      {"st", {{"maxspeed", "52"}, oneway, {"highway", "tertiary_link"}}},
      {"uv", {{"maxspeed", "53"}, oneway, {"highway", "residential_link"}}},
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
          "way": [65,66,67,68,69,70,71,72],
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
