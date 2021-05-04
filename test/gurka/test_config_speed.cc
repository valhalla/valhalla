#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, DefaultSpeedConfig) {
  const std::string urban = R"(
    A-----B-----C-----D-----E-----F-----G-----H-----I-----J
    K-----L-----M-----N-----O-----P-----Q-----R-----S-----TUVWXYZ
    a-----b-----c-----d-----e-----f-----g-----h-----i-----j
    k-----l-----m-----n-----o-----p-----q-----r-----s-----t
  )";

  auto layout = gurka::detail::map_to_coordinates(urban, 100);

  const std::string rural = R"(
            0uvwxyz
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

  const gurka::ways ways = {
      // way
      {"AB", {{"maxspeed", "100"}, {"highway", "motorway"}}},
      {"BC", {{"maxspeed", "100"}, {"highway", "trunk"}}},
      {"CD", {{"maxspeed", "100"}, {"highway", "primary"}}},
      {"DE", {{"maxspeed", "100"}, {"highway", "secondary"}}},
      {"EF", {{"maxspeed", "100"}, {"highway", "tertiary"}}},
      {"FG", {{"maxspeed", "100"}, {"highway", "unclassified"}}},
      {"GH", {{"maxspeed", "100"}, {"highway", "residential"}}},
      {"HI", {{"maxspeed", "100"}, {"highway", "service"}}},
      // link exiting
      {"IJ", {{"maxspeed", "100"}, oneway, dest, {"highway", "motorway_link"}}},
      {"JK", {{"maxspeed", "100"}, oneway, dest, {"highway", "trunk_link"}}},
      {"KL", {{"maxspeed", "100"}, oneway, dest, {"highway", "primary_link"}}},
      {"LM", {{"maxspeed", "100"}, oneway, dest, {"highway", "secondary_link"}}},
      {"MN", {{"maxspeed", "100"}, oneway, dest, {"highway", "tertiary_link"}}},
      {"NO", {{"maxspeed", "100"}, oneway, dest, {"highway", "residential_link"}}},
      // link turning
      {"TU", {{"maxspeed", "100"}, oneway, {"highway", "motorway_link"}}},
      {"UV", {{"maxspeed", "100"}, oneway, {"highway", "trunk_link"}}},
      {"VW", {{"maxspeed", "100"}, oneway, {"highway", "primary_link"}}},
      {"WX", {{"maxspeed", "100"}, oneway, {"highway", "secondary_link"}}},
      {"XY", {{"maxspeed", "100"}, oneway, {"highway", "tertiary_link"}}},
      {"YZ", {{"maxspeed", "100"}, oneway, {"highway", "residential_link"}}},
      // service
      {"OP", {{"maxspeed", "100"}, {"highway", "service"}, {"service", "driveway"}}},
      {"PQ", {{"maxspeed", "100"}, {"highway", "service"}, {"service", "alley"}}},
      {"QR", {{"maxspeed", "100"}, {"highway", "service"}, {"service", "parking_aisle"}}},
      {"RS", {{"maxspeed", "100"}, {"highway", "service"}, {"service", "drive-through"}}},
      // ferry stuff is untouched
      {"RS", {{"maxspeed", "100"}, {"route", "ferry"}, {"motor_vehicle", "yes"}}},
      {"ST", {{"maxspeed", "100"}, {"route", "shuttle_train"}, {"motor_vehicle", "yes"}}},
      // roundabout
      {"ab", {{"maxspeed", "100"}, {"junction", "roundabout"}, {"highway", "motorway"}}},
      {"bc", {{"maxspeed", "100"}, {"junction", "roundabout"}, {"highway", "trunk"}}},
      {"cd", {{"maxspeed", "100"}, {"junction", "roundabout"}, {"highway", "primary"}}},
      {"de", {{"maxspeed", "100"}, {"junction", "roundabout"}, {"highway", "secondary"}}},
      {"ef", {{"maxspeed", "100"}, {"junction", "roundabout"}, {"highway", "tertiary"}}},
      {"fg", {{"maxspeed", "100"}, {"junction", "roundabout"}, {"highway", "unclassified"}}},
      {"gh", {{"maxspeed", "100"}, {"junction", "roundabout"}, {"highway", "residential"}}},
      {"hi", {{"maxspeed", "100"}, {"junction", "roundabout"}, {"highway", "service"}}},
      // some extra density for urban areas
      {"ijklmnopqrst", {{"highway", "residential"}}},

      // way
      {"01", {{"maxspeed", "102"}, {"highway", "motorway"}}},
      {"02", {{"maxspeed", "102"}, {"highway", "trunk"}}},
      {"03", {{"maxspeed", "102"}, {"highway", "primary"}}},
      {"04", {{"maxspeed", "102"}, {"highway", "secondary"}}},
      {"05", {{"maxspeed", "102"}, {"highway", "tertiary"}}},
      {"06", {{"maxspeed", "102"}, {"highway", "unclassified"}}},
      {"07", {{"maxspeed", "102"}, {"highway", "residential"}}},
      {"08", {{"maxspeed", "102"}, {"highway", "service"}}},
      // link exiting
      {"09", {{"maxspeed", "102"}, oneway, dest, {"highway", "motorway_link"}}},
      {"12", {{"maxspeed", "102"}, oneway, dest, {"highway", "trunk_link"}}},
      {"13", {{"maxspeed", "102"}, oneway, dest, {"highway", "primary_link"}}},
      {"14", {{"maxspeed", "102"}, oneway, dest, {"highway", "secondary_link"}}},
      {"15", {{"maxspeed", "102"}, oneway, dest, {"highway", "tertiary_link"}}},
      {"16", {{"maxspeed", "102"}, oneway, dest, {"highway", "residential_link"}}},
      // link turning
      {"tu", {{"maxspeed", "102"}, oneway, {"highway", "motorway_link"}}},
      {"uv", {{"maxspeed", "102"}, oneway, {"highway", "trunk_link"}}},
      {"vw", {{"maxspeed", "102"}, oneway, {"highway", "primary_link"}}},
      {"wx", {{"maxspeed", "102"}, oneway, {"highway", "secondary_link"}}},
      {"xy", {{"maxspeed", "102"}, oneway, {"highway", "tertiary_link"}}},
      {"yz", {{"maxspeed", "102"}, oneway, {"highway", "residential_link"}}},
      // service
      {"17", {{"maxspeed", "102"}, {"highway", "service"}, {"service", "driveway"}}},
      {"18", {{"maxspeed", "102"}, {"highway", "service"}, {"service", "alley"}}},
      {"19", {{"maxspeed", "102"}, {"highway", "service"}, {"service", "parking_aisle"}}},
      {"23", {{"maxspeed", "102"}, {"highway", "service"}, {"service", "drive-through"}}},
      // roundabouts
      {"24", {{"maxspeed", "102"}, {"junction", "roundabout"}, {"highway", "motorway"}}},
      {"25", {{"maxspeed", "102"}, {"junction", "roundabout"}, {"highway", "trunk"}}},
      {"26", {{"maxspeed", "102"}, {"junction", "roundabout"}, {"highway", "primary"}}},
      {"27", {{"maxspeed", "102"}, {"junction", "roundabout"}, {"highway", "secondary"}}},
      {"28", {{"maxspeed", "102"}, {"junction", "roundabout"}, {"highway", "tertiary"}}},
      {"29", {{"maxspeed", "102"}, {"junction", "roundabout"}, {"highway", "unclassified"}}},
      {"34", {{"maxspeed", "102"}, {"junction", "roundabout"}, {"highway", "residential"}}},
      {"35", {{"maxspeed", "102"}, {"junction", "roundabout"}, {"highway", "service"}}},
      // ferry stuff is untouched
      {"36", {{"maxspeed", "102"}, {"route", "ferry"}, {"motor_vehicle", "yes"}}},
      {"37", {{"maxspeed", "102"}, {"route", "shuttle_train"}, {"motor_vehicle", "yes"}}},
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
                               {{"mjolnir.speed_config", "test/data/speed_config.json"}});
}
