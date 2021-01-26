#include "baldr/openlr.h"
#include "baldr/rapidjson_utils.h"
#include "gurka.h"
#include "test.h"

using namespace valhalla;

TEST(locate, basic_properties) {
  const std::string ascii_map = R"(
    A-1--B--2-C
    |    |    |
    3    4    5
    |    |    |
    D-6--E--7-F
    |    |    |
    8    9    0
    |    |    |
    G-a--H--b-I)";

  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}}},    {"BC", {{"highway", "primary"}}},
      {"AD", {{"highway", "residential"}}}, {"BE", {{"highway", "motorway_link"}}},
      {"CF", {{"highway", "pedestrian"}}},  {"DE", {{"highway", "trunk"}}},
      {"EF", {{"highway", "secondary"}}},   {"DG", {{"highway", "trunk_link"}}},
      {"EH", {{"highway", "cycleway"}}},    {"FI", {{"highway", "service"}}},
      {"GH", {{"highway", "tertiary"}}},    {"HI", {{"highway", "unclassified"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_locate_basic",
                        {
                            {"mjolnir.concurrency", "1"},
                            {"mjolnir.reclassify_links", "0"},
                            {"mjolnir.traffic_extract", "test/data/gurka_locate_basic/traffic.tar"},
                        });
  test::build_live_traffic_data(map.config);

  // turn on some traffic for fun
  auto traffic_cb = [](baldr::GraphReader& reader, baldr::TrafficTile& tile, int index,
                       valhalla::baldr::TrafficSpeed* current) -> void {
    current->overall_speed = 124 >> 1;
    current->speed1 = 126 >> 1;
    current->congestion1 = 0; // unknown
    current->breakpoint1 = 85;
    current->speed2 = 124 >> 1;
    current->congestion2 = 32; // middle
    current->breakpoint2 = 170;
    current->speed3 = 122 >> 1;
    current->congestion3 = 63; // high
  };
  test::customize_live_traffic_data(map.config, traffic_cb);

  // call locate to see some info about each edge
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  std::string json;
  auto result = gurka::do_action(valhalla::Options::locate, map,
                                 {"1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "a", "b"}, "none",
                                 {}, reader, &json);
  rapidjson::Document response;
  response.Parse(json);
  if (response.HasParseError())
    throw std::runtime_error("bad json response");

  // assert the information we know is correct
  ASSERT_EQ(response.GetArray().Size(), 12);
  std::vector<std::string> way_names = {
      "AB", "BC", "AD", "BE", "CF", "DE", "EF", "DG", "EH", "FI", "GH", "HI",
  };
  // check each locations resulting info
  for (size_t i = 0; i < 12; ++i) {
    // get some info
    const auto& way_name = way_names[i];
    auto classification = ways.find(way_name)->second.find("highway")->second;
    bool link = classification.find("_link") != std::string::npos;
    // this is stripped off into a bit on the edge
    if (link)
      classification = classification.substr(0, classification.size() - 5);
    // the lowest FRCs are all just service_other
    if (classification == "service" || classification == "pedestrian" || classification == "cycleway")
      classification = "service_other";

    // check both directions of the edge
    auto edges = rapidjson::Pointer("/" + std::to_string(i) + "/edges").Get(response)->GetArray();
    ASSERT_EQ(edges.Size(), 2);
    for (const auto& edge : edges) {
      ASSERT_EQ(rapidjson::Pointer("/edge_info/names/0").Get(edge)->GetString(), way_name);
      ASSERT_EQ(rapidjson::Pointer("/edge/classification/classification").Get(edge)->GetString(),
                classification);
      ASSERT_EQ(rapidjson::Pointer("/edge/classification/link").Get(edge)->GetBool(), link);

      // parse openlr
      ASSERT_NO_THROW(
          baldr::OpenLR::OpenLr(rapidjson::Pointer("/linear_reference").Get(edge)->GetString(),
                                true));

      // check out some traffic
      ASSERT_EQ(rapidjson::Pointer("/live_speed/speed_0").Get(edge)->GetInt(), 126);
      ASSERT_TRUE(rapidjson::Pointer("/live_speed/congestion_0").Get(edge)->IsNull());
      ASSERT_EQ(rapidjson::Pointer("/live_speed/breakpoint_0").Get(edge)->GetDouble(), 0.33);
      ASSERT_EQ(rapidjson::Pointer("/live_speed/speed_1").Get(edge)->GetInt(), 124);
      ASSERT_EQ(rapidjson::Pointer("/live_speed/congestion_1").Get(edge)->GetDouble(), .5);
      ASSERT_EQ(rapidjson::Pointer("/live_speed/breakpoint_1").Get(edge)->GetDouble(), 0.67);
      ASSERT_EQ(rapidjson::Pointer("/live_speed/speed_2").Get(edge)->GetInt(), 122);
      ASSERT_EQ(rapidjson::Pointer("/live_speed/congestion_2").Get(edge)->GetDouble(), 1.0);
    }
  }
}
