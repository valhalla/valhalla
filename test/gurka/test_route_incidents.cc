#include <gtest/gtest.h>

#include "gurka.h"
#include "test/util/traffic_utils.h"

using namespace valhalla;

class incidents : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
    A----------------------B-----------------------C
    |                      |
    |                      |
    |                      |
    |                      |
    |                      |
    |                      |
    |                      |
    D----------------------E-----------------------F
    |                      | \..
    |                      |    \..
    |                      |       \..
    |                      |          \..
    |                      |             \..
    |                      |                \..
    |                      |                   \..
    G----------------------H----------------------I)";

    // connect the ways via the nodes
    const gurka::ways ways = {{"AB", {{"highway", "tertiary"}}},
                              {"BC", {{"highway", "tertiary"}}},
                              {"DEF", {{"highway", "primary"}}},
                              {"GHI", {{"highway", "primary"}}},
                              {"ADG", {{"highway", "motorway"}}},
                              {"BE", {{"highway", "secondary"}}},
                              {"EI", {{"highway", "service"}}},
                              {"EH", {{"highway", "service"}}}};

    // generate the lls for the nodes in the map
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10, {.2f, .2f});

    // make the tiles
    std::string tile_dir = "test/data/route_incidents";
    map = gurka::buildtiles(layout, ways, {}, {}, tile_dir,
                                 {{"mjolnir.traffic_extract", tile_dir + "/traffic.tar"}});

    // stage up some live traffic data
    valhalla_tests::utils::build_live_traffic_data(map.config);
  }
};

// initialize static member
gurka::map incidents::map = {};

// provides us an easy way to mock having incident tiles, each test can override the tile in question
// a bit more work is needed if we want to do it for more than one tile at a time
struct test_reader : public baldr::GraphReader {
  using baldr::GraphReader::GraphReader;
  virtual std::shared_ptr<baldr::GraphReader::incident_tile_t> GetIncidentTile(const baldr::GraphId& tile_id) const {
    return incident_tile_;
  }
  std::shared_ptr<baldr::GraphReader::incident_tile_t> incident_tile_;
};

//via some macro magic we are inside the scope of the incidents class above
TEST_F(incidents, simple_cut) {
  // make our reader that we can manipulate
  map.config.put("mjolnir.simulate_incidents", true);
  test_reader reader(map.config.get_child("mjolnir"));
  std::shared_ptr<baldr::GraphReader> graphreader(&reader, [](baldr::GraphReader*){});

  // modify traffic speed info to say that this edge has an incident
  auto edge_id = std::get<0>(gurka::findEdge(reader, map.nodes, "BC", "B"));
  auto has_incident_cb = [edge_id](baldr::GraphReader& reader, baldr::TrafficTile& tile, int edge_index,
                               valhalla::baldr::TrafficSpeed* current) -> void {
    if (edge_id.Tile_Base() == tile.header->tile_id && edge_id.id() == edge_index)
      current->has_incidents = true;
  };
  valhalla_tests::utils::customize_live_traffic_data(map.config, has_incident_cb);

  // modify the incident tile to have incidents on this edge
  reader.incident_tile_.reset(new decltype(reader.incident_tile_)::element_type);
  reader.incident_tile_->emplace_back(baldr::GraphReader::Incident{edge_id.id(), .25, .75});

  // do the route
  auto result = gurka::route(map, {"C", "B"}, "auto", {}, graphreader);
  gurka::assert::osrm::expect_steps(result, {"BC"});
  gurka::assert::raw::expect_path(result, {"BC", "BC", "BC"});
}