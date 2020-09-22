#include <gtest/gtest.h>

#include "gurka.h"
#include "test/util/traffic_utils.h"

using namespace valhalla;

class incidents : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
    A----------0----------B----------1----------C
    |                     |
    |                     |
    |                     |
    2                     3
    |                     |
    |                     |
    |                     |
    D----------4----------E----------5----------F
    |                     |
    |                     |
    |                     |
    6                     7
    |                     |
    |                     |
    |                     |
    G----------8----------H----------9----------I)";

    // connect the ways via the nodes
    const gurka::ways ways = {
        {"AB", {{"highway", "tertiary"}}},  {"BC", {{"highway", "tertiary"}}},
        {"DEF", {{"highway", "primary"}}},  {"GHI", {{"highway", "primary"}}},
        {"ADG", {{"highway", "motorway"}}}, {"BE", {{"highway", "secondary"}}},
        {"EH", {{"highway", "service"}}},
    };

    // generate the lls for the nodes in the map
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10, {.2f, .2f});

    // make the tiles
    std::string tile_dir = "test/data/route_incidents";
    map = gurka::buildtiles(layout, ways, {}, {}, tile_dir,
                            {
                                {"mjolnir.traffic_extract", tile_dir + "/traffic.tar"},
                                {"mjolnir.simulate_incidents", "true"},
                            });

    // stage up some live traffic data
    valhalla_tests::utils::build_live_traffic_data(map.config);
  }
};

// initialize static member
gurka::map incidents::map = {};

// for asserting where the incident starts and ends on the route and how long it is
struct incident_location {
  uint64_t id;
  uint32_t leg_index;
  uint32_t begin_edge_index;
  double begin_pct;
  uint32_t end_edge_index;
  double end_pct;
};

void check_incident_locations(const valhalla::Api& api,
                              const std::vector<incident_location>& locations) {

  // for every length we save the distance along the leg a given shape point is
  std::vector<std::vector<double>> leg_lengths;
  size_t incident_count = 0;
  for (const auto& leg : api.trip().routes(0).legs()) {
    // keep the distances between shape points for this leg
    auto shape = midgard::decode<std::vector<midgard::PointLL>>(leg.shape());
    std::vector<double> lengths;
    for (size_t i = 0; i < shape.size(); ++i) {
      lengths.push_back(i == 0 ? 0.0 : shape[i - 1].Distance(shape[i]) + lengths.back());
    }
    leg_lengths.emplace_back(std::move(lengths));

    // also check that incidents are in the right order
    uint32_t last_shape_index = 0;
    for (const auto& i : leg.incidents()) {
      ASSERT_GE(i.begin_shape_index(), last_shape_index) << " leg incidents were out of order";
      last_shape_index = i.begin_shape_index();
    }
    incident_count += leg.incidents_size();
  }

  // check all the incidents are in the right place in the route
  for (const auto& i : locations) {
    const auto& leg = api.trip().routes(0).legs(i.leg_index);
    // find the incident
    const valhalla::TripLeg::Incident* incident = nullptr;
    for (int j = 0; j < leg.incidents_size(); ++j) {
      if (leg.incidents(j).id() == i.id) {
        incident = &leg.incidents(j);
        break;
      }
    }
    ASSERT_TRUE(incident != nullptr) << "Couldn't find incident " << i.id << " on leg " << i.leg_index
                                     << " on edge " << i.begin_edge_index;

    // check that we are on the right edges of the path
    const auto& begin_edge = leg.node(i.begin_edge_index).edge();
    ASSERT_TRUE(begin_edge.begin_shape_index() <= incident->begin_shape_index() &&
                incident->begin_shape_index() <= begin_edge.end_shape_index())
        << "Incident " << i.id << " on leg " << i.leg_index << " on edge " << i.begin_edge_index
        << " begins on wrong edge";
    const auto& end_edge = leg.node(i.end_edge_index).edge();
    ASSERT_TRUE(end_edge.begin_shape_index() <= incident->end_shape_index() &&
                incident->end_shape_index() <= end_edge.end_shape_index())
        << "Incident " << i.id << " on leg " << i.leg_index << " on edge " << i.end_edge_index
        << " ends on wrong edge";

    // check that we are on the right shape points
    const auto& lengths = leg_lengths[i.leg_index];
    auto begin_edge_length =
        lengths[begin_edge.end_shape_index()] - lengths[begin_edge.begin_shape_index()];
    auto begin_edge_pct =
        (lengths[incident->begin_shape_index()] - lengths[begin_edge.begin_shape_index()]) /
        begin_edge_length;
    ASSERT_NEAR(begin_edge_pct, i.begin_pct, .01)
        << "Incident " << i.id << " on leg " << i.leg_index << " on edge " << i.begin_edge_index
        << " begins at wrong location along the edge";
    auto end_edge_length =
        lengths[end_edge.end_shape_index()] - lengths[end_edge.begin_shape_index()];
    auto end_edge_pct =
        (lengths[incident->end_shape_index()] - lengths[end_edge.begin_shape_index()]) /
        end_edge_length;
    ASSERT_NEAR(end_edge_pct, i.end_pct, .1)
        << "Incident " << i.id << " on leg " << i.leg_index << " on edge " << i.end_edge_index
        << " ends at wrong location along the edge";
  }

  ASSERT_EQ(incident_count, locations.size()) << "Expected number of incidents does not match actual";
}

std::shared_ptr<std::vector<int>> foo() {
  std::vector<int> bar;
  return std::shared_ptr<std::vector<int>>(&bar, [](auto*) {});
}

// provides us an easy way to mock having incident tiles, each test can override the tile in question
// a bit more work is needed if we want to do it for more than one tile at a time
struct test_reader : public baldr::GraphReader {
  test_reader(const boost::property_tree::ptree& pt) : baldr::GraphReader(pt) {
    tile_extract_.reset(new baldr::GraphReader::tile_extract_t(pt));
  }
  virtual std::shared_ptr<baldr::GraphReader::incident_tile_t>
  GetIncidentTile(const baldr::GraphId& tile_id) const {
    auto i = incidents.find(tile_id.Tile_Base());
    if (i == incidents.cend())
      return {};
    return std::shared_ptr<
        baldr::GraphReader::incident_tile_t>(&i->second, [](baldr::GraphReader::incident_tile_t*) {});
  }
  void add(const baldr::GraphId& id, baldr::GraphReader::Incident&& incident) {
    incidents[id.Tile_Base()].emplace_back(incident);
  }
  void sort() {
    for (auto& kv : incidents) {
      std::sort(kv.second.begin(), kv.second.end(),
                [](const GraphReader::Incident& a, const GraphReader::Incident& b) {
                  if (a.edge_index == b.edge_index) {
                    if (a.start_offset == b.start_offset) {
                      if (a.end_offset == b.end_offset) {
                        return a.id < b.id;
                      }
                      return a.end_offset < b.end_offset;
                    }
                    return a.start_offset < b.start_offset;
                  }
                  return a.edge_index < b.edge_index;
                });
    }
  }
  mutable std::unordered_map<baldr::GraphId, baldr::GraphReader::incident_tile_t> incidents;
};

// used to make a graphreader and mark some edges as having incidents
std::shared_ptr<test_reader> setup_test(const gurka::map& map,
                                        const std::vector<std::string>& names,
                                        std::vector<baldr::GraphId>& edge_ids) {
  // make our reader that we can manipulate
  auto reader = std::make_shared<test_reader>(map.config.get_child("mjolnir"));

  // modify traffic speed info to say that this edge has an incident
  for (const auto& name : names) {
    auto begin_node = name.substr(0, 1);
    auto end_node = name.substr(1);
    auto edge_id = std::get<0>(gurka::findEdgeByNodes(*reader, map.nodes, begin_node, end_node));
    auto has_incident_cb = [edge_id](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                     int edge_index, valhalla::baldr::TrafficSpeed* current) -> void {
      if (edge_id.Tile_Base() == tile.header->tile_id && edge_id.id() == edge_index)
        current->has_incidents = true;
    };
    valhalla_tests::utils::customize_live_traffic_data(map.config, has_incident_cb);
    edge_ids.push_back(edge_id);
  }

  // prepare for incidents
  reader->incidents.clear();
  return reader;
}

// via some macro magic we are inside the scope of the incidents class above
TEST_F(incidents, simple_cut) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .25, .75, 1234});

  // do the route
  auto result = gurka::route(map, {"C", "B"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {1234, 0, 0, .25, 0, .75},
                                   });
}

TEST_F(incidents, whole_edge) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), 0., 1., 1234});

  // do the route
  auto result = gurka::route(map, {"C", "B"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {1234, 0, 0, 0., 0, 1.},
                                   });
}

TEST_F(incidents, left) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), 0., .5, 1234});

  // do the route
  auto result = gurka::route(map, {"C", "B"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {1234, 0, 0, 0., 0, .5},
                                   });
}

TEST_F(incidents, right) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .5, 1., 1234});

  // do the route
  auto result = gurka::route(map, {"C", "B"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {1234, 0, 0, .5, 0, 1.},
                                   });
}

TEST_F(incidents, multiedge) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB", "BE", "EH"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .5, 1., 1234});
  reader->add(edge_ids[1], baldr::GraphReader::Incident{edge_ids[1].id(), 0., 1., 1234});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), 0., .5, 1234});
  reader->sort();

  // do the route
  auto result = gurka::route(map, {"C", "H"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {1234, 0, 0, .5, 2, .5},
                                   });
}

TEST_F(incidents, multiincident) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB", "BE", "EH"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .25, .4, 123});
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .5, 1., 456});
  reader->add(edge_ids[1], baldr::GraphReader::Incident{edge_ids[1].id(), 0., 1., 456});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), 0., .5, 456});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), .75, .9, 789});
  reader->sort();

  // do the route
  auto result = gurka::route(map, {"C", "H"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, .25, 0, .4},
                                       {456, 0, 0, .5, 2, .5},
                                       {789, 0, 2, .75, 2, .9},
                                   });
}

TEST_F(incidents, interleaved) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB", "BE", "EH"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .25, .75, 123});
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .5, 1., 456});
  reader->add(edge_ids[1], baldr::GraphReader::Incident{edge_ids[1].id(), 0., 1., 456});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), 0., .5, 456});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), .25, .9, 789});
  reader->sort();

  // do the route
  auto result = gurka::route(map, {"C", "H"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, .25, 0, .75},
                                       {456, 0, 0, .5, 2, .5},
                                       {789, 0, 2, .25, 2, .9},
                                   });
}

TEST_F(incidents, collisions) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB", "BE", "EH"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .25, .5, 123});
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .5, 1., 456});
  reader->add(edge_ids[1], baldr::GraphReader::Incident{edge_ids[1].id(), 0., 1., 456});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), 0., .5, 456});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), .5, .9, 789});
  reader->sort();

  // do the route
  auto result = gurka::route(map, {"C", "H"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, .25, 0, .5},
                                       {456, 0, 0, .5, 2, .5},
                                       {789, 0, 2, .5, 2, .9},
                                   });
}

TEST_F(incidents, full_overlap) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB", "BE", "EH"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .5, 1., 123});
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .5, 1., 456});
  reader->sort();

  // do the route
  auto result = gurka::route(map, {"C", "H"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, .5, 0, 1.},
                                       {456, 0, 0, .5, 0, 1.},
                                   });
}

TEST_F(incidents, multileg) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB", "BE", "EH"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .25, .75, 123});
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .5, 1., 456});

  reader->add(edge_ids[1], baldr::GraphReader::Incident{edge_ids[1].id(), 0., 1., 456});
  reader->add(edge_ids[1], baldr::GraphReader::Incident{edge_ids[1].id(), 0., 1., 789});

  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), 0., .5, 456});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), 0., .9, 789});
  reader->sort();

  // do the route
  auto result = gurka::route(map, {"C", "B", "E", "H"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, .25, 0, .75},
                                       {456, 0, 0, .5, 0, 1.},
                                       {456, 1, 0, 0., 0, 1.},
                                       {789, 1, 0, 0., 0, 1.},
                                       {456, 2, 0, 0., 0, .5},
                                       {789, 2, 0, 0., 0, .9},
                                   });
}

TEST_F(incidents, clipped) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB", "BE", "EH", "HI"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .25, .75, 123});
  reader->add(edge_ids[3], baldr::GraphReader::Incident{edge_ids[3].id(), .25, .75, 456});
  reader->sort();

  // do the route
  auto result = gurka::route(map, {"1", "9"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, 0., 0, .5},
                                       {456, 0, 3, .5, 3, 1.},
                                   });
}

TEST_F(incidents, missed) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB", "BE", "EH", "HI"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .1, .2, 123});
  reader->add(edge_ids[3], baldr::GraphReader::Incident{edge_ids[3].id(), .6, .7, 456});
  reader->sort();

  // do the route
  auto result = gurka::route(map, {"1", "9"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {});
}

TEST_F(incidents, simple_point) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .25, .25, 123});
  reader->sort();

  // do the route
  auto result = gurka::route(map, {"C", "B"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, .25, 0, .25},
                                   });
}

TEST_F(incidents, left_point) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), 0., 0., 123});
  reader->sort();

  // do the route
  auto result = gurka::route(map, {"C", "H"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, 0., 0, 0.},
                                   });
}

TEST_F(incidents, right_point) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), 1., 1., 123});
  reader->sort();

  // do the route
  auto result = gurka::route(map, {"C", "B"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, 1., 0, 1.},
                                   });
}

TEST_F(incidents, multipoint) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB", "BE", "EH"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .5, .5, 123});
  reader->add(edge_ids[1], baldr::GraphReader::Incident{edge_ids[1].id(), 0., 0., 456});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), .2, .2, 789});
  reader->sort();

  // do the route
  auto result = gurka::route(map, {"C", "H"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, .5, 0, .5},
                                       {456, 0, 1, 0., 1, 0.},
                                       {789, 0, 2, .2, 2, .2},
                                   });
}

TEST_F(incidents, point_collisions) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB", "BE", "EH"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .5, .5, 123});
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .5, .5, 456});
  reader->add(edge_ids[1], baldr::GraphReader::Incident{edge_ids[1].id(), 1., 1., 789});
  reader->add(edge_ids[1], baldr::GraphReader::Incident{edge_ids[1].id(), 1., 1., 987});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), .9, .9, 654});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), .9, .9, 321});
  reader->sort();

  // do the route
  auto result = gurka::route(map, {"C", "H"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, .5, 0, .5},
                                       {456, 0, 0, .5, 0, .5},
                                       {789, 0, 1, 1., 1, 1.},
                                       {987, 0, 1, 1., 1, 1.},
                                       {654, 0, 2, .9, 2, .9},
                                       {321, 0, 2, .9, 2, .9},
                                   });
}

TEST_F(incidents, point_shared) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB", "BE", "EH"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), 1., 1., 123});
  reader->add(edge_ids[1], baldr::GraphReader::Incident{edge_ids[1].id(), 0., 0., 123});
  reader->add(edge_ids[1], baldr::GraphReader::Incident{edge_ids[1].id(), 1., 1., 456});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), 0., 0., 456});
  reader->sort();

  // do the route
  auto result = gurka::route(map, {"C", "H"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, 1., 0, 1.},
                                       {456, 0, 1, 1., 1, 1.},
                                   });
}

TEST_F(incidents, armageddon) {
  // mark the edges with incidents
  std::vector<baldr::GraphId> edge_ids;
  auto reader = setup_test(map, {"CB", "BE", "EH", "HI"}, edge_ids);
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // modify the incident tile to have incidents on this edge
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .25, .75, 123});
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .5, 1., 456});
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .5, .5, 987});
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), 1., 1., 666});

  reader->add(edge_ids[1], baldr::GraphReader::Incident{edge_ids[1].id(), .0, .0, 666});
  reader->add(edge_ids[1], baldr::GraphReader::Incident{edge_ids[1].id(), 0., 1., 456});
  reader->add(edge_ids[1], baldr::GraphReader::Incident{edge_ids[1].id(), 0., 1., 789});
  reader->add(edge_ids[1], baldr::GraphReader::Incident{edge_ids[1].id(), .6, .6, 321});

  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), 0., .5, 456});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), 0., .6, 789});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), .65, .65, 0});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), .69, .69, 1});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), .7, 1., 2});
  reader->add(edge_ids[2], baldr::GraphReader::Incident{edge_ids[2].id(), .8, 1., 3});

  reader->add(edge_ids[3], baldr::GraphReader::Incident{edge_ids[3].id(), 0., .1, 2});
  reader->add(edge_ids[3], baldr::GraphReader::Incident{edge_ids[3].id(), 0., .7, 3});

  // out of bounds
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .3, .3, 654});
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .25, .25, 4});
  reader->add(edge_ids[0], baldr::GraphReader::Incident{edge_ids[0].id(), .1, .3, 5});
  reader->add(edge_ids[3], baldr::GraphReader::Incident{edge_ids[3].id(), .9, .9, 6});
  reader->add(edge_ids[3], baldr::GraphReader::Incident{edge_ids[3].id(), .6, .95, 7});

  reader->sort();

  // do the route
  auto result = gurka::route(map, {"1", "B", "E", "9"}, "auto",
                             {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                             graphreader);

  // check its right
  check_incident_locations(result, {
                                       // first edge is only half the edge because we start in the
                                       // middle of it
                                       {123, 0, 0, .0, 0, .5},
                                       {456, 0, 0, .0, 0, 1.},
                                       {987, 0, 0, .0, 0, .0},
                                       {666, 0, 0, 1., 0, 1.},
                                       {666, 1, 0, 0., 0, 0.},
                                       {456, 1, 0, 0., 0, 1.},
                                       {789, 1, 0, 0., 0, 1.},
                                       {321, 1, 0, .6, 0, .6},
                                       {456, 2, 0, 0., 0, .5},
                                       {789, 2, 0, 0., 0, .6},
                                       {0, 2, 0, .65, 0, .65},
                                       {1, 2, 0, .69, 0, .69},
                                       {2, 2, 0, .7, 1, .2},
                                       {3, 2, 0, .8, 1, 1.},
                                   });
}