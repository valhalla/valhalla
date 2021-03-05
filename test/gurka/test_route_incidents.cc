#include "gurka.h"
#include "test.h"

using namespace valhalla;

class IncidentsTest : public ::testing::Test {
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
        {"AB", {{"highway", "tertiary"}}},  {"BC", {{"highway", "service"}}},
        {"DEF", {{"highway", "primary"}}},  {"GHI", {{"highway", "primary"}}},
        {"ADG", {{"highway", "motorway"}}}, {"BE", {{"highway", "secondary"}}},
        {"EH", {{"highway", "tertiary"}}},
    };

    // generate the lls for the nodes in the map
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10, {.2f, .2f});

    // make the tiles
    std::string tile_dir = "test/data/route_incidents";
    map = gurka::buildtiles(layout, ways, {}, {}, tile_dir,
                            {
                                {"mjolnir.traffic_extract", tile_dir + "/traffic.tar"},
                            });

    // stage up some live traffic data
    test::build_live_traffic_data(map.config);
  }
};

// initialize static member
gurka::map IncidentsTest::map = {};

// for asserting where the incident starts and ends on the route and how long it is
struct incident_location {
  uint64_t incident_id;
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
    for (const auto& incident : leg.incidents()) {
      ASSERT_GE(incident.begin_shape_index(), last_shape_index) << " leg incidents were out of order";
      last_shape_index = incident.begin_shape_index();
    }
    incident_count += leg.incidents_size();
  }

  // check all the incidents are in the right place in the route
  for (const auto& location : locations) {
    const auto& leg = api.trip().routes(0).legs(location.leg_index);
    // find the incident
    const valhalla::TripLeg::Incident* incident = nullptr;
    for (int j = 0; j < leg.incidents_size(); ++j) {
      if (leg.incidents(j).metadata().id() == location.incident_id) {
        incident = &leg.incidents(j);
        break;
      }
    }
    ASSERT_TRUE(incident != nullptr)
        << "Couldn't find incident " << location.incident_id << " on leg " << location.leg_index
        << " on edge " << location.begin_edge_index;

    // check that we are on the right edges of the path
    const auto& begin_edge = leg.node(location.begin_edge_index).edge();
    ASSERT_TRUE(begin_edge.begin_shape_index() <= incident->begin_shape_index() &&
                incident->begin_shape_index() <= begin_edge.end_shape_index())
        << "Metadata " << location.incident_id << " on leg " << location.leg_index << " on edge "
        << location.begin_edge_index << " begins on wrong edge";
    const auto& end_edge = leg.node(location.end_edge_index).edge();

    ASSERT_TRUE(end_edge.begin_shape_index() <= incident->end_shape_index() &&
                incident->end_shape_index() <= end_edge.end_shape_index())
        << "Metadata " << location.incident_id << " on leg " << location.leg_index << " on edge "
        << location.end_edge_index << " ends on wrong edge";

    // check that we are on the right shape points
    const auto& lengths = leg_lengths[location.leg_index];
    auto begin_edge_length =
        lengths[begin_edge.end_shape_index()] - lengths[begin_edge.begin_shape_index()];
    auto begin_edge_pct =
        (lengths[incident->begin_shape_index()] - lengths[begin_edge.begin_shape_index()]) /
        begin_edge_length;
    ASSERT_NEAR(begin_edge_pct, location.begin_pct, .01)
        << "Metadata " << location.incident_id << " on leg " << location.leg_index << " on edge "
        << location.begin_edge_index << " begins at wrong location along the edge";

    auto end_edge_length =
        lengths[end_edge.end_shape_index()] - lengths[end_edge.begin_shape_index()];
    auto end_edge_pct =
        (lengths[incident->end_shape_index()] - lengths[end_edge.begin_shape_index()]) /
        end_edge_length;
    ASSERT_NEAR(end_edge_pct, location.end_pct, .1)
        << "Metadata " << location.incident_id << " on leg " << location.leg_index << " on edge "
        << location.end_edge_index << " ends at wrong location along the edge";
  }

  ASSERT_EQ(incident_count, locations.size()) << "Expected number of incidents does not match actual";
}

// via some macro magic we are inside the scope of the incidents class above
TEST_F(IncidentsTest, simple_cut) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {{
                                                    {{
                                                        "CB",
                                                        .25,
                                                        .75,
                                                    }},
                                                    1234,
                                                    "foobar",
                                                }});
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "B"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {
                                       {1234, 0, 0, .25, 0, .75},
                                   });
}

TEST_F(IncidentsTest, whole_edge) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {{
                                                    {{
                                                        "CB",
                                                        .0,
                                                        1.,
                                                    }},
                                                    1234,
                                                    "foobar",
                                                }});
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "B"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {
                                       {1234, 0, 0, 0., 0, 1.},
                                   });
}

TEST_F(IncidentsTest, left) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {{
                                                    {{
                                                        "CB",
                                                        .0,
                                                        .5,
                                                    }},
                                                    1234,
                                                    "foobar",
                                                }});
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "B"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {
                                       {1234, 0, 0, 0., 0, .5},
                                   });
}

TEST_F(IncidentsTest, right) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {{
                                                    {{
                                                        "CB",
                                                        .5,
                                                        1.,
                                                    }},
                                                    1234,
                                                    "foobar",
                                                }});
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "B"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {
                                       {1234, 0, 0, .5, 0, 1.},
                                   });
}

TEST_F(IncidentsTest, multiedge) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {
                                                    {
                                                        {{
                                                            "CB",
                                                            .5,
                                                            1.,
                                                        }},
                                                        1234,
                                                        "first incident",
                                                    },
                                                    {
                                                        {
                                                            {"BE", 0., 1.},
                                                        },
                                                        8888,
                                                        "second incident",
                                                    },
                                                    {
                                                        {{
                                                            "EH",
                                                            .0,
                                                            .5,
                                                        }},
                                                        5555,
                                                        "third incident",
                                                    },
                                                });
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "H"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {
                                       {1234, 0, 0, .5, 0, 1.},
                                       {8888, 0, 0, 1., 2, 0.},
                                       {5555, 0, 2, 0., 2, 0.5},
                                   });
}

TEST_F(IncidentsTest, multiincident) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {
                                                    {
                                                        {{
                                                            "CB",
                                                            .25,
                                                            .4,
                                                        }},
                                                        123,
                                                        "first incident",
                                                    },
                                                    {
                                                        {
                                                            {"CB", .5, 1.},
                                                            {"BE", 0., 1.},
                                                            {"EH", 0., .5},
                                                        },
                                                        456,
                                                        "second incident, spanning 3 edges",
                                                    },
                                                    {
                                                        {{
                                                            "EH",
                                                            .75,
                                                            .9,
                                                        }},
                                                        789,
                                                        "third incident",
                                                    },
                                                });
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "H"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, .25, 0, .4},
                                       {456, 0, 0, .5, 2, .5},
                                       {789, 0, 2, .75, 2, .9},
                                   });
}

TEST_F(IncidentsTest, interleaved) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {
                                                    {
                                                        {{
                                                            "CB",
                                                            .25,
                                                            .75,
                                                        }},
                                                        123,
                                                        "first incident",
                                                    },
                                                    {
                                                        {
                                                            {"CB", .5, 1.},
                                                            {"BE", 0., 1.},
                                                            {"EH", 0., .5},
                                                        },
                                                        456,
                                                        "second incident, spanning 3 edges",
                                                    },
                                                    {
                                                        {{
                                                            "EH",
                                                            .25,
                                                            .9,
                                                        }},
                                                        789,
                                                        "third incident",
                                                    },
                                                });
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "H"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, .25, 0, .75},
                                       {456, 0, 0, .5, 2, .5},
                                       {789, 0, 2, .25, 2, .9},
                                   });
}

TEST_F(IncidentsTest, collisions) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {
                                                    {
                                                        {{
                                                            "CB",
                                                            .25,
                                                            .5,
                                                        }},
                                                        123,
                                                        "first incident",
                                                    },
                                                    {
                                                        {
                                                            {"CB", .5, 1.},
                                                            {"BE", 0., 1.},
                                                            {"EH", 0., .5},
                                                        },
                                                        456,
                                                        "second incident, spanning 3 edges",
                                                    },
                                                    {
                                                        {{
                                                            "EH",
                                                            .5,
                                                            .9,
                                                        }},
                                                        789,
                                                        "third incident",
                                                    },
                                                });
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "H"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, .25, 0, .5},
                                       {456, 0, 0, .5, 2, .5},
                                       {789, 0, 2, .5, 2, .9},
                                   });
}

TEST_F(IncidentsTest, full_overlap) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {
                                                    {
                                                        {{
                                                            "CB",
                                                            .5,
                                                            1.,
                                                        }},
                                                        123,
                                                        "first incident",
                                                    },
                                                    {
                                                        {
                                                            {"CB", .5, 1.},
                                                        },
                                                        456,
                                                        "second incident",
                                                    },
                                                });
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "H"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, .5, 0, 1.},
                                       {456, 0, 0, .5, 0, 1.},
                                   });
}

TEST_F(IncidentsTest, multileg) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {
                                                    {
                                                        {{
                                                            "CB",
                                                            .25,
                                                            .75,
                                                        }},
                                                        123,
                                                        "first incident",
                                                    },
                                                    {
                                                        {
                                                            {"CB", .5, 1.},
                                                            {"BE", 0., 1.},
                                                            {"EH", 0., .5},
                                                        },
                                                        456,
                                                        "second incident, spanning 3 edges",
                                                    },
                                                    {
                                                        {{"BE", 0., 1.},
                                                         {
                                                             "EH",
                                                             .0,
                                                             .9,
                                                         }},
                                                        789,
                                                        "third incident, spanning two edges",
                                                    },
                                                });
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "B", "E", "H"}, "auto",
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

TEST_F(IncidentsTest, clipped) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {
                                                    {
                                                        {{
                                                            "CB",
                                                            .25,
                                                            .75,
                                                        }},
                                                        123,
                                                        "first incident",
                                                    },
                                                    {
                                                        {
                                                            {"HI", .25, .75},
                                                        },
                                                        456,
                                                        "second incident",
                                                    },
                                                });
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"1", "9"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, 0., 0, .5},
                                       {456, 0, 3, .5, 3, 1.},
                                   });
}

TEST_F(IncidentsTest, missed) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {
                                                    {
                                                        {{
                                                            "CB",
                                                            .1,
                                                            .2,
                                                        }},
                                                        123,
                                                        "first incident",
                                                    },
                                                    {
                                                        {
                                                            {"HI", .6, .7},
                                                        },
                                                        456,
                                                        "second incident",
                                                    },
                                                });
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});
  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"1", "9"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {});
}

TEST_F(IncidentsTest, simple_point) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {
                                                    {
                                                        {{
                                                            "CB",
                                                            .25,
                                                            .25,
                                                        }},
                                                        123,
                                                        "first incident",
                                                    },
                                                });
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "B"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, .25, 0, .25},
                                   });
}

TEST_F(IncidentsTest, left_point) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {
                                                    {
                                                        {{
                                                            "CB",
                                                            .0,
                                                            .0,
                                                        }},
                                                        123,
                                                        "first incident",
                                                    },
                                                });
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "H"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, 0., 0, 0.},
                                   });
}

TEST_F(IncidentsTest, right_point) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {
                                                    {
                                                        {{
                                                            "CB",
                                                            1.,
                                                            1.,
                                                        }},
                                                        123,
                                                        "first incident",
                                                    },
                                                });
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "B"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, 1., 0, 1.},
                                   });
}

TEST_F(IncidentsTest, multipoint) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {
                                                    {
                                                        {{
                                                            "CB",
                                                            .5,
                                                            .5,
                                                        }},
                                                        123,
                                                        "first incident",
                                                    },
                                                    {
                                                        {
                                                            {"BE", .0, .0},
                                                        },
                                                        456,
                                                        "second incident",
                                                    },
                                                    {
                                                        {
                                                            {"EH", .2, .2},
                                                        },
                                                        789,
                                                        "third incident",
                                                    },
                                                });
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "H"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, .5, 0, .5},
                                       {456, 0, 1, 0., 1, 0.},
                                       {789, 0, 2, .2, 2, .2},
                                   });
}

TEST_F(IncidentsTest, point_collisions) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {
                                                    {
                                                        {{
                                                            "CB",
                                                            .5,
                                                            .5,
                                                        }},
                                                        123,
                                                        "first incident",
                                                    },
                                                    {
                                                        {
                                                            {"CB", .5, .5},
                                                        },
                                                        456,
                                                        "second incident",
                                                    },
                                                    {
                                                        {
                                                            {"BE", 1., 1.},
                                                        },
                                                        789,
                                                        "third incident",
                                                    },
                                                    {
                                                        {
                                                            {"BE", 1., 1.},
                                                        },
                                                        987,
                                                        "fourth incident",
                                                    },
                                                    {
                                                        {
                                                            {"EH", .9, .9},
                                                        },
                                                        654,
                                                        "fifth incident",
                                                    },
                                                    {
                                                        {
                                                            {"EH", .9, .9},
                                                        },
                                                        321,
                                                        "sixth incident",
                                                    },
                                                });
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "H"}, "auto",
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

TEST_F(IncidentsTest, point_shared) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {
                                                    {
                                                        {{
                                                             "CB",
                                                             1.,
                                                             1.,
                                                         },
                                                         {
                                                             "BE",
                                                             0.,
                                                             0.,
                                                         }},
                                                        123,
                                                        "first incident, two points",
                                                    },
                                                    {
                                                        {
                                                            {"BE", 1., 1.},
                                                            {"EH", 0, 0},
                                                        },
                                                        456,
                                                        "second incident, two points",
                                                    },
                                                });
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"C", "H"}, "auto",
                       {{"/filters/action", "include"}, {"/filters/attributes/0", "incidents"}},
                       graphreader);

  // check its right
  check_incident_locations(result, {
                                       {123, 0, 0, 1., 0, 1.},
                                       {456, 0, 1, 1., 1, 1.},
                                   });
}

TEST_F(IncidentsTest, armageddon) {
  // mark the edges with incidents
  auto reader = setup_graph_with_incidents(map, {
                                                    {
                                                        {
                                                            {
                                                                "CB",
                                                                .25,
                                                                .75,
                                                            },
                                                        },
                                                        123,
                                                        "first incident",
                                                    },
                                                    {
                                                        {
                                                            {
                                                                "CB",
                                                                .5,
                                                                1.,
                                                            },
                                                            {
                                                                "BE",
                                                                0.,
                                                                1.,
                                                            },
                                                            {
                                                                "EH",
                                                                0.,
                                                                .5,
                                                            },
                                                        },
                                                        456,
                                                        "second incident",
                                                    },
                                                    {
                                                        {
                                                            {
                                                                "CB",
                                                                .5,
                                                                .5,
                                                            },
                                                        },
                                                        987,
                                                        "third incident",
                                                    },
                                                    {
                                                        {
                                                            {
                                                                "CB",
                                                                1.,
                                                                1.,
                                                            },
                                                            {
                                                                "BE",
                                                                0.,
                                                                0.,
                                                            },
                                                        },
                                                        666,
                                                        "fourth incident",
                                                    },
                                                    {
                                                        {
                                                            {"BE", 0., 1.},
                                                            {
                                                                "EH",
                                                                0.,
                                                                .6,
                                                            },
                                                        },
                                                        789,
                                                        "lost count at this point",
                                                    },
                                                    {
                                                        {
                                                            {"BE", .6, .6},
                                                        },
                                                        321,
                                                        "lost count at this point",
                                                    },
                                                    {
                                                        {
                                                            {"EH", .65, .65},
                                                        },
                                                        0,
                                                        "lost count at this point",
                                                    },
                                                    {
                                                        {
                                                            {"EH", .69, .69},
                                                        },
                                                        1,
                                                        "lost count at this point",
                                                    },
                                                    {
                                                        {
                                                            {"EH", .7, 1.},
                                                            {"HI", .0, .1},
                                                        },
                                                        2,
                                                        "lost count at this point",
                                                    },
                                                    {
                                                        {
                                                            {"EH", .8, 1.},
                                                            {"HI", .0, .7},
                                                        },
                                                        3,
                                                        "lost count at this point",
                                                    },
                                                    {
                                                        {
                                                            {"CB", .3, .3},
                                                        },
                                                        654,
                                                        "lost count at this point",
                                                    },
                                                    {
                                                        {
                                                            {"CB", .25, .25},
                                                        },
                                                        4,
                                                        "lost count at this point",
                                                    },
                                                    {
                                                        {
                                                            {"HI", .9, .9},
                                                        },
                                                        6,
                                                        "lost count at this point",
                                                    },
                                                    {
                                                        {
                                                            {"HI", .6, .95},
                                                        },
                                                        7,
                                                        "lost count at this point",
                                                    },
                                                });
  std::shared_ptr<baldr::GraphReader> graphreader(reader.get(), [](baldr::GraphReader*) {});

  // do the route
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"1", "B", "E", "9"}, "auto",
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
