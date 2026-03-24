#include "baldr/openlr.h"
#include "baldr/rapidjson_utils.h"
#include "gurka.h"
#include "midgard/pointll.h"
#include "proto/options.pb.h"
#include "test.h"
#include "thor/worker.h"
#include "tyr/actor.h"

#include <gtest/gtest.h>

using namespace valhalla;
using namespace valhalla::gurka;
using namespace valhalla::midgard;
using namespace valhalla::baldr;

class Search : public ::testing::Test {
protected:
  static gurka::map map;
  static gurka::nodelayout layout;

  static void SetUpTestSuite() {
    // the numbers are not on any edge so they won't affect the graph,
    // they're just reference points for test locations
    const std::string ascii_map = R"(
            B
            |\
            | \
           8|  \
            |   \
       9    2 4  7
            A-3--D  1
            |   /   
            | 5x6
            | /   
            |/  
            C
            
            
            
            
            y
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "living_street"}}}, {"AD", {{"highway", "living_street"}}},
        {"AC", {{"highway", "living_street"}}}, {"BD", {{"highway", "living_street"}}},
        {"CD", {{"highway", "living_street"}}},
    };

    layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map = gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/gurka_search",
                            {{"service_limits.max_radius", "50000"}}); // test a high radius
  }

  using loc_modifier = std::function<void(valhalla::Location&)>;

  static Api do_locate(const PointLL& ll, const loc_modifier& mod = {}) {
    Api request;
    auto* options = request.mutable_options();
    options->set_action(Options::locate);
    auto* loc = options->add_locations();
    loc->mutable_ll()->set_lat(ll.lat());
    loc->mutable_ll()->set_lng(ll.lng());
    if (mod)
      mod(*loc);

    tyr::actor_t actor(map.config);
    actor.locate("", nullptr, &request);
    return request;
  }

  static const auto& correlated_edges(const Api& api) {
    return api.options().locations(0).correlation().edges();
  }

  static const auto& correlated_filtered_edges(const Api& api) {
    return api.options().locations(0).correlation().filtered_edges();
  }

  static int edge_count(const Api& api) {
    return correlated_edges(api).size();
  }

  static bool snapped_to_node(const Api& api) {
    for (const auto& e : correlated_edges(api)) {
      if (e.begin_node() || e.end_node())
        return true;
    }
    return false;
  }

  static bool
  has_edge(const Api& api, const std::string& from, const std::string& to, bool filtered = false) {
    GraphReader reader(map.config.get_child("mjolnir"));
    auto [id, edge] = gurka::findEdgeByNodes(reader, layout, from, to);
    for (const auto& e : (filtered ? correlated_filtered_edges(api) : correlated_edges(api))) {
      if (GraphId(e.graph_id()) == id)
        return true;
    }
    return false;
  }

  static valhalla::Location::SideOfStreet
  sos_for(const Api& api, const std::string& from, const std::string& to) {
    GraphReader reader(map.config.get_child("mjolnir"));
    auto [id, edge] = gurka::findEdgeByNodes(reader, layout, from, to);
    for (const auto& e : correlated_edges(api)) {
      if (GraphId(e.graph_id()) == id)
        return e.side_of_street();
    }
    ADD_FAILURE() << "edge " << from << "->" << to << " not in result";
    return valhalla::Location::kNone;
  }

  static PointLL pt(const std::string& name) {
    return layout.at(name);
  }
};

gurka::map Search::map = {};
gurka::nodelayout Search::layout = {};

// node snaps

TEST_F(Search, SnapToNodeA) {
  auto api = do_locate(pt("A"));
  EXPECT_TRUE(snapped_to_node(api));
  EXPECT_EQ(api.options().locations(0).correlation().edges_size(), 6);
  EXPECT_TRUE(has_edge(api, "A", "B"));
  EXPECT_TRUE(has_edge(api, "A", "D"));
  EXPECT_TRUE(has_edge(api, "A", "C"));
}

TEST_F(Search, SnapToNodeD) {
  auto api = do_locate(pt("D"));
  EXPECT_TRUE(snapped_to_node(api));
  EXPECT_TRUE(has_edge(api, "D", "B"));
  EXPECT_TRUE(has_edge(api, "D", "A"));
  EXPECT_TRUE(has_edge(api, "D", "C"));
}

// point 1 is just past D — should still snap to D and include arriving edges
TEST_F(Search, SnapNearEndOfEdgeAtD) {
  auto api = do_locate(pt("1"));
  EXPECT_TRUE(snapped_to_node(api));
  EXPECT_TRUE(has_edge(api, "D", "B"));
  EXPECT_TRUE(has_edge(api, "D", "A"));
  EXPECT_TRUE(has_edge(api, "D", "C"));
  EXPECT_TRUE(has_edge(api, "B", "D"));
  EXPECT_TRUE(has_edge(api, "A", "D"));
  EXPECT_TRUE(has_edge(api, "C", "D"));
}

// regression #2023: point 2 is just above A, beyond search_cutoff but within node_snap_tolerance
TEST_F(Search, NodeSnapToleranceVsSearchCutoff) {
  auto api = do_locate(pt("2"), [](valhalla::Location& loc) {
    loc.set_search_cutoff(94);
    loc.set_node_snap_tolerance(160);
  });
  EXPECT_TRUE(snapped_to_node(api));
  EXPECT_TRUE(has_edge(api, "A", "B"));
  EXPECT_TRUE(has_edge(api, "A", "D"));
  EXPECT_TRUE(has_edge(api, "A", "C"));
  EXPECT_TRUE(has_edge(api, "B", "A"));
  EXPECT_TRUE(has_edge(api, "D", "A"));
  EXPECT_TRUE(has_edge(api, "C", "A"));
  EXPECT_EQ(edge_count(api), 6);
}

// through location at a node returns all edges
TEST_F(Search, ThroughLocationAtNode) {
  auto api =
      do_locate(pt("A"), [](valhalla::Location& loc) { loc.set_type(valhalla::Location::kThrough); });
  EXPECT_TRUE(snapped_to_node(api));
  EXPECT_TRUE(has_edge(api, "A", "B"));
  EXPECT_TRUE(has_edge(api, "A", "D"));
  EXPECT_TRUE(has_edge(api, "A", "C"));
  EXPECT_TRUE(has_edge(api, "B", "A"));
  EXPECT_TRUE(has_edge(api, "D", "A"));
  EXPECT_TRUE(has_edge(api, "C", "A"));
  EXPECT_EQ(edge_count(api), 6);
}

// through + heading south should only match A->C and B->A
TEST_F(Search, ThroughLocationWithHeading) {
  auto api = do_locate(pt("A"), [](valhalla::Location& loc) {
    loc.set_type(valhalla::Location::kThrough);
    loc.set_heading(180);
  });
  EXPECT_TRUE(has_edge(api, "A", "C"));
  EXPECT_TRUE(has_edge(api, "B", "A"));
  EXPECT_EQ(edge_count(api), 2);
}

// mid-edge snaps — point 3 is ~50% along AD, point 4 is ~40%

TEST_F(Search, MidPointOnAD) {
  auto api = do_locate(pt("3"));
  EXPECT_FALSE(snapped_to_node(api));
  EXPECT_TRUE(has_edge(api, "A", "D"));
  EXPECT_TRUE(has_edge(api, "D", "A"));
}

TEST_F(Search, PartialAlongAD) {
  auto api = do_locate(pt("4"));
  EXPECT_FALSE(snapped_to_node(api));
  EXPECT_TRUE(has_edge(api, "A", "D"));
  EXPECT_TRUE(has_edge(api, "D", "A"));
}

TEST_F(Search, HeadingEastSelectsForwardEdge) {
  auto api = do_locate(pt("4"), [](valhalla::Location& loc) { loc.set_heading(90); });
  EXPECT_TRUE(has_edge(api, "A", "D"));
}

TEST_F(Search, HeadingWestSelectsReverseEdge) {
  auto api = do_locate(pt("4"), [](valhalla::Location& loc) { loc.set_heading(269); });
  EXPECT_TRUE(has_edge(api, "D", "A"));
  EXPECT_EQ(edge_count(api), 1);
}

// heading north is roughly perpendicular to A->D so both edges match
TEST_F(Search, HeadingNorthYieldsFilteredEdges) {
  auto api = do_locate(pt("4"), [](valhalla::Location& loc) { loc.set_heading(0); });
  EXPECT_TRUE(has_edge(api, "A", "D", true));
  EXPECT_TRUE(has_edge(api, "D", "A", true));
}

// side of street — point 5 is left of CD, point 6 is right of CD
TEST_F(Search, SideOfStreetByPerpendicularOffset) {
  auto api = do_locate(pt("5"));
  EXPECT_FALSE(snapped_to_node(api));
  auto sos_cd = sos_for(api, "C", "D");
  auto sos_dc = sos_for(api, "D", "C");
  EXPECT_NE(sos_cd, valhalla::Location::kNone);
  EXPECT_NE(sos_dc, valhalla::Location::kNone);
  EXPECT_NE(sos_cd, sos_dc) << "opposite directed edges must have opposite SOS";
}

// snap to the edge at point 4, but derive SOS from display_ll at point 5
TEST_F(Search, SideOfStreetViaDisplayLatLng) {
  auto display = pt("6");
  auto api = do_locate(pt("5"), [&](valhalla::Location& loc) {
    loc.mutable_display_ll()->set_lat(display.lat());
    loc.mutable_display_ll()->set_lng(display.lng());
  });
  auto sos_ad = sos_for(api, "C", "D");
  auto sos_da = sos_for(api, "D", "C");
  EXPECT_NE(sos_ad, valhalla::Location::kNone);
  EXPECT_NE(sos_da, valhalla::Location::kNone);
  EXPECT_NE(sos_ad, sos_da);
}

// display_ll on opposite sides (5 vs 6) should flip SOS
TEST_F(Search, DisplayLatLngFlipsSideOfStreet) {
  auto d5 = pt("5");
  auto d6 = pt("6");

  auto api_l = do_locate(pt("x"), [&](valhalla::Location& loc) {
    loc.mutable_display_ll()->set_lat(d5.lat());
    loc.mutable_display_ll()->set_lng(d5.lng());
  });
  auto api_r = do_locate(pt("x"), [&](valhalla::Location& loc) {
    loc.mutable_display_ll()->set_lat(d6.lat());
    loc.mutable_display_ll()->set_lng(d6.lng());
  });

  EXPECT_NE(sos_for(api_l, "C", "D"), sos_for(api_r, "C", "D"))
      << "display points on opposite sides should give opposite SOS";
}

// point 9 is far left of AD — beyond street_side_max_distance
TEST_F(Search, DisplayLatLngTooFarGivesNone) {
  auto far = pt("9");
  auto api = do_locate(pt("4"), [&](valhalla::Location& loc) {
    loc.mutable_display_ll()->set_lat(far.lat());
    loc.mutable_display_ll()->set_lng(far.lng());
    loc.set_street_side_max_distance(100);
  });
  EXPECT_EQ(sos_for(api, "A", "D"), valhalla::Location::kNone);
  EXPECT_EQ(sos_for(api, "D", "A"), valhalla::Location::kNone);
}

TEST_F(Search, LargeStreetSideToleranceSuppressesSOS) {
  auto api = do_locate(pt("5"), [](valhalla::Location& loc) { loc.set_street_side_tolerance(2000); });
  EXPECT_EQ(sos_for(api, "C", "D"), valhalla::Location::kNone);
  EXPECT_EQ(sos_for(api, "D", "C"), valhalla::Location::kNone);
}

// preferred side — point 5 is offset from AD so we get a definite side

TEST_F(Search, PreferredSideOpposite) {
  auto api = do_locate(pt("5"), [](valhalla::Location& loc) {
    loc.set_preferred_side(valhalla::Location::opposite);
  });
  EXPECT_EQ(edge_count(api), 1);
}

TEST_F(Search, PreferredSideSame) {
  auto api = do_locate(pt("5"), [](valhalla::Location& loc) {
    loc.set_preferred_side(valhalla::Location::same);
  });
  EXPECT_EQ(edge_count(api), 1);
}

TEST_F(Search, PreferredSideSameVsOppositeGiveDifferentEdges) {
  auto api_s = do_locate(pt("5"), [](valhalla::Location& loc) {
    loc.set_preferred_side(valhalla::Location::same);
  });
  auto api_o = do_locate(pt("5"), [](valhalla::Location& loc) {
    loc.set_preferred_side(valhalla::Location::opposite);
  });
  ASSERT_EQ(edge_count(api_s), 1);
  ASSERT_EQ(edge_count(api_o), 1);
  EXPECT_NE(correlated_edges(api_s).Get(0).graph_id(), correlated_edges(api_o).Get(0).graph_id());
}

// B->D edge — point 7 is ~40% along BD, point 8 is offset from BD

TEST_F(Search, PartialAlongBD) {
  auto api = do_locate(pt("7"));
  EXPECT_FALSE(snapped_to_node(api));
  EXPECT_TRUE(has_edge(api, "B", "D"));
  EXPECT_TRUE(has_edge(api, "D", "B"));
}

TEST_F(Search, SideOfStreetOnAB) {
  auto api = do_locate(pt("8"));
  auto sos_bd = sos_for(api, "B", "A");
  auto sos_db = sos_for(api, "A", "B");
  EXPECT_NE(sos_bd, valhalla::Location::kNone);
  EXPECT_NE(sos_db, valhalla::Location::kNone);
  EXPECT_NE(sos_bd, sos_db);
}

// reachability — use computed points since these are off-graph locations

TEST_F(Search, ReachabilityZeroEverything) {
  PointLL near_b(pt("B").lng() - .001f, pt("B").lat() - .01f);
  auto api = do_locate(near_b, [](valhalla::Location& loc) {
    loc.set_minimum_reachability(0);
    loc.set_radius(0);
  });
  EXPECT_GE(edge_count(api), 1);
}

TEST_F(Search, ReachabilityHighRadius) {
  auto longest = static_cast<unsigned int>(pt("B").Distance(pt("D")));
  auto api = do_locate(pt("8"), [longest](valhalla::Location& loc) {
    loc.set_minimum_reachability(0);
    loc.set_radius(20000);
  });
  EXPECT_EQ(edge_count(api), 10);
}

TEST_F(Search, ReachabilityMidRadius) {
  auto shortest = static_cast<unsigned int>(pt("B").Distance(pt("A")));
  auto api = do_locate(pt("B"), [shortest](valhalla::Location& loc) {
    loc.set_minimum_reachability(0);
    loc.set_radius(shortest > 100 ? shortest - 100 : 0);
  });
  EXPECT_GT(edge_count(api), 0);
  EXPECT_LT(edge_count(api), 10);
}

TEST_F(Search, ReachabilityHighRequirement) {
  PointLL near_b(pt("B").lng() - .001f, pt("B").lat() - .001f);
  auto api = do_locate(near_b, [](valhalla::Location& loc) {
    loc.set_minimum_reachability(10);
    loc.set_radius(0);
  });
  for (const auto& e : correlated_edges(api)) {
    EXPECT_GE(e.outbound_reach(), 4);
    EXPECT_GE(e.inbound_reach(), 4);
  }
}

TEST_F(Search, ReachabilityExactThreshold) {
  PointLL near_b(pt("B").lng() - .001f, pt("B").lat() - .01f);
  auto api = do_locate(near_b, [](valhalla::Location& loc) {
    loc.set_minimum_reachability(4);
    loc.set_radius(0);
  });
  for (const auto& e : correlated_edges(api)) {
    EXPECT_GE(e.outbound_reach(), 4);
    EXPECT_GE(e.inbound_reach(), 4);
  }
}

TEST_F(Search, ReachabilityLowerThreshold) {
  PointLL near_b(pt("B").lng() - .001f, pt("B").lat() - .01f);
  auto api = do_locate(near_b, [](valhalla::Location& loc) {
    loc.set_minimum_reachability(3);
    loc.set_radius(0);
  });
  for (const auto& e : correlated_edges(api)) {
    EXPECT_GE(e.outbound_reach(), 3);
    EXPECT_GE(e.inbound_reach(), 3);
  }
}

// search cutoff

TEST_F(Search, CutoffFarAwayFindsNothing) {
  auto api = do_locate({-77, -77});
  EXPECT_EQ(edge_count(api), 0);
}

TEST_F(Search, CutoffTooSmallFindsNothing) {
  PointLL far(pt("C").lng() - 1, pt("C").lat() - 1);
  auto dist = far.Distance(pt("C"));
  auto api = do_locate(far, [dist](valhalla::Location& loc) { loc.set_search_cutoff(dist - 1); });
  EXPECT_EQ(edge_count(api), 0);
}

TEST_F(Search, CutoffJustBigEnoughFindsSomething) {
  // next node should be just 500 meters away
  auto api = do_locate(pt("y"), [](valhalla::Location& loc) { loc.set_search_cutoff(501); });
  EXPECT_GT(edge_count(api), 0);
}

TEST_F(Search, EdgeSnapCutoffJustOutOfReach) {
  PointLL near_c(pt("C").lng() - .00005f, pt("C").lat() + .00005f);
  PointLL edge_pt(pt("C").lng(), pt("C").lat() + .00005f);
  auto dist = near_c.Distance(edge_pt);
  auto api = do_locate(near_c, [dist](valhalla::Location& loc) { loc.set_search_cutoff(dist - 1); });
  EXPECT_EQ(edge_count(api), 0);
}

TEST_F(Search, EdgeSnapCutoffJustWithinReach) {
  PointLL near_c(pt("C").lng() - .00005f, pt("C").lat() + .00005f);
  PointLL edge_pt(pt("C").lng(), pt("C").lat() + .00005f);
  auto dist = near_c.Distance(edge_pt);
  auto api = do_locate(near_c, [dist](valhalla::Location& loc) { loc.set_search_cutoff(dist + 10); });
  EXPECT_GT(edge_count(api), 0);
}

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
      {"AB", {{"highway", "motorway"}, {"maxweight", "3.5"}}},
      {"BC", {{"highway", "primary"}, {"maxheight", "4.8"}}},
      {"AD", {{"highway", "residential"}, {"maxaxles", "3"}}},
      {"BE", {{"highway", "motorway_link"}, {"hgv:conditional", "no @ 23:00-05:00"}}},
      {"CF", {{"highway", "pedestrian"}}},
      {"DE", {{"highway", "trunk"}}},
      {"EF", {{"highway", "secondary"}}},
      {"DG", {{"highway", "trunk_link"}}},
      {"EH", {{"highway", "cycleway"}}},
      {"FI", {{"highway", "service"}}},
      {"GH", {{"highway", "tertiary"}}},
      {"HI", {{"highway", "unclassified"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/gurka_locate_basic",
                        {
                            {"mjolnir.concurrency", "1"},
                            {"mjolnir.reclassify_links", "0"},
                            {"mjolnir.traffic_extract", "test/data/gurka_locate_basic/traffic.tar"},
                        });
  test::build_live_traffic_data(map.config);

  // turn on some traffic for fun
  auto traffic_cb = [](baldr::GraphReader& /*reader*/, baldr::TrafficTile& /*tile*/, int /*index*/,
                       valhalla::baldr::TrafficSpeed* current) -> void {
    current->overall_encoded_speed = 124 >> 1;
    current->encoded_speed1 = 126 >> 1;
    current->congestion1 = 0; // unknown
    current->breakpoint1 = 85;
    current->encoded_speed2 = 124 >> 1;
    current->congestion2 = 32; // middle
    current->breakpoint2 = 170;
    current->encoded_speed3 = 122 >> 1;
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
  std::vector<int> allowed_headings = {0, 90, 180, 270, 360};
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

    // check if headings make sense
    auto h1 = static_cast<int>(
        rapidjson::Pointer("/" + std::to_string(i) + "/edges/0/heading").Get(response)->GetDouble());
    auto h2 = static_cast<int>(
        rapidjson::Pointer("/" + std::to_string(i) + "/edges/1/heading").Get(response)->GetDouble());
    ASSERT_TRUE(h2 == (h1 + 180) % 360);

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
      ASSERT_EQ(rapidjson::Pointer("/live_speed/breakpoint_1").Get(edge)->GetDouble(), 0.66);
      ASSERT_EQ(rapidjson::Pointer("/live_speed/speed_2").Get(edge)->GetInt(), 122);
      ASSERT_EQ(rapidjson::Pointer("/live_speed/congestion_2").Get(edge)->GetDouble(), 1.0);

      // make sure the heading was determined correctly
      auto heading = static_cast<int>(rapidjson::Pointer("/heading").Get(edge)->GetDouble());
      ASSERT_TRUE(std::count(allowed_headings.begin(), allowed_headings.end(), heading));

      // are there access restrictions
      if (ways.find(way_name)->second.size() == 2) {
        ASSERT_EQ(rapidjson::Pointer("/access_restrictions").Get(edge)->GetArray().Size(), 1);
      }
    }
  }

  // if we  search with a cutoff and a certain heading and tolerance we should end up with no results
  result = gurka::do_action(valhalla::Options::locate, map, {"1", "2"}, "none",
                            {
                                {"/locations/0/heading", "45"},
                                {"/locations/0/heading_tolerance", "20"},
                                {"/locations/0/search_cutoff", "1"},
                            },
                            reader, &json);
  ASSERT_EQ(result.options().locations(0).heading_tolerance(), 20);
  ASSERT_EQ(result.options().locations(0).heading(), 45);
  ASSERT_EQ(result.options().locations(0).search_cutoff(), 1);
  EXPECT_TRUE(result.options().locations(0).correlation().edges().empty());
  EXPECT_FALSE(result.options().locations(0).correlation().filtered_edges().empty());

  result = gurka::do_action(valhalla::Options::route, map, {"1", "2"}, "auto",
                            {
                                {"/locations/0/heading", "45"},
                                {"/locations/0/heading_tolerance", "20"},
                                {"/locations/0/search_cutoff", "1"},
                            },
                            reader, &json);
  ASSERT_EQ(result.options().locations(0).heading_tolerance(), 20);
  ASSERT_EQ(result.options().locations(0).heading(), 45);
  ASSERT_EQ(result.options().locations(0).search_cutoff(), 1);

  // filtered edges should have been moved to edges
  EXPECT_FALSE(result.options().locations(0).correlation().edges().empty());
  EXPECT_TRUE(result.options().locations(0).correlation().filtered_edges().empty());
  EXPECT_EQ(result.info().warnings_size(), 1);
}

TEST(locate, locate_shoulder) {
  const std::string ascii_map = R"(
    A---B---C
    |   |   |
    D---E---F
  )";

  const gurka::ways ways = {{"AB", {{"highway", "motorway"}, {"shoulder", "yes"}}},
                            {"BC",
                             {{"highway", "motorway"}, {"shoulder", "false"}}}, // Tagged as false
                            {"AD", {{"highway", "residential"}}},               // No shoulder tag
                            {"BE", {{"highway", "residential"}, {"shoulder", "yes"}}},
                            {"CF", {{"highway", "residential"}}},
                            {"DE", {{"highway", "tertiary"}}},
                            {"EF", {{"highway", "tertiary"}, {"shoulder", "yes"}}}};

  auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {},
                               VALHALLA_BUILD_DIR "test/data/gurka_locate_shoulder_moderate");

  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  std::string json;
  [[maybe_unused]] auto result = gurka::do_action(valhalla::Options::locate, map,
                                                  {"B", "C", "E", "F"}, "none", {}, reader, &json);

  rapidjson::Document response;
  response.Parse(json);

  std::unordered_map<std::string, bool> expected_shoulders = {{"AB", true},  {"BC", false},
                                                              {"AD", false}, {"BE", true},
                                                              {"CF", false}, {"DE", false},
                                                              {"EF", true}};

  for (size_t i = 0; i < response.GetArray().Size(); ++i) {
    auto edges = rapidjson::Pointer("/" + std::to_string(i) + "/edges").Get(response)->GetArray();
    for (const auto& edge : edges) {
      std::string way_name = edge["edge_info"]["names"][0].GetString();

      ASSERT_TRUE(edge.HasMember("shoulder"));

      bool shoulder_value = edge["shoulder"].GetBool();
      EXPECT_EQ(shoulder_value, expected_shoulders[way_name])
          << "Way " << way_name << " has incorrect shoulder value.";
    }
  }
}
