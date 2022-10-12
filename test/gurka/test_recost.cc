#include "gurka.h"
#include "mjolnir/graphtilebuilder.h"
#include "sif/recost.h"
#include "test.h"

using namespace valhalla;

const std::unordered_map<std::string, std::string> build_config{{"mjolnir.shortcuts", "false"}};

TEST(recosting, forward_vs_reverse) {
  std::string tile_dir = "test/data/gurka_recost_forward_vs_reverse";

  const std::string ascii_map = R"(
    A---B----C
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "trunk"}}},
  };
  const gurka::nodes nodes = {{"B", {{"highway", "traffic_signals"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, tile_dir, build_config);

  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  test::customize_historical_traffic(map.config, [](baldr::DirectedEdge& e) {
    if (e.classification() == baldr::RoadClass::kResidential) {
      e.set_constrained_flow_speed(40);
    }
    return boost::none;
  });

  // run a route and check that the costs are the same for the same options
  valhalla::tyr::actor_t actor(map.config, *reader, true);

  {
    std::string locations = R"({"lon":)" + std::to_string(map.nodes["A"].lng()) + R"(,"lat":)" +
                            std::to_string(map.nodes["A"].lat()) + R"(},{"lon":)" +
                            std::to_string(map.nodes["C"].lng()) + R"(,"lat":)" +
                            std::to_string(map.nodes["C"].lat()) + "}";
    Api forward;
    actor.route(R"({"costing":"auto","locations":[)" + locations +
                    R"(],"date_time":{"type":1,"value":"2020-08-01T11:12"}})",
                {}, &forward);
    Api reverse;
    actor.route(R"({"costing":"auto","locations":[)" + locations +
                    R"(],"date_time":{"type":2,"value":"2020-08-01T11:12"}})",
                {}, &reverse);
    EXPECT_EQ(forward.trip().routes(0).legs(0).node_size(),
              reverse.trip().routes(0).legs(0).node_size());
    auto rn = reverse.trip().routes(0).legs(0).node().begin();
    for (const auto& n : forward.trip().routes(0).legs(0).node()) {
      // if the assert is triggered - probably something is wrong with TransitionCostReverse arguments
      // in the Unidir-A*
      EXPECT_EQ(n.cost().transition_cost().seconds(), rn->cost().transition_cost().seconds());
      EXPECT_EQ(n.cost().transition_cost().cost(), rn->cost().transition_cost().cost());
      ++rn;
    }
  }
}

TEST(recosting, forward_vs_reverse_internal_turn) {
  std::string tile_dir = "test/data/gurka_recost_forward_vs_reverse_internal_turn";

  const std::string ascii_map = R"(
           1
           |
    A------B-----C
           |
    2------E----F
  )";
  const gurka::ways ways = {
      {"ABC", {{"highway", "tertiary"}, {"oneway", "yes"}}},
      {"1B", {{"highway", "tertiary"}, {"oneway", "yes"}}},
      // BE is supposed to be an internal edge
      {"BE", {{"highway", "tertiary"}}},
      {"FE2", {{"highway", "tertiary"}, {"oneway", "yes"}}},
  };
  const gurka::nodes nodes = {{"B", {{"highway", "traffic_signals"}}},
                              {"E", {{"highway", "traffic_signals"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 4);
  auto map = gurka::buildtiles(layout, ways, nodes, {}, tile_dir, build_config);

  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  // run a route and check that the costs are the same for the same options
  valhalla::tyr::actor_t actor(map.config, *reader, true);
  {
    std::string locations = R"({"lon":)" + std::to_string(map.nodes["1"].lng()) + R"(,"lat":)" +
                            std::to_string(map.nodes["1"].lat()) + R"(},{"lon":)" +
                            std::to_string(map.nodes["2"].lng()) + R"(,"lat":)" +
                            std::to_string(map.nodes["2"].lat()) + "}";
    Api forward;
    actor.route(R"({"costing":"auto","locations":[)" + locations +
                    R"(],"date_time":{"type":1,"value":"2020-08-01T11:12"}})",
                {}, &forward);
    Api reverse;
    actor.route(R"({"costing":"auto","locations":[)" + locations +
                    R"(],"date_time":{"type":2,"value":"2020-08-01T11:12"}})",
                {}, &reverse);
    EXPECT_EQ(forward.trip().routes(0).legs(0).node_size(),
              reverse.trip().routes(0).legs(0).node_size());
    int node_count = forward.trip().routes(0).legs(0).node_size();
    auto forward_cost =
        forward.trip().routes(0).legs(0).node().Get(node_count - 1).cost().elapsed_cost();
    auto reverse_cost =
        reverse.trip().routes(0).legs(0).node().Get(node_count - 1).cost().elapsed_cost();
    // if assert is triggered - check if uturn on internal edges is detected correctly
    EXPECT_NEAR(forward_cost.cost(), reverse_cost.cost(), 0.1);
  }
}

TEST(recosting, same_historical) {
  const std::string ascii_map = R"(A--1--B-2-3-C-----G
                                         |     |     |
                                         |     |     8
                                         4     5     H
                                         |     |
                                         |     |
                                         D--6--E--7--F)";
  const gurka::ways ways = {
      {"A1B23CG8H", {{"highway", "residential"}}},
      {"D6E7F", {{"highway", "residential"}}},
      {"B4D", {{"highway", "trunk"}}},
      {"C5E", {{"highway", "trunk"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  std::string tile_dir = "test/data/gurka_recost";
  auto map = gurka::buildtiles(layout, ways, {}, {}, tile_dir, build_config);
  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));

  // add historical traffic so that we can get different costs at different times
  test::customize_historical_traffic(map.config, [](baldr::DirectedEdge& e) {
    e.set_free_flow_speed(80);
    e.set_speed(55);
    e.set_constrained_flow_speed(10);
    // TODO: add historical 5 minutely buckets
    return boost::none;
  });

  // run a route and check that the costs are the same for the same options
  valhalla::tyr::actor_t actor(map.config, *reader, true);
  std::string locations = R"({"lon":)" + std::to_string(map.nodes["F"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["F"].lat()) + R"(},{"lon":)" +
                          std::to_string(map.nodes["A"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["A"].lat()) + "}";

  // check the recostings all match using bidirectional a* with no time, should default to constrained
  {
    Api api;
    actor.route(R"({"costing":"auto","locations":[)" + locations +
                    R"(],"recostings":[{"costing":"auto","name":"same"}]})",
                {}, &api);

    // check we have the same cost at all places
    for (const auto& n : api.trip().routes(0).legs(0).node()) {
      EXPECT_EQ(n.recosts_size(), 1);
      EXPECT_NEAR(n.cost().elapsed_cost().seconds(), n.recosts(0).elapsed_cost().seconds(), 0.0001);
      EXPECT_EQ(n.cost().elapsed_cost().cost(), n.recosts(0).elapsed_cost().cost());
      EXPECT_EQ(n.cost().transition_cost().seconds(), n.recosts(0).transition_cost().seconds());
      EXPECT_EQ(n.cost().transition_cost().cost(), n.recosts(0).transition_cost().cost());
      if (n.has_edge()) {
        EXPECT_NEAR(n.edge().speed(), 10.f, .1f);
      }
    }
  }

  // disable constrained flow to make sure we get back the default speed
  {
    Api api;
    actor.route(R"({"costing":"auto","locations":[)" + locations +
                    R"(],"recostings":[{"costing":"auto","name":"same","speed_types":[]}],)" +
                    R"("costing_options":{"auto":{"speed_types":[]}}})",
                {}, &api);

    // check we have the same cost at all places
    for (const auto& n : api.trip().routes(0).legs(0).node()) {
      EXPECT_EQ(n.recosts_size(), 1);
      EXPECT_EQ(n.cost().elapsed_cost().seconds(), n.recosts(0).elapsed_cost().seconds());
      EXPECT_EQ(n.cost().elapsed_cost().cost(), n.recosts(0).elapsed_cost().cost());
      EXPECT_EQ(n.cost().transition_cost().seconds(), n.recosts(0).transition_cost().seconds());
      EXPECT_EQ(n.cost().transition_cost().cost(), n.recosts(0).transition_cost().cost());
      if (n.has_edge()) {
        EXPECT_NEAR(n.edge().speed(), 55.f, .1f);
      }
    }
  }

  // set the time should use time dep forward and freeflow at 1am
  {
    Api api;
    actor.route(R"({"costing":"auto","locations":[)" + locations +
                    R"(],"recostings":[{"costing":"auto","name":"same"}],)" +
                    R"("date_time":{"type":1,"value":"2020-08-01T01:23"}})",
                {}, &api);

    // check we have the same cost at all places
    for (const auto& n : api.trip().routes(0).legs(0).node()) {
      EXPECT_EQ(n.recosts_size(), 1);
      EXPECT_NEAR(n.cost().elapsed_cost().seconds(), n.recosts(0).elapsed_cost().seconds(), 0.0001);
      EXPECT_EQ(n.cost().elapsed_cost().cost(), n.recosts(0).elapsed_cost().cost());
      EXPECT_EQ(n.cost().transition_cost().seconds(), n.recosts(0).transition_cost().seconds());
      EXPECT_EQ(n.cost().transition_cost().cost(), n.recosts(0).transition_cost().cost());
      if (n.has_edge()) {
        EXPECT_NEAR(n.edge().speed(), 80.f, .1f);
      }
    }
  }

  // trivial route should use astar and constrained
  {
    std::string trivial = R"({"lon":)" + std::to_string(map.nodes["D"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["D"].lat()) + R"(},{"lon":)" +
                          std::to_string(map.nodes["B"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["B"].lat()) + "}";

    Api api;
    actor.route(R"({"costing":"auto","locations":[)" + trivial +
                    R"(],"recostings":[{"costing":"auto","name":"same"}]})",
                {}, &api);

    // check we have the same cost at all places
    for (const auto& n : api.trip().routes(0).legs(0).node()) {
      EXPECT_EQ(n.recosts_size(), 1);
      EXPECT_EQ(n.cost().elapsed_cost().seconds(), n.recosts(0).elapsed_cost().seconds());
      EXPECT_EQ(n.cost().elapsed_cost().cost(), n.recosts(0).elapsed_cost().cost());
      EXPECT_EQ(n.cost().transition_cost().seconds(), n.recosts(0).transition_cost().seconds());
      EXPECT_EQ(n.cost().transition_cost().cost(), n.recosts(0).transition_cost().cost());
      if (n.has_edge()) {
        EXPECT_NEAR(n.edge().speed(), 10.f, .1f);
      }
    }
  }

  // TODO: There's a difference in the way a route is costed in reverse vs forward this could be
  // a result of FormPath in timedep_reverse or it could be a result of the difference between
  // upd: the costs are pretty close but still different. Probably the order of float operations can
  // cause this diff
  const float kCostThreshold = 0.0001f;
  // set times but different types and make sure the costings are the same
  {
    std::string unambiguous = R"({"lon":)" + std::to_string(map.nodes["8"].lng()) + R"(,"lat":)" +
                              std::to_string(map.nodes["8"].lat()) + R"(},{"lon":)" +
                              std::to_string(map.nodes["A"].lng()) + R"(,"lat":)" +
                              std::to_string(map.nodes["A"].lat()) + "}";
    Api forward;
    actor.route(R"({"costing":"auto","locations":[)" + unambiguous +
                    R"(],"date_time":{"type":1,"value":"2020-08-01T11:12"}})",
                {}, &forward);
    printf("%s\n", forward.trip().routes(0).legs(0).shape().c_str());
    Api reverse;
    actor.route(R"({"costing":"auto","locations":[)" + unambiguous +
                    R"(],"date_time":{"type":2,"value":"2020-08-01T11:12"}})",
                {}, &reverse);
    EXPECT_EQ(forward.trip().routes(0).legs(0).node_size(),
              reverse.trip().routes(0).legs(0).node_size());
    auto rn = reverse.trip().routes(0).legs(0).node().begin();

    for (const auto& n : forward.trip().routes(0).legs(0).node()) {
      EXPECT_NEAR(n.cost().elapsed_cost().seconds(), rn->cost().elapsed_cost().seconds(),
                  kCostThreshold);
      EXPECT_NEAR(n.cost().elapsed_cost().cost(), rn->cost().elapsed_cost().cost(), kCostThreshold);
      EXPECT_EQ(n.cost().transition_cost().seconds(), rn->cost().transition_cost().seconds());
      EXPECT_EQ(n.cost().transition_cost().cost(), rn->cost().transition_cost().cost());
      if (n.has_edge()) {
        EXPECT_EQ(n.edge().id(), rn->edge().id());
      }
      ++rn;
    }
  }

  // set the time should use time dep reverse and constrained
  {
    Api api;
    actor.route(R"({"costing":"auto","locations":[)" + locations +
                    R"(],"recostings":[{"costing":"auto","name":"same"}],)" +
                    R"("date_time":{"type":2,"value":"2020-08-01T11:12"}})",
                {}, &api);

    // check we have the same cost at all places
    for (const auto& n : api.trip().routes(0).legs(0).node()) {
      EXPECT_EQ(n.recosts_size(), 1);
      EXPECT_NEAR(n.cost().elapsed_cost().seconds(), n.recosts(0).elapsed_cost().seconds(),
                  kCostThreshold);
      EXPECT_EQ(n.cost().elapsed_cost().cost(), n.recosts(0).elapsed_cost().cost());
      EXPECT_EQ(n.cost().transition_cost().seconds(), n.recosts(0).transition_cost().seconds());
      EXPECT_EQ(n.cost().transition_cost().cost(), n.recosts(0).transition_cost().cost());
      if (n.has_edge()) {
        EXPECT_NEAR(n.edge().speed(), 10.f, .1f);
      }
    }
  }
}

TEST(recosting, all_algorithms) {
  const std::string ascii_map = R"(A--1--B-2-3-C
                                         |     |
                                         |     |
                                         4     5
                                         |     |
                                         |     |
                                         D--6--E--7--F)";
  const gurka::ways ways = {
      {"A1B23C", {{"highway", "residential"}}},
      {"D6E7F", {{"highway", "residential"}}},
      {"B4D", {{"highway", "trunk"}}},
      {"C5E", {{"highway", "trunk"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  std::string tile_dir = "test/data/gurka_recost";
  auto map = gurka::buildtiles(layout, ways, {}, {}, tile_dir, build_config);
  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));

  // run all the permutations
  std::string named_locations = "1A2B3C4D5E6F7";
  std::vector<std::unordered_map<std::string, std::string>> options{
      {},
      {{"/date_time/type", "1"}, {"/date_time/value", "2020-06-09T14:12"}},
      {{"/date_time/type", "2"}, {"/date_time/value", "2020-06-22T14:12"}},
  };
  for (const auto& option : options) {
    for (size_t i = 0; i < named_locations.size(); ++i) {
      for (size_t j = i + 2; j < named_locations.size(); j += 2) {
        // get the api response out using the normal means
        auto start = named_locations.substr(i, 1);
        auto end = named_locations.substr(j, 1);
        auto api =
            gurka::do_action(valhalla::Options::route, map, {start, end}, "auto", option, reader);
        auto leg = api.trip().routes(0).legs(0);

        // setup a callback for the recosting to get each edge
        auto edge_itr = leg.node().begin();
        sif::EdgeCallback edge_cb = [&edge_itr]() -> baldr::GraphId {
          auto edge_id =
              edge_itr->has_edge() ? baldr::GraphId(edge_itr->edge().id()) : baldr::GraphId{};
          ++edge_itr;
          return edge_id;
        };

        // TODO: remove this when costing works the same in the reverse direction (see comment above)
        bool reverse =
            option.count("/date_time/type") && option.find("/date_time/type")->second == "2";

        // setup a callback for the recosting to tell us about the new label each made
        auto elapsed_itr = leg.node().begin();
        double length = 0;
        uint32_t pred = baldr::kInvalidLabel;
        sif::LabelCallback label_cb = [&elapsed_itr, &length, &pred,
                                       reverse](const sif::EdgeLabel& label) -> void {
          length += elapsed_itr->edge().length_km() * 1000.0;
          EXPECT_EQ(elapsed_itr->edge().id(), label.edgeid());
          EXPECT_EQ(pred++, label.predecessor());
          EXPECT_EQ(static_cast<uint8_t>(elapsed_itr->edge().travel_mode()),
                    static_cast<uint8_t>(label.mode()));
          EXPECT_NEAR(length, label.path_distance(), 2);
          EXPECT_NEAR(elapsed_itr->cost().transition_cost().seconds(), label.transition_cost().secs,
                      .1);
          if (!reverse)
            EXPECT_NEAR(elapsed_itr->cost().transition_cost().cost(), label.transition_cost().cost,
                        .1);
          // TODO: test restrictions
          // we need to move to the next node which has the elapsed time at the end of the edge
          ++elapsed_itr;
          EXPECT_NEAR(elapsed_itr->cost().elapsed_cost().seconds(), label.cost().secs, .1);
          if (!reverse)
            EXPECT_NEAR(elapsed_itr->cost().elapsed_cost().cost(), label.cost().cost, .1);
        };

        // find the percentage of the edges used
        float src_pct = 0;
        for (const auto& edge : api.options().locations(0).correlation().edges()) {
          if (leg.node(0).edge().id() == edge.graph_id()) {
            src_pct = edge.percent_along();
            break;
          }
        }
        float tgt_pct = 1;
        for (const auto& edge : api.options().locations(1).correlation().edges()) {
          if (std::next(leg.node().rbegin())->edge().id() == edge.graph_id()) {
            tgt_pct = edge.percent_along();
            break;
          }
        }

        // is there time dependence and in what direction
        auto dt_itr = option.find("/date_time/value");
        std::string date_time = dt_itr != option.cend() ? dt_itr->second : "";
        auto type_itr = option.find("/date_time/type");
        // build up the costing object
        auto costing = sif::CostFactory().Create(api.options());

        const GraphId start_edge_id(leg.node().begin()->edge().id());
        const auto* node = reader->nodeinfo(reader->edge_endnode(start_edge_id));
        const auto time_info = baldr::TimeInfo::make(date_time, node->timezone());

        // recost the path
        sif::recost_forward(*reader, *costing, edge_cb, label_cb, src_pct, tgt_pct, time_info);
      }
    }
  }
}

TEST(recosting, throwing) {
  const std::string ascii_map = R"(A--1--B-2-3-C
                                         |     |
                                         |     |
                                         4     5
                                         |     |
                                         |     |
                                         D--6--E--7--F)";
  const gurka::ways ways = {
      {"A1B23C", {{"highway", "residential"}}},
      {"D6E7F", {{"highway", "residential"}}},
      {"B4D", {{"highway", "pedestrian"}}},
      {"C5E", {{"highway", "pedestrian"}}},
  };
  const gurka::nodes nodes = {
      {"E", {{"barrier", "gate"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_recost", build_config);
  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));

  // cross the graph as a pedestrian
  auto api = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "pedestrian", {}, reader);

  // setup a callback for the recosting to get each edge
  const auto& leg = api.trip().routes(0).legs(0);
  auto edge_itr = leg.node().begin();
  sif::EdgeCallback edge_cb = [&edge_itr]() -> baldr::GraphId {
    auto edge_id = edge_itr->has_edge() ? baldr::GraphId(edge_itr->edge().id()) : baldr::GraphId{};
    ++edge_itr;
    return edge_id;
  };

  // setup a callback for the recosting to tell us about the new label each made
  bool called = false;
  sif::LabelCallback label_cb = [&called](const sif::EdgeLabel& label) -> void { called = true; };

  // build up the costing object
  auto costing = sif::CostFactory().Create(Costing::auto_);

  // those percentages are bonkers
  EXPECT_THROW(sif::recost_forward(*reader, *costing, edge_cb, label_cb, -90, 476), std::logic_error);

  // this edge id is valid but doesnt exist
  EXPECT_THROW(sif::recost_forward(
                   *reader, *costing, []() { return baldr::GraphId{123456789}; }, label_cb),
               std::runtime_error);

  // this edge id is not valid
  sif::recost_forward(
      *reader, *costing, []() { return baldr::GraphId{}; }, label_cb);
  EXPECT_EQ(called, false);

  // this path isnt possible with a car because the second edge doesnt have auto access
  called = false;
  EXPECT_THROW(sif::recost_forward(*reader, *costing, edge_cb, label_cb), std::runtime_error);
  EXPECT_EQ(called, true);

  // go in the reverse direction
  api = gurka::do_action(valhalla::Options::route, map, {"F", "A"}, "pedestrian", {}, reader);
  edge_itr = api.trip().routes(0).legs(0).node().begin();

  // this path isnt possible with a car because the first node is a gate
  called = false;
  EXPECT_THROW(sif::recost_forward(*reader, *costing, edge_cb, label_cb), std::runtime_error);
  EXPECT_EQ(called, true);

  // travel only on pedestrian edges
  api = gurka::do_action(valhalla::Options::route, map, {"B", "D"}, "pedestrian", {}, reader);
  edge_itr = api.trip().routes(0).legs(0).node().begin();

  // it wont be able to evaluate any edges because they are all pedestrian only
  called = false;
  EXPECT_THROW(sif::recost_forward(*reader, *costing, edge_cb, label_cb), std::runtime_error);
  EXPECT_EQ(called, false);
}

TEST(recosting, error_request) {
  auto config = test::make_config("foo_bar", {});
  auto reader = std::make_shared<baldr::GraphReader>(config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(config, *reader, true);

  try {
    actor.route(R"({"costing":"auto","locations":[],"recostings":[{"frosting":"chocolate"}]})");
    FAIL() << "No costing should have thrown";
  } catch (const valhalla_exception_t& e) { EXPECT_EQ(e.code, 127); }

  try {
    actor.route(R"({"costing":"auto","locations":[],"recostings":[{"costing":"foo"}]})");
    FAIL() << "Wrong costing should have thrown";
  } catch (const valhalla_exception_t& e) { EXPECT_EQ(e.code, 125); }

  try {
    actor.route(R"({"costing":"auto","locations":[],"recostings":[{"name":"foo"}]})");
    FAIL() << "No costing should have thrown";
  } catch (const valhalla_exception_t& e) { EXPECT_EQ(e.code, 127); }
}

TEST(recosting, api) {
  const std::string ascii_map = R"(A--1--B-2-3-C
                                         |     |
                                         |     |
                                         4     5
                                         |     |
                                         |     |
                                         D--6--E--7--F)";
  const gurka::ways ways = {
      {"A1B23C", {{"highway", "trunk"}, {"hgv", "no"}}},
      {"D6E7F", {{"highway", "trunk"}, {"hgv", "no"}}},
      {"B4D", {{"highway", "residential"}, {"hgv", "no"}}},
      {"C5E", {{"highway", "trunk"}, {"hgv", "no"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_recost", build_config);
  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(map.config, *reader, true);
  std::string locations = R"({"lon":)" + std::to_string(map.nodes["1"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["1"].lat()) + R"(},{"lon":)" +
                          std::to_string(map.nodes["7"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["7"].lat()) + "}";

  // lets also do a bunch of costings
  Api api;
  auto json = actor.route(R"({"costing":"auto","locations":[)" + locations + R"(],"recostings":[
      {"costing":"auto","name":"same"},
      {"costing":"auto","name":"avoid_highways","use_highways":0.1},
      {"costing":"bicycle","name":"slower"},
      {"costing":"pedestrian","name":"slower_still"},
      {"costing":"pedestrian","name":"slowest","walking_speed":2}]})",
                          {}, &api);

  auto greater_equal = [](const valhalla::TripLeg::PathCost& lesser,
                          const valhalla::TripLeg::PathCost& greater, bool cost = true,
                          bool transition = true) {
    if (cost)
      EXPECT_GE(greater.elapsed_cost().cost(), lesser.elapsed_cost().cost());
    bool const is_greater = greater.elapsed_cost().seconds() > lesser.elapsed_cost().seconds();
    bool const is_equal =
        std::abs(greater.elapsed_cost().seconds() - lesser.elapsed_cost().seconds()) < 0.0001;
    EXPECT_TRUE(is_greater || is_equal);
    if (transition) {
      if (cost)
        EXPECT_GE(greater.transition_cost().cost(), lesser.transition_cost().cost());
      EXPECT_GE(greater.transition_cost().seconds(), lesser.transition_cost().seconds());
    }
  };

  // check we have the right amount of costing information at all places
  for (const auto& n : api.trip().routes(0).legs(0).node()) {
    EXPECT_EQ(n.recosts_size(), 5);
    greater_equal(n.cost(), n.recosts(0));
    greater_equal(n.recosts(0), n.recosts(1));
    // this should be strickly bigger because of avoid highways
    if (n.cost().elapsed_cost().seconds() > 0)
      EXPECT_GT(n.recosts(1).elapsed_cost().cost(), n.recosts(0).elapsed_cost().cost());
    greater_equal(n.recosts(1), n.recosts(2), true, false);
    // bike cost uses different costing units so we only do the seconds
    greater_equal(n.recosts(2), n.recosts(3), false, false);
    greater_equal(n.recosts(3), n.recosts(4));
  }

  // verify json has the right info
  {
    rapidjson::Document d;
    d.Parse(json);
    EXPECT_FALSE(d.HasParseError());
    const auto& trip = d["trip"].GetObject();
    const auto& trip_summary = trip["summary"].GetObject();
    EXPECT_TRUE(trip_summary["time"].IsNumber());
    EXPECT_TRUE(trip_summary["time_same"].IsNumber());
    EXPECT_TRUE(trip_summary["time_avoid_highways"].IsNumber());
    EXPECT_TRUE(trip_summary["time_slower"].IsNumber());
    EXPECT_TRUE(trip_summary["time_slower_still"].IsNumber());
    EXPECT_TRUE(trip_summary["time_slowest"].IsNumber());
    for (const auto& leg : trip["legs"].GetArray()) {
      const auto& leg_summary = leg["summary"].GetObject();
      EXPECT_TRUE(leg_summary["time"].IsNumber());
      EXPECT_TRUE(leg_summary["time_same"].IsNumber());
      EXPECT_TRUE(leg_summary["time_avoid_highways"].IsNumber());
      EXPECT_TRUE(leg_summary["time_slower"].IsNumber());
      EXPECT_TRUE(leg_summary["time_slower_still"].IsNumber());
      EXPECT_TRUE(leg_summary["time_slowest"].IsNumber());
      for (const auto& man : leg["maneuvers"].GetArray()) {
        EXPECT_TRUE(man["time"].IsNumber());
        EXPECT_TRUE(man["time_same"].IsNumber());
        EXPECT_TRUE(man["time_avoid_highways"].IsNumber());
        EXPECT_TRUE(man["time_slower"].IsNumber());
        EXPECT_TRUE(man["time_slower_still"].IsNumber());
        EXPECT_TRUE(man["time_slowest"].IsNumber());
      }
    }
  }

  // do it again for osrm format
  api.Clear();
  json = actor.route(R"({"costing":"auto","format":"osrm", "locations":[)" + locations +
                         R"(],"recostings":[
      {"costing":"auto","name":"same"},
      {"costing":"auto","name":"avoid_highways","use_highways":0.1},
      {"costing":"bicycle","name":"slower"},
      {"costing":"pedestrian","name":"slower_still"},
      {"costing":"pedestrian","name":"slowest","walking_speed":2}]})",
                     {}, &api);

  // verify json has the right info
  {
    rapidjson::Document d;
    d.Parse(json);
    EXPECT_FALSE(d.HasParseError());
    for (const auto& route : d["routes"].GetArray()) {
      EXPECT_TRUE(route["duration"].IsNumber());
      EXPECT_TRUE(route["duration_same"].IsNumber());
      EXPECT_TRUE(route["duration_avoid_highways"].IsNumber());
      EXPECT_TRUE(route["duration_slower"].IsNumber());
      EXPECT_TRUE(route["duration_slower_still"].IsNumber());
      EXPECT_TRUE(route["duration_slowest"].IsNumber());
      EXPECT_TRUE(route["weight"].IsNumber());
      EXPECT_TRUE(route["weight_same"].IsNumber());
      EXPECT_TRUE(route["weight_avoid_highways"].IsNumber());
      EXPECT_TRUE(route["weight_slower"].IsNumber());
      EXPECT_TRUE(route["weight_slower_still"].IsNumber());
      EXPECT_TRUE(route["weight_slowest"].IsNumber());
      for (const auto& leg : route["legs"].GetArray()) {
        EXPECT_TRUE(leg["duration"].IsNumber());
        EXPECT_TRUE(leg["duration_same"].IsNumber());
        EXPECT_TRUE(leg["duration_avoid_highways"].IsNumber());
        EXPECT_TRUE(leg["duration_slower"].IsNumber());
        EXPECT_TRUE(leg["duration_slower_still"].IsNumber());
        EXPECT_TRUE(leg["duration_slowest"].IsNumber());
        EXPECT_TRUE(leg["weight"].IsNumber());
        EXPECT_TRUE(leg["weight_same"].IsNumber());
        EXPECT_TRUE(leg["weight_avoid_highways"].IsNumber());
        EXPECT_TRUE(leg["weight_slower"].IsNumber());
        EXPECT_TRUE(leg["weight_slower_still"].IsNumber());
        EXPECT_TRUE(leg["weight_slowest"].IsNumber());
        for (const auto& step : leg["steps"].GetArray()) {
          EXPECT_TRUE(step["duration"].IsNumber());
          EXPECT_TRUE(step["duration_same"].IsNumber());
          EXPECT_TRUE(step["duration_avoid_highways"].IsNumber());
          EXPECT_TRUE(step["duration_slower"].IsNumber());
          EXPECT_TRUE(step["duration_slower_still"].IsNumber());
          EXPECT_TRUE(step["duration_slowest"].IsNumber());
          EXPECT_TRUE(step["weight"].IsNumber());
          EXPECT_TRUE(step["weight_same"].IsNumber());
          EXPECT_TRUE(step["weight_avoid_highways"].IsNumber());
          EXPECT_TRUE(step["weight_slower"].IsNumber());
          EXPECT_TRUE(step["weight_slower_still"].IsNumber());
          EXPECT_TRUE(step["weight_slowest"].IsNumber());
        }
      }
    }
  }

  // do it again and verify that truck doesnt work
  api.Clear();
  json = actor.route(R"({"costing":"auto","locations":[)" + locations + R"(],"recostings":[
      {"costing":"truck","name":"nope"},
      {"costing":"auto","name":"same"}]})",
                     {}, &api);

  // the first recosting should always be blank
  for (const auto& n : api.trip().routes(0).legs(0).node()) {
    EXPECT_EQ(n.recosts_size(), 2);
    EXPECT_FALSE(n.recosts(0).has_elapsed_cost());
    EXPECT_FALSE(n.recosts(0).has_transition_cost());
    EXPECT_TRUE(n.recosts(1).has_elapsed_cost());
    EXPECT_TRUE(n.recosts(1).has_transition_cost());
  }

  // verify json has the right info
  {
    rapidjson::Document d;
    d.Parse(json);
    EXPECT_FALSE(d.HasParseError());
    const auto& trip = d["trip"].GetObject();
    const auto& trip_summary = trip["summary"].GetObject();
    EXPECT_TRUE(trip_summary["time"].IsNumber());
    EXPECT_TRUE(trip_summary["time_same"].IsNumber());
    EXPECT_TRUE(trip_summary["time_nope"].IsNull());
    for (const auto& leg : trip["legs"].GetArray()) {
      const auto& leg_summary = leg["summary"].GetObject();
      EXPECT_TRUE(leg_summary["time"].IsNumber());
      EXPECT_TRUE(leg_summary["time_same"].IsNumber());
      EXPECT_TRUE(leg_summary["time_nope"].IsNull());
      for (const auto& man : leg["maneuvers"].GetArray()) {
        EXPECT_TRUE(man["time"].IsNumber());
        EXPECT_TRUE(man["time_same"].IsNumber());
        EXPECT_TRUE(man["time_nope"].IsNull());
      }
    }
  }

  // do it again for osrm format
  json = actor.route(R"({"costing":"auto","format":"osrm","locations":[)" + locations +
                         R"(],"recostings":[
      {"costing":"truck","name":"nope"},
      {"costing":"auto","name":"same"}]})",
                     {}, &api);

  // verify json has the right info
  {
    rapidjson::Document d;
    d.Parse(json);
    EXPECT_FALSE(d.HasParseError());
    for (const auto& route : d["routes"].GetArray()) {
      EXPECT_TRUE(route["duration"].IsNumber());
      EXPECT_TRUE(route["duration_same"].IsNumber());
      EXPECT_TRUE(route["duration_nope"].IsNull());
      EXPECT_TRUE(route["weight"].IsNumber());
      EXPECT_TRUE(route["weight_same"].IsNumber());
      EXPECT_TRUE(route["weight_nope"].IsNull());
      for (const auto& leg : route["legs"].GetArray()) {
        EXPECT_TRUE(leg["duration"].IsNumber());
        EXPECT_TRUE(leg["duration_same"].IsNumber());
        EXPECT_TRUE(leg["duration_nope"].IsNull());
        EXPECT_TRUE(leg["weight"].IsNumber());
        EXPECT_TRUE(leg["weight_same"].IsNumber());
        EXPECT_TRUE(leg["weight_nope"].IsNull());
        for (const auto& step : leg["steps"].GetArray()) {
          EXPECT_TRUE(step["duration"].IsNumber());
          EXPECT_TRUE(step["duration_same"].IsNumber());
          EXPECT_TRUE(step["duration_nope"].IsNull());
          EXPECT_TRUE(step["weight"].IsNumber());
          EXPECT_TRUE(step["weight_same"].IsNumber());
          EXPECT_TRUE(step["weight_nope"].IsNull());
        }
      }
    }
  }
}
