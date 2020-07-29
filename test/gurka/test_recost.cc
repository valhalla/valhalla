#include "gurka.h"
#include "sif/recost.h"
#include <gtest/gtest.h>

using namespace valhalla;

const std::unordered_map<std::string, std::string> build_config{{"mjolnir.shortcuts", "false"}};

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
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_recost", build_config);
  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));

  // run all the permutations
  std::string named_locations = "1A2B3C4D5E6F7";
  std::vector<std::unordered_map<std::string, std::string>> options{
      {},
      {{"/date_time/type", "1"}, {"/date_time/value", "2020-06-16T14:12"}},
      {{"/date_time/type", "2"}, {"/date_time/value", "2020-06-16T14:12"}},
  };
  for (const auto& option : options) {
    for (size_t i = 0; i < named_locations.size(); ++i) {
      for (size_t j = i + 1; j < named_locations.size(); ++j) {
        // skip uninterestingly close routes
        if (i == j) {
          continue;
        }

        // get the api response out using the normal means
        auto start = named_locations.substr(i, 1);
        auto end = named_locations.substr(j, 1);
        auto api = gurka::route(map, start, end, "auto", option, reader);
        auto leg = api.trip().routes(0).legs(0);

        // setup a callback for the recosting to get each edge
        auto edge_itr = leg.node().begin();
        sif::EdgeCallback edge_cb = [&edge_itr]() -> baldr::GraphId {
          auto edge_id =
              edge_itr->has_edge() ? baldr::GraphId(edge_itr->edge().id()) : baldr::GraphId{};
          ++edge_itr;
          return edge_id;
        };

        // setup a callback for the recosting to tell us about the new label each made
        auto elapsed_itr = leg.node().begin();
        double length = 0;
        uint32_t pred = baldr::kInvalidLabel;
        sif::LabelCallback label_cb = [&elapsed_itr, &length,
                                       &pred](const sif::EdgeLabel& label) -> void {
          length += elapsed_itr->edge().length() * 1000.0;
          EXPECT_EQ(elapsed_itr->edge().id(), label.edgeid());
          EXPECT_EQ(pred++, label.predecessor());
          EXPECT_EQ(static_cast<uint8_t>(elapsed_itr->edge().travel_mode()),
                    static_cast<uint8_t>(label.mode()));
          EXPECT_NEAR(length, label.path_distance(), 2);
          EXPECT_NEAR(elapsed_itr->cost().transition_cost().seconds(), label.transition_cost().secs,
                      .1);
          // TODO: test restrictions
          // we need to move to the next node which has the elapsed time at the end of the edge
          ++elapsed_itr;
          EXPECT_NEAR(elapsed_itr->cost().elapsed_cost().seconds(), label.cost().secs, .1);
        };

        // find the percentage of the edges used
        float src_pct = 0;
        for (const auto& edge : api.options().locations(0).path_edges()) {
          if (leg.node(0).edge().id() == edge.graph_id()) {
            src_pct = edge.percent_along();
            break;
          }
        }
        float tgt_pct = 1;
        for (const auto& edge : api.options().locations(1).path_edges()) {
          if (std::next(leg.node().rbegin())->edge().id() == edge.graph_id()) {
            tgt_pct = edge.percent_along();
            break;
          }
        }

        // is there time dependence and in what direction
        auto dt_itr = option.find("/date_time/value");
        std::string date_time = dt_itr != option.cend() ? dt_itr->second : "";
        auto type_itr = option.find("/date_time/type");
        bool forward = type_itr != option.cend() ? std::stoi(type_itr->second) != 2 : true;
        // build up the costing object
        auto costing = sif::CostFactory().Create(api.options());

        // recost the path
        if (forward) {
          sif::recost_forward(*reader, *costing, edge_cb, label_cb, src_pct, tgt_pct, date_time);
        } else {
          // sif::recost_reverse(*reader, &mode_costing[0], date_time, edge_cb, label_cb);
        }
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
  auto api = gurka::route(map, "A", "F", "pedestrian", {}, reader);

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
  EXPECT_THROW(sif::recost_forward(*reader, *costing, []() { return baldr::GraphId{123456789}; },
                                   label_cb),
               std::runtime_error);

  // this edge id is not valid
  sif::recost_forward(*reader, *costing, []() { return baldr::GraphId{}; }, label_cb);
  EXPECT_EQ(called, false);

  // this path isnt possible with a car because the second edge doesnt have auto access
  called = false;
  EXPECT_THROW(sif::recost_forward(*reader, *costing, edge_cb, label_cb), std::runtime_error);
  EXPECT_EQ(called, true);

  // go in the reverse direction
  api = gurka::route(map, "F", "A", "pedestrian", {}, reader);
  edge_itr = api.trip().routes(0).legs(0).node().begin();

  // this path isnt possible with a car because the first node is a gate
  called = false;
  EXPECT_THROW(sif::recost_forward(*reader, *costing, edge_cb, label_cb), std::runtime_error);
  EXPECT_EQ(called, true);

  // travel only on pedestrian edges
  api = gurka::route(map, "B", "D", "pedestrian", {}, reader);
  edge_itr = api.trip().routes(0).legs(0).node().begin();

  // it wont be able to evaluate any edges because they are all pedestrian only
  called = false;
  EXPECT_THROW(sif::recost_forward(*reader, *costing, edge_cb, label_cb), std::runtime_error);
  EXPECT_EQ(called, false);
}

TEST(recosting, error_request) {
  auto config = gurka::detail::build_config("foo_bar", {});
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
    actor.route(R"({"costing":"auto","locations":[],"recostings":[{"costing":"auto"}]})");
    FAIL() << "No name should have thrown";
  } catch (const valhalla_exception_t& e) { EXPECT_EQ(e.code, 127); }

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
    EXPECT_GE(greater.elapsed_cost().seconds(), lesser.elapsed_cost().seconds());
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
