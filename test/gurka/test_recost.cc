#include "gurka.h"
#include "sif/util.h"
#include <gtest/gtest.h>

using namespace valhalla;

const std::unordered_map<std::string, std::string> build_config{{"mjolnir.shortcuts", "false"}};

void create_costing(const Api& request,
                    sif::cost_ptr_t* mode_costing,
                    const sif::CostFactory<sif::DynamicCost>& factory) {

  const auto& options = request.options();

  mode_costing[static_cast<uint8_t>(sif::TravelMode::kDrive)] =
      factory.Create(Costing::auto_, options);
  mode_costing[static_cast<uint8_t>(sif::TravelMode::kPedestrian)] =
      factory.Create(Costing::pedestrian, options);
  mode_costing[static_cast<uint8_t>(sif::TravelMode::kBicycle)] =
      factory.Create(Costing::bicycle, options);
  mode_costing[static_cast<uint8_t>(sif::TravelMode::kPublicTransit)] =
      factory.Create(Costing::transit, options);

  valhalla::sif::cost_ptr_t cost = factory.Create(options);
  mode_costing[static_cast<uint8_t>(cost->travel_mode())] = cost;
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
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_recost", build_config);
  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));

  // run all the permutations
  std::string named_locations = "1A2B3C4D5E6F7";
  std::vector<std::unordered_map<std::string, std::string>> options{
      {}, {{"/date_time/type", "1"}, {"/date_time/value", "2020-06-16T14:12"}},
      //{{"/date_time/type", "2"}, {"/date_time/value", "2020-06-16T14:12"}},
  };
  for (const auto& option : options) {
    for (size_t i = 0; i < named_locations.size(); ++i) {
      for (size_t j = 0; j < named_locations.size(); ++j) {
        // skip uninterestingly close routes
        if (i == j) {
          continue;
        }

        // get the api response out using the normal means
        auto start = named_locations.substr(i, 1);
        auto end = named_locations.substr(j, 1);
        auto api = gurka::route(map, start, end, "auto", option, reader);
        auto leg = api.trip().routes(0).legs(0);

        // build up the costing object
        sif::cost_ptr_t mode_costing[static_cast<int>(sif::TravelMode::kMaxTravelMode)];
        sif::CostFactory<sif::DynamicCost> factory;
        factory.RegisterStandardCostingModels();
        create_costing(api, mode_costing, factory);

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
          EXPECT_NEAR(elapsed_itr->transition_time(), label.transition_secs(), .1);
          // TODO: test restrictions
          // we need to move to the next node which has the elapsed time at the end of the edge
          ++elapsed_itr;
          EXPECT_NEAR(elapsed_itr->elapsed_time(), label.cost().secs, .1);
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

        // recost the path
        if (forward) {
          sif::recost_forward(*reader, *mode_costing[static_cast<uint8_t>(sif::TravelMode::kDrive)],
                              edge_cb, label_cb, src_pct, tgt_pct, date_time);
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

  // build up the costing object
  sif::cost_ptr_t mode_costing[static_cast<int>(sif::TravelMode::kMaxTravelMode)];
  sif::CostFactory<sif::DynamicCost> factory;
  factory.RegisterStandardCostingModels();
  create_costing(api, mode_costing, factory);

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

  // those percentages are bonkers
  EXPECT_THROW(sif::recost_forward(*reader,
                                   *mode_costing[static_cast<uint8_t>(sif::TravelMode::kDrive)],
                                   edge_cb, label_cb, -90, 476),
               std::logic_error);

  // this edge id is valid but doesnt exist
  EXPECT_THROW(sif::recost_forward(*reader,
                                   *mode_costing[static_cast<uint8_t>(sif::TravelMode::kDrive)],
                                   []() { return baldr::GraphId{123456789}; }, label_cb),
               std::runtime_error);

  // this edge id is not valid
  sif::recost_forward(*reader, *mode_costing[static_cast<uint8_t>(sif::TravelMode::kDrive)],
                      []() { return baldr::GraphId{}; }, label_cb);
  EXPECT_EQ(called, false);

  // this path isnt possible with a car because the second edge doesnt have auto access
  called = false;
  EXPECT_THROW(sif::recost_forward(*reader,
                                   *mode_costing[static_cast<uint8_t>(sif::TravelMode::kDrive)],
                                   edge_cb, label_cb),
               std::runtime_error);
  EXPECT_EQ(called, true);

  // go in the reverse direction
  api = gurka::route(map, "F", "A", "pedestrian", {}, reader);
  edge_itr = api.trip().routes(0).legs(0).node().begin();

  // this path isnt possible with a car because the first node is a gate
  called = false;
  EXPECT_THROW(sif::recost_forward(*reader,
                                   *mode_costing[static_cast<uint8_t>(sif::TravelMode::kDrive)],
                                   edge_cb, label_cb),
               std::runtime_error);
  EXPECT_EQ(called, true);

  // travel only on pedestrian edges
  api = gurka::route(map, "B", "D", "pedestrian", {}, reader);
  edge_itr = api.trip().routes(0).legs(0).node().begin();

  // it wont be able to evaluate any edges because they are all pedestrian only
  called = false;
  EXPECT_THROW(sif::recost_forward(*reader,
                                   *mode_costing[static_cast<uint8_t>(sif::TravelMode::kDrive)],
                                   edge_cb, label_cb),
               std::runtime_error);
  EXPECT_EQ(called, false);
}