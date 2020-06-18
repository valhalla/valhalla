#include "gurka.h"
#include "sif/util.h"
#include <gtest/gtest.h>

using namespace valhalla;

const std::unordered_map<std::string, std::string> build_config{{"mjolnir.shortcuts", "false"}};

void create_costing(const Api& request,
                    sif::cost_ptr_t* mode_costing,
                    const sif::CostFactory<sif::DynamicCost>& factory) {
  // Parse out the type of route - this provides the costing method to use
  const auto& options = request.options();
  const auto& costing = options.costing();
  const auto& costing_str = Costing_Enum_Name(costing);

  // Set travel mode and construct costing
  if (costing == Costing::multimodal || costing == Costing::transit) {
    // For multi-modal we construct costing for all modes and set the
    // initial mode to pedestrian. (TODO - allow other initial modes)
    mode_costing[0] = factory.Create(Costing::auto_, options);
    mode_costing[1] = factory.Create(Costing::pedestrian, options);
    mode_costing[2] = factory.Create(Costing::bicycle, options);
    mode_costing[3] = factory.Create(Costing::transit, options);
  } else {
    valhalla::sif::cost_ptr_t cost = factory.Create(options);
    mode_costing[static_cast<uint8_t>(cost->travel_mode())] = cost;
  }
}

TEST(recosting, mode_changes) {
  const std::string ascii_map = R"(A--1--B-2-3-C
                                         |     |
                                         |     |
                                         4     5
                                         |     |
                                         |     |
                                         D--6--E--7--F)";
  const gurka::ways ways = {
      {"A1B2C", {{"highway", "residential"}}},
      {"D5E6F", {{"highway", "residential"}}},
      {"B3D", {{"highway", "trunk"}}},
      {"C4E", {{"highway", "trunk"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_flat_loop", build_config);
  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));

  // run all the permutations
  std::string named_locations = "1A2B3C4D5E6F7";
  std::vector<std::unordered_map<std::string, std::string>> options{
      {}, {{"/date_time/type", "1"}, {"/date_time/value", "2020-06-16T14:12"}},
      //{{"/date_time/type", "2"}, {"/date_time/value", "2020-06-16T14:12"}},
  };
  for (const auto& option : options)
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

        // build up the costing object
        sif::cost_ptr_t mode_costing[static_cast<int>(sif::TravelMode::kMaxTravelMode)];
        sif::CostFactory<sif::DynamicCost> factory;
        factory.RegisterStandardCostingModels();
        create_costing(api, mode_costing, factory);

        // find the percentage of the edges used
        auto leg = api.trip().routes(0).legs(0);
        float end_pct = 1;
        for (const auto& edge : api.options().locations(1).path_edges()) {
          if (std::next(leg.node().rbegin())->edge().id() == edge.graph_id()) {
            end_pct = edge.percent_along();
            break;
          }
        }
        float begin_pct = 0;
        for (const auto& edge : api.options().locations(0).path_edges()) {
          if (leg.node(0).edge().id() == edge.graph_id()) {
            // handle trivial route
            begin_pct = (leg.node_size() == 2 ? end_pct : 1.f) - edge.percent_along();
            break;
          }
        }

        // setup a callback for the recosting to get each edge
        auto edge_itr = leg.node().begin();
        sif::EdgeCallback edge_cb = [&edge_itr, &begin_pct, end_pct]() -> sif::PathEdge {
          // done
          if (!edge_itr->has_edge())
            return {};

          // how much of the edge
          auto edge_pct =
              begin_pct != -1 ? begin_pct : (!std::next(edge_itr)->has_edge() ? end_pct : 1.f);
          begin_pct = -1;

          // here's an edge
          sif::PathEdge pe{baldr::GraphId(edge_itr->edge().id()),
                           sif::TravelMode(edge_itr->edge().travel_mode()), edge_pct};
          ++edge_itr;
          return pe;
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

        // recost the path
        auto dt_itr = option.find("/date_time/value");
        std::string date_time = dt_itr != option.cend() ? dt_itr->second : "";
        auto type_itr = option.find("/date_time/type");
        bool forward = type_itr != option.cend() ? std::stoi(type_itr->second) != 2 : true;
        if (forward) {
          sif::recost_forward(*reader, &mode_costing[0], date_time, edge_cb, label_cb);
        } else {
          // sif::recost_reverse(*reader, &mode_costing[0], date_time, edge_cb, label_cb);
        }
      }
    }
}