#include "gurka.h"
#include "sif/util.h"
#include <gtest/gtest.h>

using namespace valhalla;

const std::unordered_map<std::string, std::string> build_config{{"mjolnir.shortcuts", "false"}};

TEST(recosting, mode_changes) {
  const std::string ascii_map = R"(A--1--B--2--C
                                         |     |
                                         |     |
                                         3     4
                                         |     |
                                         |     |
                                         D--5--E--6--F)";
  const gurka::ways ways = {
      {"A1B2C", {{"highway", "residential"}}},
      {"D5E6F", {{"highway", "residential"}}},
      {"B3D", {{"highway", "trunk"}}},
      {"C4E", {{"highway", "trunk"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_flat_loop", build_config);
  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));

  std::string named_locations = "ABCDEF123456";
  for (const auto& i : named_locations) {
    for (const auto& j : named_locations) {
      if (i == j) {
        continue;
      }

      // get the api response out using the normal means
      auto api = gurka::route(map, std::string(i, 1), std::string(j, 1), "auto", "", reader);

      // setup a callback for the recosting to get each edge
      auto node_itr = api.trip().routes(0).legs(0).node().begin();
      sif::PathEdgeCallback edge_cb = [&node_itr]() -> sif::PathEdge {
        // done
        if (!node_itr->has_edge())
          return {};
        // here's an edge
        sif::PathEdge pe{baldr::GraphId(node_itr->edge().id()),
                         sif::TravelMode(node_itr->edge().travel_mode())};
        ++node_itr;
        return pe;
      };

      // setup a callback for the recosting to tell us about the new label each made
      sif::EdgeLabelCallback label_cb = []() -> void {};

      // recost the path

      // check the results are the same
    }
  }
}