#include "gurka.h"
#include "microtar.h"
#include <gtest/gtest.h>

#include "baldr/graphreader.h"
#include "baldr/traffictile.h"
#include "test/util/traffic_utils.h"

#include <boost/property_tree/ptree.hpp>

#include <cmath>
#include <sstream>
#include <sys/mman.h>
#include <sys/stat.h>

using namespace valhalla;

TEST(Traffic, BasicUpdates) {

  const std::string ascii_map = R"(
    A----B----C
         |    |
         D----E)";

  const gurka::ways ways = {{"AB", {{"highway", "primary"}, {"maxspeed", "10"}}},
                            {"BC", {{"highway", "primary"}, {"maxspeed", "10"}}},
                            {"BD", {{"highway", "primary"}, {"maxspeed", "10"}}},
                            {"CE", {{"highway", "primary"}, {"maxspeed", "10"}}},
                            {"DE", {{"highway", "primary"}, {"maxspeed", "10"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  std::string tile_dir = "test/data/traffic_basicupdates";
  auto map = gurka::buildtiles(layout, ways, {}, {}, tile_dir);

  map.config.put("mjolnir.traffic_extract", tile_dir + "/traffic.tar");

  valhalla_tests::utils::build_live_traffic_data(map.config);

  {
    auto clean_reader =
        valhalla_tests::utils::make_clean_graphreader(map.config.get_child("mjolnir"));
    printf("Do a route with initial traffic");
    {
      auto result = gurka::route(map, "A", "C", "auto", {{"/date_time/type", "0"}}, clean_reader);
      gurka::assert::osrm::expect_steps(result, {"AB"});
      gurka::assert::raw::expect_path(result, {"AB", "BC"});
      gurka::assert::raw::expect_eta(result, 361.5);
    }
    printf("Make some updates to the traffic .tar file. "
           "Mostly just updates every edge in the file to 24km/h, except for one "
           "specific edge (B->D) where we simulate a closure (speed=0, congestion high)");
    std::function<void(baldr::GraphReader&, baldr::TrafficTile&, int, valhalla::baldr::TrafficSpeed*)>
        cb_setter_24kmh = [&map](baldr::GraphReader& reader, baldr::TrafficTile& tile, int index,
                                 valhalla::baldr::TrafficSpeed* current) -> void {
      baldr::GraphId tile_id(tile.header->tile_id);
      auto BD = gurka::findEdge(reader, map.nodes, "BD", "D", tile_id);
      current->breakpoint1 = 255;
      if (std::get<1>(BD) != nullptr && std::get<0>(BD).id() == index) {
        current->overall_speed = 0;
        current->speed1 = 0;
      } else {
        current->overall_speed = 24 >> 1;
        current->speed1 = 24 >> 1;
      }
    };
    valhalla_tests::utils::customize_live_traffic_data(map.config, cb_setter_24kmh);

    printf(" Now do another route with the same (not restarted) actor to see if"
           " it's noticed the changes in the live traffic file");
    {
      auto result = gurka::route(map, "A", "C", "auto", {{"/date_time/type", "0"}}, clean_reader);
      gurka::assert::osrm::expect_steps(result, {"AB"});
      gurka::assert::raw::expect_path(result, {"AB", "BC"});
      gurka::assert::raw::expect_eta(result, 151.5);
    }
    printf("Next, set the speed to the highest possible to ensure nothing breaks");
    std::function<void(baldr::GraphReader&, baldr::TrafficTile&, int, baldr::TrafficSpeed*)>
        cb_setter_max = [&map](baldr::GraphReader& reader, baldr::TrafficTile& tile, int index,
                               baldr::TrafficSpeed* current) -> void {
      baldr::GraphId tile_id(tile.header->tile_id);
      auto BD = gurka::findEdge(reader, map.nodes, "BD", "D", tile_id);
      current->breakpoint1 = 255;
      if (std::get<1>(BD) != nullptr && std::get<0>(BD).id() == index) {
        current->overall_speed = 0;
      } else {
        current->overall_speed = valhalla::baldr::MAX_TRAFFIC_SPEED_KPH >> 1;
      }
    };
    valhalla_tests::utils::customize_live_traffic_data(map.config, cb_setter_max);
    {
      auto result = gurka::route(map, "A", "C", "auto", {{"/date_time/type", "0"}}, clean_reader);
      gurka::assert::osrm::expect_steps(result, {"AB"});
      gurka::assert::raw::expect_path(result, {"AB", "BC"});
      gurka::assert::raw::expect_eta(result, 15.785715);
    }
    printf("Back to previous speed");
    valhalla_tests::utils::customize_live_traffic_data(map.config, cb_setter_24kmh);
    // And verify that without the "current" timestamp, the live traffic
    // results aren't used
    {
      auto result = gurka::route(map, "A", "C", "auto", {}, clean_reader);
      gurka::assert::osrm::expect_steps(result, {"AB"});
      gurka::assert::raw::expect_path(result, {"AB", "BC"});
      gurka::assert::raw::expect_eta(result, 361.5);
    }
    printf("Now do another route with the same (not restarted) actor to see if "
           "it's noticed the changes in the live traffic file");
    {

      auto result = gurka::route(map, "B", "D", "auto", {{"/date_time/type", "0"}}, clean_reader);
      gurka::assert::osrm::expect_steps(result, {"BC", "CE", "DE"});
      gurka::assert::raw::expect_path(result, {"BC", "CE", "DE"});
      gurka::assert::raw::expect_eta(result, 180., 0.01);
    }
    {
      auto result = gurka::route(map, "D", "B", "auto", {{"/date_time/type", "0"}}, clean_reader);
      gurka::assert::osrm::expect_steps(result, {"BD"});
      gurka::assert::raw::expect_path(result, {"BD"});
      gurka::assert::raw::expect_eta(result, 30., 0.01);
    }
    printf(" Repeat the B->D route, but this time with no timestamp - this should disable "
           "using live traffc and the road should be open again");
    {
      auto result = gurka::route(map, "B", "D", "auto", {}, clean_reader);
      gurka::assert::osrm::expect_steps(result, {"BD"});
      gurka::assert::raw::expect_path(result, {"BD"});
      gurka::assert::raw::expect_eta(result, 72);
    }
  }
}

TEST(Traffic, CutGeoms) {

  const std::string ascii_map = R"(
    A----B----C-F
         |    1 2
         |    | G
         |    | 3
         |    | H
         D----E-I )";

  const gurka::ways ways = {{"AB", {{"highway", "primary"}}}, {"BC", {{"highway", "primary"}}},
                            {"BD", {{"highway", "primary"}}}, {"CE", {{"highway", "primary"}}},
                            {"CF", {{"highway", "primary"}}}, {"FGHI", {{"highway", "primary"}}},
                            {"EI", {{"highway", "primary"}}}, {"DE", {{"highway", "primary"}}}};

  // build the tiles
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  std::string tile_dir = "test/data/traffic_cutgeoms";
  auto map = gurka::buildtiles(layout, ways, {}, {}, tile_dir,
                               {{"mjolnir.traffic_extract", tile_dir + "/traffic.tar"}});

  // empty traffic for now
  valhalla_tests::utils::build_live_traffic_data(map.config);

  // first we get the edge without traffic on it
  {
    auto clean_reader =
        valhalla_tests::utils::make_clean_graphreader(map.config.get_child("mjolnir"));
    tyr::actor_t actor(map.config, *clean_reader);
    valhalla::Api api;
    actor.route(
        R"({"locations":[
        {"lat":)" +
            std::to_string(map.nodes["C"].second) + R"(,"lon":)" +
            std::to_string(map.nodes["C"].first) +
            R"(},
        {"lat":)" +
            std::to_string(map.nodes["E"].second) + R"(,"lon":)" +
            std::to_string(map.nodes["E"].first) +
            R"(}
      ],"costing":"auto","date_time":{"type":0},
      "filters":{"attributes":["edge.length","edge.speed","edge.begin_shape_index",
      "edge.end_shape_index","shape","shape_attributes.length","shape_attributes.time","shape_attributes.speed"],
      "action":"include"}})",
        nullptr, &api);

    const auto& leg = api.trip().routes(0).legs(0);
    auto shapes = midgard::decode<std::vector<valhalla::midgard::PointLL>>(leg.shape());
    EXPECT_EQ(leg.node_size(), 2); // 1 edge
    EXPECT_EQ(shapes.size(), 2);   // 2 shape points
    // An attribute for each pair formed by the shape-points
    EXPECT_EQ(leg.shape_attributes().time_size(), shapes.size() - 1);
    EXPECT_EQ(leg.shape_attributes().length_size(), shapes.size() - 1);
    EXPECT_EQ(leg.shape_attributes().speed_size(), shapes.size() - 1);
  }

  // then we add one portion of the edge having traffic
  {
    valhalla::baldr::TrafficSpeed ts{valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                     valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                     valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                     valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                     0u,
                                     0u,
                                     0u,
                                     0u,
                                     0u};
    ts.overall_speed = 42 >> 1;
    ts.speed1 = 42 >> 1;
    ts.congestion1 = baldr::MAX_CONGESTION_VAL - 1;
    ts.breakpoint1 = 127;

    std::function<void(baldr::GraphReader&, baldr::TrafficTile&, int, baldr::TrafficSpeed*)>
        cb_setter_speed = [&map, &ts](baldr::GraphReader& reader, baldr::TrafficTile& tile, int index,
                                      baldr::TrafficSpeed* current) -> void {
      baldr::GraphId tile_id(tile.header->tile_id);
      auto BD = gurka::findEdge(reader, map.nodes, "BD", "D", tile_id);
      current->breakpoint1 = 255;
      if (std::get<1>(BD) != nullptr && std::get<0>(BD).id() == index) {
        current->overall_speed = 0;
        current->speed1 = 0;
        current->breakpoint1 = 255;
      } else {
        *current = ts;
      }
    };

    valhalla_tests::utils::customize_live_traffic_data(map.config, cb_setter_speed);

    auto clean_reader =
        valhalla_tests::utils::make_clean_graphreader(map.config.get_child("mjolnir"));
    tyr::actor_t actor(map.config, *clean_reader);
    valhalla::Api api;
    actor.route(
        R"({"locations":[
        {"lat":)" +
            std::to_string(map.nodes["C"].second) + R"(,"lon":)" +
            std::to_string(map.nodes["C"].first) +
            R"(},
        {"lat":)" +
            std::to_string(map.nodes["E"].second) + R"(,"lon":)" +
            std::to_string(map.nodes["E"].first) +
            R"(}
      ],"costing":"auto","date_time":{"type":0},
      "filters":{"attributes":["edge.length","edge.speed","edge.begin_shape_index",
      "edge.end_shape_index","shape","shape_attributes.length","shape_attributes.time","shape_attributes.speed"],
      "action":"include"}})",
        nullptr, &api);

    const auto& leg = api.trip().routes(0).legs(0);
    auto shapes = midgard::decode<std::vector<valhalla::midgard::PointLL>>(leg.shape());
    EXPECT_EQ(leg.node_size(), 2); // 1 edge
    EXPECT_EQ(shapes.size(), 3);   // 3 shape points
    // An attribute for each pair formed by the shape-points
    EXPECT_EQ(leg.shape_attributes().time_size(), shapes.size() - 1);
    EXPECT_EQ(leg.shape_attributes().length_size(), shapes.size() - 1);
    EXPECT_EQ(leg.shape_attributes().speed_size(), shapes.size() - 1);

    EXPECT_TRUE(map.nodes["C"].ApproximatelyEqual(shapes[0]));
    auto b1 = map.nodes["C"].PointAlongSegment(map.nodes["E"], 127 / 255.0);
    EXPECT_TRUE(b1.ApproximatelyEqual(shapes[1]));
  }

  // Another permutation of subsegments
  {
    {
      valhalla::baldr::TrafficSpeed ts{valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                       valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                       valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                       valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                       0u,
                                       0u,
                                       0u,
                                       0u,
                                       0u};
      ts.overall_speed = 30 >> 1;

      ts.speed1 = 20 >> 1;
      ts.congestion1 = baldr::MAX_CONGESTION_VAL - 1;
      ts.breakpoint1 = 100;

      ts.speed2 = 40 >> 1;
      ts.congestion2 = 1; // low but not unknown - 0 = unknown
      ts.breakpoint2 = 200;

      std::function<void(baldr::GraphReader&, baldr::TrafficTile&, int, baldr::TrafficSpeed*)>
          cb_setter_speed = [&map, &ts](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                        int index, baldr::TrafficSpeed* current) -> void {
        baldr::GraphId tile_id(tile.header->tile_id);
        auto BD = gurka::findEdge(reader, map.nodes, "BD", "D", tile_id);
        baldr::TrafficSpeed* existing =
            const_cast<valhalla::baldr::TrafficSpeed*>(tile.speeds + index);
        current->breakpoint1 = 255;
        if (std::get<1>(BD) != nullptr && std::get<0>(BD).id() == index) {
          current->overall_speed = 0;
          current->speed1 = 0;
          current->breakpoint1 = 255;
        } else {
          *current = ts;
        }
      };

      valhalla_tests::utils::customize_live_traffic_data(map.config, cb_setter_speed);
    }

    auto clean_reader =
        valhalla_tests::utils::make_clean_graphreader(map.config.get_child("mjolnir"));
    tyr::actor_t actor(map.config, *clean_reader);
    valhalla::Api api;
    actor.route(
        R"({"locations":[
        {"lat":)" +
            std::to_string(map.nodes["C"].second) + R"(,"lon":)" +
            std::to_string(map.nodes["C"].first) +
            R"(},
        {"lat":)" +
            std::to_string(map.nodes["E"].second) + R"(,"lon":)" +
            std::to_string(map.nodes["E"].first) +
            R"(}
      ],"costing":"auto","date_time":{"type":0},
      "filters":{"attributes":["edge.length","edge.speed","edge.begin_shape_index",
      "edge.end_shape_index","shape","shape_attributes.length","shape_attributes.time","shape_attributes.speed"],
      "action":"include"}})",
        nullptr, &api);

    const auto& leg = api.trip().routes(0).legs(0);
    auto shapes = midgard::decode<std::vector<valhalla::midgard::PointLL>>(leg.shape());
    EXPECT_EQ(leg.node_size(), 2); // 1 edge
    EXPECT_EQ(shapes.size(), 4);
    // An attribute for each pair formed by the shape-points
    EXPECT_EQ(leg.shape_attributes().time_size(), shapes.size() - 1);
    EXPECT_EQ(leg.shape_attributes().length_size(), shapes.size() - 1);
    EXPECT_EQ(leg.shape_attributes().speed_size(), shapes.size() - 1);

    {
      auto b1 = map.nodes["C"].PointAlongSegment(map.nodes["E"], 100 / 255.0);
      EXPECT_TRUE(b1.ApproximatelyEqual(shapes[1]));
    }
    {
      auto b2 = map.nodes["C"].PointAlongSegment(map.nodes["E"], 200 / 255.0);
      EXPECT_TRUE(b2.ApproximatelyEqual(shapes[2]));
    }
  }

  // Another permutation of subsegments
  {
    {
      valhalla::baldr::TrafficSpeed ts{valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                       valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                       valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                       valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                       0u,
                                       0u,
                                       0u,
                                       0u,
                                       0u};
      ts.overall_speed = 36 >> 1;

      ts.speed1 = 20 >> 1;
      ts.congestion1 = baldr::MAX_CONGESTION_VAL - 1;
      ts.breakpoint1 = 100;

      ts.speed2 = 40 >> 1;
      ts.congestion2 = 1;
      ts.breakpoint2 = 200;

      ts.speed3 = 60 >> 1;
      ts.congestion3 = 1;
      std::function<void(baldr::GraphReader&, baldr::TrafficTile&, int, baldr::TrafficSpeed*)>
          cb_setter_speed = [&map, &ts](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                        int index, baldr::TrafficSpeed* current) -> void {
        baldr::GraphId tile_id(tile.header->tile_id);
        auto BD = gurka::findEdge(reader, map.nodes, "BD", "D", tile_id);
        current->breakpoint1 = 255;
        if (std::get<1>(BD) != nullptr && std::get<0>(BD).id() == index) {
          current->overall_speed = 0;
          current->speed1 = 0;
          current->breakpoint1 = 255;
        } else {
          *current = ts;
        }
      };

      valhalla_tests::utils::customize_live_traffic_data(map.config, cb_setter_speed);
    }

    auto clean_reader =
        valhalla_tests::utils::make_clean_graphreader(map.config.get_child("mjolnir"));
    tyr::actor_t actor(map.config, *clean_reader);
    valhalla::Api api;
    {
      // Test the full edge CE
      actor.route(
          R"({"locations":[
        {"lat":)" +
              std::to_string(map.nodes["C"].second) + R"(,"lon":)" +
              std::to_string(map.nodes["C"].first) +
              R"(},
        {"lat":)" +
              std::to_string(map.nodes["E"].second) + R"(,"lon":)" +
              std::to_string(map.nodes["E"].first) +
              R"(}
      ],"costing":"auto","date_time":{"type":0},
      "filters":{"attributes":["edge.length","edge.speed","edge.begin_shape_index",
      "edge.end_shape_index","shape","shape_attributes.length","shape_attributes.time","shape_attributes.speed"],
      "action":"include"}})",
          nullptr, &api);

      const auto& leg = api.trip().routes(0).legs(0);
      auto shapes = midgard::decode<std::vector<valhalla::midgard::PointLL>>(leg.shape());
      EXPECT_EQ(leg.node_size(), 2); // 1 edge
      EXPECT_EQ(shapes.size(), 4);
      // An attribute for each pair formed by the shape-points
      EXPECT_EQ(leg.shape_attributes().time_size(), shapes.size() - 1);
      EXPECT_EQ(leg.shape_attributes().length_size(), shapes.size() - 1);
      EXPECT_EQ(leg.shape_attributes().speed_size(), shapes.size() - 1);

      { EXPECT_TRUE(map.nodes["C"].ApproximatelyEqual(shapes[0])); }
      {
        auto b1 = map.nodes["C"].PointAlongSegment(map.nodes["E"], 100 / 255.0);
        EXPECT_TRUE(b1.ApproximatelyEqual(shapes[1]));
      }
      {
        auto b2 = map.nodes["C"].PointAlongSegment(map.nodes["E"], 200 / 255.0);
        EXPECT_TRUE(b2.ApproximatelyEqual(shapes[2]));
      }
    }
    {
      // Test partial CE
      actor.route(
          R"({"locations":[
        {"lat":)" +
              std::to_string(map.nodes["1"].second) + R"(,"lon":)" +
              std::to_string(map.nodes["1"].first) +
              R"(},
        {"lat":)" +
              std::to_string(map.nodes["E"].second) + R"(,"lon":)" +
              std::to_string(map.nodes["E"].first) +
              R"(}
      ],"costing":"auto","date_time":{"type":0},
      "filters":{"attributes":["edge.length","edge.speed","edge.begin_shape_index",
      "edge.end_shape_index","shape","shape_attributes.length","shape_attributes.time","shape_attributes.speed"],
      "action":"include"}})",
          nullptr, &api);

      const auto& leg = api.trip().routes(0).legs(0);
      auto shapes = midgard::decode<std::vector<valhalla::midgard::PointLL>>(leg.shape());
      EXPECT_EQ(leg.node_size(), 2); // 1 edge
      EXPECT_EQ(shapes.size(), 4);
      // An attribute for each pair formed by the shape-points
      EXPECT_EQ(leg.shape_attributes().time_size(), shapes.size() - 1);
      EXPECT_EQ(leg.shape_attributes().length_size(), shapes.size() - 1);
      EXPECT_EQ(leg.shape_attributes().speed_size(), shapes.size() - 1);
      EXPECT_TRUE(map.nodes["1"].ApproximatelyEqual(shapes[0]));
      {
        auto b1 = map.nodes["C"].PointAlongSegment(map.nodes["E"], 100 / 255.0);
        EXPECT_TRUE(b1.ApproximatelyEqual(shapes[1]));
      }
      {
        auto b2 = map.nodes["C"].PointAlongSegment(map.nodes["E"], 200 / 255.0);
        EXPECT_TRUE(b2.ApproximatelyEqual(shapes[2]));
      }
    }
    {
      {
        valhalla::baldr::TrafficSpeed ts{valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                         valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                         valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                         valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
                                         0u,
                                         0u,
                                         0u,
                                         0u,
                                         0u};
        ts.overall_speed = 36 >> 1;

        ts.speed1 = 20 >> 1;
        ts.congestion1 = baldr::MAX_CONGESTION_VAL - 1;
        ts.breakpoint1 = 100;

        ts.speed2 = 40 >> 1;
        ts.congestion2 = 1;
        ts.breakpoint2 = 200;

        ts.speed3 = 60 >> 1;
        ts.congestion3 = 50;
        std::function<void(baldr::GraphReader&, baldr::TrafficTile&, int, baldr::TrafficSpeed*)>
            cb_setter_speed = [&map, &ts](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                          int index, baldr::TrafficSpeed* current) -> void {
          baldr::GraphId tile_id(tile.header->tile_id);
          auto BD = gurka::findEdge(reader, map.nodes, "BD", "D", tile_id);
          current->breakpoint1 = 255;
          if (std::get<1>(BD) != nullptr && std::get<0>(BD).id() == index) {
            current->overall_speed = 0;
            current->speed1 = 0;
          } else {
            *current = ts;
          }
        };

        valhalla_tests::utils::customize_live_traffic_data(map.config, cb_setter_speed);
      }

      auto clean_reader =
          valhalla_tests::utils::make_clean_graphreader(map.config.get_child("mjolnir"));
      tyr::actor_t actor(map.config, *clean_reader);
      valhalla::Api api;
      {
        // Test the full edge CE
        actor.route(
            R"({"locations":[
        {"lat":)" +
                std::to_string(map.nodes["C"].second) + R"(,"lon":)" +
                std::to_string(map.nodes["C"].first) +
                R"(},
        {"lat":)" +
                std::to_string(map.nodes["E"].second) + R"(,"lon":)" +
                std::to_string(map.nodes["E"].first) +
                R"(}
      ],"costing":"auto","date_time":{"type":0},
      "filters":{"attributes":["edge.length","edge.speed","edge.begin_shape_index",
      "edge.end_shape_index","shape","shape_attributes.length","shape_attributes.time","shape_attributes.speed"],
      "action":"include"}})",
            nullptr, &api);

        const auto& leg = api.trip().routes(0).legs(0);
        auto shapes = midgard::decode<std::vector<valhalla::midgard::PointLL>>(leg.shape());

        EXPECT_EQ(leg.node_size(), 2); // 1 edge
        EXPECT_EQ(shapes.size(), 4);
        // An attribute for each pair formed by the shape-points
        EXPECT_EQ(leg.shape_attributes().time_size(), shapes.size() - 1);
        EXPECT_EQ(leg.shape_attributes().length_size(), shapes.size() - 1);
        EXPECT_EQ(leg.shape_attributes().speed_size(), shapes.size() - 1);

        EXPECT_TRUE(map.nodes["C"].ApproximatelyEqual(shapes[0]));
        {
          auto b1 = map.nodes["C"].PointAlongSegment(map.nodes["E"], 100 / 255.0);
          EXPECT_TRUE(b1.ApproximatelyEqual(shapes[1]));
        }
        {
          auto b2 = map.nodes["C"].PointAlongSegment(map.nodes["E"], 200 / 255.0);
          EXPECT_TRUE(b2.ApproximatelyEqual(shapes[2]));
        }
        EXPECT_TRUE(map.nodes["E"].ApproximatelyEqual(shapes[3]));
      }
      {
        // Test partial CE
        actor.route(
            R"({"locations":[
        {"lat":)" +
                std::to_string(map.nodes["1"].second) + R"(,"lon":)" +
                std::to_string(map.nodes["1"].first) +
                R"(},
        {"lat":)" +
                std::to_string(map.nodes["E"].second) + R"(,"lon":)" +
                std::to_string(map.nodes["E"].first) +
                R"(}
      ],"costing":"auto","date_time":{"type":0},
      "filters":{"attributes":["edge.length","edge.speed","edge.begin_shape_index",
      "edge.end_shape_index","shape","shape_attributes.length","shape_attributes.time","shape_attributes.speed"],
      "action":"include"}})",
            nullptr, &api);

        const auto& leg = api.trip().routes(0).legs(0);
        auto shapes = midgard::decode<std::vector<valhalla::midgard::PointLL>>(leg.shape());

        EXPECT_EQ(leg.node_size(), 2); // 1 edge
        EXPECT_EQ(shapes.size(), 4);
        // An attribute for each pair formed by the shape-points
        EXPECT_EQ(leg.shape_attributes().time_size(), shapes.size() - 1);
        EXPECT_EQ(leg.shape_attributes().length_size(), shapes.size() - 1);
        EXPECT_EQ(leg.shape_attributes().speed_size(), shapes.size() - 1);

        { EXPECT_TRUE(map.nodes["1"].ApproximatelyEqual(shapes[0])); }
        {
          auto b1 = map.nodes["C"].PointAlongSegment(map.nodes["E"], 100 / 255.0);
          EXPECT_TRUE(b1.ApproximatelyEqual(shapes[1]));
        }
        {
          auto b2 = map.nodes["C"].PointAlongSegment(map.nodes["E"], 200 / 255.0);
          EXPECT_TRUE(b2.ApproximatelyEqual(shapes[2]));
        }
      }
      {
        // Test multishape FE
        actor.route(
            R"({"locations":[
        {"lat":)" +
                std::to_string(map.nodes["F"].second) + R"(,"lon":)" +
                std::to_string(map.nodes["F"].first) +
                R"(},
        {"lat":)" +
                std::to_string(map.nodes["I"].second) + R"(,"lon":)" +
                std::to_string(map.nodes["I"].first) +
                R"(}
      ],"costing":"auto","date_time":{"type":0},
      "filters":{"attributes":["edge.length","edge.speed","edge.begin_shape_index",
      "edge.end_shape_index","shape","shape_attributes.length","shape_attributes.time","shape_attributes.speed"],
      "action":"include"}})",
            nullptr, &api);

        const auto& leg = api.trip().routes(0).legs(0);
        auto shapes = midgard::decode<std::vector<valhalla::midgard::PointLL>>(leg.shape());
        EXPECT_EQ(leg.node_size(), 2); // 1 edge
        EXPECT_EQ(shapes.size(), 6);
        // An attribute for each pair formed by the shape-points
        EXPECT_EQ(leg.shape_attributes().time_size(), shapes.size() - 1);
        EXPECT_EQ(leg.shape_attributes().length_size(), shapes.size() - 1);
        EXPECT_EQ(leg.shape_attributes().speed_size(), shapes.size() - 1);

        EXPECT_TRUE(map.nodes["F"].ApproximatelyEqual(shapes[0]));
        {
          auto b1 = map.nodes["F"].PointAlongSegment(map.nodes["I"], 100 / 255.0);
          EXPECT_TRUE(b1.ApproximatelyEqual(shapes[1]));
        }
        EXPECT_TRUE(map.nodes["G"].ApproximatelyEqual(shapes[2]));
        {
          auto b2 = map.nodes["F"].PointAlongSegment(map.nodes["I"], 200 / 255.0);
          EXPECT_TRUE(b2.ApproximatelyEqual(shapes[3]));
        }
        EXPECT_TRUE(map.nodes["H"].ApproximatelyEqual(shapes[4]));
        EXPECT_TRUE(map.nodes["I"].ApproximatelyEqual(shapes[5]));
      }
      {
        // Test partial multishape 2->E
        actor.route(
            R"({"locations":[
        {"lat":)" +
                std::to_string(map.nodes["2"].second) + R"(,"lon":)" +
                std::to_string(map.nodes["2"].first) +
                R"(},
        {"lat":)" +
                std::to_string(map.nodes["E"].second) + R"(,"lon":)" +
                std::to_string(map.nodes["E"].first) +
                R"(}
      ],"costing":"auto","date_time":{"type":0},
      "filters":{"attributes":["edge.length","edge.speed","edge.begin_shape_index",
      "edge.end_shape_index","shape","shape_attributes.length","shape_attributes.time","shape_attributes.speed"],
      "action":"include"}})",
            nullptr, &api);

        const auto& leg = api.trip().routes(0).legs(0);
        auto shapes = midgard::decode<std::vector<valhalla::midgard::PointLL>>(leg.shape());
        //{
        //  std::string buf;
        //  google::protobuf::util::JsonPrintOptions opt;
        //  opt.add_whitespace = true;
        //  google::protobuf::util::MessageToJsonString(leg, &buf, opt);
        //  std::cout << buf << std::endl;

        //  auto node_of_interest = "F";
        //  std::cout << "node['" << node_of_interest << "'] "
        //            << std::to_string(map.nodes[node_of_interest].first) << ", "
        //            << std::to_string(map.nodes[node_of_interest].second) << std::endl;
        //  node_of_interest = "I";
        //  std::cout << "node['" << node_of_interest << "'] "
        //            << std::to_string(map.nodes[node_of_interest].first) << ", "
        //            << std::to_string(map.nodes[node_of_interest].second) << std::endl;
        //  int i=0;
        //  for (auto& shape : shapes) {

        //    std::cout << "shape #" <<std::to_string(i) << ": "<< std::to_string(shape.first) << ", "
        //              << std::to_string(shape.second) << std::endl;
        //    ++i;
        //  }
        //}
        EXPECT_EQ(leg.node_size(), 3); // FI + IE
        EXPECT_EQ(shapes.size(), 9);
        // An attribute for each pair formed by the shape-points
        EXPECT_EQ(leg.shape_attributes().time_size(), shapes.size() - 1);
        EXPECT_EQ(leg.shape_attributes().length_size(), shapes.size() - 1);
        EXPECT_EQ(leg.shape_attributes().speed_size(), shapes.size() - 1);

        EXPECT_TRUE(map.nodes["2"].ApproximatelyEqual(shapes[0]));
        {
          auto b1 = map.nodes["F"].PointAlongSegment(map.nodes["I"], 100 / 255.0);
          EXPECT_TRUE(b1.ApproximatelyEqual(shapes[1]));
        }
        EXPECT_TRUE(map.nodes["G"].ApproximatelyEqual(shapes[2]));
        {
          auto b2 = map.nodes["F"].PointAlongSegment(map.nodes["I"], 200 / 255.0);
          EXPECT_TRUE(b2.ApproximatelyEqual(shapes[3]));
        }
        EXPECT_TRUE(map.nodes["H"].ApproximatelyEqual(shapes[4]));
        EXPECT_TRUE(map.nodes["I"].ApproximatelyEqual(shapes[5]));
        {
          auto b2 = map.nodes["I"].PointAlongSegment(map.nodes["E"], 100 / 255.0);
          EXPECT_TRUE(b2.ApproximatelyEqual(shapes[6]));
        }
        {
          auto b2 = map.nodes["I"].PointAlongSegment(map.nodes["E"], 200 / 255.0);
          EXPECT_TRUE(b2.ApproximatelyEqual(shapes[7]));
        }
        EXPECT_TRUE(map.nodes["E"].ApproximatelyEqual(shapes[8]));
      }
    }
  }
}
