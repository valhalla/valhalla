#include "gurka.h"
#include "test.h"

#include "baldr/graphreader.h"
#include "baldr/traffictile.h"

#include <cmath>
#include <sys/mman.h>
#include <sys/stat.h>

using namespace valhalla;
using LiveTrafficCustomize = test::LiveTrafficCustomize;

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

  test::build_live_traffic_data(map.config);

  auto clean_reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  std::cout << "[          ] Do a route with initial traffic" << std::endl;
  {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "auto",
                                   {{"/date_time/type", "0"}}, clean_reader);
    gurka::assert::osrm::expect_steps(result, {"AB"});
    gurka::assert::raw::expect_path(result, {"AB", "BC"});
    gurka::assert::raw::expect_eta(result, 360.0177);
  }
  std::cout << "[          ] Make some updates to the traffic .tar file. "
               "Mostly just updates every edge in the file to 24km/h, except for one "
               "specific edge (B->D) where we simulate a closure (speed=0, congestion high)"
            << std::endl;
  auto cb_setter_24kmh = [&map](baldr::GraphReader& reader, baldr::TrafficTile& tile, uint32_t index,
                                valhalla::baldr::TrafficSpeed* current) -> void {
    baldr::GraphId tile_id(tile.header->tile_id);
    auto BD = gurka::findEdge(reader, map.nodes, "BD", "D", tile_id);
    current->breakpoint1 = 255;
    if (std::get<1>(BD) != nullptr && std::get<0>(BD).id() == index) {
      current->overall_encoded_speed = 0;
      current->encoded_speed1 = 0;
    } else {
      current->overall_encoded_speed = 24 >> 1;
      current->encoded_speed1 = 24 >> 1;
    }
  };
  test::customize_live_traffic_data(map.config, cb_setter_24kmh);

  std::cout << "[          ] Now do another route with the same (not restarted) actor to see if"
               " it's noticed the changes in the live traffic file"
            << std::endl;
  {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "auto",
                                   {{"/date_time/type", "0"}}, clean_reader);
    gurka::assert::osrm::expect_steps(result, {"AB"});
    gurka::assert::raw::expect_path(result, {"AB", "BC"});
    // live-traffic is mixed with default traffic
    gurka::assert::raw::expect_eta(result, 153.27858, 0.5);
  }

  std::cout << "[          ] Next, set the speed to the highest possible to ensure nothing breaks"
            << std::endl;
  auto cb_setter_max = [&map](baldr::GraphReader& reader, baldr::TrafficTile& tile, uint32_t index,
                              baldr::TrafficSpeed* current) -> void {
    baldr::GraphId tile_id(tile.header->tile_id);
    auto BD = gurka::findEdge(reader, map.nodes, "BD", "D", tile_id);
    current->breakpoint1 = 255;
    if (std::get<1>(BD) != nullptr && std::get<0>(BD).id() == index) {
      current->overall_encoded_speed = 0;
    } else {
      current->overall_encoded_speed = UNKNOWN_TRAFFIC_SPEED_RAW - 1;
    }
  };
  test::customize_live_traffic_data(map.config, cb_setter_max);
  {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "auto",
                                   {{"/date_time/type", "0"}}, clean_reader);
    gurka::assert::osrm::expect_steps(result, {"AB"});
    gurka::assert::raw::expect_path(result, {"AB", "BC"});
    gurka::assert::raw::expect_eta(result, 25.731991);
  }

  std::cout << "[          ] Back to previous speed" << std::endl;
  test::customize_live_traffic_data(map.config, cb_setter_24kmh);
  // And verify that without the "current" timestamp, the live traffic
  // results aren't used
  {
    auto result =
        gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "auto", {}, clean_reader);
    gurka::assert::osrm::expect_steps(result, {"AB"});
    gurka::assert::raw::expect_path(result, {"AB", "BC"});
    gurka::assert::raw::expect_eta(result, 360.0177);
  }

  std::cout << "[          ] Now do another route with the same (not restarted) actor to see if "
               "it's noticed the changes in the live traffic file"
            << std::endl;
  {
    auto result = gurka::do_action(valhalla::Options::route, map, {"B", "D"}, "auto",
                                   {{"/date_time/type", "0"}}, clean_reader);
    gurka::assert::osrm::expect_steps(result, {"BC", "CE", "DE"});
    // BD is excluded
    gurka::assert::raw::expect_path(result, {"BC", "CE", "DE"});
    // live-traffic is mixed with default traffic.
    gurka::assert::raw::expect_eta(result, 184.5, 0.5);
  }
  {
    auto result = gurka::do_action(valhalla::Options::route, map, {"D", "B"}, "auto",
                                   {{"/date_time/type", "0"}}, clean_reader);
    gurka::assert::osrm::expect_steps(result, {"BD"});
    gurka::assert::raw::expect_path(result, {"BD"});
    gurka::assert::raw::expect_eta(result, 30., 0.01);
  }

  std::cout
      << "[          ] Repeat the B->D route, but this time with no timestamp - this should disable "
         "using live traffc and the road should be open again"
      << std::endl;
  {
    auto result =
        gurka::do_action(valhalla::Options::route, map, {"B", "D"}, "auto", {}, clean_reader);
    gurka::assert::osrm::expect_steps(result, {"BD"});
    gurka::assert::raw::expect_path(result, {"BD"});
    gurka::assert::raw::expect_eta(result, 72);
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
  test::build_live_traffic_data(map.config);

  // first we get the edge without traffic on it
  {
    auto clean_reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));

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
    valhalla::baldr::TrafficSpeed ts{
        valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
        valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
        valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
        valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
        0u,
        0u,
        0u,
        0u,
        0u,
        0u,
    };
    ts.overall_encoded_speed = 42 >> 1;
    ts.encoded_speed1 = 42 >> 1;
    ts.congestion1 = baldr::MAX_CONGESTION_VAL - 1;
    ts.breakpoint1 = 127;

    auto cb_setter_speed = [&map, &ts](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                       uint32_t index, baldr::TrafficSpeed* current) -> void {
      baldr::GraphId tile_id(tile.header->tile_id);
      auto BD = gurka::findEdge(reader, map.nodes, "BD", "D", tile_id);
      current->breakpoint1 = 255;
      if (std::get<1>(BD) != nullptr && std::get<0>(BD).id() == index) {
        current->overall_encoded_speed = 0;
        current->encoded_speed1 = 0;
        current->breakpoint1 = 255;
      } else {
        *current = ts;
      }
    };

    test::customize_live_traffic_data(map.config, cb_setter_speed);

    auto clean_reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
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
      valhalla::baldr::TrafficSpeed ts{
          valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
          valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
          valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
          valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
          0u,
          0u,
          0u,
          0u,
          0u,
          0u,
      };
      ts.overall_encoded_speed = 30 >> 1;

      ts.encoded_speed1 = 20 >> 1;
      ts.congestion1 = baldr::MAX_CONGESTION_VAL - 1;
      ts.breakpoint1 = 100;

      ts.encoded_speed2 = 40 >> 1;
      ts.congestion2 = 1; // low but not unknown - 0 = unknown
      ts.breakpoint2 = 200;

      auto cb_setter_speed = [&map, &ts](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
        baldr::GraphId tile_id(tile.header->tile_id);
        auto BD = gurka::findEdge(reader, map.nodes, "BD", "D", tile_id);
        current->breakpoint1 = 255;
        if (std::get<1>(BD) != nullptr && std::get<0>(BD).id() == index) {
          current->overall_encoded_speed = 0;
          current->encoded_speed1 = 0;
        } else {
          *current = ts;
        }
      };

      test::customize_live_traffic_data(map.config, cb_setter_speed);
    }

    auto clean_reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
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
      valhalla::baldr::TrafficSpeed ts{
          valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
          valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
          valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
          valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
          0u,
          0u,
          0u,
          0u,
          0u,
          0u,
      };
      ts.overall_encoded_speed = 36 >> 1;

      ts.encoded_speed1 = 20 >> 1;
      ts.congestion1 = baldr::MAX_CONGESTION_VAL - 1;
      ts.breakpoint1 = 100;

      ts.encoded_speed2 = 40 >> 1;
      ts.congestion2 = 1;
      ts.breakpoint2 = 200;

      ts.encoded_speed3 = 60 >> 1;
      ts.congestion3 = 1;

      auto cb_setter_speed = [&map, &ts](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
        baldr::GraphId tile_id(tile.header->tile_id);
        auto BD = gurka::findEdge(reader, map.nodes, "BD", "D", tile_id);
        current->breakpoint1 = 255;
        if (std::get<1>(BD) != nullptr && std::get<0>(BD).id() == index) {
          current->overall_encoded_speed = 0;
          current->encoded_speed1 = 0;
        } else {
          *current = ts;
        }
      };

      test::customize_live_traffic_data(map.config, cb_setter_speed);
    }

    auto clean_reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
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
        valhalla::baldr::TrafficSpeed ts{
            valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
            valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
            valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
            valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
            0u,
            0u,
            0u,
            0u,
            0u,
            0u,
        };
        ts.overall_encoded_speed = 36 >> 1;

        ts.encoded_speed1 = 20 >> 1;
        ts.congestion1 = baldr::MAX_CONGESTION_VAL - 1;
        ts.breakpoint1 = 100;

        ts.encoded_speed2 = 40 >> 1;
        ts.congestion2 = 1;
        ts.breakpoint2 = 200;

        ts.encoded_speed3 = 60 >> 1;
        ts.congestion3 = 50;

        auto cb_setter_speed = [&map, &ts](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                           uint32_t index, baldr::TrafficSpeed* current) -> void {
          baldr::GraphId tile_id(tile.header->tile_id);
          auto BD = gurka::findEdge(reader, map.nodes, "BD", "D", tile_id);
          current->breakpoint1 = 255;
          if (std::get<1>(BD) != nullptr && std::get<0>(BD).id() == index) {
            current->overall_encoded_speed = 0;
            current->encoded_speed1 = 0;
          } else {
            *current = ts;
          }
        };

        test::customize_live_traffic_data(map.config, cb_setter_speed);
      }

      auto clean_reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
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
    // Test regression when cutting
    {
      {
        valhalla::baldr::TrafficSpeed ts{
            valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
            valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
            valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
            valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
            0u,
            0u,
            0u,
            0u,
            0u,
            0u,
        };
        ts.overall_encoded_speed = 36 >> 1;

        ts.encoded_speed1 = 20 >> 1;
        ts.congestion1 = baldr::MAX_CONGESTION_VAL - 1;
        ts.breakpoint1 = 100;

        ts.encoded_speed2 = 40 >> 1;
        ts.congestion2 = 1;

        // Regression is when breakpoint2 is set to 255,
        // but speed3 is not set to UNKNOWN - we would incorrectly
        // emit an additional 0-length slice in the API response
        ts.breakpoint2 = 255;

        // This needs to be set
        ts.encoded_speed3 = 60 >> 1;
        ts.congestion3 = 50;

        auto cb_setter_speed = [&map, &ts](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                           uint32_t index, baldr::TrafficSpeed* current) -> void {
          baldr::GraphId tile_id(tile.header->tile_id);
          auto BD = gurka::findEdge(reader, map.nodes, "BD", "D", tile_id);
          current->breakpoint1 = 255;
          if (std::get<1>(BD) != nullptr && std::get<0>(BD).id() == index) {
            current->overall_encoded_speed = 0;
            current->encoded_speed1 = 0;
          } else {
            *current = ts;
          }
        };

        test::customize_live_traffic_data(map.config, cb_setter_speed);
      }

      auto clean_reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
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
        EXPECT_EQ(shapes.size(), 3);   // The third record should not create a slice
        // An attribute for each pair formed by the shape-points
        EXPECT_EQ(leg.shape_attributes().time_size(), shapes.size() - 1);
        EXPECT_EQ(leg.shape_attributes().length_size(), shapes.size() - 1);
        EXPECT_EQ(leg.shape_attributes().speed_size(), shapes.size() - 1);

        EXPECT_TRUE(map.nodes["C"].ApproximatelyEqual(shapes[0]));
        {
          auto b1 = map.nodes["C"].PointAlongSegment(map.nodes["E"], 100 / 255.0);
          EXPECT_TRUE(b1.ApproximatelyEqual(shapes[1]));
        }
        EXPECT_TRUE(map.nodes["E"].ApproximatelyEqual(shapes[2]));
      }
    }
  }
}

/*************************************************************/
class WaypointsOnClosuresTest : public ::testing::Test {
protected:
  static gurka::map closure_map;
  static int const default_speed;
  static std::string const tile_dir;
  static std::shared_ptr<baldr::GraphReader> clean_reader;
  ;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
      A-2-------D
      |         |
      |         |
      |         |
      |         |
      B-1-------C
  )";

    const std::string speed_str = std::to_string(default_speed);
    const gurka::ways ways = {{"AB", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"BC", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"CD", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"DA", {{"highway", "primary"}, {"maxspeed", speed_str}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10, {.05f, .2f});
    closure_map = gurka::buildtiles(layout, ways, {}, {}, tile_dir);

    closure_map.config.put("mjolnir.traffic_extract", tile_dir + "/traffic.tar");
    test::build_live_traffic_data(closure_map.config);

    clean_reader = test::make_clean_graphreader(closure_map.config.get_child("mjolnir"));
  }
};

gurka::map WaypointsOnClosuresTest::closure_map = {};
const int WaypointsOnClosuresTest::default_speed = 36;
const std::string WaypointsOnClosuresTest::tile_dir = "test/data/traffic_waypoints_on_closures";
std::shared_ptr<baldr::GraphReader> WaypointsOnClosuresTest::clean_reader;

namespace {
inline void SetLiveSpeed(baldr::TrafficSpeed* live_speed, uint64_t speed) {
  live_speed->breakpoint1 = 255;
  live_speed->overall_encoded_speed = speed >> 1;
  live_speed->encoded_speed1 = speed >> 1;
}
} // namespace

TEST_F(WaypointsOnClosuresTest, DepartPointAtClosure) {
  const float eta_margin = 0.01f;

  // start from an edge that is closed in a one direction
  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      baldr::GraphId tile_id(tile.header->tile_id);
      auto BC = gurka::findEdge(reader, closure_map.nodes, "BC", "C", tile_id);

      bool const is_bc = (std::get<1>(BC) != nullptr && std::get<0>(BC).id() == index);

      SetLiveSpeed(current, (is_bc ? 0 : default_speed));
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "A"}, "auto",
                                   {{"/date_time/type", "0"}}, clean_reader);
    gurka::assert::raw::expect_path(result, {"BC", "AB"});
    gurka::assert::raw::expect_eta(result, 7.f, eta_margin);
  }

  // start from an edge that is closed in a one direction
  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      baldr::GraphId tile_id(tile.header->tile_id);
      auto CB = gurka::findEdge(reader, closure_map.nodes, "BC", "B", tile_id);

      bool const is_cb = (std::get<1>(CB) != nullptr && std::get<0>(CB).id() == index);

      SetLiveSpeed(current, (is_cb ? 0 : default_speed));
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "A"}, "auto",
                                   {{"/date_time/type", "0"}}, clean_reader);
    gurka::assert::raw::expect_path(result, {"BC", "CD", "DA"});
    gurka::assert::raw::expect_eta(result, 23.f, eta_margin);
  }

  // depart point at closed edge (ignore):
  // - closed edge should be ignored;
  // - depart point should be matched to the nearest node;
  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      baldr::GraphId tile_id(tile.header->tile_id);

      auto BC = gurka::findEdge(reader, closure_map.nodes, "BC", "C", tile_id);
      auto CB = gurka::findEdge(reader, closure_map.nodes, "BC", "B", tile_id);

      bool const is_bc = (std::get<1>(BC) != nullptr && std::get<0>(BC).id() == index);
      bool const is_cb = (std::get<1>(CB) != nullptr && std::get<0>(CB).id() == index);

      SetLiveSpeed(current, ((is_cb || is_bc) ? 0 : default_speed));
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "A"}, "auto",
                                   {{"/date_time/type", "0"}}, clean_reader);
    gurka::assert::raw::expect_path(result, {"AB"});
    gurka::assert::raw::expect_eta(result, 5.f, eta_margin);
  }
}

TEST_F(WaypointsOnClosuresTest, ArrivePointAtClosure) {
  const float eta_margin = 0.01f;

  // end at edge that is closed in a one direction
  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      baldr::GraphId tile_id(tile.header->tile_id);
      auto DA = gurka::findEdge(reader, closure_map.nodes, "DA", "A", tile_id);

      bool const is_da = (std::get<1>(DA) != nullptr && std::get<0>(DA).id() == index);

      SetLiveSpeed(current, (is_da ? 0 : default_speed));
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"B", "2"}, "auto",
                                   {{"/date_time/type", "0"}}, clean_reader);
    gurka::assert::raw::expect_path(result, {"AB", "DA"});
    gurka::assert::raw::expect_eta(result, 7.f, eta_margin);
  }

  // end at edge that is closed in a one direction
  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      baldr::GraphId tile_id(tile.header->tile_id);
      auto AD = gurka::findEdge(reader, closure_map.nodes, "DA", "D", tile_id);

      bool const is_ad = (std::get<1>(AD) != nullptr && std::get<0>(AD).id() == index);

      SetLiveSpeed(current, (is_ad ? 0 : default_speed));
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"B", "2"}, "auto",
                                   {{"/date_time/type", "0"}}, clean_reader);
    gurka::assert::raw::expect_path(result, {"BC", "CD", "DA"});
    gurka::assert::raw::expect_eta(result, 23.f, eta_margin);
  }

  // arrive point at closed edge (ignore):
  // - closed edge should be ignored;
  // - arrive point should be matched to the nearest node;
  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      baldr::GraphId tile_id(tile.header->tile_id);

      auto DA = gurka::findEdge(reader, closure_map.nodes, "DA", "A", tile_id);
      auto AD = gurka::findEdge(reader, closure_map.nodes, "DA", "D", tile_id);

      bool const is_da = (std::get<1>(DA) != nullptr && std::get<0>(DA).id() == index);
      bool const is_ad = (std::get<1>(AD) != nullptr && std::get<0>(AD).id() == index);

      SetLiveSpeed(current, ((is_da || is_ad) ? 0 : default_speed));
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"B", "2"}, "auto",
                                   {{"/date_time/type", "0"}}, clean_reader);
    gurka::assert::raw::expect_path(result, {"AB"});
    gurka::assert::raw::expect_eta(result, 5.f, eta_margin);
  }
}

TEST_F(WaypointsOnClosuresTest, IgnoreDepartPointAtClosure) {
  // the edge is not closed
  {
    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "A"}, "auto",
                                   {{"/date_time/type", "0"}}, clean_reader);
    gurka::assert::raw::expect_path(result, {"BC", "AB"});
  }

  // the edge is closed in both directions
  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      baldr::GraphId tile_id(tile.header->tile_id);
      auto BC = std::get<0>(gurka::findEdge(reader, closure_map.nodes, "BC", "C"));
      auto CB = std::get<0>(gurka::findEdge(reader, closure_map.nodes, "BC", "B"));
      bool should_close = (BC.Tile_Base() == tile_id && BC.id() == index) ||
                          (CB.Tile_Base() == tile_id && CB.id() == index);
      SetLiveSpeed(current, should_close ? 0 : default_speed);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "A"}, "auto",
                                   {{"/date_time/type", "0"}}, clean_reader);
    gurka::assert::raw::expect_path(result, {"AB"});
  }

  // the edge is closed in both directions but you say you dont care
  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      baldr::GraphId tile_id(tile.header->tile_id);
      auto BC = std::get<0>(gurka::findEdge(reader, closure_map.nodes, "BC", "C"));
      auto CB = std::get<0>(gurka::findEdge(reader, closure_map.nodes, "BC", "B"));
      bool should_close = (BC.Tile_Base() == tile_id && BC.id() == index) ||
                          (CB.Tile_Base() == tile_id && CB.id() == index);
      SetLiveSpeed(current, should_close ? 0 : default_speed);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    auto result =
        gurka::do_action(valhalla::Options::route, closure_map, {"1", "A"}, "auto",
                         {{"/date_time/type", "0"}, {"/costing_options/auto/ignore_closures", "1"}},
                         clean_reader);
    gurka::assert::raw::expect_path(result, {"BC", "AB"});
  }
}
