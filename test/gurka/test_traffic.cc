#include "gurka.h"
#include "microtar.h"
#include <gtest/gtest.h>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/traffictile.h>

#include <boost/property_tree/ptree.hpp>

#include <cmath>
#include <sstream>
#include <sys/mman.h>
#include <sys/stat.h>

using namespace valhalla;

/** Copy of raw header for use with sizeof() **/
typedef struct {
  char name[100];
  char mode[8];
  char owner[8];
  char group[8];
  char size[12];
  char mtime[12];
  char checksum[8];
  char type;
  char linkname[100];
  char _padding[255];
} mtar_raw_header_t_;

/*************************************************************/

// Helper function for updating the BD edge in the following test
void update_all_edges_but_bd(const valhalla::gurka::map& map,
                             const valhalla::baldr::TrafficSpeed& new_speed) {
  int fd = open(map.config.get<std::string>("mjolnir.traffic_extract").c_str(), O_RDWR);
  struct stat s;
  fstat(fd, &s);

  mtar_t tar;
  tar.pos = 0;
  tar.stream = mmap(0, s.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  tar.read = [](mtar_t* tar, void* data, unsigned size) -> int {
    memcpy(data, reinterpret_cast<char*>(tar->stream) + tar->pos, size);
    return MTAR_ESUCCESS;
  };
  tar.write = [](mtar_t* tar, const void* data, unsigned size) -> int {
    memcpy(reinterpret_cast<char*>(tar->stream) + tar->pos, data, size);
    return MTAR_ESUCCESS;
  };
  tar.seek = [](mtar_t* tar, unsigned pos) -> int { return MTAR_ESUCCESS; };
  tar.close = [](mtar_t* tar) -> int { return MTAR_ESUCCESS; };

  // Read every speed tile, and update it with fixed speed of `new_speed` km/h (original speeds are
  // 10km/h)
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  mtar_header_t tar_header;
  while ((mtar_read_header(&tar, &tar_header)) != MTAR_ENULLRECORD) {
    baldr::TrafficTile tile(reinterpret_cast<char*>(tar.stream) + tar.pos +
                            sizeof(mtar_raw_header_t_));

    baldr::GraphId tile_id(tile.header->tile_id);
    auto BD = gurka::findEdge(reader, map.nodes, "BD", "D", tile_id);

    for (int index = 0; index < tile.header->directed_edge_count; index++) {
      if (std::get<1>(BD) != nullptr && std::get<0>(BD).id() == index) {
        (tile.speeds + index)->overall_speed = 0;
        (tile.speeds + index)->speed1 = 0;
        (tile.speeds + index)->breakpoint1 = 255;
      } else {
        valhalla::baldr::TrafficSpeed* existing =
            const_cast<valhalla::baldr::TrafficSpeed*>(tile.speeds + index);
        *existing = new_speed;
      }
    }
    mtar_next(&tar);
  }
  munmap(tar.stream, s.st_size);
  close(fd);
}
void update_all_edges_but_bd(const valhalla::gurka::map& map, uint16_t new_speed) {
  valhalla::baldr::TrafficSpeed ts{};
  ts.speed1 = new_speed >> 1;
  ts.overall_speed = new_speed >> 1;
  ts.breakpoint1 = 255;
  update_all_edges_but_bd(map, ts);
}

void blank_traffic(const valhalla::gurka::map& map,
                   uint32_t traffic_tile_version = valhalla::baldr::TRAFFIC_TILE_VERSION) {
  const auto& traffic_extract = map.config.get<std::string>("mjolnir.traffic_extract");
  mtar_t tar;
  auto tar_open_result = mtar_open(&tar, traffic_extract.c_str(), "w");
  ASSERT_EQ(tar_open_result, MTAR_ESUCCESS);

  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  auto tile_ids = reader.GetTileSet();
  // Traffic data works like this:
  //   1. There is a separate .tar file containing tile entries matching the main tiles
  //   2. Each tile is a fixed-size, with a header, and entries
  // This loop iterates ofer the routing tiles, and creates blank
  // traffic tiles with empty records.
  // Valhalla mmap()'s this file and reads from it during route calculation.
  // This loop below creates initial .tar file entries .  Lower down, we make changes to
  // values within the generated traffic tiles and test that routes reflect those changes
  // as expected.
  for (auto tile_id : tile_ids) {
    auto tile = reader.GetGraphTile(tile_id);
    std::stringstream buffer;
    baldr::TrafficTileHeader header = {};
    header.tile_id = tile_id;
    header.traffic_tile_version = traffic_tile_version;
    std::vector<baldr::TrafficSpeed> speeds;
    header.directed_edge_count = tile->header()->directededgecount();
    buffer.write(reinterpret_cast<char*>(&header), sizeof(header));
    baldr::TrafficSpeed dummy_speed =
        {valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW, valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
         valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW,
         valhalla::baldr::UNKNOWN_TRAFFIC_SPEED_RAW}; // Initialize break points to 0s and speeds to
                                                      // unknown
    for (int i = 0; i < header.directed_edge_count; ++i) {
      buffer.write(reinterpret_cast<char*>(&dummy_speed), sizeof(dummy_speed));
    }
    uint32_t dummy_uint32 = 0;
    buffer.write(reinterpret_cast<char*>(&dummy_uint32), sizeof(dummy_uint32));
    buffer.write(reinterpret_cast<char*>(&dummy_uint32), sizeof(dummy_uint32));

    /* Write strings to files `test1.txt` and `test2.txt` */
    std::string blanktile = buffer.str();
    std::string filename = valhalla::baldr::GraphTile::FileSuffix(tile_id);
    auto e1 = mtar_write_file_header(&tar, filename.c_str(), blanktile.size());
    EXPECT_EQ(e1, MTAR_ESUCCESS);
    auto e2 = mtar_write_data(&tar, blanktile.c_str(), blanktile.size());
    EXPECT_EQ(e2, MTAR_ESUCCESS);
  }

  mtar_finalize(&tar);
  mtar_close(&tar);
}

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
  blank_traffic(map);

  {
    auto clean_reader = gurka::make_clean_graphreader(map.config.get_child("mjolnir"));
    // Do a route with initial traffic
    {
      auto result = gurka::route(map, "A", "C", "auto", "current", clean_reader);
      gurka::assert::osrm::expect_steps(result, {"AB"});
      gurka::assert::raw::expect_path(result, {"AB", "BC"});
      gurka::assert::raw::expect_eta(result, 361.5);
    }
    // Make some updates to the traffic .tar file.
    // Mostly just updates every edge in the file to 25km/h, except for one
    // specific edge (B->D) where we simulate a closure (speed=0, congestion high)
    update_all_edges_but_bd(map, 25);

    // Now do another route with the same (not restarted) actor to see if
    // it's noticed the changes in the live traffic file
    {
      auto result = gurka::route(map, "A", "C", "auto", "current", clean_reader);
      gurka::assert::osrm::expect_steps(result, {"AB"});
      gurka::assert::raw::expect_path(result, {"AB", "BC"});
      gurka::assert::raw::expect_eta(result, 151.5);
    }
    // Next, set the speed to the highest possible to ensure nothing breaks
    update_all_edges_but_bd(map, valhalla::baldr::kMaxSpeedKph);
    {
      auto result = gurka::route(map, "A", "C", "auto", "current", clean_reader);
      gurka::assert::osrm::expect_steps(result, {"AB"});
      gurka::assert::raw::expect_path(result, {"AB", "BC"});
      gurka::assert::raw::expect_eta(result, 15.785715);
    }
    // Back to previous speed
    update_all_edges_but_bd(map, 25);
    // And verify that without the "current" timestamp, the live traffic
    // results aren't used
    {
      auto result = gurka::route(map, "A", "C", "auto", "", clean_reader);
      gurka::assert::osrm::expect_steps(result, {"AB"});
      gurka::assert::raw::expect_path(result, {"AB", "BC"});
      gurka::assert::raw::expect_eta(result, 361.5);
    }
    // Now do another route with the same (not restarted) actor to see if
    // it's noticed the changes in the live traffic file
    {
      auto result = gurka::route(map, "B", "D", "auto", "current", clean_reader);
      gurka::assert::osrm::expect_steps(result, {"BC", "CE", "DE"});
      gurka::assert::raw::expect_path(result, {"BC", "CE", "DE"});
      gurka::assert::raw::expect_eta(result, 180., 0.01);
    }
    {
      auto result = gurka::route(map, "D", "B", "auto", "current", clean_reader);
      gurka::assert::osrm::expect_steps(result, {"BD"});
      gurka::assert::raw::expect_path(result, {"BD"});
      gurka::assert::raw::expect_eta(result, 30., 0.01);
    }
    // Repeat the B->D route, but this time with no timestamp - this should
    // disable using live traffc and the road should be open again.
    {
      auto result = gurka::route(map, "B", "D", "auto", "", clean_reader);
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
  blank_traffic(map);

  // first we get the edge without traffic on it
  {
    auto clean_reader = gurka::make_clean_graphreader(map.config.get_child("mjolnir"));
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
                                     0u,
                                     0u};
    ts.overall_speed = 42 >> 1;
    ts.speed1 = 42 >> 1;
    ts.breakpoint1 = 127;
    update_all_edges_but_bd(map, ts);

    auto clean_reader = gurka::make_clean_graphreader(map.config.get_child("mjolnir"));
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
                                       0u,
                                       0u};
      ts.overall_speed = 30 >> 1;

      ts.speed1 = 20 >> 1;
      ts.breakpoint1 = 100;

      ts.speed2 = 40 >> 1;
      ts.breakpoint2 = 200;
      update_all_edges_but_bd(map, ts);
    }

    auto clean_reader = gurka::make_clean_graphreader(map.config.get_child("mjolnir"));
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
                                       0u,
                                       0u};
      ts.overall_speed = 36 >> 1;

      ts.speed1 = 20 >> 1;
      ts.breakpoint1 = 100;

      ts.speed2 = 40 >> 1;
      ts.breakpoint2 = 200;

      ts.speed3 = 60 >> 1;
      update_all_edges_but_bd(map, ts);
    }

    auto clean_reader = gurka::make_clean_graphreader(map.config.get_child("mjolnir"));
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
                                         0u,
                                         0u};
        ts.overall_speed = 36 >> 1;

        ts.speed1 = 20 >> 1;
        ts.breakpoint1 = 100;

        ts.speed2 = 40 >> 1;
        ts.breakpoint2 = 200;

        ts.speed3 = 60 >> 1;
        update_all_edges_but_bd(map, ts);
      }

      auto clean_reader = gurka::make_clean_graphreader(map.config.get_child("mjolnir"));
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
