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
void update_bd_traffic_speed(valhalla::gurka::map& map, uint16_t new_speed) {
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
    baldr::traffic::Tile tile(reinterpret_cast<char*>(tar.stream) + tar.pos +
                              sizeof(mtar_raw_header_t_));

    baldr::GraphId tile_id(tile.header->tile_id);
    auto BD = gurka::findEdge(reader, map.nodes, "BD", "D", tile_id);

    for (int i = 0; i < tile.header->directed_edge_count; i++) {
      (tile.speeds + i)->age_bucket = baldr::traffic::MAX_SPEED_AGE_BUCKET;
      if (std::get<1>(BD) != nullptr && std::get<0>(BD).id() == i) {
        (tile.speeds + i)->speed_kmh = 0;
      } else {
        (tile.speeds + i)->speed_kmh = new_speed;
      }
    }
    mtar_next(&tar);
  }
  munmap(tar.stream, s.st_size);
  close(fd);
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

  std::string traffic_extract = tile_dir + "/traffic.tar";

  {
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
      baldr::traffic::TileHeader header = {};
      header.tile_id = tile_id;
      std::vector<baldr::traffic::Speed> speeds;
      header.directed_edge_count = tile->header()->directededgecount();
      buffer.write(reinterpret_cast<char*>(&header), sizeof(header));
      baldr::traffic::Speed dummy_speed = {}; // Initialize to all zeros
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
  {
    map.config.put("mjolnir.traffic_extract", traffic_extract);
    auto clean_reader = gurka::make_clean_graphreader(map.config.get_child("mjolnir"));
    // Do a route with initial traffic
    {
      auto result = gurka::route(map, "A", "C", "auto", "current", clean_reader);
      gurka::assert::osrm::expect_steps(result, {"AB", "BC"});
      gurka::assert::raw::expect_path(result, {"AB", "BC"});
      gurka::assert::raw::expect_eta(result, 361.5);
    }
    // Make some updates to the traffic .tar file.
    // Mostly just updates ever edge in the file to 25km/h, except for one
    // specific edge (B->D) where we simulate a closure (speed=0, congestion high)
    update_bd_traffic_speed(map, 25);

    // Now do another route with the same (not restarted) actor to see if
    // it's noticed the changes in the live traffic file
    {
      auto result = gurka::route(map, "A", "C", "auto", "current", clean_reader);
      gurka::assert::osrm::expect_steps(result, {"AB", "BC"});
      gurka::assert::raw::expect_path(result, {"AB", "BC"});
      gurka::assert::raw::expect_eta(result, 145.5);
    }
    // Next, set the speed to the highest possible to ensure nothing breaks
    update_bd_traffic_speed(map, valhalla::baldr::kMaxSpeedKph);
    {
      auto result = gurka::route(map, "A", "C", "auto", "current", clean_reader);
      gurka::assert::osrm::expect_steps(result, {"AB", "BC"});
      gurka::assert::raw::expect_path(result, {"AB", "BC"});
      gurka::assert::raw::expect_eta(result, 15.617648);
    }
    // Back to previous speed
    update_bd_traffic_speed(map, 25);
    // And verify that without the "current" timestamp, the live traffic
    // results aren't used
    {
      auto result = gurka::route(map, "A", "C", "auto", "", clean_reader);
      gurka::assert::osrm::expect_steps(result, {"AB", "BC"});
      gurka::assert::raw::expect_path(result, {"AB", "BC"});
      gurka::assert::raw::expect_eta(result, 361.5);
    }
    // Now do another route with the same (not restarted) actor to see if
    // it's noticed the changes in the live traffic file
    {
      auto result = gurka::route(map, "B", "D", "auto", "current", clean_reader);
      gurka::assert::osrm::expect_steps(result, {"BC", "CE", "DE"});
      gurka::assert::raw::expect_path(result, {"BC", "CE", "DE"});
      gurka::assert::raw::expect_eta(result, 172.8, 0.01);
    }
    {
      auto result = gurka::route(map, "D", "B", "auto", "current", clean_reader);
      gurka::assert::osrm::expect_steps(result, {"BD"});
      gurka::assert::raw::expect_path(result, {"BD"});
      gurka::assert::raw::expect_eta(result, 28.8, 0.01);
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
