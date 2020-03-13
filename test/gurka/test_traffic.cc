#include "gurka.h"
#include <gtest/gtest.h>
#include "microtar.h"

#include <valhalla/baldr/traffictile.h>
#include <valhalla/baldr/graphreader.h>

#include <boost/property_tree/ptree.hpp>

#include <sstream>
#include <cmath>
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
TEST(Traffic, BasicUpdates) {

  const std::string ascii_map = R"(
    A----B----C
         |
         D)";

  const gurka::ways ways = {{"ABC", {{"highway", "primary"}, {"maxspeed", "10"}}},
                            {"BD", {{"highway", "primary"}, {"maxspeed", "10"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  std::string tile_dir = "test/data/traffic_basicupdates";
  auto map = gurka::buildtiles(layout, ways, {}, {}, tile_dir);

  {
    mtar_t tar;
    auto tar_open_result = mtar_open(&tar, "test/data/traffic_basicupdates/traffic.tar", "w");
    ASSERT_EQ(tar_open_result, MTAR_ESUCCESS);

    baldr::GraphReader reader(map.config.get_child("mjolnir"));
    auto tile_ids = reader.GetTileSet();
    for (auto tile_id : tile_ids) {
      auto tile = reader.GetGraphTile(tile_id);

      std::stringstream buffer;
      baldr::traffic::TileHeader header = {};
      std::vector<baldr::traffic::Speed> speeds;
      header.directed_edge_count = tile->header()->directededgecount();
      header.incident_buffer_size = static_cast<std::uint32_t>(std::ceil(header.directed_edge_count * 0.1));
      header.active_incident_buffer = 0;
      buffer.write(reinterpret_cast<char*>(&header), sizeof(header));
      baldr::traffic::Speed dummy_speed = {}; // Initialize to all zeros
      for (int i = 0; i < header.directed_edge_count; ++i) {
        buffer.write(reinterpret_cast<char*>(&dummy_speed), sizeof(dummy_speed));
      }
      uint32_t dummy_uint32 = 0;
      buffer.write(reinterpret_cast<char*>(&dummy_uint32), sizeof(dummy_uint32));
      buffer.write(reinterpret_cast<char*>(&dummy_uint32), sizeof(dummy_uint32));
      baldr::traffic::Incident dummy_incident;
      for (int i=0; i<header.incident_buffer_size*2; i++) {
          buffer.write(reinterpret_cast<char*>(&dummy_incident), sizeof(dummy_incident));
      }

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
    valhalla::tyr::actor_t actor(map.config, true);
    // Do a route with initial traffic
    {
      auto result = gurka::route(map, "A", "C", "auto", &actor);
      gurka::assert::osrm::expect_route(result, {"ABC"});
      gurka::assert::raw::expect_eta(result, 361.5);
    }
    // Update the live traffic in-place
    {
        int fd = open("test/data/traffic_basicupdates/traffic.tar",O_RDWR);
        struct stat s;
        fstat(fd, &s);

        mtar_t tar;
        tar.pos = 0;
        tar.stream = mmap(0, s.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        tar.read = [](mtar_t *tar, void *data, unsigned size) -> int {
            memcpy(data, reinterpret_cast<char *>(tar->stream) + tar->pos, size);
            return MTAR_ESUCCESS;
        };
        tar.write = [](mtar_t *tar, const void *data, unsigned size) -> int {
            memcpy(reinterpret_cast<char *>(tar->stream) + tar->pos, data, size);
            return MTAR_ESUCCESS;
        };
        tar.seek = [](mtar_t *tar, unsigned pos) -> int { return MTAR_ESUCCESS; };
        tar.close = [](mtar_t *tar) -> int { return MTAR_ESUCCESS; };

        // Read every speed tile, and update it with fixed speed of 10km/h
        mtar_header_t tar_header;
        while ( (mtar_read_header(&tar, &tar_header)) != MTAR_ENULLRECORD ) {
            std::cout << "Found tile " << tar_header.name << " of size " << tar_header.size << std::endl;
            std::cout << "pos is now at " << tar.pos << std::endl;
            baldr::traffic::Tile tile(reinterpret_cast<char *>(tar.stream) + tar.pos + sizeof(mtar_raw_header_t_));
            for (int i=0; i<tile.header->directed_edge_count; i++) {
                (tile.speeds + i)->speed_kmh = 10;
            }
            mtar_next(&tar);
        }
        munmap(tar.stream, s.st_size);

    }
    // Now do another route with the same (not restarted) actor to see if
    // it's noticed the changes in the live traffic file
    {
      auto result = gurka::route(map, "A", "C", "auto", &actor);
      gurka::assert::osrm::expect_route(result, {"ABC"});
      gurka::assert::raw::expect_eta(result, 1.0);
    }
  }

}