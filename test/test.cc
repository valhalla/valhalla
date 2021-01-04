#include "test.h"

#include "baldr/graphmemory.h"
#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/traffictile.h"
#include "mjolnir/graphtilebuilder.h"

#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <boost/property_tree/ptree.hpp>

#include "microtar.h"

namespace test {

struct MMap {
  MMap(const char* filename) {
    fd = open(filename, O_RDWR);
    struct stat s;
    fstat(fd, &s);
    data = mmap(0, s.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    length = s.st_size;
  }

  ~MMap() {
    munmap(data, length);
    close(fd);
  }

  int fd;
  void* data;
  size_t length;
};

class MMapGraphMemory final : public valhalla::baldr::GraphMemory {
public:
  MMapGraphMemory(std::shared_ptr<MMap> mmap, char* data_, size_t size_) : mmap_(std::move(mmap)) {
    data = data_;
    size = size_;
  }

private:
  const std::shared_ptr<MMap> mmap_;
};

std::string load_binary_file(const std::string& filename) {
  std::string bytes;
  std::ifstream input_pbf(filename, std::ios::in | std::ios::binary);
  if (input_pbf.is_open()) {
    input_pbf.seekg(0, std::ios::end);
    bytes.resize(input_pbf.tellg());
    input_pbf.seekg(0, std::ios::beg);
    input_pbf.read(&bytes[0], bytes.size());
    input_pbf.close();
  } else {
    throw std::runtime_error("Failed to read " + filename);
  }
  return bytes;
}

boost::property_tree::ptree json_to_pt(const std::string& json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  rapidjson::read_json(ss, pt);
  return pt;
}

std::shared_ptr<valhalla::baldr::GraphReader>
make_clean_graphreader(const boost::property_tree::ptree& mjolnir_conf) {

  // Wrapper sub-class to allow replacing the statically initialized
  // tile_extract member variable
  struct ResettingGraphReader : valhalla::baldr::GraphReader {
    ResettingGraphReader(const boost::property_tree::ptree& pt) : GraphReader(pt) {
      // Reset the statically initialized tile_extract_ member variable
      tile_extract_.reset(new valhalla::baldr::GraphReader::tile_extract_t(pt));
    }
  };
  return std::make_shared<ResettingGraphReader>(mjolnir_conf);
}

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
// Creates an empty traffic file
//
// To actually customize the traffic data, use `customize_live_traffic_data`
//
/*************************************************************/
void build_live_traffic_data(const boost::property_tree::ptree& config,
                             uint32_t traffic_tile_version) {

  std::string tile_dir = config.get<std::string>("mjolnir.tile_dir");
  std::string traffic_extract = config.get<std::string>("mjolnir.traffic_extract");

  // Begin by seeding the traffic file,
  // per-edge customizations come in the step after
  {
    mtar_t tar;
    auto tar_open_result = mtar_open(&tar, traffic_extract.c_str(), "w");
    if (tar_open_result != MTAR_ESUCCESS) {
      throw std::runtime_error("Could not create traffic tar file");
    }

    valhalla::baldr::GraphReader reader(config.get_child("mjolnir"));

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
      valhalla::baldr::TrafficTileHeader header = {};
      header.tile_id = tile_id;
      header.traffic_tile_version = traffic_tile_version;
      std::vector<valhalla::baldr::TrafficSpeed> speeds;
      header.directed_edge_count = tile->header()->directededgecount();
      buffer.write(reinterpret_cast<char*>(&header), sizeof(header));
      valhalla::baldr::TrafficSpeed dummy_speed = {}; // Initialize to all zeros
      for (uint32_t i = 0; i < header.directed_edge_count; ++i) {
        buffer.write(reinterpret_cast<char*>(&dummy_speed), sizeof(dummy_speed));
      }

      uint32_t dummy_uint32 = 0;
      buffer.write(reinterpret_cast<char*>(&dummy_uint32), sizeof(dummy_uint32));
      buffer.write(reinterpret_cast<char*>(&dummy_uint32), sizeof(dummy_uint32));

      /* Write strings to files `test1.txt` and `test2.txt` */
      std::string blanktile = buffer.str();
      std::string filename = valhalla::baldr::GraphTile::FileSuffix(tile_id);
      auto e1 = mtar_write_file_header(&tar, filename.c_str(), blanktile.size());
      if (e1 != MTAR_ESUCCESS) {
        throw std::runtime_error("Could not write tar-file header");
      }
      auto e2 = mtar_write_data(&tar, blanktile.c_str(), blanktile.size());
      if (e2 != MTAR_ESUCCESS) {
        throw std::runtime_error("Could not write tar-file data");
      }
    }

    mtar_finalize(&tar);
    mtar_close(&tar);
  }
}

/*************************************************************/
// Helper function for customizing traffic data in unit-tests
//
// `setter_cb` is a callback that can modify traffic for each edge
// when building traffic data
/*************************************************************/
void customize_live_traffic_data(const boost::property_tree::ptree& config,
                                 const LiveTrafficCustomize& setter_cb) {
  // Now we have the tar-file and can go ahead with per edge customizations
  {
    const auto memory =
        std::make_shared<MMap>(config.get<std::string>("mjolnir.traffic_extract").c_str());

    mtar_t tar;
    tar.pos = 0;
    tar.stream = memory->data;
    tar.read = [](mtar_t* tar, void* data, unsigned size) -> int {
      memcpy(data, reinterpret_cast<char*>(tar->stream) + tar->pos, size);
      return MTAR_ESUCCESS;
    };
    tar.write = [](mtar_t* tar, const void* data, unsigned size) -> int {
      memcpy(reinterpret_cast<char*>(tar->stream) + tar->pos, data, size);
      return MTAR_ESUCCESS;
    };
    tar.seek = [](mtar_t* /*tar*/, unsigned /*pos*/) -> int { return MTAR_ESUCCESS; };
    tar.close = [](mtar_t * /*tar*/) -> int { return MTAR_ESUCCESS; };

    // Read every speed tile, and update it with fixed speed of `new_speed` km/h (original speeds are
    // 10km/h)
    valhalla::baldr::GraphReader reader(config.get_child("mjolnir"));
    mtar_header_t tar_header;
    while ((mtar_read_header(&tar, &tar_header)) != MTAR_ENULLRECORD) {
      valhalla::baldr::TrafficTile tile(
          std::make_unique<MMapGraphMemory>(memory,
                                            reinterpret_cast<char*>(tar.stream) + tar.pos +
                                                sizeof(mtar_raw_header_t_),
                                            tar_header.size));

      valhalla::baldr::GraphId tile_id(tile.header->tile_id);

      for (uint32_t index = 0; index < tile.header->directed_edge_count; index++) {
        valhalla::baldr::TrafficSpeed* current =
            const_cast<valhalla::baldr::TrafficSpeed*>(tile.speeds + index);
        setter_cb(reader, tile, index, current);
      }
      mtar_next(&tar);
    }
  }
}

void customize_historical_traffic(const boost::property_tree::ptree& config,
                                  const HistoricalTrafficCustomize& cb) {
  // loop over all tiles in the tileset
  valhalla::baldr::GraphReader reader(config.get_child("mjolnir"));
  auto tile_dir = config.get<std::string>("mjolnir.tile_dir");
  for (const auto& tile_id : reader.GetTileSet()) {
    valhalla::mjolnir::GraphTileBuilder tile(tile_dir, tile_id, false);
    std::vector<valhalla::baldr::DirectedEdge> edges;
    edges.reserve(tile.header()->directededgecount());
    for (const auto& edge : tile.GetDirectedEdges()) {
      edges.push_back(edge);
      auto historical = cb(edges.back());
      if (!historical.empty())
        tile.AddPredictedSpeed(edges.size() - 1, historical, tile.header()->directededgecount());
      edges.back().set_has_predicted_speed(!historical.empty());
    }
    tile.UpdatePredictedSpeeds(edges);
  }
}

} // namespace test
