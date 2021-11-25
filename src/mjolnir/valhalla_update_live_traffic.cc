#include "baldr/graphreader.h"
#include "baldr/predictedspeeds.h"
#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include "mjolnir/graphtilebuilder.h"

#include <boost/program_options.hpp>
#include <cstdint>

#include <algorithm>
#include <boost/algorithm/string/replace.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/tokenizer.hpp>

#include "config.h"
#include "microtar.h"

namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vj = valhalla::mjolnir;

namespace bpo = boost::program_options;
using namespace valhalla;

namespace {

struct CSV_Speed {
  CSV_Speed(valhalla::baldr::GraphId _tileid, std::vector<std::pair<uint32_t, uint32_t>> _speed_data)
      : tileid{_tileid}, speed_data{std::move(_speed_data)} {
  }
  explicit CSV_Speed(valhalla::baldr::GraphId _tileid) : tileid{_tileid} {
  }

  valhalla::baldr::GraphId tileid;
  std::vector<std::pair<uint32_t, uint32_t>> speed_data;
  // uint32_t speed;
  bool operator==(const CSV_Speed& rhs) const {
    return this->tileid == rhs.tileid;
  }
};

// Mmap and mtar structs copied from test/test.cc
struct MMap {
  MMap(const char* filename) {
    fd = open(filename, O_RDWR);
    struct stat s;
#ifdef _MSC_VER
    _fstat64(fd, &s);
#else
    fstat(fd, &s);
#endif
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

// csv format [tileid, index, speed]
std::vector<CSV_Speed> ParseTrafficFile(const std::string filename,
                                        const boost::property_tree::ptree& pt) {

  typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
  boost::char_separator<char> sep{","};
  std::vector<CSV_Speed> ts;
  std::string line;
  std::ifstream file(filename);
  uint32_t line_num = 0;

  std::vector<std::pair<uint32_t, uint32_t>> speed_data;
  baldr::GraphId tileid;
  baldr::GraphId prev_tileid{baldr::kInvalidGraphId};
  if (file.is_open()) {
    // for each row in the file
    while (getline(file, line) && ++line_num) {
      tokenizer tok{line, sep};
      uint32_t field_num = 0;
      uint32_t index = 0, speed = 0;

      // parse each column
      for (const auto& t : tok) {
        switch (field_num) {
          case 0:
            tileid = baldr::GraphId{std::stoull(t)};
            break;
          case 1:
            index = std::stoi(t);
            break;
          case 2:
            speed = std::stoi(t);
            break;
          default:
            break;
        }
        field_num++;
      } // for

      if (prev_tileid.value == kInvalidGraphId) {
        prev_tileid = tileid;
      }
      if (tileid.value != prev_tileid.value && !file.eof()) {
        std::vector<std::pair<uint32_t, uint32_t>> temp;
        temp.swap(speed_data);
        ts.emplace_back(CSV_Speed{prev_tileid, std::move(temp)});

        prev_tileid = tileid;
        speed_data.emplace_back(std::make_pair(index, speed));
      } else {
        speed_data.emplace_back(std::make_pair(index, speed));
      }
    } // while

    // insert the last tile
    ts.emplace_back(CSV_Speed{tileid, std::move(speed_data)});
  } // if
  file.close();
  return ts;
}

// // Copied from test/test.cc
// void build_live_traffic_data(const boost::property_tree::ptree& config,
//                              std::string tile_id,
//                              uint32_t constant_encoded_speed,
//                              uint64_t traffic_update_timestamp) {

//   std::string tile_dir = config.get<std::string>("mjolnir.tile_dir");
//   std::string traffic_extract = config.get<std::string>("mjolnir.traffic_extract");

//   filesystem::path parent_dir = filesystem::path(traffic_extract).parent_path();
//   if (!filesystem::exists(parent_dir)) {
//     std::stringstream ss;
//     ss << "Traffic extract directory " << parent_dir.string() << " does not exist";
//     throw std::runtime_error(ss.str());
//   }

//   // Begin by seeding the traffic file,
//   // per-edge customizations come in the step after
//   {
//     mtar_t tar;
//     auto tar_open_result = mtar_open(&tar, traffic_extract.c_str(), "w");
//     if (tar_open_result != MTAR_ESUCCESS) {
//       throw std::runtime_error("Could not create traffic tar file");
//     }

//     valhalla::baldr::GraphReader reader(config.get_child("mjolnir"));

//     // Traffic data works like this:
//     //   1. There is a separate .tar file containing tile entries matching the main tiles
//     //   2. Each tile is a fixed-size, with a header, and entries
//     // This loop iterates over the routing tiles, and creates blank
//     // traffic tiles with empty records.
//     // Valhalla mmap()'s this file and reads from it during route calculation.
//     // This loop below creates initial .tar file entries .  Lower down, we make changes to
//     // values within the generated traffic tiles and test that routes reflect those changes
//     // as expected.

//     valhalla::baldr::GraphId tile_graph_id(tile_id);
//     auto tile = reader.GetGraphTile(tile_graph_id);

//     std::stringstream buffer;
//     valhalla::baldr::TrafficTileHeader header = {};
//     header.tile_id = tile_graph_id.value;
//     header.last_update = traffic_update_timestamp;
//     header.traffic_tile_version = valhalla::baldr::TRAFFIC_TILE_VERSION;
//     header.directed_edge_count = tile->header()->directededgecount();
//     buffer.write(reinterpret_cast<char*>(&header), sizeof(header));
//     valhalla::baldr::TrafficSpeed dummy_speed = {constant_encoded_speed,
//                                                  constant_encoded_speed,
//                                                  constant_encoded_speed,
//                                                  constant_encoded_speed,
//                                                  255,
//                                                  255,
//                                                  0,
//                                                  0,
//                                                  0,
//                                                  0};
//     for (uint32_t i = 0; i < header.directed_edge_count; ++i) {
//       buffer.write(reinterpret_cast<char*>(&dummy_speed), sizeof(dummy_speed));
//     }

//     uint32_t dummy_uint32 = 0;
//     buffer.write(reinterpret_cast<char*>(&dummy_uint32), sizeof(dummy_uint32));
//     buffer.write(reinterpret_cast<char*>(&dummy_uint32), sizeof(dummy_uint32));

//     /* Write strings to files `test1.txt` and `test2.txt` */
//     std::string blanktile = buffer.str();
//     std::string filename = valhalla::baldr::GraphTile::FileSuffix(tile_graph_id);
//     auto e1 = mtar_write_file_header(&tar, filename.c_str(), blanktile.size());
//     if (e1 != MTAR_ESUCCESS) {
//       throw std::runtime_error("Could not write tar-file header");
//     }
//     auto e2 = mtar_write_data(&tar, blanktile.c_str(), blanktile.size());
//     if (e2 != MTAR_ESUCCESS) {
//       throw std::runtime_error("Could not write tar-file data");
//     }

//     mtar_finalize(&tar);
//     mtar_close(&tar);
//   }
// }

/*************************************************************/
// Creates an empty traffic file
//
// To actually customize the traffic data, use `customize_live_traffic_data`
//
/*************************************************************/
void build_live_traffic_data(const boost::property_tree::ptree& config,
                             uint32_t traffic_tile_version = valhalla::baldr::TRAFFIC_TILE_VERSION) {

  std::string tile_dir = config.get<std::string>("mjolnir.tile_dir");
  std::string traffic_extract = config.get<std::string>("mjolnir.traffic_extract");

  filesystem::path parent_dir = filesystem::path(traffic_extract).parent_path();
  if (!filesystem::exists(parent_dir)) {
    std::stringstream ss;
    ss << "Traffic extract directory " << parent_dir.string() << " does not exist";
    throw std::runtime_error(ss.str());
  }

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
    // This loop iterates over the routing tiles, and creates blank
    // traffic tiles with empty records.
    // Valhalla mmap()'s this file and reads from it during route calculation.
    // This loop below creates initial .tar file entries .  Lower down, we make changes to
    // values within the generated traffic tiles and test that routes reflect those changes
    // as expected.
    for (auto tile_id : tile_ids) {
      // LOG_INFO("tile_id=" + std::to_string(tile_id.value));
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

void customize_live_traffic_data(const boost::property_tree::ptree& config,
                                 const std::vector<CSV_Speed>& ts) {
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
    tar.close = [](mtar_t* /*tar*/) -> int { return MTAR_ESUCCESS; };

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

      CSV_Speed findvalue{tile_id};

      auto it = find(ts.begin(), ts.end(), findvalue);
      if (it != ts.end()) {
        for (auto speeddata : it->speed_data) {
          valhalla::baldr::TrafficSpeed* current =
              const_cast<valhalla::baldr::TrafficSpeed*>(tile.speeds + speeddata.first);
          current->breakpoint1 = 255;
          current->overall_encoded_speed = speeddata.second >> 1;
          current->encoded_speed1 = speeddata.second >> 1;
        }
      } else {
        // LOG_INFO("No traffic data for tile " + std::to_string(tile_id));
      }
      mtar_next(&tar);
    }
  }
}

// Copied from test/test.cc
// Updates all edges of all tiles in a traffic tar with the given speed
void customize_live_traffic_data(const boost::property_tree::ptree& config,
                                 valhalla::baldr::TrafficSpeed speed) {
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
    tar.close = [](mtar_t* /*tar*/) -> int { return MTAR_ESUCCESS; };

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
        // current->overall_encoded_speed = constant_encoded_speed;
        // current->breakpoint1 = 255;
        *current = speed;
      }
      mtar_next(&tar);
    }
  }
}

// Copied from test/test.cc
// Updates one edge in a traffic tar with the given speed
void customize_live_traffic_data(const boost::property_tree::ptree& config,
                                 valhalla::baldr::GraphId edge_id,
                                 uint32_t constant_encoded_speed) {
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
    tar.close = [](mtar_t* /*tar*/) -> int { return MTAR_ESUCCESS; };

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
        if (valhalla::baldr::GraphId{tile_id.tileid(), tile_id.level(), index} == edge_id) {
          current->overall_encoded_speed = constant_encoded_speed;
          current->breakpoint1 = 255;
        }
      }
      mtar_next(&tar);
    }
  }
}

} // namespace

int main(int argc, char** argv) {

  std::string config_file_path;
  std::string csv_file_path;

  bpo::options_description options(
      "valhalla_update_live_traffic \n"
      "\n"
      " Usage: valhalla_update_live_traffic [options]\n"
      "\n"
      "provides utils for create default traffic.tar and updating by csv file. "
      "\n"
      "\n");

  options.add_options()("help,h", "Print this help message.")(
      "config,c", boost::program_options::value<std::string>(&config_file_path),
      "Path to the json configuration file.")("traffic,t",
                                              boost::program_options::value<std::string>(
                                                  &csv_file_path),
                                              "Path to the csv traffic file");

  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).run(), vm);
    bpo::notify(vm);
  } catch (std::exception& e) {
  std:
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  if (vm.count("help") || vm.empty()) {
    std::cout << options << "\n";
    return EXIT_SUCCESS;
  }

  // Read the config file
  boost::property_tree::ptree pt;
  if (vm.count("config") && filesystem::is_regular_file(config_file_path)) {
    rapidjson::read_json(config_file_path, pt);
  } else {
    LOG_ERROR("Configuration is required");
    return EXIT_FAILURE;
  }

  // check whether the traffic file exists
  if (!filesystem::is_regular_file(pt.get<std::string>("mjolnir.traffic_extract"))) {
    build_live_traffic_data(pt);
    LOG_INFO("Generated " + pt.get<std::string>("mjolnir.traffic_extract") + " succesfully");
  } else {
    // valhalla::baldr::TrafficSpeed speed = {};
    // customize_live_traffic_data(pt, speed);
    // LOG_INFO(pt.get<std::string>("mjolnir.traffic_extract") + " exists!");
  }

  if (vm.count("traffic")) {
    if (!filesystem::is_regular_file(csv_file_path)) {
      LOG_ERROR("Traffic file " + csv_file_path + " does not exist!");
      return EXIT_FAILURE;
    }

    auto ts = ParseTrafficFile(csv_file_path, pt);
    customize_live_traffic_data(pt, ts);

    LOG_INFO("Updated " + pt.get<std::string>("mjolnir.traffic_extract") + " succesfully!");
    return EXIT_SUCCESS;
  }

  return EXIT_FAILURE;
}
