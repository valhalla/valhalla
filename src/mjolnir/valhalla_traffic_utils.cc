#include "baldr/graphreader.h"
#include "baldr/predictedspeeds.h"
#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "filesystem.h"
#include "microtar.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/valhalla_traffic_utils.h"
#include "rapidjson/document.h"

#include <boost/algorithm/string/replace.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>

using namespace valhalla::baldr;
using namespace valhalla::midgard;

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

struct TileSet {
  int level;
  double size;
};

// INVALID_ID calculation
// constexpr uint64_t INVALID_ID =
//     (static_cast<uint64_t>(ID_INDEX_MASK) << (TILE_INDEX_BITS + LEVEL_BITS)) |
//     (static_cast<uint64_t>(TILE_INDEX_MASK) << LEVEL_BITS) | LEVEL_MASK;

std::unordered_map<uint64_t, bool> tile_map;

struct tile_index_entry {
  uint64_t offset;
  uint32_t tile_id;
  uint32_t size;
};

std::unordered_map<std::string, uint64_t> traffic_tiles;

auto index_loader = [](const std::string& filename,
                       const char* index_begin,
                       const char* file_begin,
                       size_t size) -> decltype(valhalla::midgard::tar::contents) {
  if (filename != "index.bin")
    return {};

  decltype(valhalla::midgard::tar::contents) contents;

  auto entries = valhalla::midgard::iterable_t<tile_index_entry>(reinterpret_cast<tile_index_entry*>(
                                                                     const_cast<char*>(index_begin)),
                                                                 size / sizeof(tile_index_entry));

  for (const auto& entry : entries) {
    // std::cout << "tile_id: " << entry.tile_id << ", offset: " << entry.offset << std::endl;
    valhalla::baldr::GraphId graph_id(entry.tile_id);
    // std::cout << std::to_string(graph_id) << std::endl;
    traffic_tiles.emplace(std::to_string(graph_id), entry.offset - 512);
  }

  return contents;
};

void update_traffic_tile(uint64_t tile_offset,
                         const std::vector<uint64_t>& traffic_params,
                         uint64_t last_updated,
                         std::string traffic_path) {

  const auto memory = std::make_shared<MMap>(traffic_path.c_str());

  mtar_t tar;
  tar.pos = tile_offset;

  tar.stream = memory->data;
  tar.read = [](mtar_t* tar, void* data, unsigned size) -> int {
    memcpy(data, reinterpret_cast<char*>(tar->stream) + tar->pos, size);
    return MTAR_ESUCCESS;
  };

  tar.write = [](mtar_t* tar, const void* data, unsigned size) -> int {
    memcpy(reinterpret_cast<char*>(tar->stream) + tar->pos, data, size);
    return MTAR_ESUCCESS;
  };
  tar.seek = [](mtar_t*, unsigned) -> int { return MTAR_ESUCCESS; };
  tar.close = [](mtar_t*) -> int { return MTAR_ESUCCESS; };

  // Read the tile header
  mtar_header_t tar_header;
  mtar_read_header(&tar, &tar_header);

  valhalla::baldr::TrafficTile tile(
      std::make_unique<MMapGraphMemory>(memory,
                                        reinterpret_cast<char*>(tar.stream) + tile_offset +
                                            sizeof(mtar_raw_header_t_),
                                        tar_header.size));

  LOG_INFO("Updating tile with tile id: " + std::to_string(tile.header->tile_id));

  int count = 0;

  for (size_t paramOffset = 0; paramOffset < traffic_params.size();) {
    uint64_t edge_index = static_cast<uint64_t>(traffic_params[paramOffset]);

    if (edge_index >= tile.header->directed_edge_count) { // edge index out of bounds skip

      paramOffset = paramOffset + 7;
      continue;
      throw std::runtime_error("Edge index out of bounds");
    }
    count++;

    // Access and update the edge's traffic data
    valhalla::baldr::TrafficSpeed* target_edge =
        const_cast<valhalla::baldr::TrafficSpeed*>(tile.speeds + edge_index);

    target_edge->overall_encoded_speed = static_cast<uint64_t>(traffic_params[paramOffset + 1]);
    target_edge->encoded_speed1 = static_cast<uint64_t>(traffic_params[paramOffset + 2]);
    target_edge->encoded_speed2 = static_cast<uint64_t>(traffic_params[paramOffset + 3]);
    target_edge->encoded_speed3 = static_cast<uint64_t>(
        traffic_params[paramOffset + 4]); // encoded minute timestamp into speed3 (received from
                                          // Valhalla traffic builder)
    target_edge->breakpoint1 = static_cast<uint64_t>(traffic_params[paramOffset + 5]);
    target_edge->breakpoint2 = static_cast<uint64_t>(traffic_params[paramOffset + 6]);
    target_edge->congestion1 = static_cast<uint64_t>(0);
    target_edge->congestion2 = static_cast<uint64_t>(0);
    target_edge->congestion3 = static_cast<uint64_t>(0);

    target_edge->has_incidents = static_cast<uint64_t>(0);

    paramOffset = paramOffset + 7;
  }

  tile.header->last_update = last_updated;

  LOG_INFO("Updated " + std::to_string(count) + " edges out of " +
           std::to_string(traffic_params.size() / 7) + " in tile " +
           std::to_string(tile.header->tile_id) + " with traffic data.");
}

int handle_tile_offset_index(std::string config_file_path) {
  boost::property_tree::ptree config;
  if (filesystem::is_regular_file(config_file_path)) {
    rapidjson::read_json(config_file_path, config);
  } else {
    std::cerr << "Configuration is required" << std::endl;
    return EXIT_FAILURE;
  }

  std::unordered_map<std::string, size_t> tar_index;

  const auto memory =
      std::make_shared<MMap>(config.get<std::string>("mjolnir.traffic_extract").c_str());

  std::unique_ptr<valhalla::midgard::tar> archive(
      new valhalla::midgard::tar(config.get<std::string>("mjolnir.traffic_extract"), true, true,
                                 index_loader));
  std::cout << "Loaded index from .tar archive." << std::endl;

  std::ofstream tar_index_file;
  std::string fname = config.get<std::string>("mjolnir.tile_dir") +
                      filesystem::path::preferred_separator + "traffic_tile_offset.csv";
  tar_index_file.open(fname, std::ofstream::out | std::ofstream::trunc);
  for (const auto& index : traffic_tiles) {
    tar_index_file << index.first << "," << index.second << std::endl;
  }
  tar_index_file.close();

  LOG_INFO("Finished with " + std::to_string(traffic_tiles.size()) + " ways.");

  return EXIT_SUCCESS;
}

int handle_build_verification(std::string config_file_path) {
  boost::property_tree::ptree config;
  if (filesystem::is_regular_file(config_file_path)) {
    rapidjson::read_json(config_file_path, config);
  } else {
    std::cerr << "Configuration is required" << std::endl;
    return EXIT_FAILURE;
  }

  std::unique_ptr<valhalla::midgard::tar> archive(
      new valhalla::midgard::tar(config.get<std::string>("mjolnir.traffic_extract"), true, true,
                                 index_loader));

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
  tar.seek = [](mtar_t*, unsigned) -> int { return MTAR_ESUCCESS; };
  tar.close = [](mtar_t*) -> int { return MTAR_ESUCCESS; };

  // Read the tile header
  mtar_header_t tar_header;
  mtar_read_header(&tar, &tar_header);

  std::uintmax_t size =
      std::filesystem::file_size(config.get<std::string>("mjolnir.traffic_extract"));
  std::ofstream verification_file;
  std::string fname = "traffic_verification.txt";
  verification_file.open(fname, std::ofstream::out | std::ofstream::trunc);

  verification_file << size << std::endl;

  int i = 0;
  for (const auto& index : traffic_tiles) {
    if (i == 0 || i == 200 ||
        i == 500) { // Copy specific tile information to reverify that file wasn't corrupted
      valhalla::baldr::TrafficTile tile(
          std::make_unique<MMapGraphMemory>(memory,
                                            reinterpret_cast<char*>(tar.stream) + index.second +
                                                sizeof(mtar_raw_header_t_),
                                            tar_header.size));

      verification_file << index.second << "," << tile.header->tile_id << ","
                        << tile.header->directed_edge_count << std::endl;
    }

    i++;
  }

  verification_file.close();
  return 0;
}

int handle_verify(std::string traffic_file_path, std::string verify_path) {

  const auto memory = std::make_shared<MMap>(traffic_file_path.c_str());

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
  tar.seek = [](mtar_t*, unsigned) -> int { return MTAR_ESUCCESS; };
  tar.close = [](mtar_t*) -> int { return MTAR_ESUCCESS; };

  // Read the tile header
  mtar_header_t tar_header;
  mtar_read_header(&tar, &tar_header);

  std::ifstream file(verify_path);
  if (!file) {
    std::cerr << "Failed to open file\n";
    return 1;
  }

  std::string line;

  uintmax_t size = 0;
  if (std::getline(file, line)) {
    std::istringstream iss(line);
    iss >> size;
  }

  std::uintmax_t to_verify_size = std::filesystem::file_size(traffic_file_path);

  if (size != to_verify_size) {
    std::cerr << "File size mismatch: expected " << size << ", got " << to_verify_size << "\n";
    return 1;
  }

  while (std::getline(file, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');

    std::istringstream iss(line);
    uint64_t offset = 0, tile_id = 0, edge_count = 0;

    if (iss >> offset >> tile_id >> edge_count) {
      valhalla::baldr::TrafficTile tile(
          std::make_unique<MMapGraphMemory>(memory,
                                            reinterpret_cast<char*>(tar.stream) + offset +
                                                sizeof(mtar_raw_header_t_),
                                            tar_header.size));
      if (tile.header->tile_id != tile_id) {
        std::cerr << "Tile ID mismatch at offset " << offset << ": expected " << tile_id << ", got "
                  << tile.header->tile_id << "\n";
        return 1;
      }

      if (tile.header->directed_edge_count != edge_count) {
        std::cerr << "Edge count mismatch at offset " << offset << ": expected " << edge_count
                  << ", got " << tile.header->directed_edge_count << "\n";
        return 1;
      }

    } else {
      std::cerr << "Malformed line: " << line << "\n";
    }
  }

  std::cout << "Verification successful. File is valid." << std::endl;
  file.close();

  return 0;
}

int handle_copy_traffic(std::string traffic_src_path, std::string traffic_dest_path) {
  LOG_INFO("Starting to copy traffic data from " + traffic_src_path + " to " + traffic_dest_path);

  const auto src_memory = std::make_shared<MMap>(traffic_src_path.c_str());

  mtar_t src_tar;
  src_tar.pos = 0;

  src_tar.stream = src_memory->data;
  src_tar.read = [](mtar_t* src_tar, void* data, unsigned size) -> int {
    memcpy(data, reinterpret_cast<char*>(src_tar->stream) + src_tar->pos, size);
    return MTAR_ESUCCESS;
  };

  src_tar.write = [](mtar_t* src_tar, const void* data, unsigned size) -> int {
    memcpy(reinterpret_cast<char*>(src_tar->stream) + src_tar->pos, data, size);
    return MTAR_ESUCCESS;
  };
  src_tar.seek = [](mtar_t*, unsigned) -> int { return MTAR_ESUCCESS; };
  src_tar.close = [](mtar_t*) -> int { return MTAR_ESUCCESS; };

  // Read the tile header
  mtar_header_t src_tar_header;
  mtar_read_header(&src_tar, &src_tar_header);

  const auto dest_memory = std::make_shared<MMap>(traffic_dest_path.c_str());

  mtar_t dest_tar;
  dest_tar.pos = 0;

  dest_tar.stream = dest_memory->data;
  dest_tar.read = [](mtar_t* dest_tar, void* data, unsigned size) -> int {
    memcpy(data, reinterpret_cast<char*>(dest_tar->stream) + dest_tar->pos, size);
    return MTAR_ESUCCESS;
  };

  dest_tar.write = [](mtar_t* dest_tar, const void* data, unsigned size) -> int {
    memcpy(reinterpret_cast<char*>(dest_tar->stream) + dest_tar->pos, data, size);
    return MTAR_ESUCCESS;
  };
  dest_tar.seek = [](mtar_t*, unsigned) -> int { return MTAR_ESUCCESS; };
  dest_tar.close = [](mtar_t*) -> int { return MTAR_ESUCCESS; };

  // Read the tile header
  mtar_header_t dest_tar_header;
  mtar_read_header(&dest_tar, &dest_tar_header);

  // std::unordered_map<std::string, size_t> tar_index;

  std::unique_ptr<valhalla::midgard::tar> archive(
      new valhalla::midgard::tar(traffic_dest_path, true, true, index_loader));

  for (const auto& index : traffic_tiles) {

    valhalla::baldr::TrafficTile src_tile(
        std::make_unique<MMapGraphMemory>(src_memory,
                                          reinterpret_cast<char*>(src_tar.stream) + index.second +
                                              sizeof(mtar_raw_header_t_),
                                          src_tar_header.size));

    valhalla::baldr::TrafficTile dest_tile(
        std::make_unique<MMapGraphMemory>(dest_memory,
                                          reinterpret_cast<char*>(dest_tar.stream) + index.second +
                                              sizeof(mtar_raw_header_t_),
                                          dest_tar_header.size));

    if (src_tile.header->tile_id == dest_tile.header->tile_id) {
      std::memcpy(const_cast<TrafficSpeed*>(dest_tile.speeds),
                  const_cast<const TrafficSpeed*>(src_tile.speeds),
                  sizeof(TrafficSpeed) * src_tile.header->directed_edge_count);
    }
  }

  std::cout << "Finished copying traffic data from " << traffic_src_path << " to "
            << traffic_dest_path << std::endl;

  return EXIT_SUCCESS;
}
