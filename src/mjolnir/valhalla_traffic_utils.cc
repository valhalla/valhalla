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
#include <fstream>
#include <iostream>
#include <filesystem>

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

struct EdgeAndDirection {
  bool forward;
  uint32_t length;
  GraphId tileid;
  GraphId edgeid;

  EdgeAndDirection(const bool f, uint32_t len, valhalla::baldr::GraphId graphid, const GraphId& id)
      : forward(f), length(len), tileid(graphid), edgeid(id) {
  }
};

struct TileSet {
  int level;
  double size;
};

const std::vector<TileSet> valhalla_tiles = {{2, 0.25}, {1, 1.0}, {0, 4.0}};

// Constants
constexpr int LEVEL_BITS = 3;
constexpr int TILE_INDEX_BITS = 22;
constexpr int ID_INDEX_BITS = 21;

// Masks
constexpr uint32_t LEVEL_MASK = (1 << LEVEL_BITS) - 1;           // 0b111 (3 bits set)
constexpr uint32_t TILE_INDEX_MASK = (1 << TILE_INDEX_BITS) - 1; // 22 bits set
constexpr uint32_t ID_INDEX_MASK = (1 << ID_INDEX_BITS) - 1;     // 21 bits set

// INVALID_ID calculation
// constexpr uint64_t INVALID_ID =
//     (static_cast<uint64_t>(ID_INDEX_MASK) << (TILE_INDEX_BITS + LEVEL_BITS)) |
//     (static_cast<uint64_t>(TILE_INDEX_MASK) << LEVEL_BITS) | LEVEL_MASK;

// Bounding box coordinates for USA and Alaska
// Visualize the bboxes using links below:
// http://bboxfinder.com/#51.214183,-179.148909,71.538800,-129.974167
// http://bboxfinder.com/#24.396308,-125.0,49.384358,-66.934570
const double usa_left = -125.0 + 180.0;
const double usa_bottom = 24.396308 + 90.0;
const double usa_right = -66.934570 + 180.0;
const double usa_top = 49.384358 + 90.0;

const double alaska_left = -179.148909 + 180.0;
const double alaska_bottom = 51.214183 + 90.0;
const double alaska_right = -129.974167 + 180.0;
const double alaska_top = 71.538800 + 90.0;

std::unordered_map<uint64_t, bool> tile_map;

void tiles_for_bounding_box(double left, double bottom, double right, double top) {
  // If the bounding box crosses the anti-meridian, split into two and combine results
  if (left > right) {
    tiles_for_bounding_box(left, bottom, 180.0, top);
    tiles_for_bounding_box(-180.0, bottom, right, top);
    return;
  }

  // Iterate over each tile set level
  for (const auto& tile_set : valhalla_tiles) {
    double tile_size = tile_set.size;

    int x_start = static_cast<int>(std::floor(left / tile_size));
    int x_end = static_cast<int>(std::ceil(right / tile_size));
    int y_start = static_cast<int>(std::floor(bottom / tile_size));
    int y_end = static_cast<int>(std::ceil(top / tile_size));

    for (int x = x_start; x <= x_end; ++x) {
      for (int y = y_start; y <= y_end; ++y) {
        int tile_index = y * static_cast<int>(360.0 / tile_size) + x;

        uint64_t tile_id =
            (static_cast<uint64_t>(tile_set.level) & LEVEL_MASK) |
            ((static_cast<uint64_t>(tile_index) & TILE_INDEX_MASK) << LEVEL_BITS) |
            ((static_cast<uint64_t>(0) & ID_INDEX_MASK) << (TILE_INDEX_BITS + LEVEL_BITS));

        tile_map[tile_id] = true;
      }
    }
  }
}

bool is_tile_in_usa(uint64_t tile_id) {
  return tile_map.find(tile_id) != tile_map.end();
};

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
    std::cout << "tile_id: " << entry.tile_id << ", offset: " << entry.offset << "size: " << entry.size << std::endl;
    valhalla::baldr::GraphId graph_id(entry.tile_id);
    std::cout << std::to_string(graph_id) << std::endl;
    traffic_tiles.emplace(std::to_string(graph_id), entry.offset - 512);
  }

  return contents;
};

void update_traffic_tile(uint64_t tile_offset,
                         const std::vector<uint64_t>& traffic_params,
                         uint64_t last_updated, std::string traffic_path) {
      std::cout << "Current path: " << std::filesystem::current_path() << std::endl;

  std::cout << tile_offset << ',' << traffic_params.at(0) << ',' << last_updated << std::endl;

  const auto memory =
      std::make_shared<MMap>(traffic_path.c_str());


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

  // std::cout << tile.header->directed_edge_count  << " dir edge count" << std::endl;
  // std::cout << tile.header->tile_id << " tile id" << std::endl;

  for (size_t paramOffset = 0; paramOffset < traffic_params.size();) {
    uint64_t edge_index = static_cast<uint64_t>(traffic_params[paramOffset]);


    if (edge_index >= tile.header->directed_edge_count) {

      std::cout << tile.header->directed_edge_count << "edgecount" << edge_index << "," << tile_offset << std::endl;
      paramOffset = paramOffset + 7;
      continue;
      throw std::runtime_error("Edge index out of bounds");
    }

    // Access and update the edge's traffic data
    valhalla::baldr::TrafficSpeed* target_edge =
        const_cast<valhalla::baldr::TrafficSpeed*>(tile.speeds + edge_index);
    target_edge->overall_encoded_speed = static_cast<uint64_t>(traffic_params[paramOffset + 1]);
    target_edge->encoded_speed1 = static_cast<uint64_t>(traffic_params[paramOffset + 2]);
    target_edge->encoded_speed2 = static_cast<uint64_t>(traffic_params[paramOffset + 3]);
    target_edge->encoded_speed3 = static_cast<uint64_t>(traffic_params[paramOffset + 4]);
    target_edge->breakpoint1 =  static_cast<uint64_t>(traffic_params[paramOffset + 5]);
    target_edge->breakpoint2 =  static_cast<uint64_t>(traffic_params[paramOffset + 6]);
    target_edge->congestion1 = static_cast<uint64_t>(0);
    target_edge->congestion2 = static_cast<uint64_t>(0);
    target_edge->congestion3 = static_cast<uint64_t>(0);

    target_edge->has_incidents = static_cast<uint64_t>(0);

    paramOffset = paramOffset + 7;
  }

  tile.header->last_update = last_updated;

  std::cout << "Updated edge_id successfully at " << traffic_path << std::endl;
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



int handle_ways_to_edges(std::string config_file_path) {
  tiles_for_bounding_box(usa_left, usa_bottom, usa_right, usa_top);
  tiles_for_bounding_box(alaska_left, alaska_bottom, alaska_right, alaska_top);

  boost::property_tree::ptree config;
  if (filesystem::is_regular_file(config_file_path)) {
    rapidjson::read_json(config_file_path, config);
  } else {
    std::cerr << "Configuration is required" << std::endl;
    return EXIT_FAILURE;
  }

  std::unordered_map<uint64_t, std::vector<EdgeAndDirection>> ways_edges;

  GraphReader reader(config.get_child("mjolnir"));
  for (auto edge_id : reader.GetTileSet()) {
    if (!reader.DoesTileExist(edge_id)) {
      continue;
    }
    if (reader.OverCommitted()) {
      reader.Trim();
    }

    if (!is_tile_in_usa(edge_id))
      continue;

    graph_tile_ptr tile = reader.GetGraphTile(edge_id);
    for (uint32_t n = 0; n < tile->header()->directededgecount(); n++, ++edge_id) {
      const DirectedEdge* edge = tile->directededge(edge_id);
      if (edge->IsTransitLine() || edge->use() == Use::kTransitConnection ||
          edge->use() == Use::kEgressConnection || edge->use() == Use::kPlatformConnection ||
          edge->is_shortcut()) {
        continue;
      }

      if (!(edge->forwardaccess() & kAutoAccess)) {
        continue;
      }

      valhalla::baldr::GraphId graph_id(edge_id);

      uint64_t wayid = tile->edgeinfo(edge).wayid();
      ways_edges[wayid].push_back({edge->forward(), edge->length(), graph_id, edge_id});
    }
  }

  std::ofstream ways_file;
  std::string fname = config.get<std::string>("mjolnir.tile_dir") +
                      filesystem::path::preferred_separator + "way_edges_traffic.csv";
  ways_file.open(fname, std::ofstream::out | std::ofstream::trunc);
  for (const auto& way : ways_edges) {
    ways_file << way.first;
    bool isFirst = true;
    for (auto edge : way.second) {
      if (isFirst) {
        ways_file << "," << (uint32_t)edge.forward << ";" << edge.length << ";" << edge.tileid << ";"
                  << (uint64_t)edge.edgeid;
        isFirst = false;
      } else {
        ways_file << ";" << (uint32_t)edge.forward << ";" << edge.length << ";" << edge.tileid << ";"
                  << (uint64_t)edge.edgeid;
      }
    }
    ways_file << std::endl;
  }
  ways_file.close();

  LOG_INFO("Finished with " + std::to_string(ways_edges.size()) + " ways.");

  return EXIT_SUCCESS;
}
