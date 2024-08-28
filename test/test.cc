#include "test.h"

#include "baldr/graphmemory.h"
#include "baldr/graphreader.h"
#include "baldr/predictedspeeds.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/traffictile.h"
#include "mjolnir/graphtilebuilder.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#ifndef _MSC_VER
#include <sys/mman.h>
#endif
#include <sys/stat.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <boost/algorithm/string.hpp>

#include "microtar.h"

namespace {
// TODO: this should support boost::property_tree::path
// like get_child does to make it obvious that it supports
// the path separator notation for specifying sub children
bool remove_child(boost::property_tree::ptree& pt, const std::string& path) {
  // split up the path into each sub part
  std::vector<std::string> path_parts;
  boost::split(path_parts, path, boost::is_any_of("."));

  // check each part of the path
  auto* root = &pt;
  for (const auto& part : path_parts) {
    // if we dont have this sub child bail
    auto found = root->find(part);
    if (found == root->not_found())
      return false;

    // if this was the last one to look for remove it
    if (&part == &path_parts.back()) {
      root->erase(root->to_iterator(found));
    } // next sub child
    else {
      root = &found->second;
    }
  }

  // made it to the last sub child without bailing on not found
  return true;
}
} // namespace

namespace test {

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

boost::property_tree::ptree make_config(const std::string& path_prefix,
                                        const std::unordered_map<std::string, std::string>& overrides,
                                        const std::unordered_set<std::string>& removes) {

  std::string defaults = R"(
    {
      "additional_data": {
        "elevation": "%%/elevation/"
      },
      "httpd": {
        "service": {
          "interrupt": "ipc://%%/interrupt",
          "listen": "ipc://%%/listen",
          "loopback": "ipc://%%/loopback"
        }
      },
      "loki": {
        "actions": [
          "locate",
          "route",
          "height",
          "sources_to_targets",
          "optimized_route",
          "isochrone",
          "trace_route",
          "trace_attributes",
          "transit_available",
          "expansion",
          "centroid",
          "status"
        ],
        "logging": {
          "color": false,
          "type": "std_out"
        },
        "service": {
          "proxy": "ipc://%%/loki"
        },
        "service_defaults": {
          "heading_tolerance": 60,
          "minimum_reachability": 50,
          "node_snap_tolerance": 5,
          "radius": 0,
          "search_cutoff": 35000,
          "street_side_max_distance": 1000,
          "street_side_tolerance": 5
        },
        "use_connectivity": true
      },
      "meili": {
        "auto": {
          "search_radius": 50,
          "turn_penalty_factor": 200
        },
        "bicycle": {
          "turn_penalty_factor": 140
        },
        "customizable": [
          "mode",
          "search_radius",
          "turn_penalty_factor",
          "gps_accuracy",
          "interpolation_distance",
          "sigma_z",
          "beta",
          "max_route_distance_factor",
          "max_route_time_factor"
        ],
        "default": {
          "beta": 3,
          "breakage_distance": 2000,
          "geometry": false,
          "gps_accuracy": 5.0,
          "interpolation_distance": 10,
          "max_route_distance_factor": 5,
          "max_route_time_factor": 5,
          "max_search_radius": 100,
          "route": true,
          "search_radius": 50,
          "sigma_z": 4.07,
          "turn_penalty_factor": 0
        },
        "grid": {
          "cache_size": 100240,
          "size": 500
        },
        "logging": {
          "color": false,
          "type": "std_out"
        },
        "mode": "auto",
        "multimodal": {
          "turn_penalty_factor": 70
        },
        "pedestrian": {
          "search_radius": 50,
          "turn_penalty_factor": 100
        },
        "service": {
          "proxy": "ipc://%%/meili"
        },
        "verbose": false
      },
      "mjolnir": {
        "concurrency": 1,
        "admin": "%%/admin.sqlite",
        "landmarks": "%%/landmarks.sqlite",
        "data_processing": {
          "allow_alt_name": false,
          "apply_country_overrides": true,
          "infer_internal_intersections": true,
          "infer_turn_channels": true,
          "use_admin_db": true,
          "use_direction_on_ways": false,
          "use_rest_area": false,
          "use_urban_tag": false
        },
        "global_synchronized_cache": false,
        "hierarchy": true,
        "id_table_size": 1300000000,
        "import_bike_share_stations": false,
        "include_bicycle": true,
        "include_driveways": true,
        "include_construction": true,
        "include_driving": true,
        "include_pedestrian": true,
        "logging": {
          "color": false,
          "type": "std_out"
        },
        "lru_mem_cache_hard_control": false,
        "max_cache_size": 1000000000,
        "max_concurrent_reader_users": 1,
        "reclassify_links": true,
        "shortcuts": true,
        "tile_dir": "%%",
        "tile_extract": "",
        "timezone": "%%/tz_world.sqlite",
        "traffic_extract": "%%/traffic.tar",
        "transit_dir": "%%/transit",
        "transit_feeds_dir": "%%/transit_feeds",
        "use_lru_mem_cache": false
      },
      "odin": {
        "logging": {
          "color": false,
          "type": "std_out"
        },
        "service": {
          "proxy": "ipc://%%/odin"
        }
      },
      "service_limits": {
        "auto": {
          "max_distance": 5000000.0,
          "max_locations": 20,
          "max_matrix_distance": 400000.0,
          "max_matrix_location_pairs": 2500
        },
        "bicycle": {
          "max_distance": 500000.0,
          "max_locations": 50,
          "max_matrix_distance": 200000.0,
          "max_matrix_location_pairs": 2500
        },
        "bikeshare": {
          "max_distance": 500000.0,
          "max_locations": 50,
          "max_matrix_distance": 200000.0,
          "max_matrix_location_pairs": 2500
        },
        "bus": {
          "max_distance": 5000000.0,
          "max_locations": 50,
          "max_matrix_distance": 400000.0,
          "max_matrix_location_pairs": 2500
        },
        "centroid": {
          "max_distance": 200000.0,
          "max_locations": 5
        },
        "isochrone": {
          "max_contours": 4,
          "max_distance": 25000.0,
          "max_locations": 1,
          "max_time_contour": 120,
          "max_distance_contour": 200
        },
        "max_alternates": 2,
        "max_exclude_locations": 50,
        "max_exclude_polygons_length": 10000,
        "max_radius": 200,
        "max_reachability": 100,
        "max_timedep_distance": 500000,
        "max_distance_disable_hierarchy_culling": 0,
        "motor_scooter": {
          "max_distance": 500000.0,
          "max_locations": 50,
          "max_matrix_distance": 200000.0,
          "max_matrix_location_pairs": 2500
        },
        "motorcycle": {
          "max_distance": 500000.0,
          "max_locations": 50,
          "max_matrix_distance": 200000.0,
          "max_matrix_location_pairs": 2500
        },
        "multimodal": {
          "max_distance": 500000.0,
          "max_locations": 50,
          "max_matrix_distance": 0.0,
          "max_matrix_location_pairs": 0
        },
        "pedestrian": {
          "max_distance": 250000.0,
          "max_locations": 50,
          "max_matrix_distance": 200000.0,
          "max_matrix_location_pairs": 2500,
          "max_transit_walking_distance": 10000,
          "min_transit_walking_distance": 1
        },
        "skadi": {
          "max_shape": 750000,
          "min_resample": 10.0
        },
        "status": {
          "allow_verbose": true
        },
        "taxi": {
          "max_distance": 5000000.0,
          "max_locations": 20,
          "max_matrix_distance": 400000.0,
          "max_matrix_location_pairs": 2500
        },
        "trace": {
          "max_alternates": 3,
          "max_alternates_shape": 100,
          "max_distance": 200000.0,
          "max_gps_accuracy": 100.0,
          "max_search_radius": 100.0,
          "max_shape": 16000
        },
        "transit": {
          "max_distance": 500000.0,
          "max_locations": 50,
          "max_matrix_distance": 200000.0,
          "max_matrix_location_pairs": 2500
        },
        "truck": {
          "max_distance": 5000000.0,
          "max_locations": 20,
          "max_matrix_distance": 400000.0,
          "max_matrix_location_pairs": 2500
        }
      },
      "thor": {
        "logging": {
          "color": false,
          "long_request": 110.0,
          "type": "std_out"
        },
        "service": {
          "proxy": "ipc://%%/thor"
        },
        "source_to_target_algorithm": "select_optimal"
      }
    }
  )";

  // force the paths to be different
  boost::replace_all(defaults, "%%", path_prefix);

  // make ptree and override defaults
  auto pt = json_to_pt(defaults);
  for (const auto& override : overrides) {
    pt.put(override.first, override.second);
  }

  // remove keys we dont want
  for (const auto& remove : removes) {
    remove_child(pt, remove);
  }

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

  std::filesystem::path parent_dir = std::filesystem::path(traffic_extract).parent_path();
  if (!std::filesystem::exists(parent_dir)) {
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
        setter_cb(reader, tile, index, current);
      }
      mtar_next(&tar);
    }
  }
}

#ifdef DATA_TOOLS
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
      const auto historical = cb(edges.back());
      if (historical) {
        auto coefs = valhalla::baldr::compress_speed_buckets(historical->data());
        tile.AddPredictedSpeed(edges.size() - 1, coefs, tile.header()->directededgecount());
      }
      edges.back().set_has_predicted_speed(static_cast<bool>(historical));
    }
    tile.UpdatePredictedSpeeds(edges);
  }
}

void customize_edges(const boost::property_tree::ptree& config, const EdgesCustomize& setter_cb) {
  // loop over all tiles in the tileset
  valhalla::baldr::GraphReader reader(config.get_child("mjolnir"));
  auto tile_dir = config.get<std::string>("mjolnir.tile_dir");
  for (const auto& tile_id : reader.GetTileSet()) {
    valhalla::mjolnir::GraphTileBuilder tile(tile_dir, tile_id, false);
    std::vector<valhalla::baldr::NodeInfo> nodes;
    nodes.reserve(tile.header()->nodecount());
    for (const auto& node : tile.GetNodes())
      nodes.push_back(node);

    std::vector<valhalla::baldr::DirectedEdge> edges;
    edges.reserve(tile.header()->directededgecount());

    GraphId edgeid = tile_id;
    for (size_t j = 0; j < tile.header()->directededgecount(); ++j, ++edgeid) {
      edges.push_back(tile.directededge(j));
      setter_cb(edgeid, edges.back());
    }
    tile.Update(nodes, edges);
  }
}
#endif

} // namespace test
