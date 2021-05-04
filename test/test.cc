#include "test.h"

#include "baldr/graphreader.h"
#include "baldr/predictedspeeds.h"
#include "baldr/rapidjson_utils.h"

#include <random>
#include <sstream>
#include <stdexcept>
#include <string>

#include <boost/algorithm/string.hpp>

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
        "tile_extract": "%%/tiles.tar",
        "timezone": "%%/tz_world.sqlite",
        "traffic_extract": "%%/traffic.tar",
        "transit_dir": "%%/transit",
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
          "max_matrix_locations": 50
        },
        "auto_shorter": {
          "max_distance": 5000000.0,
          "max_locations": 20,
          "max_matrix_distance": 400000.0,
          "max_matrix_locations": 50
        },
        "bicycle": {
          "max_distance": 500000.0,
          "max_locations": 50,
          "max_matrix_distance": 200000.0,
          "max_matrix_locations": 50
        },
        "bikeshare": {
          "max_distance": 500000.0,
          "max_locations": 50,
          "max_matrix_distance": 200000.0,
          "max_matrix_locations": 50
        },
        "bus": {
          "max_distance": 5000000.0,
          "max_locations": 50,
          "max_matrix_distance": 400000.0,
          "max_matrix_locations": 50
        },
        "centroid": {
          "max_distance": 200000.0,
          "max_locations": 5
        },
        "hov": {
          "max_distance": 5000000.0,
          "max_locations": 20,
          "max_matrix_distance": 400000.0,
          "max_matrix_locations": 50
        },
        "isochrone": {
          "max_contours": 4,
          "max_distance": 25000.0,
          "max_locations": 1,
          "max_time_contour": 120,
          "max_distance_contour": 200
        },
        "max_alternates": 2,
        "max_avoid_locations": 50,
        "max_avoid_polygons_length": 10000,
        "max_radius": 200,
        "max_reachability": 100,
        "max_timedep_distance": 500000,
        "motor_scooter": {
          "max_distance": 500000.0,
          "max_locations": 50,
          "max_matrix_distance": 200000.0,
          "max_matrix_locations": 50
        },
        "motorcycle": {
          "max_distance": 500000.0,
          "max_locations": 50,
          "max_matrix_distance": 200000.0,
          "max_matrix_locations": 50
        },
        "multimodal": {
          "max_distance": 500000.0,
          "max_locations": 50,
          "max_matrix_distance": 0.0,
          "max_matrix_locations": 0
        },
        "pedestrian": {
          "max_distance": 250000.0,
          "max_locations": 50,
          "max_matrix_distance": 200000.0,
          "max_matrix_locations": 50,
          "max_transit_walking_distance": 10000,
          "min_transit_walking_distance": 1
        },
        "skadi": {
          "max_shape": 750000,
          "min_resample": 10.0
        },
        "taxi": {
          "max_distance": 5000000.0,
          "max_locations": 20,
          "max_matrix_distance": 400000.0,
          "max_matrix_locations": 50
        },
        "trace": {
          "max_best_paths": 4,
          "max_best_paths_shape": 100,
          "max_distance": 200000.0,
          "max_gps_accuracy": 100.0,
          "max_search_radius": 100.0,
          "max_shape": 16000
        },
        "transit": {
          "max_distance": 500000.0,
          "max_locations": 50,
          "max_matrix_distance": 200000.0,
          "max_matrix_locations": 50
        },
        "truck": {
          "max_distance": 5000000.0,
          "max_locations": 20,
          "max_matrix_distance": 400000.0,
          "max_matrix_locations": 50
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

} // namespace test
