#include "test.h"
#include <string>

#include "configuration.h"

namespace {

std::string make_config(const std::string& path_prefix) {
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

  return defaults;
}

TEST(Configuration, UseBeforeConfiguration) {
  EXPECT_ANY_THROW(valhalla::config());
}

TEST(Configuration, ReadInlineConfig) {
  using namespace valhalla;

  auto inline_config = make_config("/home/test");
  configuration::configure(inline_config);

  auto conf = config();

  EXPECT_EQ(conf.get<uint32_t>("mjolnir.concurrency"), 1);
  EXPECT_EQ(conf.get<std::string>("additional_data.elevation"), "/home/test/elevation/");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
