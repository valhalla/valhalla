#include "test.h"

#include <iostream>
#include <string>
#include <vector>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "midgard/encoded.h"
#include "midgard/util.h"
#include "src/baldr/shortcut_recovery.h"
#include <boost/property_tree/ptree.hpp>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace std {
std::string to_string(const midgard::PointLL& p) {
  return "[" + to_string(p.first) + "," + to_string(p.second) + "]";
}
} // namespace std

boost::property_tree::ptree get_conf() {
  std::stringstream ss;
  ss << R"({
      "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1},
      "loki":{
        "actions":["route"],
        "logging":{"long_request": 100},
        "service_defaults":{"minimum_reachability": 50,"radius": 0,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "street_side_max_distance": 1000, "heading_tolerance": 60}
      },
      "thor":{"logging":{"long_request": 100}},
      "odin":{"logging":{"long_request": 100}},
      "skadi":{"actons":["height"],"logging":{"long_request": 5}},
      "meili":{"customizable": ["turn_penalty_factor","max_route_distance_factor","max_route_time_factor","search_radius"],
              "mode":"auto","grid":{"cache_size":100240,"size":500},
              "default":{"beta":3,"breakage_distance":2000,"geometry":false,"gps_accuracy":5.0,"interpolation_distance":10,
              "max_route_distance_factor":5,"max_route_time_factor":5,"max_search_radius":200,"route":true,
              "search_radius":15.0,"sigma_z":4.07,"turn_penalty_factor":200}},
      "service_limits": {
        "auto": {"max_distance": 5000000.0, "max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "auto_shorter": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "bicycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
        "bus": {"max_distance": 5000000.0,"max_locations": 50,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "hov": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time_contour": 120, "max_distance_contour":200},
        "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,"max_avoid_polygons_length":100,
        "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
        "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
        "skadi": {"max_shape": 750000,"min_resample": 10.0},
        "trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
        "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
        "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
      }
    })";
  boost::property_tree::ptree conf;
  rapidjson::read_json(ss, conf);
  return conf;
}

// expose the constructor
struct testable_recovery : public shortcut_recovery_t {
  testable_recovery(GraphReader* reader) : shortcut_recovery_t(reader) {
  }
};

void recover(bool cache) {
  auto conf = get_conf();
  GraphReader graphreader(conf.get_child("mjolnir"));
  testable_recovery recovery{cache ? &graphreader : nullptr};
  size_t total = 0;
  size_t bad = 0;

  // for each tile set per level
  for (const auto& level : TileHierarchy::levels()) {
    // we dont get shortcuts on level 2 and up
    if (level.level > 1)
      continue;

    // for each tile in the tile set
    auto tileset = graphreader.GetTileSet(level.level);
    for (const auto tileid : tileset) {
      printf("bad: %zu, total: %zu\n", bad, total);
      if (graphreader.OverCommitted())
        graphreader.Trim();

      // for each edge in the tile
      auto tile = graphreader.GetGraphTile(tileid);
      for (size_t j = 0; j < tile->header()->directededgecount(); ++j) {
        // skip it if its not a shortcut or the shortcut is one we will never traverse
        const auto* edge = tile->directededge(j);
        if (!edge->is_shortcut() ||
            (!(edge->forwardaccess() & kAutoAccess) && !(edge->reverseaccess() & kAutoAccess)))
          continue;

        // we'll have the shape to compare to
        auto shortcut_shape = tile->edgeinfo(edge->edgeinfo_offset()).shape();
        if (!edge->forward())
          std::reverse(shortcut_shape.begin(), shortcut_shape.end());

        // make a graph id out of the shortcut to send to recover
        auto shortcutid = tileid;
        shortcutid.set_id(j);
        auto edgeids = recovery.get(shortcutid, graphreader);

        // if it gave us back the shortcut we failed
        if (edgeids.front() == shortcutid) {
          //          FAIL() << "We couldnt recover the shortcut\nShortcut was: " +
          //              midgard::encode(shortcut_shape);
          ++bad;
          ++total;
          continue;
        }

        // accumulate the shape along the edges that we recovered
        std::vector<PointLL> recovered_shape;
        for (auto edgeid : edgeids) {
          auto tile = graphreader.GetGraphTile(edgeid);
          const auto* de = tile->directededge(edgeid);
          auto de_shape = tile->edgeinfo(de->edgeinfo_offset()).shape();
          if (!de->forward()) {
            std::reverse(de_shape.begin(), de_shape.end());
          }
          recovered_shape.insert(recovered_shape.end(),
                                 (de_shape.begin() + (recovered_shape.size() == 0 ? 0 : 1)),
                                 de_shape.end());
        }

        // check the number of coords match
        if (shortcut_shape.size() != recovered_shape.size()) {
          //          FAIL() << "shape lengths do not match: " + std::to_string(shortcut_shape.size())
          //          +
          //              " != " + std::to_string(recovered_shape.size()) +
          //              "\nShortcut was: " + midgard::encode(shortcut_shape) +
          //              "\nRecovered was: " + midgard::encode(recovered_shape);
          ++bad;
          ++total;
          continue;
        }

        // check if the shape matches approximatly
        for (size_t k = 0; k < shortcut_shape.size(); ++k) {
          if (!shortcut_shape[k].ApproximatelyEqual(recovered_shape[k])) {
            //            FAIL() << "edge shape points are not equal: " +
            //            std::to_string(shortcut_shape[k]) +
            //                " != " + std::to_string(recovered_shape[k]) +
            //                "\nShortcut was: " + midgard::encode(shortcut_shape) +
            //                "\nRecovered was: " + midgard::encode(recovered_shape);
            ++bad;
            break;
          }
        }
        ++total;
      }
    }
  }
  printf("bad: %zu, total: %zu\n", bad, total);
  EXPECT_LE(double(bad) / double(total), .001) << "More than 0.1% is too much";
}

TEST(RecoverShortcut, test_recover_shortcut_edges_no_cache) {
  recover(false);
}

TEST(RecoverShortcut, test_recover_shortcut_edges_cache) {
  recover(true);
}

int main(int argc, char* argv[]) {
  // valhalla::midgard::logging::Configure({{"type", ""}});
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
