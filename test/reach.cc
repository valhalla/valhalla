#include "test.h"

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "loki/reach.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"

#include <algorithm>
#include <boost/property_tree/ptree.hpp>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;

namespace {

boost::property_tree::ptree get_conf() {
  std::stringstream ss;
  ss << R"({
      "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1},
      "loki":{
        "actions":["route"],
        "logging":{"long_request": 100},
        "service_defaults":{"minimum_reachability": 50,"radius": 0,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "heading_tolerance": 60}
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
        "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time": 120},
        "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,
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

GraphId begin_node(GraphReader& reader, const DirectedEdge* edge) {
  // grab the node
  const auto* tile = reader.GetGraphTile(edge->endnode());
  const auto* node = tile->node(edge->endnode());
  // grab the opp edges end node
  const auto* opp_edge = tile->directededge(node->edge_index() + edge->opp_index());
  return opp_edge->endnode();
}

void check_all_reach() {
  // get tile access
  auto conf = get_conf();
  GraphReader reader(conf.get_child("mjolnir"));

  // use basic car filters
  auto edge_filter = [](const DirectedEdge* e) { return e->forwardaccess() & kAutoAccess; };
  auto node_filter = [](const NodeInfo* n) { return !(n->access() & kAutoAccess); };

  // look at all the edges
  for (auto tile_id : reader.GetTileSet()) {
    const auto* tile = reader.GetGraphTile(tile_id);
    // loop over edges
    for (GraphId edge_id = tile->header()->graphid();
         edge_id.id() < tile->header()->directededgecount(); ++edge_id) {
      // use the simple method to find the reach for the edge in both directions
      const auto* edge = tile->directededge(edge_id);
      auto reach = SimpleReach(edge, 50, reader, edge_filter, node_filter, kInbound | kOutbound);

      // shape is nice to have
      auto shape = tile->edgeinfo(edge->edgeinfo_offset()).shape();
      if (!edge->forward())
        std::reverse(shape.begin(), shape.end());
      auto shape_str = midgard::encode(shape);

      // begin and end nodes are nice to check
      auto node_id = begin_node(reader, edge);
      const auto* t = reader.GetGraphTile(node_id);
      const auto* begin = t->node(node_id);
      t = reader.GetGraphTile(edge->endnode());
      const auto* end = t->node(edge->endnode());

      // if you have non zero reach but you dont have access, something is wrong
      if ((reach.outbound > 0 || reach.inbound > 0) && !(edge->forwardaccess() & kAutoAccess) &&
          !(edge->reverseaccess() & kAutoAccess)) {
        throw std::logic_error("This edge should have 0 reach as its not accessable: " +
                               std::to_string(edge_id.value) + " " + shape_str);
      }

      // if inbound is 0 and outbound is not then it must be an edge leaving a dead end
      // meaning a begin node that is not accessable
      if (reach.inbound == 0 && reach.outbound > 0 && !node_filter(begin)) {
        throw std::logic_error("Only outbound reach should mean an edge that leaves a dead end: " +
                               std::to_string(edge_id.value) + " " + shape_str);
      }

      // if outbound is 0 and inbound is not then it must be an edge entering a dead end
      // meaning an end node that is not accessable
      if (reach.inbound > 0 && reach.outbound == 0 && !node_filter(end)) {
        throw std::logic_error("Only inbound reach should mean an edge that enters a dead end: " +
                               std::to_string(edge_id.value) + " " + shape_str);
      }
    }
  }
}

} // namespace

int main(int argc, char* argv[]) {
  test::suite suite("reach");

  logging::Configure({{"type", ""}});

  suite.test(TEST_CASE(check_all_reach));

  return suite.tear_down();
}
