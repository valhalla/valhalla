#include <array>
#include <iostream>
#include <random>
#include <string>

#include "baldr/graphreader.h"
#include "loki/search.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "sif/autocost.h"
#include "sif/costfactory.h"
#include "test.h"
#include "thor/bidirectional_astar.h"
#include "thor/unidirectional_astar.h"
#include <valhalla/proto/options.pb.h>

using namespace valhalla;

namespace {

// TODO: only support one edge id each time. To be improve performance, List shall be supported
void customize_traffic(const boost::property_tree::ptree& config,
                       baldr::GraphId& target_edge_id,
                       const int target_speed) {
  // Make some updates to the traffic .tar file.
  // Generate traffic data
  std::function<void(baldr::GraphReader&, baldr::TrafficTile&, int, baldr::TrafficSpeed*)>
      generate_traffic = [&target_edge_id, &target_speed](baldr::GraphReader& reader,
                                                          baldr::TrafficTile& tile, int index,
                                                          baldr::TrafficSpeed* current) -> void {
    baldr::GraphId tile_id(tile.header->tile_id);
    auto edge_id = baldr::GraphId(tile_id.tileid(), tile_id.level(), index);
    if (edge_id == target_edge_id) {
      current->breakpoint1 = 255;
      current->overall_encoded_speed = target_speed >> 1;
      current->encoded_speed1 = target_speed >> 1;
    }
  };
  test::customize_live_traffic_data(config, generate_traffic);
}

} // namespace

int main(int argc, char** argv) {

  boost::property_tree::ptree pt;
  rapidjson::read_json("/home/router/valhalla/data/bw/valhalla_bw.json", pt);

  // Build the default traffic data with invalid values
  // test::build_live_traffic_data(pt);

  // Boebingen, Lessingstraße: way id: 13865735: 0,621568686498,1,621770013090
  auto tgt_edge_id = baldr::GraphId{621568686498};
  customize_traffic(pt, tgt_edge_id, 30); // original: 30

  tgt_edge_id = baldr::GraphId{621770013090};
  customize_traffic(pt, tgt_edge_id, 30); // original: 30

  // STR > Sifi A81: 309175327,1,3802388259096
  tgt_edge_id = baldr::GraphId{3802388259096};
  customize_traffic(pt, tgt_edge_id, 0); // original: 100

  // auto tgt_edge_id = baldr::GraphId{3802388259096};

  // Read the information
  GraphReader reader(pt.get_child("mjolnir"));
  auto tile = reader.GetGraphTile(baldr::GraphId(tgt_edge_id));
  auto edge = tile->directededge(tgt_edge_id);

  LOG_INFO("speed: " + std::to_string(tile->GetSpeed(edge, 255, 1)));
  LOG_INFO("wayid: " + std::to_string(tile->edgeinfo(edge).wayid()));
}
