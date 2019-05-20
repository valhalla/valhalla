#include "test.h"

#include <iostream>
#include <string>
#include <vector>

#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/nodeinfo.h"
#include "mjolnir/graphtilebuilder.h"

#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>

using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

boost::property_tree::ptree json_to_pt(const std::string& json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  rapidjson::read_json(ss, pt);
  return pt;
}

const auto config = json_to_pt(R"({
    "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1}
  })");

} // namespace

void test_predictive_traffic() {
  boost::property_tree::ptree hierarchy_properties = config.get_child("mjolnir");
  GraphReader reader(hierarchy_properties);

  uint32_t count = 0;
  auto level = TileHierarchy::levels().rbegin();
  for (; level != TileHierarchy::levels().rend(); ++level) {
    // Create a randomized queue of tiles to work from
    auto tile_level = level->second;
    auto level_tiles = reader.GetTileSet(tile_level.level);
    for (const auto& tile_id : level_tiles) {
      const GraphTile* tile = reader.GetGraphTile(tile_id);
      uint32_t nodecount = tile->header()->nodecount();
      GraphId node = tile_id;
      for (uint32_t i = 0; i < nodecount; i++, ++node) {
        // The node we will modify
        auto ni = tile->node(i);
        uint32_t idx = ni->edge_index();
        GraphId edgeid(node.tileid(), node.level(), idx);
        for (uint32_t j = 0, n = ni->edge_count(); j < n; j++, idx++, ++edgeid) {
          auto de = tile->directededge(idx);

          if (edgeid.value == 67134432) {
            if (de->free_flow_speed() != 5)
              throw std::runtime_error("Free flow speed should be 5 for edge 67134432, but it is: " +
                                       std::to_string(de->free_flow_speed()));
            if (de->constrained_flow_speed() != 6)
              throw std::runtime_error(
                  "Constrained flow speed should be 6 for edge 67134432, but it is: " +
                  std::to_string(de->constrained_flow_speed()));
          } else if (edgeid.value == 264348364578) {
            if (de->free_flow_speed() != 13)
              throw std::runtime_error(
                  "Free fow speed should be 13 for edge 264348364578, but it is: " +
                  std::to_string(de->free_flow_speed()));
            if (de->constrained_flow_speed() != 9)
              throw std::runtime_error(
                  "Constrained flow speed should be 9 for edge 264348364578, but it is: " +
                  std::to_string(de->constrained_flow_speed()));
          }
          if (de->constrained_flow_speed() || de->free_flow_speed())
            count++;
        }
      }
    }
  }
  if (count != 23751)
    throw std::runtime_error("Incorrect number of edges updated.  Count: " + std::to_string(count));
}

int main(int argc, char* argv[]) {
  test::suite suite("predictive_traffic");
  // TODO - add this back in when updated data is available!
  //  suite.test(TEST_CASE(test_predictive_traffic));

  return suite.tear_down();
}
