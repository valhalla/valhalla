#ifndef VALHALLA_MJOLNIR_GRAPHBUILDER_H
#define VALHALLA_MJOLNIR_GRAPHBUILDER_H

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/baldr/signinfo.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/mjolnir/osmdata.h>
#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmway.h>

namespace valhalla {
namespace mjolnir {

using boost::property_tree::ptree;

/**
 * Class used to construct temporary data used to build the initial graph.
 */
class GraphBuilder {
public:
  /**
   * Tell the builder to build the tiles from the provided datasource
   * and configs
   * @param  config                         properties file
   * @param  osmdata                        OSM data used to build the graph.
   * @param  ways_file                      where to store the ways so they are not in memory
   * @param  way_nodes_file                 where to store the nodes so they are not in memory
   * @param  nodes_file                     where to store node information so it isn't in memory
   * @param  edges_file                     where to store edge information so it isn't in memory
   * @param  complex_from_restriction_file  where to store the from complex restrictions so they are
   * not in memory
   * @param  complex_to_restriction_file    where to store the to complex restrictions so they are not
   * in memory
   */
  static void Build(const ptree& pt,
                    const OSMData& osmdata,
                    const std::string& ways_file,
                    const std::string& way_nodes_file,
                    const std::string& nodes_file,
                    const std::string& edges_file,
                    const std::string& complex_from_restriction_file,
                    const std::string& complex_to_restriction_file,
                    const std::map<baldr::GraphId, size_t>& tiles);

  static std::map<baldr::GraphId, size_t> BuildEdges(const ptree& conf,
                                                     const OSMData& osmdata,
                                                     const std::string& ways_file,
                                                     const std::string& way_nodes_file,
                                                     const std::string& nodes_file,
                                                     const std::string& edges_file);

  static std::string GetRef(const std::string& way_ref, const std::string& relation_ref);

  static bool CreateSignInfoList(const OSMNode& node,
                                 const OSMWay& way,
                                 const OSMData& osmdata,
                                 std::vector<baldr::SignInfo>& exits,
                                 bool fork,
                                 bool forward,
                                 bool ramp,
                                 bool tc);
};

// The tile manifest is a JSON-serializable index of tiles to be processed during the build stage of
// valhalla_build_tiles'. It can be used to distribute shard keys when building tiles with
// parallelized, distributed batch processing. For example, a workflow orchestrator can partition
// and distribute this manifest to workers in a 'build' stage worker pool.
//
// This file written out during 'constructedges' and a prerequisite for the 'build' stage run by
// valhalla_build_tiles.
//
// Example manifest :
//
// {
//   "tiles": [
//     {
//       "node_index": 0,
//       "graphid": {
//         "value": 5970538,
//         "id": 0,
//         "tile_id": 746317,
//         "level": 2
//       }
//     }
//   ]
// }
struct TileManifest {

  std::map<baldr::GraphId, size_t> tileset;

  std::string ToString() const {
    baldr::json::ArrayPtr array = baldr::json::array({});
    for (const auto& tile : tileset) {
      const baldr::json::Value& graphid = tile.first.json();
      const baldr::json::MapPtr& item = baldr::json::map(
          {{"graphid", graphid}, {"node_index", static_cast<uint64_t>(tile.second)}});
      array->emplace_back(item);
    }
    std::stringstream manifest;
    manifest << *baldr::json::map({{"tiles", array}});
    return manifest.str();
  }

  void LogToFile(const std::string& filename) const {
    std::ofstream handle;
    handle.open(filename);
    handle << ToString();
    handle.close();
    LOG_INFO("Writing tile manifest to " + filename);
  }

  static TileManifest ReadFromFile(const std::string& filename) {
    ptree manifest;
    rapidjson::read_json(filename, manifest);
    LOG_INFO("Reading tile manifest from " + filename);
    std::map<baldr::GraphId, size_t> tileset;
    for (const auto& tile_info : manifest.get_child("tiles")) {
      const ptree& graph_id = tile_info.second.get_child("graphid");
      const baldr::GraphId id(graph_id.get<uint64_t>("value"));
      const size_t node_index = tile_info.second.get<size_t>("node_index");
      tileset.insert({id, node_index});
    }
    LOG_INFO("Reading " + std::to_string(tileset.size()) + " tiles from tile manifest file " +
             filename);
    return TileManifest{tileset};
  }
}; // namespace mjolnir

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_GRAPHBUILDER_H
