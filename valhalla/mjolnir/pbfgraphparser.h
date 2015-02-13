#ifndef VALHALLA_MJOLNIR_PBFGRAPHPARSER_H
#define VALHALLA_MJOLNIR_PBFGRAPHPARSER_H

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <set>
#include <utility>
#include <algorithm>
#include <memory>
#include <boost/property_tree/ptree.hpp>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmway.h>
#include <valhalla/mjolnir/osmdata.h>
#include <valhalla/mjolnir/idtable.h>

#include <valhalla/mjolnir/luatagtransform.h>

// Forward declare
namespace CanalTP {
  class Reference;
}

namespace valhalla {
namespace mjolnir {

/**
 * Class used to parse OSM protocol buffer extracts.
 */
class PBFGraphParser {
 public:
  //not default constructable or copyable
  PBFGraphParser() = delete;
  PBFGraphParser(const PBFGraphParser&) = delete;

  /**
   * Constructor
   */
  PBFGraphParser(const boost::property_tree::ptree& pt);

  /**
   * Loads a given input file
   */
  OSMData Load(const std::vector<std::string>& input_file);

  //TODO: put these callbacks inside of an pimpl or just rewrite
  //the bits of canalTP that we care about

  /**
   * Callback method for OSMPBFReader. Called when a node is parsed.
   */
  void node_callback(uint64_t osmid, double lng, double lat,
                     const Tags &/*tags*/);

  /**
   * Callback method for OSMPBFReader. Called when a way is parsed.
   */
  void way_callback(uint64_t osmid, const Tags &tags,
                    const std::vector<uint64_t> &refs);

  /**
   * Callback method for OSMPBFReader. Called when a relation is parsed.
   */
  void relation_callback(uint64_t /*osmid*/, const Tags &tags,
                         const std::vector<CanalTP::Reference> &refs);

 protected:

  /**
   * Initialize Lua with the scripts and functions.
   */
  void LuaInit(const std::string& nodetagtransformscript,
               const std::string& nodetagtransformfunction,
               const std::string& waytagtransformscript,
               const std::string& waytagtransformfunction,
               const std::string& reltagtransformscript,
               const std::string& reltagtransformfunction);

  size_t speed_assignment_count_;

  // List of the tile levels to be created
  baldr::TileHierarchy tile_hierarchy_;

  // Lua Tag Transformation class
  LuaTagTransform lua_;

  // Mark the OSM Node Ids used by ways
  IdTable shape_, intersection_;

  // Pointer to all the OSM data (for use by callbacks)
  OSMData* osm_;

  // How many threads to run
  const unsigned int threads_;
};

}
}

#endif  // VALHALLA_MJOLNIR_PBFGRAPHPARSER_H
