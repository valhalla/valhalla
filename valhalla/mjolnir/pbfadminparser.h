#ifndef VALHALLA_MJOLNIR_PBFADMINPARSER_H
#define VALHALLA_MJOLNIR_PBFADMINPARSER_H

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <boost/property_tree/ptree.hpp>

#include <valhalla/midgard/pointll.h>
#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmway.h>
#include <valhalla/mjolnir/osmadmin.h>
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
 * Class used to parse OSM administrative protocol buffer extracts.
 */
class PBFAdminParser {
 public:
  //not default constructable or copyable
  PBFAdminParser() = delete;
  PBFAdminParser(const PBFAdminParser&) = delete;

  /**
   * Constructor
   */
  PBFAdminParser(const boost::property_tree::ptree& pt);

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

  // Vector of all the admin member/way ids.
  std::vector<uint64_t> memberids_;

  // Vector of admins.
  std::vector<OSMAdmin> admins_;

  std::unordered_map<uint64_t, OSMWay> admin_ways_;

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

  // Lua Tag Transformation class
  LuaTagTransform lua_;

  // Mark the OSM Ids used by the ways and relations
  IdTable shape_, ways_;

  // Pointer to all the OSM data (for use by callbacks)
  OSMData* osm_;

};

}
}

#endif  // VALHALLA_MJOLNIR_PBFADMINPARSER_H
