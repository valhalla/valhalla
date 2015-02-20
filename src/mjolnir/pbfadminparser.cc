
#include "mjolnir/pbfadminparser.h"
#include "mjolnir/util.h"

// Use open source PBF reader from:
//     https://github.com/CanalTP/libosmpbfreader
#include "osmpbfreader.h"

#include <future>
#include <utility>
#include <thread>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <valhalla/midgard/logging.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Will throw an error if this is exceeded. Then we can increase.
const uint64_t kMaxOSMNodeId = 4000000000;

// Construct PBFAdminParser based on properties file and input PBF extract
PBFAdminParser::PBFAdminParser(const boost::property_tree::ptree& pt)
    : shape_(kMaxOSMNodeId),
      ways_(kMaxOSMNodeId) {

  // Initialize Lua based on config
  LuaInit(pt.get<std::string>("admintagtransform.node_script"),
          pt.get<std::string>("admintagtransform.node_function"),
          pt.get<std::string>("admintagtransform.way_script"),
          pt.get<std::string>("admintagtransform.way_function"),
          pt.get<std::string>("admintagtransform.relation_script"),
          pt.get<std::string>("admintagtransform.relation_function"));
}

OSMData PBFAdminParser::Load(const std::vector<std::string>& input_files) {
  // Create OSM data. Set the member pointer so that the parsing callback
  // methods can use it.
  OSMData osmdata{};
  osm_ = &osmdata;

  // Parse each input file - first pass
  for (const auto& input_file : input_files) {

    // Parse relations.
    auto t1 = std::chrono::high_resolution_clock::now();
    CanalTP::read_osm_pbf(input_file, *this, CanalTP::Interest::RELATIONS);
    auto t2 = std::chrono::high_resolution_clock::now();
    uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
    LOG_INFO("Parsing relations took " + std::to_string(msecs) + " ms");
    LOG_INFO("Admin count = " +
             std::to_string(admins_.size()));
    LOG_INFO("Member/ways count = " +
             std::to_string(memberids_.size()));

    // Parse the ways. Find all node Ids needed. Shrink the OSM ways vector
    // and the OSM node reference vector (list of nodes that the ways include).
    t1 = std::chrono::high_resolution_clock::now();
    CanalTP::read_osm_pbf(input_file, *this, CanalTP::Interest::WAYS);
    osmdata.ways.shrink_to_fit();
    osmdata.noderefs.shrink_to_fit();
    t2 = std::chrono::high_resolution_clock::now();
    msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
    LOG_INFO("Parsing ways took " + std::to_string(msecs) + " ms");
    LOG_INFO("Admin ways count = " + std::to_string(osmdata.ways.size()));
    LOG_INFO("Number of noderefs = " + std::to_string(osmdata.noderefs.size()));
  }

  // Parse node in all the input files. Skip any that are not marked from
  // being used in a way.
  // TODO: we know how many knows we expect, stop early once we have that many
  for (const auto& input_file : input_files) {
    auto t1 = std::chrono::high_resolution_clock::now();
    LOG_INFO("Parsing nodes but only keeping " + std::to_string(osmdata.node_count));
    osmdata.ReserveNodes(osmdata.node_count);
    CanalTP::read_osm_pbf(input_file, *this, CanalTP::Interest::NODES);
    auto t2 = std::chrono::high_resolution_clock::now();
    uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
    LOG_INFO("Parsing nodes took " + std::to_string(msecs) + " ms");
    LOG_INFO("Nodes included on Admin ways, count = " +
             std::to_string(osmdata.node_map.size()));
  }

  // Return OSM data
  return osmdata;
}

// Initialize Lua tag transformations
void PBFAdminParser::LuaInit(const std::string& nodetagtransformscript,
                           const std::string& nodetagtransformfunction,
                           const std::string& waytagtransformscript,
                           const std::string& waytagtransformfunction,
                           const std::string& reltagtransformscript,
                           const std::string& reltagtransformfunction) {
  lua_.SetLuaNodeScript(nodetagtransformscript);
  lua_.SetLuaNodeFunc(nodetagtransformfunction);
  lua_.SetLuaWayScript(waytagtransformscript);
  lua_.SetLuaWayFunc(waytagtransformfunction);
  lua_.SetLuaRelationScript(reltagtransformscript);
  lua_.SetLuaRelationFunc(reltagtransformfunction);
  lua_.OpenLib();
}

void PBFAdminParser::node_callback(uint64_t osmid, double lng, double lat,
                                 const Tags &tags) {
  // Check if it is in the list of nodes used by ways
  if (!shape_.IsUsed(osmid)) {
    return;
  }

  // Get tags
  Tags results = lua_.TransformInLua(OSMType::kNode, tags);
  if (results.size() == 0)
    return;

  // Create a new node and set its attributes
  OSMNode* n = osm_->WriteNode(osmid);
  n->set_latlng(std::move(std::make_pair(lng, lat)));

  if (osm_->node_map.size() % 5000000 == 0) {
    LOG_INFO("Processed " + std::to_string(osm_->node_map.size()) + " nodes on ways");
  }
}

void PBFAdminParser::way_callback(uint64_t osmid, const Tags &tags,
                                const std::vector<uint64_t> &refs) {

  // Check if it is in the list of ways used by relations
  if (!ways_.IsUsed(osmid)) {
    return;
  }

  // Transform tags. If no results that means the way does not have tags
  // suitable for use in routing.
  Tags results = lua_.TransformInLua(OSMType::kWay, tags);
  if (results.size() == 0) {
    return;
  }

  // Add the refs to the node reference list
  uint32_t idx = osm_->noderefs.size();
  for (const auto ref : refs) {
    osm_->noderefs.push_back(ref);
    ++osm_->node_count;
    // Mark the nodes that we will care about when processing nodes
    shape_.set(ref);
  }

  // Construct OSMWay and set the node ref index and count
  OSMWay w(osmid);
  w.set_noderef_index(idx);
  w.set_node_count(refs.size());

  // Add the way to the list
  osm_->ways.push_back(std::move(w));
}

void PBFAdminParser::relation_callback(uint64_t osmid, const Tags &tags,
                                     const CanalTP::References &refs) {
  // Get tags
  Tags results = lua_.TransformInLua(OSMType::kRelation, tags);
  if (results.size() == 0)
    return;

  OSMAdmin admin(osmid);
  uint64_t from_way_id = 0;

  for (const auto& tag : results) {

    if (tag.first == "name")
      admin.set_name(tag.second);
    else if (tag.first == "admin_level")
      admin.set_admin_level(std::stoi(tag.second));

  }

  uint32_t idx = memberids_.size();
  for (const auto& ref : refs) {

    if (ref.member_type == OSMPBF::Relation::MemberType::Relation_MemberType_WAY) {
      ways_.set(ref.member_id);
      memberids_.push_back(ref.member_id);
    }
  }

  admin.set_member_index(idx);
  admin.set_member_count(refs.size());

  admins_.push_back(std::move(admin));
}

}
}
