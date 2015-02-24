
#include "mjolnir/pbfadminparser.h"
#include "mjolnir/util.h"
#include "mjolnir/osmpbfparser.h"

#include <future>
#include <utility>
#include <thread>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/mjolnir/osmadmin.h>
#include <valhalla/mjolnir/luatagtransform.h>
#include <valhalla/mjolnir/idtable.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {
// Will throw an error if this is exceeded. Then we can increase.
const uint64_t kMaxOSMNodeId = 4000000000;

struct admin_callback : public OSMPBF::Callback {
 public:
  admin_callback() = delete;
  admin_callback(const admin_callback&) = delete;
  virtual ~admin_callback() {}
  // Construct PBFAdminParser based on properties file and input PBF extract
  admin_callback(const boost::property_tree::ptree& pt, OSMData& osmdata)
      : shape_(kMaxOSMNodeId), ways_(kMaxOSMNodeId), osmdata_(osmdata) {

    // Initialize Lua based on config
    lua_.SetLuaNodeScript(pt.get<std::string>("admintagtransform.node_script"));
    lua_.SetLuaNodeFunc(pt.get<std::string>("admintagtransform.node_function"));
    lua_.SetLuaWayScript(pt.get<std::string>("admintagtransform.way_script"));
    lua_.SetLuaWayFunc(pt.get<std::string>("admintagtransform.way_function"));
    lua_.SetLuaRelationScript(pt.get<std::string>("admintagtransform.relation_script"));
    lua_.SetLuaRelationFunc(pt.get<std::string>("admintagtransform.relation_function"));
    lua_.OpenLib();
  }

  void node_callback(uint64_t osmid, double lng, double lat, const OSMPBF::Tags &tags) {
    // Check if it is in the list of nodes used by ways
    if (!shape_.IsUsed(osmid)) {
      return;
    }

    // Get tags
    auto results = lua_.Transform(OSMType::kNode, tags);
    if (results.size() == 0)
      return;

    // Create a new node and set its attributes
    osmdata_.nodes.emplace_back(osmid, lng, lat);

    if (osmdata_.nodes.size() % 5000000 == 0) {
      LOG_INFO("Processed " + std::to_string(osmdata_.nodes.size()) + " nodes on ways");
    }
  }

  void way_callback(uint64_t osmid, const OSMPBF::Tags &tags, const std::vector<uint64_t> &nodes) {

    // Check if it is in the list of ways used by relations
    if (!ways_.IsUsed(osmid)) {
      return;
    }

    // Transform tags. If no results that means the way does not have tags
    // suitable for use in routing.
    auto results = lua_.Transform(OSMType::kWay, tags);
    if (results.size() == 0) {
      return;
    }

    // Add the refs to the node reference list
    uint32_t idx = osmdata_.noderefs.size();
    for (const auto node : nodes) {
      osmdata_.noderefs.push_back(node);
      ++osmdata_.node_count;
      // Mark the nodes that we will care about when processing nodes
      shape_.set(node);
    }

    // Construct OSMWay and set the node ref index and count
    OSMWay w(osmid);
    w.set_noderef_index(idx);
    w.set_node_count(nodes.size());

    // Add the way to the list
    osmdata_.ways.push_back(std::move(w));
  }

  void relation_callback(const uint64_t osmid, const OSMPBF::Tags &tags, const std::vector<OSMPBF::Member> &members) {
    // Get tags
    auto results = lua_.Transform(OSMType::kRelation, tags);
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
    for (const auto& member : members) {

      if (member.member_type == OSMPBF::Relation::MemberType::Relation_MemberType_WAY) {
        ways_.set(member.member_id);
        memberids_.push_back(member.member_id);
      }
    }

    admin.set_member_index(idx);
    admin.set_member_count(members.size());

    admins_.push_back(std::move(admin));
  }

// Lua Tag Transformation class
LuaTagTransform lua_;

// Mark the OSM Ids used by the ways and relations
IdTable shape_, ways_;

// Pointer to all the OSM data (for use by callbacks)
OSMData& osmdata_;

// Vector of all the admin member/way ids.
std::vector<uint64_t> memberids_;

// Vector of admins.
std::vector<OSMAdmin> admins_;

};

}

namespace valhalla {
namespace mjolnir {

OSMData PBFAdminParser::Parse(const boost::property_tree::ptree& pt, const std::vector<std::string>& input_files) {
  // Create OSM data. Set the member pointer so that the parsing callback
  // methods can use it.
  OSMData osmdata{};
  admin_callback callback(pt, osmdata);
  OSMPBF::Parser parser(callback);

  // Parse each input file - first pass
  for (const auto& input_file : input_files) {
    // Parse relations.
    auto t1 = std::chrono::high_resolution_clock::now();
    parser.parse(input_file, OSMPBF::Interest::RELATIONS);
    auto t2 = std::chrono::high_resolution_clock::now();
    uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
    LOG_INFO("Parsing relations took " + std::to_string(msecs) + " ms");
    LOG_INFO("Admin count = " + std::to_string(callback.admins_.size()));
    LOG_INFO("Member/ways count = " + std::to_string(callback.memberids_.size()));

    // Parse the ways. Find all node Ids needed. Shrink the OSM ways vector
    // and the OSM node reference vector (list of nodes that the ways include).
    t1 = std::chrono::high_resolution_clock::now();
    parser.parse(input_file, OSMPBF::Interest::WAYS);
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
    osmdata.nodes.reserve(osmdata.node_count);
    parser.parse(input_file, OSMPBF::Interest::NODES);
    auto t2 = std::chrono::high_resolution_clock::now();
    uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
    LOG_INFO("Parsing nodes took " + std::to_string(msecs) + " ms");
    LOG_INFO("Nodes included on Admin ways, count = " + std::to_string(osmdata.nodes.size()));
  }

  // Sort the OSM nodes vector by OSM Id
  std::sort(osmdata.nodes.begin(), osmdata.nodes.end());

  // Return OSM data
  return osmdata;
}

}
}
