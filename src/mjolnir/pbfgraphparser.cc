
#include "mjolnir/pbfadminparser.h"
#include "mjolnir/util.h"
#include "mjolnir/osmpbfparser.h"
#include "mjolnir/sequence.h"
#include "mjolnir/osmadmin.h"
#include "mjolnir/luatagtransform.h"
#include "mjolnir/idtable.h"

#include <future>
#include <utility>
#include <thread>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {
// Will throw an error if this is exceeded. Then we can increase.
constexpr uint64_t kMaxOSMNodeId = 4000000000;

// Node equality
const auto WayNodeEquals = [](const OSMWayNode& a, const OSMWayNode& b) {
  return a.node_id == b.node_id;
};

struct admin_callback : public OSMPBF::Callback {
 public:
  admin_callback() = delete;
  admin_callback(const admin_callback&) = delete;
  virtual ~admin_callback() {}
  // Construct PBFAdminParser based on properties file and input PBF extract
  admin_callback(const boost::property_tree::ptree& pt, OSMData& osmdata)
      : shape_(kMaxOSMNodeId), members_(kMaxOSMNodeId), osmdata_(osmdata) {

    // Initialize Lua based on config
    lua_.SetLuaNodeScript(pt.get<std::string>("admintagtransform.node_script"));
    lua_.SetLuaNodeFunc(pt.get<std::string>("admintagtransform.node_function"));
    lua_.SetLuaWayScript(pt.get<std::string>("admintagtransform.way_script"));
    lua_.SetLuaWayFunc(pt.get<std::string>("admintagtransform.way_function"));
    lua_.SetLuaRelationScript(pt.get<std::string>("admintagtransform.relation_script"));
    lua_.SetLuaRelationFunc(pt.get<std::string>("admintagtransform.relation_function"));
    lua_.OpenLib();

    current_way_node_index_ = last_node_ = last_way_ = last_relation_ = 0;
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

    //unsorted extracts are just plain nasty, so they can bugger off!
    if(osmid < last_node_)
      throw std::runtime_error("Detected unsorted input data");
    last_node_ = osmid;

    //find a node we need to update
    current_way_node_index_ = way_nodes_->find_first_of(OSMWayNode{osmid}, WayNodeEquals, current_way_node_index_);
    //we found the first one
    if(current_way_node_index_ < way_nodes_->size()) {
      //update all the nodes that match it
      OSMWayNode way_node;
      sequence_element<OSMWayNode> element = (*way_nodes_)[current_way_node_index_];
      while(current_way_node_index_ < way_nodes_->size() && (way_node = element = (*way_nodes_)[current_way_node_index_]).node_id == osmid) {
        way_node.node.lng = static_cast<float>(lng);
        way_node.node.lat = static_cast<float>(lat);
        element = way_node;
        ++current_way_node_index_;
      }

      if (++osmdata_.osm_node_count % 5000000 == 0) {
        LOG_INFO("Processed " + std::to_string(osmdata_.osm_node_count) + " nodes on ways");
      }
    }//if we hit the end of the nodes and didnt find it that is a problem
    else {
      throw std::runtime_error("Didn't find OSMWayNode for node id: " + std::to_string(osmid));
    }
  }

  void way_callback(uint64_t osmid, const OSMPBF::Tags &tags, const std::vector<uint64_t> &nodes) {

    // Check if it is in the list of ways used by relations
    if (!members_.IsUsed(osmid)) {
      return;
    }

    // Transform tags. If no results that means the way does not have tags
    // suitable for use in routing.
    auto results = lua_.Transform(OSMType::kWay, tags);
    if (results.size() == 0) {
      return;
    }

    //unsorted extracts are just plain nasty, so they can bugger off!
    if(osmid < last_way_)
      throw std::runtime_error("Detected unsorted input data");
    last_way_ = osmid;

    // Add the refs to the node reference list
    for (size_t i = 0; i < nodes.size(); ++i) {
      const auto& node = nodes[i];
      //keep the node
      way_nodes_->push_back({node, ways_->size(), i});
      ++osmdata_.node_count;
      // Mark the nodes that we will care about when processing nodes
      shape_.set(node);
    }
    ++osmdata_.osm_way_count;
    osmdata_.osm_way_node_count += nodes.size();

    // Construct OSMWay and set the node ref index and count
    OSMWay w{osmid};
    w.set_node_count(nodes.size());

    // Add the way to the list
    ways_->push_back(w);
  }

  void relation_callback(const uint64_t osmid, const OSMPBF::Tags &tags, const std::vector<OSMPBF::Member> &members) {
    // Get tags
    auto results = lua_.Transform(OSMType::kRelation, tags);
    if (results.size() == 0)
      return;

    //unsorted extracts are just plain nasty, so they can bugger off!
    if(osmid < last_relation_)
      throw std::runtime_error("Detected unsorted input data");
    last_relation_ = osmid;

    OSMAdmin admin{osmid};
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
        members_.set(member.member_id);
        memberids_.push_back(member.member_id);
      }
    }

    admin.set_member_index(idx);
    admin.set_member_count(members.size());

    admins_.push_back(std::move(admin));
  }

  //lets the sequences be set and reset
  void reset(sequence<OSMWay>* ways, sequence<OSMWayNode>* way_nodes){
    //reset the pointers (either null them out or set them to something valid)
    ways_.reset(ways);
    way_nodes_.reset(way_nodes);
  }

  // Lua Tag Transformation class
  LuaTagTransform lua_;

  // Mark the OSM Ids used by the ways and relations
  IdTable shape_, members_;

  // Pointer to all the OSM data (for use by callbacks)
  OSMData& osmdata_;

  // Vector of all the admin member/way ids.
  std::vector<uint64_t> memberids_;

  // Vector of admins.
  std::vector<OSMAdmin> admins_;

  // Ways and nodes written to file, nodes are written in the order they appear in way (shape)
  std::unique_ptr<sequence<OSMWay> > ways_;
  std::unique_ptr<sequence<OSMWayNode> > way_nodes_;
  // When updating the references with the node information we keep the last index we looked at
  // this lets us only have to iterate over the whole set once
  size_t current_way_node_index_;
  uint64_t last_node_, last_way_, last_relation_;

};

}

namespace valhalla {
namespace mjolnir {

OSMData PBFAdminParser::Parse(const boost::property_tree::ptree& pt, const std::vector<std::string>& input_files) {
  // Create OSM data. Set the member pointer so that the parsing callback
  // methods can use it.
  OSMData osmdata{"admin_ways.bn", "admin_way_node_ref.bn"};
  admin_callback callback(pt, osmdata);

  // Parse each input file for relations
  auto t = std::chrono::high_resolution_clock::now();
  for (const auto& input_file : input_files) {
    callback.current_way_node_index_ = callback.last_node_ = callback.last_way_ = callback.last_relation_ = 0;
    OSMPBF::Parser::parse(input_file, OSMPBF::Interest::RELATIONS, callback);
  }
  uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-t).count();
  LOG_INFO("Parsing relations took " + std::to_string(msecs) + " ms");
  LOG_INFO("Admin count = " + std::to_string(callback.admins_.size()));
  LOG_INFO("Member/ways count = " + std::to_string(callback.memberids_.size()));

  // Parse the ways. Find all node Ids needed. Shrink the OSM ways vector
  // and the OSM node reference vector (list of nodes that the ways include).
  t = std::chrono::high_resolution_clock::now();
  callback.reset(new sequence<OSMWay>(osmdata.ways_file, true),
    new sequence<OSMWayNode>(osmdata.way_node_references_file, true));
  for (const auto& input_file : input_files) {
    callback.current_way_node_index_ = callback.last_node_ = callback.last_way_ = callback.last_relation_ = 0;
    OSMPBF::Parser::parse(input_file, OSMPBF::Interest::WAYS, callback);
  }
  callback.reset(nullptr, nullptr);
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-t).count();
  LOG_INFO("Parsing ways took " + std::to_string(msecs) + " ms");
  LOG_INFO("Admin ways count = " + std::to_string(osmdata.osm_way_count));
  LOG_INFO("Number of noderefs = " + std::to_string(osmdata.node_count));

  //we need to sort the refs so that we can easily (sequentially) update them
  //during node processing, we use memory mapping here because otherwise we aren't
  //using much mem, the scoping makes sure to let it go when done sorting
  LOG_INFO("Sorting osm way node references by node id");
  t = std::chrono::high_resolution_clock::now();
  {
    sequence<OSMWayNode> way_nodes(osmdata.way_node_references_file, false);
    way_nodes.sort(
      [](const OSMWayNode& a, const OSMWayNode& b){
        return a.node_id < b.node_id;
      }
    );
  }
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-t).count();
  LOG_INFO("Sorting osm way node references took " + std::to_string(msecs) + " ms");

  // Parse node in all the input files. Skip any that are not marked from
  // being used in a way.
  LOG_INFO("Parsing nodes but only keeping " + std::to_string(osmdata.node_count));
  t = std::chrono::high_resolution_clock::now();
  for (const auto& input_file : input_files) {
    callback.reset(nullptr, new sequence<OSMWayNode>(osmdata.way_node_references_file, false));
    callback.current_way_node_index_ = callback.last_node_ = callback.last_way_ = callback.last_relation_ = 0;
    OSMPBF::Parser::parse(input_file, OSMPBF::Interest::NODES, callback);
  }
  callback.reset(nullptr, nullptr);
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-t).count();
  LOG_INFO("Parsing nodes took " + std::to_string(msecs) + " ms");
  LOG_INFO("Nodes included on Admin ways, count = " + std::to_string(osmdata.osm_node_count));

  //we need to sort the refs so that we easily iterate over them for building edges
  //so we line them first by way index then by shape index of the node
  LOG_INFO("Sorting osm way node references by way index and node shape index");
  t = std::chrono::high_resolution_clock::now();
  {
    sequence<OSMWayNode> way_nodes(osmdata.way_node_references_file, false);
    way_nodes.sort(
      [](const OSMWayNode& a, const OSMWayNode& b){
        if(a.way_index == b.way_index) {
          //TODO: if its equal we have screwed something up, should we check and throw here?
          return a.way_shape_node_index < b.way_shape_node_index;
        }
        return a.way_index < b.way_index;
      }
    );
  }
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-t).count();
  LOG_INFO("Sorting osm way node references took " + std::to_string(msecs) + " ms");


  // Return OSM data
  return osmdata;
}

}
}
