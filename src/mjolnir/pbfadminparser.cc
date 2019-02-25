
#include "mjolnir/pbfadminparser.h"
#include "admin_lua_proc.h"
#include "mjolnir/idtable.h"
#include "mjolnir/luatagtransform.h"
#include "mjolnir/osmadmin.h"
#include "mjolnir/osmpbfparser.h"
#include "mjolnir/util.h"

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <future>
#include <thread>
#include <utility>

#include "baldr/tilehierarchy.h"
#include "midgard/logging.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {
// Will throw an error if this is exceeded. Then we can increase.
constexpr uint64_t kMaxOSMNodeId = 5500000000;

// Node equality
const auto WayNodeEquals = [](const OSMWayNode& a, const OSMWayNode& b) {
  return a.node.osmid_ == b.node.osmid_;
};

struct admin_callback : public OSMPBF::Callback {
public:
  admin_callback() = delete;
  admin_callback(const admin_callback&) = delete;
  virtual ~admin_callback() {
  }
  // Construct PBFAdminParser based on properties file and input PBF extract
  admin_callback(const boost::property_tree::ptree& pt, OSMAdminData& osmdata)
      : shape_(kMaxOSMNodeId), members_(kMaxOSMNodeId), osm_admin_data_(osmdata),
        lua_(std::string(lua_admin_lua, lua_admin_lua + lua_admin_lua_len)) {
  }

  virtual void
  node_callback(const uint64_t osmid, double lng, double lat, const OSMPBF::Tags& tags) override {
    // Check if it is in the list of nodes used by ways
    if (!shape_.get(osmid)) {
      return;
    }

    ++osm_admin_data_.osm_node_count;

    osm_admin_data_.shape_map.emplace(osmid, PointLL(lng, lat));

    if (osm_admin_data_.shape_map.size() % 500000 == 0) {
      LOG_INFO("Processed " + std::to_string(osm_admin_data_.shape_map.size()) + " nodes on ways");
    }
  }

  virtual void way_callback(const uint64_t osmid,
                            const OSMPBF::Tags& tags,
                            const std::vector<uint64_t>& nodes) override {

    // Check if it is in the list of ways used by relations
    if (!members_.get(osmid)) {
      return;
    }

    for (const auto node : nodes) {
      ++osm_admin_data_.node_count;
      // Mark the nodes that we will care about when processing nodes
      shape_.set(node);
    }

    osm_admin_data_.way_map.emplace(osmid, std::list<uint64_t>(nodes.begin(), nodes.end()));
  }

  virtual void relation_callback(const uint64_t osmid,
                                 const OSMPBF::Tags& tags,
                                 const std::vector<OSMPBF::Member>& members) override {
    // Get tags
    auto results = lua_.Transform(OSMType::kRelation, tags);
    if (results.size() == 0) {
      return;
    }

    OSMAdmin admin{osmid};

    for (const auto& tag : results) {
      // TODO:  Store multiple all the names
      if (tag.first == "name" && !tag.second.empty()) {
        admin.set_name_index(osm_admin_data_.name_offset_map.index(tag.second));
      } else if (tag.first == "name:en" && !tag.second.empty()) {
        admin.set_name_en_index(osm_admin_data_.name_offset_map.index(tag.second));
      } else if (tag.first == "admin_level") {
        admin.set_admin_level(std::stoi(tag.second));
      } else if (tag.first == "drive_on_right") {
        admin.set_drive_on_right(tag.second == "true" ? true : false);
      } else if (tag.first == "iso_code" && !tag.second.empty()) {
        admin.set_iso_code_index(osm_admin_data_.name_offset_map.index(tag.second));
      }
    }

    std::list<uint64_t> member_ids;

    for (const auto& member : members) {

      if (member.member_type == OSMPBF::Relation::MemberType::Relation_MemberType_WAY) {
        members_.set(member.member_id);
        member_ids.push_back(member.member_id);
        ++osm_admin_data_.osm_way_count;
      }
    }

    if (admin.name_index() == admin.name_en_index()) {
      admin.set_name_en_index(0);
    }

    admin.set_ways(member_ids);

    osm_admin_data_.admins_.push_back(std::move(admin));
  }

  virtual void changeset_callback(const uint64_t changeset_id) override {
    osm_admin_data_.max_changeset_id_ = std::max(osm_admin_data_.max_changeset_id_, changeset_id);
  }

  // Lua Tag Transformation class
  LuaTagTransform lua_;

  // Mark the OSM Ids used by the ways and relations
  IdTable shape_, members_;

  // Pointer to all the OSM data (for use by callbacks)
  OSMAdminData& osm_admin_data_;
};

} // namespace

namespace valhalla {
namespace mjolnir {

OSMAdminData PBFAdminParser::Parse(const boost::property_tree::ptree& pt,
                                   const std::vector<std::string>& input_files) {
  // Create OSM data. Set the member pointer so that the parsing callback
  // methods can use it.
  OSMAdminData osmdata{};
  admin_callback callback(pt, osmdata);

  LOG_INFO("Parsing files: " + boost::algorithm::join(input_files, ", "));

  // hold open all the files so that if something else (like diff application)
  // needs to mess with them we wont have troubles with inodes changing underneath us
  std::list<std::ifstream> file_handles;
  for (const auto& input_file : input_files) {
    file_handles.emplace_back(input_file, std::ios::binary);
    if (!file_handles.back().is_open()) {
      throw std::runtime_error("Unable to open: " + input_file);
    }
  }

  // Parse each input file for relations
  LOG_INFO("Parsing relations...");
  for (auto& file_handle : file_handles) {
    OSMPBF::Parser::parse(file_handle,
                          static_cast<OSMPBF::Interest>(OSMPBF::Interest::RELATIONS |
                                                        OSMPBF::Interest::CHANGESETS),
                          callback);
  }
  LOG_INFO("Finished with " + std::to_string(osmdata.admins_.size()) +
           " admin polygons comprised of " + std::to_string(osmdata.osm_way_count) + " ways");

  // Parse the ways.
  LOG_INFO("Parsing ways...");
  for (auto& file_handle : file_handles) {
    OSMPBF::Parser::parse(file_handle,
                          static_cast<OSMPBF::Interest>(OSMPBF::Interest::WAYS |
                                                        OSMPBF::Interest::CHANGESETS),
                          callback);
  }
  LOG_INFO("Finished with " + std::to_string(osmdata.way_map.size()) + " ways comprised of " +
           std::to_string(osmdata.node_count) + " nodes");

  // Parse node in all the input files. Skip any that are not marked from
  // being used in a way.
  LOG_INFO("Parsing nodes...");
  for (auto& file_handle : file_handles) {
    OSMPBF::Parser::parse(file_handle,
                          static_cast<OSMPBF::Interest>(OSMPBF::Interest::NODES |
                                                        OSMPBF::Interest::CHANGESETS),
                          callback);
  }
  LOG_INFO("Finished with " + std::to_string(osmdata.osm_node_count) + " nodes");

  // Return OSM data
  return osmdata;
}

} // namespace mjolnir
} // namespace valhalla
