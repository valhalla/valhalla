#include "mjolnir/pbfadminparser.h"
#include "admin_lua_proc.h"
#include "idtable.h"
#include "midgard/logging.h"
#include "mjolnir/luatagtransform.h"
#include "mjolnir/osmadmindata.h"

#include <boost/algorithm/string.hpp>
#include <osmium/io/pbf_input.hpp>
#include <osmium/osm/entity_bits.hpp>

#include <string>
#include <utility>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {
// This value controls the initial size of the Id table. If this is exceeded
// the table will be resized. We should keep this number just higher than
// reality to maintain performance
constexpr uint64_t kMaxOSMNodesHint = 1300000000;

struct admin_parser {
  // Construct PBFAdminParser based on properties file and input PBF extract
  admin_parser(const boost::property_tree::ptree& pt, OSMAdminData& osmdata)
      : lua_(std::string(lua_admin_lua, lua_admin_lua + lua_admin_lua_len)),
        shape_(pt.get<unsigned long>("id_table_size", kMaxOSMNodesHint)),
        members_(pt.get<unsigned long>("id_table_size", kMaxOSMNodesHint)), osm_admin_data_(osmdata) {
  }

  void node(const osmium::Node& node) {
    // Check if it is in the list of nodes used by ways
    if (!shape_.get(node.id())) {
      return;
    }

    ++osm_admin_data_.osm_node_count;

    osm_admin_data_.shape_map.emplace(node.id(),
                                      PointLL(node.location().lon(), node.location().lat()));

    if (osm_admin_data_.shape_map.size() % 500000 == 0) {
      LOG_INFO("Processed " + std::to_string(osm_admin_data_.shape_map.size()) + " nodes on ways");
    }
  }

  void way(const osmium::Way& way) {

    // Check if it is in the list of ways used by relations
    if (!members_.get(way.id())) {
      return;
    }

    std::vector<uint64_t> node_ids;
    node_ids.reserve(way.nodes().size());
    for (const auto& node : way.nodes()) {
      node_ids.push_back(node.ref());
    }

    for (const auto node : node_ids) {
      ++osm_admin_data_.node_count;
      // Mark the nodes that we will care about when processing nodes
      shape_.set(node);
    }

    osm_admin_data_.way_map.emplace(way.id(), std::move(node_ids));
  }

  void relation(const osmium::Relation& relation) {
    auto tags = lua_.Transform(OSMType::kRelation, relation.id(), relation.tags());
    if (tags.empty()) {
      return;
    }

    OSMAdmin admin{static_cast<uint64_t>(relation.id())};

    for (const auto& tag : tags) {
      // TODO:  Store multiple all the names
      if (tag.first == "name" && !tag.second.empty()) {
        admin.name_index = osm_admin_data_.name_offset_map.index(tag.second);
      } else if (tag.first == "name:en" && !tag.second.empty()) {
        admin.name_en_index = osm_admin_data_.name_offset_map.index(tag.second);
      } else if (tag.first == "admin_level") {
        admin.admin_level = std::stoi(tag.second);
      } else if (tag.first == "drive_on_right") {
        admin.drive_on_right = tag.second == "true" ? true : false;
      } else if (tag.first == "allow_intersection_names") {
        admin.allow_intersection_names = tag.second == "true" ? true : false;
      } else if (tag.first == "iso_code" && !tag.second.empty()) {
        admin.iso_code_index = osm_admin_data_.name_offset_map.index(tag.second);
      } else if (tag.first == "default_language" && !tag.second.empty()) {
        admin.default_language_index = osm_admin_data_.name_offset_map.index(tag.second);
      }
    }

    const auto& members = relation.members();
    admin.ways.reserve(members.size());
    admin.roles.reserve(members.size());
    for (const auto& member : members) {

      if (member.type() == osmium::item_type::way) {
        members_.set(member.ref());
        admin.ways.push_back(member.ref());
        admin.roles.push_back(std::string(member.role()) != "inner"); // assume outer
        ++osm_admin_data_.osm_way_count;
      }
    }

    if (admin.name_index == admin.name_en_index) {
      admin.name_en_index = 0;
    }

    osm_admin_data_.admins.push_back(std::move(admin));
  }

  // Lua Tag Transformation class
  LuaTagTransform lua_;

  // Mark the OSM Ids used by the ways and relations
  // TODO: just use robin_hood::unordered_set<uint64_t> since these are probably relatively small
  UnorderedIdTable shape_, members_;

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
  admin_parser parser(pt, osmdata);

  LOG_INFO("Parsing files: " + boost::algorithm::join(input_files, ", "));

  // Parse each input file for relations
  LOG_INFO("Parsing relations...");
  for (auto& file : input_files) {
    osmium::io::Reader reader(file, osmium::osm_entity_bits::relation);
    while (const osmium::memory::Buffer buffer = reader.read()) {
      for (const osmium::memory::Item& item : buffer) {
        parser.relation(static_cast<const osmium::Relation&>(item));
      }
    }
    reader.close(); // Explicit close to get an exception in case of an error.
  }
  LOG_INFO("Finished with " + std::to_string(osmdata.admins.size()) +
           " admin polygons comprised of " + std::to_string(osmdata.osm_way_count) + " ways");

  // Parse the ways.
  LOG_INFO("Parsing ways...");
  for (auto& file : input_files) {
    osmium::io::Reader reader(file, osmium::osm_entity_bits::way);
    while (const osmium::memory::Buffer buffer = reader.read()) {
      for (const osmium::memory::Item& item : buffer) {
        parser.way(static_cast<const osmium::Way&>(item));
      }
    }
    reader.close(); // Explicit close to get an exception in case of an error.
  }
  LOG_INFO("Finished with " + std::to_string(osmdata.way_map.size()) + " ways comprised of " +
           std::to_string(osmdata.node_count) + " nodes");

  // Parse node in all the input files. Skip any that are not marked from
  // being used in a way.
  LOG_INFO("Parsing nodes...");
  for (auto& file : input_files) {
    osmium::io::Reader reader(file, osmium::osm_entity_bits::node);
    while (const osmium::memory::Buffer buffer = reader.read()) {
      for (const osmium::memory::Item& item : buffer) {
        parser.node(static_cast<const osmium::Node&>(item));
      }
    }
    reader.close(); // Explicit close to get an exception in case of an error.
  }
  LOG_INFO("Finished with " + std::to_string(osmdata.osm_node_count) + " nodes");

  // Return OSM data
  return osmdata;
}

} // namespace mjolnir
} // namespace valhalla
