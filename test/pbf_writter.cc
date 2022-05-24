#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <osmium/builder/attr.hpp>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/output_iterator.hpp>
#include <osmium/io/pbf_output.hpp>
#include <osmium/object_pointer_collection.hpp>
#include <osmium/osm/object_comparisons.hpp>

#include <iostream>
#include <regex>
#include <set>
#include <string>
#include <sys/stat.h>
#include <tuple>
#include <unordered_set>
#include <utility>

// valhalla
#include "midgard/logging.h"

const std::string NODE_SOURCE_IDS_FILE = "original_node_id";
const std::string NODE_POS_LAT_FILE = "node_pos_lat";
const std::string NODE_POS_LON_FILE = "node_pos_lon";
const std::string NODE_IS_ARTIFICIAL_FILE = "node_is_artificial";

const std::string LINK_SOURCE_IDS_FILE = "original_link_id";
const std::string LINK_FROM_FILE = "link_tail";
const std::string LINK_TO_FILE = "link_head";
const std::string LINK_FRC_FILE = "link_frc";
const std::string LINK_LENGTH_FILE = "link_length";
const std::string LINK_SPEED_FILE = "link_speed";
const std::string LINK_ONEWAY_FILE = "link_oneway";

namespace valhalla {
namespace detail {

long GetFileSize(std::string filename) {
  struct stat stat_buf;
  int rc = stat(filename.c_str(), &stat_buf);
  return rc == 0 ? stat_buf.st_size : -1;
}

template <class T> inline std::vector<T> read_vector(std::string file_path) {
  uint64_t count = GetFileSize(file_path) / sizeof(T);
  std::ifstream file(file_path, std::ios::in | std::ios::binary);
  std::vector<T> vec(count);
  file.read(reinterpret_cast<char*>(vec.data()), count * sizeof(T));
  file.close();
  return std::move(vec);
}

inline void build_pbf(const std::string inPath, const uint64_t initial_osm_id = 0) {

  const size_t initial_buffer_size = 10000;
  osmium::memory::Buffer buffer{initial_buffer_size, osmium::memory::Buffer::auto_grow::yes};

  LOG_INFO("reading files for nodes");
  std::vector<uint64_t> node_ids =
      valhalla::detail::read_vector<uint64_t>(inPath + NODE_SOURCE_IDS_FILE);
  std::vector<uint32_t> node_lats =
      valhalla::detail::read_vector<uint32_t>(inPath + NODE_POS_LAT_FILE);
  std::vector<uint32_t> node_lons =
      valhalla::detail::read_vector<uint32_t>(inPath + NODE_POS_LON_FILE);
  std::vector<uint32_t> node_artificials =
      valhalla::detail::read_vector<uint32_t>(inPath + NODE_IS_ARTIFICIAL_FILE);

  LOG_INFO("converting " + std::to_string(node_ids.size()) + " nodes");
  for (auto i = 0; i < node_ids.size(); ++i) {
    std::vector<std::pair<std::string, std::string>> tags;
    if (node_artificials.at(i) == 1)
      tags.push_back({"artificial", "true"});
    else
      tags.push_back({"artificial", "false"});
    osmium::builder::add_node(buffer, osmium::builder::attr::_id(node_ids.at(i)),
                              osmium::builder::attr::_version(1),
                              osmium::builder::attr::_timestamp(std::time(nullptr)),
                              osmium::builder::attr::_location(
                                  osmium::Location{static_cast<float>(node_lons.at(i)) / 100000,
                                                   static_cast<float>(node_lats.at(i)) / 100000}),
                              osmium::builder::attr::_tags(tags));
  }

  node_ids.clear();
  node_ids.shrink_to_fit();
  node_lats.clear();
  node_lats.shrink_to_fit();
  node_lons.clear();
  node_lons.shrink_to_fit();
  node_artificials.clear();
  node_artificials.shrink_to_fit();

  LOG_INFO("reading files for links");
  std::vector<uint64_t> link_ids =
      valhalla::detail::read_vector<uint64_t>(inPath + LINK_SOURCE_IDS_FILE);
  std::vector<uint64_t> link_froms = valhalla::detail::read_vector<uint64_t>(inPath + LINK_FROM_FILE);
  std::vector<uint64_t> link_tos = valhalla::detail::read_vector<uint64_t>(inPath + LINK_TO_FILE);
  std::vector<uint32_t> link_frcs = valhalla::detail::read_vector<uint32_t>(inPath + LINK_FRC_FILE);
  std::vector<uint32_t> linkd_speeds =
      valhalla::detail::read_vector<uint32_t>(inPath + LINK_SPEED_FILE);
  std::vector<uint32_t> link_oneways =
      valhalla::detail::read_vector<uint32_t>(inPath + LINK_ONEWAY_FILE);

  std::string ROAD_CLASS[] = {"motorway",  "trunk",    "primary",
                              "secondary", "tertiary", "unclassified"};

  LOG_INFO("converting " + std::to_string(link_ids.size()) + " nodes");
  for (auto i = 0; i < link_ids.size(); ++i) {
    std::vector<uint64_t> nodeids;
    nodeids.emplace_back(link_froms.at(i));
    nodeids.emplace_back(link_tos.at(i));

    std::vector<std::pair<std::string, std::string>> tags;

    tags.push_back({"highway", ROAD_CLASS[link_frcs.at(i) - 1]});
    tags.push_back({"maxspeed", std::to_string(linkd_speeds.at(i))});
    if (link_oneways.at(i) == 2) {
      tags.push_back({"oneway", "yes"});
    } else if (link_oneways.at(i) == 1) {
      tags.push_back({"oneway", "-1"});
    } else if (link_oneways.at(i) == 3) {
      tags.push_back({"oneway", "yes"});
      tags.push_back({"oneway", "-1"});
    }

    osmium::builder::add_way(buffer, osmium::builder::attr::_id(link_ids.at(i)),
                             osmium::builder::attr::_version(1), osmium::builder::attr::_cid(1001),
                             osmium::builder::attr::_timestamp(std::time(nullptr)),
                             osmium::builder::attr::_nodes(nodeids),
                             osmium::builder::attr::_tags(tags));
  }

  link_ids.clear();
  link_ids.shrink_to_fit();
  link_froms.clear();
  link_froms.shrink_to_fit();
  link_tos.clear();
  link_tos.shrink_to_fit();
  link_frcs.clear();
  link_frcs.shrink_to_fit();
  linkd_speeds.clear();
  linkd_speeds.shrink_to_fit();
  link_oneways.clear();
  link_oneways.shrink_to_fit();
  /*

    for (const auto& relation : relations) {

      std::vector<osmium::builder::attr::member_type> members;
      for (const auto& member : relation.members) {
        if (member.type == node_member) {
          members.push_back({osmium::item_type::node,
    static_cast<int64_t>(node_osm_id_map[member.ref]), member.role.c_str()}); } else { if
    (way_osm_id_map.count(member.ref) == 0) { throw std::runtime_error("Relation member refers to an
    undefined way " + member.ref);
          }
          members.push_back({osmium::item_type::way, static_cast<int64_t>(way_osm_id_map[member.ref]),
                             member.role.c_str()});
        }
      }

      std::vector<std::pair<std::string, std::string>> tags;
      for (const auto& tag : relation.tags) {
        tags.push_back({tag.first, tag.second});
      }

      osmium::builder::add_relation(buffer, osmium::builder::attr::_id(osm_id++),
                                    osmium::builder::attr::_version(1),
                                    osmium::builder::attr::_timestamp(std::time(nullptr)),
                                    osmium::builder::attr::_members(members),
                                    osmium::builder::attr::_tags(tags));
    }
*/
  // Create header and set generator.
  osmium::io::Header header;
  header.set("generator", "valhalla-test-creator");

  osmium::io::File output_file{inPath + "test.osm.pbf", "pbf"};

  // Initialize Writer using the header from above and tell it that it
  // is allowed to overwrite a possibly existing file.
  osmium::io::Writer writer{output_file, header, osmium::io::overwrite::allow, osmium::io::fsync::no};

  // Sort by id..
  // TODO: why does everything use object_id_type of signed int64?
  osmium::ObjectPointerCollection objects;
  osmium::apply(buffer, objects);
  struct object_order_type_unsigned_id_version {
    bool operator()(const osmium::OSMObject* lhs, const osmium::OSMObject* rhs) const noexcept {
      if (lhs->type() == rhs->type()) {
        if (lhs->id() == rhs->id()) {
          return lhs->version() < rhs->version();
        }
        return static_cast<uint64_t>(lhs->id()) < static_cast<uint64_t>(rhs->id());
      }
      return lhs->type() < rhs->type();
    }
  };

  LOG_INFO("sorting objects by id");
  objects.sort(object_order_type_unsigned_id_version{});

  LOG_INFO("write out the objects in sorted order");
  auto out = osmium::io::make_output_iterator(writer);
  std::copy(objects.begin(), objects.end(), out);

  // Explicitly close the writer. Will throw an exception if there is
  // a problem. If you wait for the destructor to close the writer, you
  // will not notice the problem, because destructors must not throw.
  writer.close();
}

} // namespace detail
} // namespace valhalla

int main(int argc, char** argv) {
  using namespace valhalla;
  const std::string path = "/SDD_datadrive/ndslive/data/weu_out_2/";
  valhalla::detail::build_pbf(path); // nodes
  LOG_INFO("Finished");
  return 0;
}