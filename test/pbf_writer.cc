// boost
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

// osmium
#include <osmium/builder/attr.hpp>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/output_iterator.hpp>
#include <osmium/io/pbf_output.hpp>
#include <osmium/object_pointer_collection.hpp>
#include <osmium/osm/object_comparisons.hpp>

// stl
#include <iostream>
#include <regex>
#include <set>
#include <string>
#include <sys/stat.h>
#include <tuple>
#include <unordered_set>
#include <utility>

// valhalla
#include "filesystem.h"
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
const std::string LINK_SHAPEPOINTS_LAT_FILE = "link_shapepoints_lat";
const std::string LINK_SHAPEPOINTS_LON_FILE = "link_shapepoints_lon";
const std::string LINK_SHAPEPOINTS_INDEX_FILE = "link_shapepoints_index";

constexpr auto RESTRICTION_SIMPLE_FILE = "restriction_simple";

constexpr auto ndsUnitsPerWgs84Deg = (std::numeric_limits<uint32_t>::max() + 1.0) / 360.0;

namespace valhalla {
namespace detail {

struct MyRestriction {
  uint64_t from_edge_id;
  uint64_t via_node_id;
  uint64_t to_edge_id;
};

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

inline void build_pbf(const std::string inPath,
                      const std::string osm_file_name,
                      const uint64_t initial_osm_id = 0) {

  const size_t initial_buffer_size = 10000;
  osmium::memory::Buffer buffer{initial_buffer_size, osmium::memory::Buffer::auto_grow::yes};

  {
    LOG_INFO("reading files for nodes");
    std::vector<uint64_t> node_ids =
        valhalla::detail::read_vector<uint64_t>(inPath + NODE_SOURCE_IDS_FILE);
    std::vector<int32_t> node_lats =
        valhalla::detail::read_vector<int32_t>(inPath + NODE_POS_LAT_FILE);
    std::vector<int32_t> node_lons =
        valhalla::detail::read_vector<int32_t>(inPath + NODE_POS_LON_FILE);
    std::vector<uint32_t> node_artificials =
        valhalla::detail::read_vector<uint32_t>(inPath + NODE_IS_ARTIFICIAL_FILE);

    LOG_INFO("processing " + std::to_string(node_ids.size()) + " nodes");
    for (auto i = 0; i < node_ids.size(); ++i) {
      std::vector<std::pair<std::string, std::string>> tags;
      if (node_artificials.at(i) == 1)
        tags.push_back({"artificial", "true"});
      else
        tags.push_back({"artificial", "false"});

      // TODO: int to float for lat and lon
      osmium::builder::add_node(buffer, osmium::builder::attr::_id(node_ids.at(i)),
                                osmium::builder::attr::_version(1),
                                osmium::builder::attr::_timestamp(std::time(nullptr)),
                                osmium::builder::attr::_location(
                                    osmium::Location{static_cast<float>(node_lons.at(i)) /
                                                         ndsUnitsPerWgs84Deg,
                                                     static_cast<float>(node_lats.at(i)) /
                                                         ndsUnitsPerWgs84Deg}),
                                osmium::builder::attr::_tags(tags));
    }
  }

  // {
  //   std::vector<int32_t> link_shapepoints_lat =
  //       valhalla::detail::read_vector<int32_t>(inPath + LINK_SHAPEPOINTS_LAT_FILE);
  //   std::vector<int32_t> link_shapepoints_lon =
  //       valhalla::detail::read_vector<int32_t>(inPath + LINK_SHAPEPOINTS_LON_FILE);

  //   LOG_INFO("processing " + std::to_string(link_shapepoints_lon.size()) + " shapepoints");
  //   for (uint64_t i = 0; i < link_shapepoints_lon.size(); ++i) {
  //     std::vector<std::pair<std::string, std::string>> tags;
  //     tags.push_back({"artificial", "false"});

  //     // TODO: int to float for lat and lon
  //     osmium::builder::add_node(buffer, osmium::builder::attr::_id(i),
  //                               osmium::builder::attr::_version(1),
  //                               osmium::builder::attr::_timestamp(std::time(nullptr)),
  //                               osmium::builder::attr::_location(
  //                                   osmium::Location{static_cast<float>(link_shapepoints_lon.at(i))
  //                                   /
  //                                                        ndsUnitsPerWgs84Deg,
  //                                                    static_cast<float>(link_shapepoints_lat.at(i))
  //                                                    /
  //                                                        ndsUnitsPerWgs84Deg}),
  //                               osmium::builder::attr::_tags(tags));
  //   }
  // }

  {
    LOG_INFO("reading files for links");
    std::vector<uint64_t> link_ids =
        valhalla::detail::read_vector<uint64_t>(inPath + LINK_SOURCE_IDS_FILE);
    std::vector<uint64_t> link_froms =
        valhalla::detail::read_vector<uint64_t>(inPath + LINK_FROM_FILE);
    std::vector<uint64_t> link_tos = valhalla::detail::read_vector<uint64_t>(inPath + LINK_TO_FILE);
    std::vector<uint32_t> link_frcs = valhalla::detail::read_vector<uint32_t>(inPath + LINK_FRC_FILE);
    std::vector<uint32_t> linkd_speeds =
        valhalla::detail::read_vector<uint32_t>(inPath + LINK_SPEED_FILE);
    std::vector<uint32_t> link_oneways =
        valhalla::detail::read_vector<uint32_t>(inPath + LINK_ONEWAY_FILE);

    std::vector<uint64_t> link_shapepoints_index =
        valhalla::detail::read_vector<uint64_t>(inPath + LINK_SHAPEPOINTS_INDEX_FILE);

    const std::string ROAD_CLASS[] = {"motorway", "trunk", "secondary", "tertiary", "residential"};
    // const std::string ROAD_CLASS[] = {"motorway", "trunk",        "secondary",
    //                                   "tertiary", "unclassified", "residential"};
    uint64_t pos = 0;

    LOG_INFO("processing " + std::to_string(link_ids.size()) + " links");
    for (auto i = 0; i < link_ids.size(); ++i) {
      std::vector<uint64_t> nodeids;
      nodeids.emplace_back(link_froms.at(i)); // from node

      // // add shapepoints
      // for (auto j = pos; j < link_shapepoints_index[i]; ++j) {
      //   nodeids.emplace_back(j);
      // }
      // pos = link_shapepoints_index[i];

      nodeids.emplace_back(link_tos.at(i)); // to node

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
  }

  {

    std::vector<MyRestriction> relations =
        valhalla::detail::read_vector<MyRestriction>(inPath + RESTRICTION_SIMPLE_FILE);
    int osm_id = 0;
    for (const auto& relation : relations) {

      std::vector<osmium::builder::attr::member_type> members;
      members.push_back({osmium::item_type::way, relation.from_edge_id, "from"});
      members.push_back({osmium::item_type::node, relation.via_node_id, "via"});
      members.push_back({osmium::item_type::way, relation.to_edge_id, "to"});

      std::vector<std::pair<std::string, std::string>> tags;
      tags.push_back({"type", "restriction"});
      tags.push_back({"restriction", "no_straight_on"});
      osmium::builder::add_relation(buffer, osmium::builder::attr::_id(osm_id++),
                                    osmium::builder::attr::_version(1),
                                    osmium::builder::attr::_timestamp(std::time(nullptr)),
                                    osmium::builder::attr::_members(members),
                                    osmium::builder::attr::_tags(tags));
    }
  }

  // Create header and set generator.
  osmium::io::Header header;
  header.set("generator", "valhalla-test-creator");

  osmium::io::File output_file{inPath + osm_file_name, "pbf"};

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
  std::string input_path;
  std::string filename = "result.osm.pbf";
  cxxopts::Options options("pbf_writer",
                           "\n\npbf_writer is a program that creates *.osm.pbf from binary files\n");

  options.add_options()("h,help", "Print this help message.")(
      "filename", "filename of output .osm.pbf, defaulat (result.osm.pbf)",
      cxxopts::value<std::string>())("input_path", "positional arguments",
                                     cxxopts::value<std::string>(input_path));

  auto result = options.parse(argc, argv);

  if (result.count("help")) {
    std::cout << options.help() << "\n";
    return EXIT_SUCCESS;
  }

  if (result.count("filename")) {
    filename = result["filename"].as<std::string>();
  }

  if (result.count("input_path")) {
    input_path = result["input_path"].as<std::string>();
    if (input_path.back() != '/') {
      input_path.push_back('/');
    }
  }

  valhalla::detail::build_pbf(input_path, filename); // nodes
  LOG_INFO("Finished");
  return 0;
}