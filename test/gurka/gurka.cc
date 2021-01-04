#include "baldr/directededge.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include "loki/worker.h"
#include "midgard/constants.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "mjolnir/util.h"
#include "odin/worker.h"
#include "proto/trip.pb.h"
#include "thor/worker.h"
#include "tyr/actor.h"
#include "tyr/serializers.h"

#include "gurka.h"
#include "test.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <osmium/builder/attr.hpp>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/output_iterator.hpp>
#include <osmium/io/pbf_output.hpp>
#include <osmium/object_pointer_collection.hpp>
#include <osmium/osm/object_comparisons.hpp>

#include <regex>
#include <string>
#include <tuple>
#include <utility>

#include <gtest/gtest.h>

namespace valhalla {
namespace gurka {
namespace detail {

boost::property_tree::ptree
build_config(const std::string& tiledir,
             const std::unordered_map<std::string, std::string>& config_options) {

  const std::string default_config = R"(
    {"mjolnir":{"id_table_size":1000,"tile_dir":"", "concurrency": 1},
     "thor":{
       "logging" : {"long_request" : 100}
     },
     "meili":{
       "logging" : {"long_request" : 100},
       "grid" : {"cache_size" : 100, "size": 100 },
       "default": {
         "beta": 3,
         "breakage_distance": 2000,
         "geometry": false,
         "gps_accuracy": 5.0,
         "interpolation_distance": 10,
         "max_route_distance_factor": 5,
         "max_route_time_factor": 5,
         "max_search_radius": 100,
         "route": true,
         "search_radius": 50,
         "sigma_z": 4.07,
         "turn_penalty_factor": 0,
         "penalize_immediate_uturn": true
       },
       "customizable": [
         "mode",
         "search_radius",
         "turn_penalty_factor",
         "gps_accuracy",
         "interpolation_distance",
         "sigma_z",
         "beta",
         "max_route_distance_factor",
         "max_route_time_factor",
         "penalize_immediate_uturn"
       ]
     },
     "loki":{
       "actions": [
         "locate",
         "route",
         "height",
         "sources_to_targets",
         "optimized_route",
         "isochrone",
         "trace_route",
         "trace_attributes",
         "transit_available"
       ],
       "logging" : {"long_request" : 100},
       "service_defaults" : {
         "minimum_reachability" : 50,
         "radius" : 0,
         "search_cutoff" : 35000,
         "node_snap_tolerance" : 5,
         "street_side_tolerance" : 5,
         "street_side_max_distance": 1000,
         "heading_tolerance" : 60
        }
     },
     "service_limits": {
      "auto": {"max_distance": 5000000.0, "max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "auto_data_fix": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "auto_shorter": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "bicycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "bus": {"max_distance": 5000000.0,"max_locations": 50,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "hov": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "motorcycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "motor_scooter": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "taxi": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},

      "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time": 120},
      "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,
      "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
      "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
      "skadi": {"max_shape": 750000,"min_resample": 10.0},
      "trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
      "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
    }
  })";

  std::stringstream stream(default_config);
  boost::property_tree::ptree ptree;
  boost::property_tree::json_parser::read_json(stream, ptree);
  ptree.put("mjolnir.tile_dir", tiledir);
  for (const auto& kv : config_options) {
    ptree.put(kv.first, kv.second);
  }
  return ptree;
}

midgard::PointLL to_ll(const nodelayout& nodes, const std::string& node_name) {
  return nodes.at(node_name);
}
std::vector<midgard::PointLL> to_lls(const nodelayout& nodes,
                                     const std::vector<std::string>& node_names) {
  std::vector<midgard::PointLL> lls;
  lls.reserve(node_names.size());
  for (const auto& node_name : node_names)
    lls.emplace_back(to_ll(nodes, node_name));
  return lls;
}

/**
 * build a valhalla json request body
 *
 * @param location_type  locations or shape
 * @param waypoints      sequence of pointlls representing the locations
 * @param costing        which costing name to use, defaults to auto
 * @param options        overrides parts of the request, supports rapidjson pointer semantics
 * @param stop_type      break, through, via, break_through
 * @return json string
 */
std::string build_valhalla_request(const std::string& location_type,
                                   const std::vector<midgard::PointLL>& waypoints,
                                   const std::string& costing = "auto",
                                   const std::unordered_map<std::string, std::string>& options = {},
                                   const std::string& stop_type = "break") {

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();

  rapidjson::Value locations(rapidjson::kArrayType);
  for (const auto& waypoint : waypoints) {
    rapidjson::Value p(rapidjson::kObjectType);
    p.AddMember("lon", waypoint.lng(), allocator);
    p.AddMember("lat", waypoint.lat(), allocator);
    if (!stop_type.empty()) {
      p.AddMember("type", stop_type, allocator);
    }
    locations.PushBack(p, allocator);
  }

  doc.AddMember(rapidjson::Value(location_type, allocator), locations, allocator);
  doc.AddMember("costing", costing, allocator);

  // check if we are overriding speed types etc
  bool custom_speed_types = false;
  for (const auto& option : options) {
    if (option.first.find(costing + "/speed_types") != std::string::npos) {
      custom_speed_types = true;
      break;
    }
  }

  rapidjson::Value co(rapidjson::kObjectType);
  if (!custom_speed_types) {
    rapidjson::Value dt(rapidjson::kObjectType);
    rapidjson::Value speed_types(rapidjson::kArrayType);
    speed_types.PushBack("freeflow", allocator);
    speed_types.PushBack("constrained", allocator);
    speed_types.PushBack("predicted", allocator);

    auto found = options.find("/date_time/type");
    if (found != options.cend() && found->second == "0") {
      speed_types.PushBack("current", allocator);
    }
    co.AddMember("speed_types", speed_types, allocator);
  }

  rapidjson::Value costing_options(rapidjson::kObjectType);
  costing_options.AddMember(rapidjson::Value(costing, allocator), co, allocator);
  doc.AddMember("costing_options", costing_options, allocator);
  doc.AddMember("verbose", true, allocator);

  // we do this last so that options are additive/overwrite
  for (const auto& kv : options) {
    rapidjson::Pointer(kv.first).Set(doc, kv.second);
  }

  // TODO: allow selecting this
  // for map matching for now we just want to do real map matching rather than fast
  doc.AddMember("shape_match", "map_snap", allocator);

  rapidjson::StringBuffer sb;
  rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
  doc.Accept(writer);
  return sb.GetString();
}

std::vector<std::string> splitter(const std::string& in_pattern, const std::string& content) {
  std::vector<std::string> split_content;
  std::regex pattern(in_pattern);
  std::copy(std::sregex_token_iterator(content.begin(), content.end(), pattern, -1),
            std::sregex_token_iterator(), std::back_inserter(split_content));
  return split_content;
}

void ltrim(std::string& s) {
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) { return !std::isspace(ch); }));
}
void rtrim(std::string& s) {
  s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) { return !std::isspace(ch); }).base(),
          s.end());
}

std::string trim(std::string s) {
  ltrim(s);
  rtrim(s);
  return s;
}

/**
 * Given a string that's an "ASCII map", will decide on coordinates
 * for the nodes drawn on the grid.
 *
 * @returns a dictionary of node IDs to lon/lat values
 */
nodelayout map_to_coordinates(const std::string& map,
                              const double gridsize_metres,
                              const midgard::PointLL& topleft) {

  // Gridsize is in meters per character

  /// Mercator projection
  auto y2lat_m = [](double y) {
    return (2 * atan(exp(y / midgard::kRadEarthMeters)) - midgard::kPiD / 2) * midgard::kDegPerRadD;
  };
  auto x2lon_m = [](double x) { return (x / midgard::kRadEarthMeters) * midgard::kDegPerRadD; };

  // Split string into lines
  // Strip whitespace lines, if they exist
  // Strip common leading whitespace
  // Decide locations for nodes based on remaining grid

  // Split string into lines
  auto lines = splitter("\n", map);
  if (lines.empty())
    return {};

  // Remove the leading whitespace lines, if they exist
  while (trim(lines.front()).empty()) {
    lines.erase(lines.begin());
  }

  // Find out the min whitespace on each line, then remove that from each line
  long min_whitespace = std::numeric_limits<long>::max();
  for (const auto& line : lines) {
    // Skip blank lines, as they might have no space at all, but shouldn't
    // be allowed to affect positioning
    if (trim(line).empty())
      continue;
    auto pos =
        std::find_if(line.begin(), line.end(), [](const auto& ch) { return !std::isspace(ch); });
    min_whitespace = std::min(min_whitespace, static_cast<long>(std::distance(line.begin(), pos)));
    if (min_whitespace == 0) // No need to continue if something is up against the left
      break;
  }

  // In-place remove leading whitespace from each line
  for (auto& line : lines) {
    // This must be a blank line or something
    if (line.size() < static_cast<size_t>(min_whitespace))
      continue;
    line.erase(line.begin(), line.begin() + min_whitespace);
  }

  // TODO: the way this projects coordinates onto the sphere could do with some work.
  //       it's not always going to be sensible to lay a grid down onto a sphere
  nodelayout result;
  for (std::size_t y = 0; y < lines.size(); y++) {
    for (std::size_t x = 0; x < lines[y].size(); x++) {
      auto ch = lines[y][x];
      // Only do A-Za-z0-9 for nodes - all other things are ignored
      if (std::isalnum(ch)) {
        // Always project west, then south, for consistency
        double lon = topleft.lng() + x2lon_m(x * gridsize_metres);
        double lat = topleft.lat() - y2lat_m(y * gridsize_metres);
        auto inserted = result.insert({std::string(1, ch), {lon, lat}});
        // TODO: Change the type to char instead of std::string so that its obvious
        if (!inserted.second) {
          throw std::logic_error(
              "Duplicate node name in ascii map, only single char names are supported");
        }
      }
    }
  }

  return result;
}

/**
 * Given a map of node locations, ways, node properties and relations, will
 * generate an OSM compatible PBF file, suitable for loading into Valhalla
 */
inline void build_pbf(const nodelayout& node_locations,
                      const ways& ways,
                      const nodes& nodes,
                      const relations& relations,
                      const std::string& filename,
                      const uint64_t initial_osm_id) {

  const size_t initial_buffer_size = 10000;
  osmium::memory::Buffer buffer{initial_buffer_size, osmium::memory::Buffer::auto_grow::yes};

  std::unordered_set<std::string> used_nodes;
  for (const auto& way : ways) {
    for (const auto& ch : way.first) {
      used_nodes.insert(std::string(1, ch));
    }
  }
  for (const auto& node : nodes) {
    for (const auto& ch : node.first) {
      used_nodes.insert(std::string(1, ch));
    }
  }
  for (const auto& relation : relations) {
    for (const auto& member : relation.members) {
      if (member.type == node_member) {
        used_nodes.insert(member.ref);
      }
    }
  }

  for (auto& used_node : used_nodes) {
    if (node_locations.count(used_node) == 0) {
      throw std::runtime_error("Node " + used_node + " was referred to but was not in the ASCII map");
    }
  }

  std::unordered_map<std::string, int> node_id_map;
  std::unordered_map<std::string, uint64_t> node_osm_id_map;
  int id = 0;
  for (auto& loc : node_locations) {
    node_id_map[loc.first] = id++;
  }
  uint64_t osm_id = initial_osm_id;
  for (auto& loc : node_locations) {
    if (used_nodes.count(loc.first) > 0) {
      node_osm_id_map[loc.first] = osm_id++;

      std::vector<std::pair<std::string, std::string>> tags;

      if (nodes.count(loc.first) == 0) {
        tags.push_back({"name", loc.first});
      } else {
        auto otags = nodes.at(loc.first);
        if (otags.count("name") == 0) {
          tags.push_back({"name", loc.first});
        }
        for (const auto& keyval : otags) {
          tags.push_back({keyval.first, keyval.second});
        }
      }

      osmium::builder::add_node(buffer, osmium::builder::attr::_id(node_osm_id_map[loc.first]),
                                osmium::builder::attr::_version(1),
                                osmium::builder::attr::_timestamp(std::time(nullptr)),
                                osmium::builder::attr::_location(
                                    osmium::Location{loc.second.lng(), loc.second.lat()}),
                                osmium::builder::attr::_tags(tags));
    }
  }

  std::unordered_map<std::string, uint64_t> way_osm_id_map;
  for (const auto& way : ways) {
    // allow setting custom id
    auto way_id = osm_id++;
    auto found = way.second.find("osm_id");
    if (found != way.second.cend()) {
      uint64_t id = std::stoull(found->second);
      if (id < osm_id) {
        throw std::invalid_argument("Osm way id has already been used");
      }
      way_id = id;
    }

    way_osm_id_map[way.first] = way_id;
    std::vector<int> nodeids;
    for (const auto& ch : way.first) {
      nodeids.push_back(node_osm_id_map[std::string(1, ch)]);
    }
    std::vector<std::pair<std::string, std::string>> tags;
    if (way.second.count("name") == 0) {
      tags.push_back({"name", way.first});
    }
    for (const auto& keyval : way.second) {
      tags.push_back({keyval.first, keyval.second});
    }
    osmium::builder::add_way(buffer, osmium::builder::attr::_id(way_osm_id_map[way.first]),
                             osmium::builder::attr::_version(1), osmium::builder::attr::_cid(1001),
                             osmium::builder::attr::_timestamp(std::time(nullptr)),
                             osmium::builder::attr::_nodes(nodeids),
                             osmium::builder::attr::_tags(tags));
  }

  for (const auto& relation : relations) {

    std::vector<osmium::builder::attr::member_type> members;
    for (const auto& member : relation.members) {
      if (member.type == node_member) {
        members.push_back({osmium::item_type::node, static_cast<int64_t>(node_osm_id_map[member.ref]),
                           member.role.c_str()});
      } else {
        if (way_osm_id_map.count(member.ref) == 0) {
          throw std::runtime_error("Relation member refers to an undefined way " + member.ref);
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

  // Create header and set generator.
  osmium::io::Header header;
  header.set("generator", "valhalla-test-creator");

  osmium::io::File output_file{filename, "pbf"};

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
  objects.sort(object_order_type_unsigned_id_version{});

  // Write out the objects in sorted order
  auto out = osmium::io::make_output_iterator(writer);
  std::copy(objects.begin(), objects.end(), out);

  // Explicitly close the writer. Will throw an exception if there is
  // a problem. If you wait for the destructor to close the writer, you
  // will not notice the problem, because destructors must not throw.
  writer.close();
}

std::string
to_string(const ::google::protobuf::RepeatedPtrField<::valhalla::StreetName>& street_names) {
  std::string str;

  for (const auto& street_name : street_names) {
    if (!str.empty()) {
      str += "/";
    }
    str += street_name.value();
  }
  return str;
}

} // namespace detail

/**
 * Given a node layout, set of ways, node properties and relations, generates an OSM PBF file,
 * and builds a set of Valhalla tiles for it.
 *
 * @param layout the locations of all the nodes
 * @param ways the way definitions (which nodes are connected, and their properties
 * @param nodes properties on any of the defined nodes
 * @param relations OSM relations that related nodes and ways together
 * @param workdir where to build the PBF and the tiles
 * @param config_options optional key value pairs where the key is ptree style dom traversal and
 *        the value is the value to put into the config. You can do things like add timezones database
 *        path
 * @return a map object that contains the Valhalla config (to pass to GraphReader) and node layout
 *         (for converting node names to coordinates)
 */
map buildtiles(const nodelayout& layout,
               const ways& ways,
               const nodes& nodes,
               const relations& relations,
               const std::string& workdir,
               const std::unordered_map<std::string, std::string>& config_options) {

  map result;
  result.config = detail::build_config(workdir, config_options);
  result.nodes = layout;

  // Sanity check so that we don't blow away / by mistake
  if (workdir == "/") {
    throw std::runtime_error("Can't use / for tests, as we need to clean it out first");
  }

  if (filesystem::exists(workdir))
    filesystem::remove_all(workdir);
  filesystem::create_directories(workdir);

  auto pbf_filename = workdir + "/map.pbf";
  std::cerr << "[          ] generating map PBF at " << pbf_filename << std::endl;
  detail::build_pbf(result.nodes, ways, nodes, relations, pbf_filename);
  std::cerr << "[          ] building tiles in " << result.config.get<std::string>("mjolnir.tile_dir")
            << std::endl;
  midgard::logging::Configure({{"type", ""}});

  mjolnir::build_tile_set(result.config, {pbf_filename}, mjolnir::BuildStage::kInitialize,
                          mjolnir::BuildStage::kValidate, false);

  return result;
}

/**
 * Finds a directed edge in the generated map.  Helpful because the IDs assigned
 * to edges depends on the shape of the map.
 *
 * @param reader a reader configured to read graph tiles
 * @param nodes a lookup table from node names to coordinates
 * @param way_name the way name you want a directed edge for
 * @param end_node the node that should be the target of the directed edge you want
 * @param tile_id optional tile_id to limit the search to
 * @return the directed edge that matches, or nullptr if there was no match
 */
std::tuple<const baldr::GraphId,
           const baldr::DirectedEdge*,
           const baldr::GraphId,
           const baldr::DirectedEdge*>
findEdge(valhalla::baldr::GraphReader& reader,
         const nodelayout& nodes,
         const std::string& way_name,
         const std::string& end_node,
         const baldr::GraphId& tile_id) {
  // if the tile was specified use it otherwise scan everything
  auto tileset =
      tile_id.Is_Valid() ? std::unordered_set<baldr::GraphId>{tile_id} : reader.GetTileSet();

  // Iterate over all the tiles, there wont be many in unit tests..
  const auto& end_node_coordinates = nodes.at(end_node);
  for (auto tile_id : tileset) {
    auto tile = reader.GetGraphTile(tile_id);
    // Iterate over all directed edges to find one with the name we want
    for (uint32_t i = 0; i < tile->header()->directededgecount(); i++) {
      const auto* forward_directed_edge = tile->directededge(i);
      // Now, see if the endnode for this edge is our end_node
      auto de_endnode = forward_directed_edge->endnode();
      auto de_endnode_coordinates = tile->get_node_ll(de_endnode);
      const auto threshold = 0.00001; // Degrees.  About 1m at the equator
      if (std::abs(de_endnode_coordinates.lng() - end_node_coordinates.lng()) < threshold &&
          std::abs(de_endnode_coordinates.lat() - end_node_coordinates.lat()) < threshold) {
        auto names = tile->GetNames(forward_directed_edge->edgeinfo_offset());
        for (const auto& name : names) {
          if (name == way_name) {
            auto forward_edge_id = tile_id;
            forward_edge_id.set_id(i);
            auto reverse_edge_id = tile->GetOpposingEdgeId(forward_directed_edge);
            auto* reverse_directed_edge = tile->directededge(i);
            return std::make_tuple(forward_edge_id, forward_directed_edge, reverse_edge_id,
                                   reverse_directed_edge);
          }
        }
      }
    }
  }

  return std::make_tuple(baldr::GraphId{}, nullptr, baldr::GraphId{}, nullptr);
}

/**
 * Finds an edge in the graph based on its begin and end node names
 *
 * @param reader           graph reader to look up tiles and edges
 * @param begin_node_name  name of the begin node
 * @param end_node_name    name of the end node
 * @return the edge_id and its edge
 */
std::tuple<const baldr::GraphId, const baldr::DirectedEdge*>
findEdgeByNodes(valhalla::baldr::GraphReader& reader,
                const nodelayout& nodes,
                const std::string& begin_node_name,
                const std::string& end_node_name) {
  // Iterate over all the tiles, there wont be many in unit tests..
  for (auto tile_id : reader.GetTileSet()) {
    auto tile = reader.GetGraphTile(tile_id);
    // Iterate over all directed edges to find one with the name we want
    for (const auto& e : tile->GetDirectedEdges()) {
      // Bail if wrong end node
      auto ll = reader.GetGraphTile(e.endnode())->get_node_ll(e.endnode());
      if (!ll.ApproximatelyEqual(nodes.at(end_node_name))) {
        continue;
      }
      // Bail if wrong begin node
      auto edge_id = tile_id;
      edge_id.set_id(&e - tile->directededge(0));
      auto begin_node_id = reader.edge_startnode(edge_id);
      ll = tile->get_node_ll(begin_node_id);
      if (!ll.ApproximatelyEqual(nodes.at(begin_node_name))) {
        continue;
      }

      // TODO: could check that the edges name contains the nodes (confirm connectivity)
      return std::make_tuple(edge_id, &e);
    }
  }
  throw std::runtime_error("Couldnt not find edge for nodes " + begin_node_name + ", " +
                           end_node_name);
}

valhalla::Api route(const map& map,
                    const std::string& request_json,
                    std::shared_ptr<valhalla::baldr::GraphReader> reader) {
  if (!reader)
    reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(map.config, *reader, true);
  valhalla::Api api;
  actor.route(request_json, nullptr, &api);
  return api;
}

/**
 * Calculates a route along a set of waypoints with a given costing model, and returns the
 * valhalla::Api result.
 *
 * @param map a map returned by buildtiles
 * @param waypoints an array of node names to use as waypoints
 * @param costing the name of the costing model to use
 */
valhalla::Api route(const map& map,
                    const std::vector<std::string>& waypoints,
                    const std::string& costing,
                    const std::unordered_map<std::string, std::string>& options,
                    const std::shared_ptr<valhalla::baldr::GraphReader>& reader) {
  std::cerr << "[          ] Routing with mjolnir.tile_dir = "
            << map.config.get<std::string>("mjolnir.tile_dir") << " with waypoints ";
  bool first = true;
  for (const auto& waypoint : waypoints) {
    if (!first)
      std::cerr << " -> ";
    std::cerr << waypoint;
    first = false;
  };
  std::cerr << " with costing " << costing << std::endl;
  auto lls = detail::to_lls(map.nodes, waypoints);
  auto request_json = detail::build_valhalla_request("locations", lls, costing, options);
  std::cerr << "[          ] Valhalla request is: " << request_json << std::endl;

  return route(map, request_json, reader);
}

valhalla::Api route(const map& map,
                    const std::string& origin,
                    const std::string& destination,
                    const std::string& costing,
                    const std::unordered_map<std::string, std::string>& options,
                    std::shared_ptr<valhalla::baldr::GraphReader> reader) {
  return route(map, {origin, destination}, costing, options, std::move(reader));
}

valhalla::Api match(const map& map,
                    const std::vector<std::string>& waypoints,
                    const std::string& stop_type,
                    const std::string& costing,
                    const std::unordered_map<std::string, std::string>& options,
                    std::shared_ptr<valhalla::baldr::GraphReader> reader) {
  if (!reader)
    reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));

  std::cerr << "[          ] Matching with mjolnir.tile_dir = "
            << map.config.get<std::string>("mjolnir.tile_dir") << " with waypoints ";
  bool first = true;
  for (const auto& waypoint : waypoints) {
    if (!first)
      std::cerr << " -> ";
    std::cerr << waypoint;
    first = false;
  };
  std::cerr << " with costing " << costing << std::endl;
  auto lls = detail::to_lls(map.nodes, waypoints);
  auto request_json = detail::build_valhalla_request("shape", lls, costing, options, stop_type);
  std::cerr << "[          ] Valhalla request is: " << request_json << std::endl;

  valhalla::tyr::actor_t actor(map.config, *reader, true);
  valhalla::Api api;
  actor.trace_route(request_json, nullptr, &api);
  return api;
}

valhalla::Api locate(const map& map,
                     const std::vector<std::string>& waypoints,
                     const std::string& costing,
                     const std::unordered_map<std::string, std::string>& options,
                     std::shared_ptr<valhalla::baldr::GraphReader> reader,
                     std::string* json) {
  if (!reader)
    reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));

  std::cerr << "[          ] Locate with mjolnir.tile_dir = "
            << map.config.get<std::string>("mjolnir.tile_dir") << " with locations ";
  bool first = true;
  for (const auto& waypoint : waypoints) {
    if (!first)
      std::cerr << ",";
    std::cerr << waypoint;
    first = false;
  };
  std::cerr << " with costing " << costing << std::endl;
  auto lls = detail::to_lls(map.nodes, waypoints);
  auto request_json = detail::build_valhalla_request("locations", lls, costing, options);
  std::cerr << "[          ] Valhalla request is: " << request_json << std::endl;

  valhalla::tyr::actor_t actor(map.config, *reader, true);
  valhalla::Api api;
  auto json_str = actor.locate(request_json, nullptr, &api);
  if (json) {
    *json = json_str;
  }
  return api;
}

/* Returns the raw_result formatted as a JSON document in the given format.
 *
 * @param raw_result the result of a /route or /match request
 * @param format the response format to use for the JSON document
 * @return A JSON document created from serialized raw_result. Caller should
 * call HasParseError() on the returned document to verify its validity.
 */
rapidjson::Document convert_to_json(valhalla::Api& raw_result, valhalla::Options_Format format) {
  raw_result.mutable_options()->set_format(format);

  std::string json = tyr::serializeDirections(raw_result);
  rapidjson::Document result;
  result.Parse(json.c_str());
  return result;
}

// just draw one shape for both edges and add both edges properties
std::string dump_geojson_graph(const map& graph) {
  // flesh out the geojson to start
  rapidjson::Document doc(rapidjson::kObjectType);
  doc.AddMember("type", "FeatureCollection", doc.GetAllocator());
  rapidjson::Value features(rapidjson::kArrayType);

  // for all tiles for all edges
  valhalla::baldr::GraphReader reader(graph.config.get_child("mjolnir"));
  for (auto tile_id : reader.GetTileSet()) {
    if (reader.OverCommitted())
      reader.Trim();
    auto tile = reader.GetGraphTile(tile_id);
    for (const auto& edge : tile->GetDirectedEdges()) {
      valhalla::baldr::GraphId edge_id(tile_id.tileid(), tile_id.level(),
                                       &edge - tile->directededge(0));
      auto info = tile->edgeinfo(edge.edgeinfo_offset());

      // add some properties
      rapidjson::Value properties(rapidjson::kObjectType);
      rapidjson::Value names(rapidjson::kArrayType);
      for (const std::string& name : info.GetNames()) {
        names.PushBack(rapidjson::Value(name, doc.GetAllocator()).Move(), doc.GetAllocator());
      }
      properties.AddMember("edge_id", std::to_string(edge_id), doc.GetAllocator());
      properties.AddMember("opp_edge_id", std::to_string(reader.GetOpposingEdgeId(edge_id)),
                           doc.GetAllocator());
      properties.AddMember("names", names, doc.GetAllocator());

      // add the geom
      rapidjson::Value geometry(rapidjson::kObjectType);
      geometry.AddMember("type", "LineString", doc.GetAllocator());
      rapidjson::Value coordinates(rapidjson::kArrayType);
      for (const auto& point : info.shape()) {
        rapidjson::Value coordinate(rapidjson::kArrayType);
        coordinate.PushBack(point.lng(), doc.GetAllocator());
        coordinate.PushBack(point.lat(), doc.GetAllocator());
        coordinates.PushBack(coordinate, doc.GetAllocator());
      }
      geometry.AddMember("coordinates", coordinates, doc.GetAllocator());

      // keep all the stuff we made
      rapidjson::Value feature(rapidjson::kObjectType);
      feature.AddMember("type", "Feature", doc.GetAllocator());
      feature.AddMember("properties", properties, doc.GetAllocator());
      feature.AddMember("geometry", geometry, doc.GetAllocator());
      features.PushBack(feature, doc.GetAllocator());
    }
  }

  // add any named points
  for (const auto& point : graph.nodes) {
    // add some properties
    rapidjson::Value properties(rapidjson::kObjectType);
    properties.AddMember("name", point.first, doc.GetAllocator());

    // add the geom
    rapidjson::Value geometry(rapidjson::kObjectType);
    geometry.AddMember("type", "Point", doc.GetAllocator());
    rapidjson::Value coordinate(rapidjson::kArrayType);
    coordinate.PushBack(point.second.lng(), doc.GetAllocator());
    coordinate.PushBack(point.second.lat(), doc.GetAllocator());
    geometry.AddMember("coordinates", coordinate, doc.GetAllocator());

    // keep all the stuff we made
    rapidjson::Value feature(rapidjson::kObjectType);
    feature.AddMember("type", "Feature", doc.GetAllocator());
    feature.AddMember("properties", properties, doc.GetAllocator());
    feature.AddMember("geometry", geometry, doc.GetAllocator());
    features.PushBack(feature, doc.GetAllocator());
  }

  // serialize
  doc.AddMember("features", features, doc.GetAllocator());
  rapidjson::StringBuffer sb;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
  doc.Accept(writer);
  return sb.GetString();
}

namespace assert {
namespace osrm {

/**
 * Tests if a found path traverses the expected steps in the expected order
 *
 * @param result the result of a /route or /match request
 * @param expected_names the names of the step roads the path should traverse in order
 * @param dedupe whether subsequent same-name roads should appear multiple times or not (default not)
 */
void expect_steps(valhalla::Api& raw_result,
                  const std::vector<std::string>& expected_names,
                  bool dedupe,
                  const std::string& route_name) {

  rapidjson::Document result = convert_to_json(raw_result, valhalla::Options_Format_osrm);
  if (result.HasParseError()) {
    FAIL() << "Error converting route response to JSON";
  }

  EXPECT_TRUE(result.HasMember(route_name));
  EXPECT_TRUE(result[route_name].IsArray());
  EXPECT_EQ(result[route_name].Size(), 1);

  EXPECT_TRUE(result[route_name][0].IsObject());
  EXPECT_TRUE(result[route_name][0].HasMember("legs"));
  EXPECT_TRUE(result[route_name][0]["legs"].IsArray());

  std::vector<std::string> actual_names;
  for (auto leg_iter = result[route_name][0]["legs"].Begin();
       leg_iter != result[route_name][0]["legs"].End(); ++leg_iter) {
    EXPECT_TRUE(leg_iter->IsObject());
    EXPECT_TRUE(leg_iter->HasMember("steps"));
    EXPECT_TRUE(leg_iter->FindMember("steps")->value.IsArray());
    for (auto step_iter = leg_iter->FindMember("steps")->value.Begin();
         step_iter != leg_iter->FindMember("steps")->value.End(); ++step_iter) {

      // If we're on the last leg, append the last step, otherwise skip it, because it'll
      // exist on the first step of the next leg
      if (leg_iter == result[route_name][0]["legs"].End() ||
          step_iter != leg_iter->FindMember("steps")->value.End()) {
        EXPECT_TRUE(step_iter->IsObject());
        auto name = step_iter->FindMember("name");
        EXPECT_NE(name, step_iter->MemberEnd());
        EXPECT_TRUE(name->value.IsString());
        actual_names.push_back(name->value.GetString());
      }
    }
  }

  if (dedupe) {
    auto last = std::unique(actual_names.begin(), actual_names.end());
    actual_names.erase(last, actual_names.end());
  }

  EXPECT_EQ(actual_names, expected_names) << "Actual steps didn't match expected steps";
}
/**
 * Tests if a found path traverses the expected roads in the expected order
 *
 * @param result the result of a /route or /match request
 * @param expected_names the names of the roads the path should traverse in order
 * @param dedupe whether subsequent same-name roads should appear multiple times or not (default not)
 */
void expect_match(valhalla::Api& raw_result,
                  const std::vector<std::string>& expected_names,
                  bool dedupe) {

  rapidjson::Document result = convert_to_json(raw_result, valhalla::Options_Format_osrm);
  if (result.HasParseError()) {
    FAIL() << "Error converting route response to JSON";
  }

  EXPECT_TRUE(result.HasMember("matchings"));
  EXPECT_TRUE(result["matchings"].IsArray());
  EXPECT_EQ(result["matchings"].Size(), 1);

  EXPECT_TRUE(result["matchings"][0].IsObject());
  EXPECT_TRUE(result["matchings"][0].HasMember("legs"));
  EXPECT_TRUE(result["matchings"][0]["legs"].IsArray());

  std::vector<std::string> actual_names;
  for (auto leg_iter = result["matchings"][0]["legs"].Begin();
       leg_iter != result["matchings"][0]["legs"].End(); ++leg_iter) {
    EXPECT_TRUE(leg_iter->IsObject());
    EXPECT_TRUE(leg_iter->HasMember("steps"));
    EXPECT_TRUE(leg_iter->FindMember("steps")->value.IsArray());
    for (auto step_iter = leg_iter->FindMember("steps")->value.Begin();
         step_iter != leg_iter->FindMember("steps")->value.End(); ++step_iter) {

      // If we're on the last leg, append the last step, otherwise skip it, because it'll
      // exist on the first step of the next leg
      if (leg_iter == result["matchings"][0]["legs"].End() ||
          step_iter != leg_iter->FindMember("steps")->value.End()) {
        EXPECT_TRUE(step_iter->IsObject());
        auto name = step_iter->FindMember("name");
        EXPECT_NE(name, step_iter->MemberEnd());
        EXPECT_TRUE(name->value.IsString());
        actual_names.push_back(name->value.GetString());
      }
    }
  }

  if (dedupe) {
    auto last = std::unique(actual_names.begin(), actual_names.end());
    actual_names.erase(last, actual_names.end());
  }

  EXPECT_EQ(actual_names, expected_names) << "Actual path didn't match expected path";
}
} // namespace osrm

namespace raw {
/**
 * Tests whether the expected sequence of maneuvers is emitted for the route.
 * Looks at the output of Odin in the result.
 *
 * @param result the result of a /route or /match request
 * @param expected_maneuvers all the maneuvers expected in the DirectionsLeg for the route
 */
void expect_maneuvers(const valhalla::Api& result,
                      const std::vector<valhalla::DirectionsLeg_Maneuver_Type>& expected_maneuvers) {

  EXPECT_EQ(result.directions().routes_size(), 1);

  std::vector<valhalla::DirectionsLeg_Maneuver_Type> actual_maneuvers;
  for (const auto& leg : result.directions().routes(0).legs()) {
    for (const auto& maneuver : leg.maneuver()) {
      actual_maneuvers.push_back(maneuver.type());
    }
  }

  EXPECT_EQ(actual_maneuvers, expected_maneuvers)
      << "Actual maneuvers didn't match expected maneuvers";
}

/**
 * Tests whether the expected sequence of maneuver begin path indexes is emitted for the route.
 * Looks at the output of Odin in the result.
 *
 * @param result the result of a /route or /match request
 * @param expected_indexes all the maneuver begin path indexes expected in the DirectionsLeg
 *                         for the route
 */
void expect_maneuver_begin_path_indexes(const valhalla::Api& result,
                                        const std::vector<uint32_t>& expected_indexes) {

  EXPECT_EQ(result.directions().routes_size(), 1);

  std::vector<uint32_t> actual_indexes;
  for (const auto& leg : result.directions().routes(0).legs()) {
    for (const auto& maneuver : leg.maneuver()) {
      actual_indexes.push_back(maneuver.begin_path_index());
    }
  }

  EXPECT_EQ(actual_indexes, expected_indexes)
      << "Actual maneuver begin path indexes didn't match expected indexes";
}

/**
 * Tests whether the expected set of instructions is emitted for the specified maneuver index.
 * Looks at the output of Odin in the result.
 *
 * @param result the result of a /route or /match request
 * @param maneuver_index the specified maneuver index to inspect
 * @param expected_text_instruction the expected text instruction
 * @param expected_verbal_transition_alert_instruction the expected verbal transition alert
 *                                                     instruction
 * @param expected_verbal_pre_transition_instruction the expected verbal pre-transition instruction
 * @param expected_verbal_post_transition_instruction the expected verbal post-transition instruction
 */
void expect_instructions_at_maneuver_index(
    const valhalla::Api& result,
    int maneuver_index,
    const std::string& expected_text_instruction,
    const std::string& expected_verbal_transition_alert_instruction,
    const std::string& expected_verbal_pre_transition_instruction,
    const std::string& expected_verbal_post_transition_instruction) {

  ASSERT_EQ(result.directions().routes_size(), 1);
  ASSERT_EQ(result.directions().routes(0).legs_size(), 1);
  ASSERT_TRUE((maneuver_index >= 0) &&
              (maneuver_index < result.directions().routes(0).legs(0).maneuver_size()));
  const auto& maneuver = result.directions().routes(0).legs(0).maneuver(maneuver_index);

  EXPECT_EQ(maneuver.text_instruction(), expected_text_instruction);
  EXPECT_EQ(maneuver.verbal_transition_alert_instruction(),
            expected_verbal_transition_alert_instruction);
  EXPECT_EQ(maneuver.verbal_pre_transition_instruction(), expected_verbal_pre_transition_instruction);
  EXPECT_EQ(maneuver.verbal_post_transition_instruction(),
            expected_verbal_post_transition_instruction);
}

void expect_path_length(const valhalla::Api& result,
                        const float expected_length_km,
                        const float error_margin) {
  EXPECT_EQ(result.trip().routes_size(), 1);

  double length_m = 0;
  for (const auto& route : result.trip().routes()) {
    for (const auto& leg : route.legs()) {
      auto points = midgard::decode<std::vector<midgard::PointLL>>(leg.shape());
      length_m += midgard::length(points);
    }
  }

  if (error_margin == 0) {
    EXPECT_FLOAT_EQ(static_cast<float>(length_m), expected_length_km * 1000);
  } else {
    EXPECT_NEAR(static_cast<float>(length_m), expected_length_km * 1000, 1.f);
  }

  double length_km = 0;
  for (const auto& leg : result.trip().routes(0).legs()) {
    for (const auto& node : leg.node()) {
      if (node.has_edge())
        length_km += node.edge().length_km();
    }
  }

  if (error_margin == 0) {
    EXPECT_FLOAT_EQ(static_cast<float>(length_km), expected_length_km);
  } else {
    EXPECT_NEAR(static_cast<float>(length_km), expected_length_km, error_margin);
  }

  length_km = 0;
  for (const auto& leg : result.directions().routes(0).legs()) {
    length_km += leg.summary().length();
  }

  if (error_margin == 0) {
    EXPECT_FLOAT_EQ(static_cast<float>(length_km), expected_length_km);
  } else {
    EXPECT_NEAR(static_cast<float>(length_km), expected_length_km, error_margin);
  }
}

void expect_eta(const valhalla::Api& result,
                const float expected_eta_seconds,
                const float error_margin) {
  EXPECT_EQ(result.trip().routes_size(), 1);

  double eta_sec = 0;
  for (const auto& leg : result.directions().routes(0).legs()) {
    eta_sec += leg.summary().time();
  }

  if (error_margin == 0) {
    EXPECT_FLOAT_EQ(static_cast<float>(eta_sec), expected_eta_seconds);
  } else {
    EXPECT_NEAR(static_cast<float>(eta_sec), expected_eta_seconds, error_margin);
  }
}

/**
 * Tests if a found path traverses the expected edges in the expected order
 *
 * @param result the result of a /route or /match request
 * @param expected_names the names of the edges the path should traverse in order
 */
void expect_path(const valhalla::Api& result, const std::vector<std::string>& expected_names) {
  EXPECT_EQ(result.trip().routes_size(), 1);

  std::vector<std::string> actual_names;
  for (const auto& leg : result.trip().routes(0).legs()) {
    for (const auto& node : leg.node()) {
      if (node.has_edge()) {
        actual_names.push_back(detail::to_string(node.edge().name()));
      }
    }
  }

  EXPECT_EQ(actual_names, expected_names) << "Actual path didn't match expected path";
}

} // namespace raw
} // namespace assert

} // namespace gurka
} // namespace valhalla
