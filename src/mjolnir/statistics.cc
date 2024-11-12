#include <cstdint>

#include "filesystem.h"
#include "mjolnir/graphvalidator.h"
#include "statistics.h"

#include <boost/property_tree/ptree.hpp>
#include <fstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/json.h"
#include "baldr/tilehierarchy.h"
#include "midgard/aabb2.h"
#include "midgard/logging.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {
// merges contents of sets and maps that do not have overlapping keys
template <class T> T merge(T& a, T b) {
  T tmp(a);
  tmp.insert(b.begin(), b.end());
  return tmp;
}

// accumulates counts into a new map for maps that have counts associated with its keys
template <class T> T merge_counts(T& a, T b) {
  T tmp(a);
  for (auto it = b.begin(); it != b.end(); it++) {
    tmp[it->first] += it->second;
  }
  return tmp;
}

// merge two hashes by key and merge underlying hash buckets
template <class T> T deep_merge_counts(T& a, T const& b) {
  T tmp(a);
  for (auto it = b.begin(); it != b.end(); it++) {
    tmp[it->first] = merge_counts(tmp[it->first], it->second);
  }
  return tmp;
}

} // namespace

namespace valhalla {
namespace mjolnir {

void statistics::add_tile_road(const uint64_t& tile_id, const RoadClass& rclass, const float length) {
  tile_ids.insert(tile_id);
  tile_lengths[tile_id][rclass] += length;
}

void statistics::add_country_road(const std::string& ctry_code,
                                  const RoadClass& rclass,
                                  const float length) {
  iso_codes.insert(ctry_code);
  country_lengths[ctry_code][rclass] += length;
}

void statistics::add_tile_int_edge(const uint64_t& tile_id,
                                   const RoadClass& rclass,
                                   const size_t& count) {
  tile_int_edges[tile_id][rclass] += count;
}

void statistics::add_country_int_edge(const std::string& ctry_code,
                                      const RoadClass& rclass,
                                      const size_t& count) {
  country_int_edges[ctry_code][rclass] += count;
}

void statistics::add_tile_one_way(const uint64_t& tile_id,
                                  const RoadClass& rclass,
                                  const float length) {
  tile_one_way[tile_id][rclass] += length;
}

void statistics::add_country_one_way(const std::string& ctry_code,
                                     const RoadClass& rclass,
                                     const float length) {
  country_one_way[ctry_code][rclass] += length;
}

void statistics::add_tile_speed_info(const uint64_t& tile_id,
                                     const RoadClass& rclass,
                                     const float length) {
  tile_speed_info[tile_id][rclass] += length;
}

void statistics::add_country_speed_info(const std::string& ctry_code,
                                        const RoadClass& rclass,
                                        const float length) {
  country_speed_info[ctry_code][rclass] += length;
}

void statistics::add_tile_named(const uint64_t& tile_id,
                                const RoadClass& rclass,
                                const float length) {
  tile_named[tile_id][rclass] += length;
}

void statistics::add_country_named(const std::string& ctry_code,
                                   const RoadClass& rclass,
                                   const float length) {
  country_named[ctry_code][rclass] += length;
}

void statistics::add_tile_hazmat(const uint64_t& tile_id,
                                 const RoadClass& rclass,
                                 const float length) {
  tile_hazmat[tile_id][rclass] += length;
}

void statistics::add_country_hazmat(const std::string& ctry_code,
                                    const RoadClass& rclass,
                                    const float length) {
  country_hazmat[ctry_code][rclass] += length;
}

void statistics::add_tile_truck_route(const uint64_t& tile_id,
                                      const RoadClass& rclass,
                                      const float length) {
  tile_truck_route[tile_id][rclass] += length;
}

void statistics::add_country_truck_route(const std::string& ctry_code,
                                         const RoadClass& rclass,
                                         const float length) {
  country_truck_route[ctry_code][rclass] += length;
}

void statistics::add_tile_height(const uint64_t& tile_id,
                                 const RoadClass& rclass,
                                 const size_t& count) {
  tile_height[tile_id][rclass] += count;
}

void statistics::add_country_height(const std::string& ctry_code,
                                    const RoadClass& rclass,
                                    const size_t& count) {
  country_height[ctry_code][rclass] += count;
}

void statistics::add_tile_width(const uint64_t& tile_id,
                                const RoadClass& rclass,
                                const size_t& count) {
  tile_width[tile_id][rclass] += count;
}
void statistics::add_country_width(const std::string& ctry_code,
                                   const RoadClass& rclass,
                                   const size_t& count) {
  country_width[ctry_code][rclass] += count;
}

void statistics::add_tile_length(const uint64_t& tile_id,
                                 const RoadClass& rclass,
                                 const size_t& count) {
  tile_length[tile_id][rclass] += count;
}

void statistics::add_country_length(const std::string& ctry_code,
                                    const RoadClass& rclass,
                                    const size_t& count) {
  country_length[ctry_code][rclass] += count;
}

void statistics::add_tile_weight(const uint64_t& tile_id,
                                 const RoadClass& rclass,
                                 const size_t& count) {
  tile_weight[tile_id][rclass] += count;
}

void statistics::add_country_weight(const std::string& ctry_code,
                                    const RoadClass& rclass,
                                    const size_t& count) {
  country_weight[ctry_code][rclass] += count;
}

void statistics::add_tile_axle_load(const uint64_t& tile_id,
                                    const RoadClass& rclass,
                                    const size_t& count) {
  tile_axle_load[tile_id][rclass] += count;
}

void statistics::add_country_axle_load(const std::string& ctry_code,
                                       const RoadClass& rclass,
                                       const size_t& count) {
  country_axle_load[ctry_code][rclass] += count;
}

void statistics::add_exitinfo(const std::pair<uint64_t, short>& exitinfo) {
  tile_exit_signs[exitinfo.first] += exitinfo.second;
  tile_exit_count[exitinfo.first] += 1;
}

void statistics::add_fork_exitinfo(const std::pair<uint64_t, short>& forkinfo) {
  tile_fork_signs[forkinfo.first] += forkinfo.second;
  tile_fork_count[forkinfo.first] += 1;
}

void statistics::add_exitinfo(const std::pair<std::string, short>& exitinfo) {
  ctry_exit_signs[exitinfo.first] += exitinfo.second;
  ctry_exit_count[exitinfo.first] += 1;
}

void statistics::add_fork_exitinfo(const std::pair<std::string, short>& forkinfo) {
  ctry_fork_signs[forkinfo.first] += forkinfo.second;
  ctry_fork_count[forkinfo.first] += 1;
}

void statistics::add_tile_area(const uint64_t& tile_id, const float area) {
  tile_areas[tile_id] = area;
}

void statistics::add_tile_geom(const uint64_t& tile_id, const AABB2<PointLL> geom) {
  tile_geometries[tile_id] = geom;
}

const std::unordered_set<uint64_t>& statistics::get_ids() const {
  return tile_ids;
}

const std::unordered_set<std::string>& statistics::get_isos() const {
  return iso_codes;
}

const std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
statistics::get_tile_lengths() const {
  return tile_lengths;
}
const std::unordered_map<std::string, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
statistics::get_country_lengths() const {
  return country_lengths;
}

const std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
statistics::get_tile_int_edges() const {
  return tile_int_edges;
}
const std::unordered_map<std::string,
                         std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
statistics::get_country_int_edges() const {
  return country_int_edges;
}

const std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
statistics::get_tile_one_way() const {
  return tile_one_way;
}
const std::unordered_map<std::string, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
statistics::get_country_one_way() const {
  return country_one_way;
}

const std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
statistics::get_tile_speed_info() const {
  return tile_speed_info;
}
const std::unordered_map<std::string, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
statistics::get_country_speed_info() const {
  return country_speed_info;
}

const std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
statistics::get_tile_named() const {
  return tile_named;
}
const std::unordered_map<std::string, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
statistics::get_country_named() const {
  return country_named;
}

const std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
statistics::get_tile_hazmat() const {
  return tile_hazmat;
}
const std::unordered_map<std::string, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
statistics::get_country_hazmat() const {
  return country_hazmat;
}

const std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
statistics::get_tile_truck_route() const {
  return tile_truck_route;
}
const std::unordered_map<std::string, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
statistics::get_country_truck_route() const {
  return country_truck_route;
}

const std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
statistics::get_tile_height() const {
  return tile_height;
}
const std::unordered_map<std::string,
                         std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
statistics::get_country_height() const {
  return country_height;
}

const std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
statistics::get_tile_width() const {
  return tile_width;
}
const std::unordered_map<std::string,
                         std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
statistics::get_country_width() const {
  return country_width;
}

const std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
statistics::get_tile_length() const {
  return tile_length;
}
const std::unordered_map<std::string,
                         std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
statistics::get_country_length() const {
  return country_length;
}

const std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
statistics::get_tile_weight() const {
  return tile_weight;
}
const std::unordered_map<std::string,
                         std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
statistics::get_country_weight() const {
  return country_weight;
}

const std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
statistics::get_tile_axle_load() const {
  return tile_axle_load;
}
const std::unordered_map<std::string,
                         std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
statistics::get_country_axle_load() const {
  return country_axle_load;
}

const std::unordered_map<uint64_t, float>& statistics::get_tile_areas() const {
  return tile_areas;
}

const std::unordered_map<uint64_t, AABB2<PointLL>>& statistics::get_tile_geometries() const {
  return tile_geometries;
}

const std::unordered_map<uint64_t, size_t>& statistics::get_tile_fork_info() const {
  return tile_fork_signs;
}

const std::unordered_map<uint64_t, size_t>& statistics::get_tile_exit_info() const {
  return tile_exit_signs;
}

const std::unordered_map<std::string, size_t>& statistics::get_ctry_fork_info() const {
  return ctry_fork_signs;
}

const std::unordered_map<std::string, size_t>& statistics::get_ctry_exit_info() const {
  return ctry_exit_signs;
}

const std::unordered_map<uint64_t, size_t>& statistics::get_tile_fork_count() const {
  return tile_fork_count;
}

const std::unordered_map<uint64_t, size_t>& statistics::get_tile_exit_count() const {
  return tile_exit_count;
}

const std::unordered_map<std::string, size_t>& statistics::get_ctry_fork_count() const {
  return ctry_fork_count;
}

const std::unordered_map<std::string, size_t>& statistics::get_ctry_exit_count() const {
  return ctry_exit_count;
}

void statistics::add(const statistics& stats) {
  // Combine ids and isos
  tile_ids = merge(tile_ids, stats.get_ids());
  iso_codes = merge(iso_codes, stats.get_isos());

  // Combine tile statistics
  tile_areas = merge(tile_areas, stats.get_tile_areas());
  tile_geometries = merge(tile_geometries, stats.get_tile_geometries());
  tile_lengths = merge(tile_lengths, stats.get_tile_lengths());
  tile_one_way = merge(tile_one_way, stats.get_tile_one_way());
  tile_speed_info = merge(tile_speed_info, stats.get_tile_speed_info());
  tile_int_edges = merge(tile_int_edges, stats.get_tile_int_edges());
  tile_named = merge(tile_named, stats.get_tile_named());
  tile_hazmat = merge(tile_hazmat, stats.get_tile_hazmat());
  tile_truck_route = merge(tile_truck_route, stats.get_tile_truck_route());
  tile_height = merge(tile_height, stats.get_tile_height());
  tile_width = merge(tile_width, stats.get_tile_width());
  tile_length = merge(tile_length, stats.get_tile_length());
  tile_weight = merge(tile_weight, stats.get_tile_weight());
  tile_axle_load = merge(tile_axle_load, stats.get_tile_axle_load());

  // Combine country statistics
  country_lengths = deep_merge_counts(country_lengths, stats.get_country_lengths());
  country_one_way = deep_merge_counts(country_one_way, stats.get_country_one_way());
  country_speed_info = deep_merge_counts(country_speed_info, stats.get_country_speed_info());
  country_int_edges = deep_merge_counts(country_int_edges, stats.get_country_int_edges());
  country_named = deep_merge_counts(country_named, stats.get_country_named());
  country_hazmat = deep_merge_counts(country_hazmat, stats.get_country_hazmat());
  country_truck_route = deep_merge_counts(country_truck_route, stats.get_country_truck_route());
  country_height = deep_merge_counts(country_height, stats.get_country_height());
  country_width = deep_merge_counts(country_width, stats.get_country_width());
  country_length = deep_merge_counts(country_length, stats.get_country_length());
  country_weight = deep_merge_counts(country_weight, stats.get_country_weight());
  country_axle_load = deep_merge_counts(country_axle_load, stats.get_country_axle_load());

  // Combine exit statistics
  tile_exit_signs = merge_counts(tile_exit_signs, stats.get_tile_exit_info());
  ctry_exit_signs = merge_counts(ctry_exit_signs, stats.get_ctry_exit_info());

  tile_exit_count = merge_counts(tile_exit_count, stats.get_tile_exit_count());
  ctry_exit_count = merge_counts(ctry_exit_count, stats.get_ctry_exit_count());

  tile_fork_signs = merge_counts(tile_fork_signs, stats.get_tile_fork_info());
  ctry_fork_signs = merge_counts(ctry_fork_signs, stats.get_ctry_fork_info());

  tile_fork_count = merge_counts(tile_fork_count, stats.get_tile_fork_count());
  ctry_fork_count = merge_counts(ctry_fork_count, stats.get_ctry_fork_count());

  // Combine roulette data
  roulette_data.Add(stats.roulette_data);
}

statistics::RouletteData::RouletteData() : shape_bb(), way_IDs(), way_shapes(), unroutable_nodes() {
}

void statistics::RouletteData::AddTask(const AABB2<PointLL>& bb,
                                       const uint64_t id,
                                       const std::vector<PointLL>& shape) {
  auto result = way_IDs.insert(id);
  if (result.second) {
    shape_bb.insert({id, bb});
  }
  way_shapes.insert({id, shape});
}

void statistics::RouletteData::AddNode(const PointLL& p) {
  unroutable_nodes.insert(p);
}

void statistics::RouletteData::Add(const RouletteData& rd) {
  way_IDs = merge(way_IDs, rd.way_IDs);
  way_shapes = merge(way_shapes, rd.way_shapes);
  shape_bb = merge(shape_bb, rd.shape_bb);
  unroutable_nodes = merge(unroutable_nodes, rd.unroutable_nodes);
}

void statistics::RouletteData::GenerateTasks(const boost::property_tree::ptree& /*pt*/) const {
  // build a task list for each collected wayid
  json::ArrayPtr tasks = json::array({});
  for (auto& id : way_IDs) {
    // build shape array before the rest of the json
    json::ArrayPtr coords =
        json::array({json::array({json::fixed_t{way_shapes.at(id)[0].lng(), 5},
                                  json::fixed_t{way_shapes.at(id)[0].lat(), 5}})});
    for (size_t i = 1; i < way_shapes.at(id).size(); ++i) {
      const auto& way_point = way_shapes.at(id)[i];
      coords->emplace_back(
          json::array({json::fixed_t{way_point.lng(), 5}, json::fixed_t{way_point.lat(), 5}}));
    }
    // build each task into the json array
    tasks->emplace_back(json::map(
        {{"geometry", json::map({{"coordinates", coords}, {"type", std::string("LineString")}})},
         {"properties", json::map({{"osmid", id}, {"type", std::string("Loop")}})},
         {"type", std::string("Feature")},
         {"instruction", std::string("This one way road loops back on itself. Edit it so that the "
                                     "road is properly accessible")}}));
  }
  // Add all unroutable nodes to the json array
  auto hasher = std::hash<PointLL>();
  for (auto it = unroutable_nodes.cbegin(); it != unroutable_nodes.cend(); it++) {
    tasks->emplace_back(json::map(
        {{"geometry", json::map({{"coordinates", json::array({{json::fixed_t{it->lng(), 5}},
                                                              {json::fixed_t{it->lat(), 5}}})},
                                 {"type", std::string("Point")}})},
         {"properties",
          json::map({{"type", std::string("Node")}, {"key", static_cast<uint64_t>(hasher(*it))}})},
         {"type", std::string("Feature")},
         {"instruction",
          std::string("This node is either unreachable or unleavable. Edit the surrounding roads "
                      "so that the node can be accessed properly")}}));
  }
  // Put the tasks' json into the feature collection
  json::MapPtr geo_json = json::map(
      {{"type", std::string("FeatureCollection")},
       {"features", tasks},
       {"properties",
        json::map(
            {{"instructions",
              json::map(
                  {{"Loop", std::string("This one way road loops back on itself. Edit it so "
                                        "that the road is properly accessible")},
                   {"Node",
                    std::string("This node is either unreachable or unleavable. Edit the surrounding "
                                "roads so that the node can be accessed properly")}})}})}});
  // write out to a file
  if (filesystem::exists("maproulette_tasks.geojson")) {
    filesystem::remove("maproulette_tasks.geojson");
  }
  std::ofstream file;
  file.open("maproulette_tasks.geojson");
  file << *geo_json << std::endl;
  file.close();
  LOG_INFO("MapRoulette tasks saved to maproulette_tasks.geojson");
  LOG_INFO(std::to_string(tasks->size()) + " tasks generated");
}
} // namespace mjolnir
} // namespace valhalla
