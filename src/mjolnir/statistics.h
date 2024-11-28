#ifndef VALHALLA_MJOLNIR_STATISTICS_H_
#define VALHALLA_MJOLNIR_STATISTICS_H_

#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <sqlite3.h>

#include "baldr/graphconstants.h"
#include "midgard/aabb2.h"
#include <boost/property_tree/ptree.hpp>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
namespace {
const std::map<RoadClass, std::string> roadClassToString =
    {{RoadClass::kMotorway, "Motorway"},       {RoadClass::kTrunk, "Trunk"},
     {RoadClass::kPrimary, "Primary"},         {RoadClass::kSecondary, "Secondary"},
     {RoadClass::kTertiary, "Tertiary"},       {RoadClass::kUnclassified, "Unclassified"},
     {RoadClass::kResidential, "Residential"}, {RoadClass::kServiceOther, "ServiceOther"}};
const std::vector<RoadClass> rclasses = {RoadClass::kMotorway,     RoadClass::kPrimary,
                                         RoadClass::kSecondary,    RoadClass::kTertiary,
                                         RoadClass::kTrunk,        RoadClass::kResidential,
                                         RoadClass::kServiceOther, RoadClass::kUnclassified};
} // namespace
namespace valhalla {
namespace mjolnir {

/**
 * This class gathers statistics on the road lengths within tile
 *  and country boundaries and breaks them down by road classification.
 *  The statistics are separated by the tile ID and by the
 *  country ISO code. The statistics are logged to debug by
 *  default and put into a sqlite3 DB file as defined in valhalla.json.
 *  This class also handles the values returned by the threads
 *  in GraphValidator.
 */

class statistics {
  struct rclassHasher {
    std::size_t operator()(const RoadClass& r) const {
      return static_cast<size_t>(r);
    }
  };
  // Total Length
  std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, rclassHasher>> tile_lengths;
  std::unordered_map<std::string, std::unordered_map<RoadClass, float, rclassHasher>> country_lengths;
  // Internal Edges Count
  std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, rclassHasher>> tile_int_edges;
  std::unordered_map<std::string, std::unordered_map<RoadClass, size_t, rclassHasher>>
      country_int_edges;
  // Length of one way road
  std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, rclassHasher>> tile_one_way;
  std::unordered_map<std::string, std::unordered_map<RoadClass, float, rclassHasher>> country_one_way;
  // Length of road with speed info
  std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, rclassHasher>> tile_speed_info;
  std::unordered_map<std::string, std::unordered_map<RoadClass, float, rclassHasher>>
      country_speed_info;
  // Length of named road
  std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, rclassHasher>> tile_named;
  std::unordered_map<std::string, std::unordered_map<RoadClass, float, rclassHasher>> country_named;
  // Length of road with hazmat
  std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, rclassHasher>> tile_hazmat;
  std::unordered_map<std::string, std::unordered_map<RoadClass, float, rclassHasher>> country_hazmat;
  // Length of road that is a truck route
  std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, rclassHasher>> tile_truck_route;
  std::unordered_map<std::string, std::unordered_map<RoadClass, float, rclassHasher>>
      country_truck_route;
  // Count of roads with height
  std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, rclassHasher>> tile_height;
  std::unordered_map<std::string, std::unordered_map<RoadClass, size_t, rclassHasher>> country_height;
  // Count of roads with width
  std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, rclassHasher>> tile_width;
  std::unordered_map<std::string, std::unordered_map<RoadClass, size_t, rclassHasher>> country_width;
  // Count of roads with length
  std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, rclassHasher>> tile_length;
  std::unordered_map<std::string, std::unordered_map<RoadClass, size_t, rclassHasher>> country_length;
  // Count of roads with weight
  std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, rclassHasher>> tile_weight;
  std::unordered_map<std::string, std::unordered_map<RoadClass, size_t, rclassHasher>> country_weight;
  // Count of roads with axle_load
  std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, rclassHasher>> tile_axle_load;
  std::unordered_map<std::string, std::unordered_map<RoadClass, size_t, rclassHasher>>
      country_axle_load;
  // Exit sign info for forks in tiles
  std::unordered_map<uint64_t, size_t> tile_fork_signs;
  std::unordered_map<uint64_t, size_t> tile_exit_signs;
  std::unordered_map<uint64_t, size_t> tile_fork_count;
  std::unordered_map<uint64_t, size_t> tile_exit_count;

  // Exit sign info for forks in countries
  std::unordered_map<std::string, size_t> ctry_fork_signs;
  std::unordered_map<std::string, size_t> ctry_exit_signs;
  std::unordered_map<std::string, size_t> ctry_fork_count;
  std::unordered_map<std::string, size_t> ctry_exit_count;

  std::unordered_set<uint64_t> tile_ids;
  std::unordered_set<std::string> iso_codes;
  std::unordered_map<uint64_t, float> tile_areas;
  std::unordered_map<uint64_t, AABB2<PointLL>> tile_geometries;

public:
  struct RouletteData {
    std::unordered_map<uint64_t, AABB2<PointLL>> shape_bb;
    ;
    std::unordered_map<uint64_t, std::vector<PointLL>> way_shapes;
    std::unordered_set<uint64_t> way_IDs;
    std::unordered_set<PointLL, std::hash<PointLL>> unroutable_nodes;

    RouletteData();

    void
    AddTask(const AABB2<PointLL>& shape_bb, const uint64_t id, const std::vector<PointLL>& shape);

    void AddNode(const PointLL& p);

    void Add(const RouletteData& rd);

    void GenerateTasks(const boost::property_tree::ptree& pt) const;
  } roulette_data;

  void add_tile_road(const uint64_t& tile_id, const RoadClass& rclass, const float length);
  void add_country_road(const std::string& ctry_code, const RoadClass& rclass, const float length);

  void add_tile_int_edge(const uint64_t& tile_id, const RoadClass& rclass, const size_t& count = 1);
  void add_country_int_edge(const std::string& ctry_code,
                            const RoadClass& rclass,
                            const size_t& count = 1);

  void add_tile_one_way(const uint64_t& tile_id, const RoadClass& rclass, const float length);
  void add_country_one_way(const std::string& ctry_code, const RoadClass& rclass, const float length);

  void add_tile_speed_info(const uint64_t& tile_id, const RoadClass& rclass, const float length);
  void
  add_country_speed_info(const std::string& ctry_code, const RoadClass& rclass, const float length);

  void add_tile_named(const uint64_t& tile_id, const RoadClass& rclass, const float length);
  void add_country_named(const std::string& ctry_code, const RoadClass& rclass, const float length);

  void add_tile_hazmat(const uint64_t& tile_id, const RoadClass& rclass, const float length);
  void add_country_hazmat(const std::string& ctry_code, const RoadClass& rclass, const float length);

  void add_tile_truck_route(const uint64_t& tile_id, const RoadClass& rclass, const float length);
  void
  add_country_truck_route(const std::string& ctry_code, const RoadClass& rclass, const float length);

  void add_tile_height(const uint64_t& tile_id, const RoadClass& rclass, const size_t& count = 1);
  void
  add_country_height(const std::string& ctry_code, const RoadClass& rclass, const size_t& count = 1);

  void add_tile_width(const uint64_t& tile_id, const RoadClass& rclass, const size_t& count = 1);
  void
  add_country_width(const std::string& ctry_code, const RoadClass& rclass, const size_t& count = 1);

  void add_tile_length(const uint64_t& tile_id, const RoadClass& rclass, const size_t& count = 1);
  void
  add_country_length(const std::string& ctry_code, const RoadClass& rclass, const size_t& count = 1);

  void add_tile_weight(const uint64_t& tile_id, const RoadClass& rclass, const size_t& count = 1);
  void
  add_country_weight(const std::string& ctry_code, const RoadClass& rclass, const size_t& count = 1);

  void add_tile_axle_load(const uint64_t& tile_id, const RoadClass& rclass, const size_t& count = 1);
  void add_country_axle_load(const std::string& ctry_code,
                             const RoadClass& rclass,
                             const size_t& count = 1);

  void add_tile_area(const uint64_t& tile_id, const float area);
  void add_tile_geom(const uint64_t& tile_id, const AABB2<PointLL> geom);

  // Overloads for tiles
  void add_fork_exitinfo(const std::pair<uint64_t, short>& fork_signs);
  void add_exitinfo(const std::pair<uint64_t, short>& exitinfo);

  // Overloads for countries
  void add_fork_exitinfo(const std::pair<std::string, short>& fork_signs);
  void add_exitinfo(const std::pair<std::string, short>& exitinfo);

  const std::unordered_set<uint64_t>& get_ids() const;

  const std::unordered_set<std::string>& get_isos() const;

  const std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
  get_tile_lengths() const;
  const std::unordered_map<std::string,
                           std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
  get_country_lengths() const;

  const std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
  get_tile_int_edges() const;
  const std::unordered_map<std::string,
                           std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
  get_country_int_edges() const;

  const std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
  get_tile_one_way() const;
  const std::unordered_map<std::string,
                           std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
  get_country_one_way() const;

  const std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
  get_tile_speed_info() const;
  const std::unordered_map<std::string,
                           std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
  get_country_speed_info() const;

  const std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
  get_tile_named() const;
  const std::unordered_map<std::string,
                           std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
  get_country_named() const;

  const std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
  get_tile_hazmat() const;
  const std::unordered_map<std::string,
                           std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
  get_country_hazmat() const;

  const std::unordered_map<uint64_t, std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
  get_tile_truck_route() const;
  const std::unordered_map<std::string,
                           std::unordered_map<RoadClass, float, statistics::rclassHasher>>&
  get_country_truck_route() const;

  const std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
  get_tile_height() const;
  const std::unordered_map<std::string,
                           std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
  get_country_height() const;

  const std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
  get_tile_width() const;
  const std::unordered_map<std::string,
                           std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
  get_country_width() const;

  const std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
  get_tile_length() const;
  const std::unordered_map<std::string,
                           std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
  get_country_length() const;

  const std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
  get_tile_weight() const;
  const std::unordered_map<std::string,
                           std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
  get_country_weight() const;

  const std::unordered_map<uint64_t, std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
  get_tile_axle_load() const;
  const std::unordered_map<std::string,
                           std::unordered_map<RoadClass, size_t, statistics::rclassHasher>>&
  get_country_axle_load() const;

  const std::unordered_map<uint64_t, size_t>& get_tile_fork_info() const;

  const std::unordered_map<uint64_t, size_t>& get_tile_exit_info() const;

  const std::unordered_map<std::string, size_t>& get_ctry_fork_info() const;

  const std::unordered_map<std::string, size_t>& get_ctry_exit_info() const;

  const std::unordered_map<uint64_t, size_t>& get_tile_fork_count() const;

  const std::unordered_map<uint64_t, size_t>& get_tile_exit_count() const;

  const std::unordered_map<std::string, size_t>& get_ctry_fork_count() const;

  const std::unordered_map<std::string, size_t>& get_ctry_exit_count() const;

  const std::unordered_map<uint64_t, float>& get_tile_areas() const;

  const std::unordered_map<uint64_t, AABB2<PointLL>>& get_tile_geometries() const;

  void add(const statistics& stats);

  void build_db();

private:
  void create_tile_tables(sqlite3* db_handle);

  void create_country_tables(sqlite3* db_handle);

  void create_exit_tables(sqlite3* db_handle);

  void insert_tile_data(sqlite3* db_handle, sqlite3_stmt* stmt);

  void insert_country_data(sqlite3* db_handle, sqlite3_stmt* stmt);

  void insert_exit_data(sqlite3* db_handle, sqlite3_stmt* stmt);
};
} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_STATISTICS_H_
