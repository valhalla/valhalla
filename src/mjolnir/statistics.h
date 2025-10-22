#ifndef VALHALLA_MJOLNIR_STATISTICS_H_
#define VALHALLA_MJOLNIR_STATISTICS_H_

#include "baldr/graphconstants.h"
#include "midgard/aabb2.h"
#include "midgard/pointll.h"

#include <boost/property_tree/ptree_fwd.hpp>

#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct sqlite3_stmt;

namespace valhalla {
namespace mjolnir {

namespace {
const std::map<baldr::RoadClass, std::string> roadClassToString =
    {{baldr::RoadClass::kMotorway, "Motorway"},
     {baldr::RoadClass::kTrunk, "Trunk"},
     {baldr::RoadClass::kPrimary, "Primary"},
     {baldr::RoadClass::kSecondary, "Secondary"},
     {baldr::RoadClass::kTertiary, "Tertiary"},
     {baldr::RoadClass::kUnclassified, "Unclassified"},
     {baldr::RoadClass::kResidential, "Residential"},
     {baldr::RoadClass::kServiceOther, "ServiceOther"}};
const std::vector<baldr::RoadClass> rclasses =
    {baldr::RoadClass::kMotorway,     baldr::RoadClass::kPrimary,     baldr::RoadClass::kSecondary,
     baldr::RoadClass::kTertiary,     baldr::RoadClass::kTrunk,       baldr::RoadClass::kResidential,
     baldr::RoadClass::kServiceOther, baldr::RoadClass::kUnclassified};
} // namespace

class Sqlite3;

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
    std::size_t operator()(const baldr::RoadClass& r) const {
      return static_cast<size_t>(r);
    }
  };
  // Total Length
  std::unordered_map<uint64_t, std::unordered_map<baldr::RoadClass, float, rclassHasher>>
      tile_lengths;
  std::unordered_map<std::string, std::unordered_map<baldr::RoadClass, float, rclassHasher>>
      country_lengths;
  // Internal Edges Count
  std::unordered_map<uint64_t, std::unordered_map<baldr::RoadClass, size_t, rclassHasher>>
      tile_int_edges;
  std::unordered_map<std::string, std::unordered_map<baldr::RoadClass, size_t, rclassHasher>>
      country_int_edges;
  // Length of one way road
  std::unordered_map<uint64_t, std::unordered_map<baldr::RoadClass, float, rclassHasher>>
      tile_one_way;
  std::unordered_map<std::string, std::unordered_map<baldr::RoadClass, float, rclassHasher>>
      country_one_way;
  // Length of road with speed info
  std::unordered_map<uint64_t, std::unordered_map<baldr::RoadClass, float, rclassHasher>>
      tile_speed_info;
  std::unordered_map<std::string, std::unordered_map<baldr::RoadClass, float, rclassHasher>>
      country_speed_info;
  // Length of named road
  std::unordered_map<uint64_t, std::unordered_map<baldr::RoadClass, float, rclassHasher>> tile_named;
  std::unordered_map<std::string, std::unordered_map<baldr::RoadClass, float, rclassHasher>>
      country_named;
  // Length of road with hazmat
  std::unordered_map<uint64_t, std::unordered_map<baldr::RoadClass, float, rclassHasher>> tile_hazmat;
  std::unordered_map<std::string, std::unordered_map<baldr::RoadClass, float, rclassHasher>>
      country_hazmat;
  // Length of road that is a truck route
  std::unordered_map<uint64_t, std::unordered_map<baldr::RoadClass, float, rclassHasher>>
      tile_truck_route;
  std::unordered_map<std::string, std::unordered_map<baldr::RoadClass, float, rclassHasher>>
      country_truck_route;
  // Count of roads with height
  std::unordered_map<uint64_t, std::unordered_map<baldr::RoadClass, size_t, rclassHasher>>
      tile_height;
  std::unordered_map<std::string, std::unordered_map<baldr::RoadClass, size_t, rclassHasher>>
      country_height;
  // Count of roads with width
  std::unordered_map<uint64_t, std::unordered_map<baldr::RoadClass, size_t, rclassHasher>> tile_width;
  std::unordered_map<std::string, std::unordered_map<baldr::RoadClass, size_t, rclassHasher>>
      country_width;
  // Count of roads with length
  std::unordered_map<uint64_t, std::unordered_map<baldr::RoadClass, size_t, rclassHasher>>
      tile_length;
  std::unordered_map<std::string, std::unordered_map<baldr::RoadClass, size_t, rclassHasher>>
      country_length;
  // Count of roads with weight
  std::unordered_map<uint64_t, std::unordered_map<baldr::RoadClass, size_t, rclassHasher>>
      tile_weight;
  std::unordered_map<std::string, std::unordered_map<baldr::RoadClass, size_t, rclassHasher>>
      country_weight;
  // Count of roads with axle_load
  std::unordered_map<uint64_t, std::unordered_map<baldr::RoadClass, size_t, rclassHasher>>
      tile_axle_load;
  std::unordered_map<std::string, std::unordered_map<baldr::RoadClass, size_t, rclassHasher>>
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
  std::unordered_map<uint64_t, midgard::AABB2<midgard::PointLL>> tile_geometries;

public:
  struct RouletteData {
    std::unordered_map<uint64_t, midgard::AABB2<midgard::PointLL>> shape_bb;
    ;
    std::unordered_map<uint64_t, std::vector<midgard::PointLL>> way_shapes;
    std::unordered_set<uint64_t> way_IDs;
    std::unordered_set<midgard::PointLL, std::hash<midgard::PointLL>> unroutable_nodes;

    RouletteData();

    void AddTask(const midgard::AABB2<midgard::PointLL>& shape_bb,
                 const uint64_t id,
                 const std::vector<midgard::PointLL>& shape);

    void AddNode(const midgard::PointLL& p);

    void Add(const RouletteData& rd);

    void GenerateTasks(const boost::property_tree::ptree& pt) const;
  } roulette_data;

  void add_tile_road(const uint64_t& tile_id, const baldr::RoadClass& rclass, const float length);
  void
  add_country_road(const std::string& ctry_code, const baldr::RoadClass& rclass, const float length);

  void
  add_tile_int_edge(const uint64_t& tile_id, const baldr::RoadClass& rclass, const size_t& count = 1);
  void add_country_int_edge(const std::string& ctry_code,
                            const baldr::RoadClass& rclass,
                            const size_t& count = 1);

  void add_tile_one_way(const uint64_t& tile_id, const baldr::RoadClass& rclass, const float length);
  void add_country_one_way(const std::string& ctry_code,
                           const baldr::RoadClass& rclass,
                           const float length);

  void
  add_tile_speed_info(const uint64_t& tile_id, const baldr::RoadClass& rclass, const float length);
  void add_country_speed_info(const std::string& ctry_code,
                              const baldr::RoadClass& rclass,
                              const float length);

  void add_tile_named(const uint64_t& tile_id, const baldr::RoadClass& rclass, const float length);
  void
  add_country_named(const std::string& ctry_code, const baldr::RoadClass& rclass, const float length);

  void add_tile_hazmat(const uint64_t& tile_id, const baldr::RoadClass& rclass, const float length);
  void add_country_hazmat(const std::string& ctry_code,
                          const baldr::RoadClass& rclass,
                          const float length);

  void
  add_tile_truck_route(const uint64_t& tile_id, const baldr::RoadClass& rclass, const float length);
  void add_country_truck_route(const std::string& ctry_code,
                               const baldr::RoadClass& rclass,
                               const float length);

  void
  add_tile_height(const uint64_t& tile_id, const baldr::RoadClass& rclass, const size_t& count = 1);
  void add_country_height(const std::string& ctry_code,
                          const baldr::RoadClass& rclass,
                          const size_t& count = 1);

  void
  add_tile_width(const uint64_t& tile_id, const baldr::RoadClass& rclass, const size_t& count = 1);
  void add_country_width(const std::string& ctry_code,
                         const baldr::RoadClass& rclass,
                         const size_t& count = 1);

  void
  add_tile_length(const uint64_t& tile_id, const baldr::RoadClass& rclass, const size_t& count = 1);
  void add_country_length(const std::string& ctry_code,
                          const baldr::RoadClass& rclass,
                          const size_t& count = 1);

  void
  add_tile_weight(const uint64_t& tile_id, const baldr::RoadClass& rclass, const size_t& count = 1);
  void add_country_weight(const std::string& ctry_code,
                          const baldr::RoadClass& rclass,
                          const size_t& count = 1);

  void add_tile_axle_load(const uint64_t& tile_id,
                          const baldr::RoadClass& rclass,
                          const size_t& count = 1);
  void add_country_axle_load(const std::string& ctry_code,
                             const baldr::RoadClass& rclass,
                             const size_t& count = 1);

  void add_tile_area(const uint64_t& tile_id, const float area);
  void add_tile_geom(const uint64_t& tile_id, const midgard::AABB2<midgard::PointLL> geom);

  // Overloads for tiles
  void add_fork_exitinfo(const std::pair<uint64_t, short>& fork_signs);
  void add_exitinfo(const std::pair<uint64_t, short>& exitinfo);

  // Overloads for countries
  void add_fork_exitinfo(const std::pair<std::string, short>& fork_signs);
  void add_exitinfo(const std::pair<std::string, short>& exitinfo);

  const std::unordered_set<uint64_t>& get_ids() const;

  const std::unordered_set<std::string>& get_isos() const;

  const std::unordered_map<uint64_t,
                           std::unordered_map<baldr::RoadClass, float, statistics::rclassHasher>>&
  get_tile_lengths() const;
  const std::unordered_map<std::string,
                           std::unordered_map<baldr::RoadClass, float, statistics::rclassHasher>>&
  get_country_lengths() const;

  const std::unordered_map<uint64_t,
                           std::unordered_map<baldr::RoadClass, size_t, statistics::rclassHasher>>&
  get_tile_int_edges() const;
  const std::unordered_map<std::string,
                           std::unordered_map<baldr::RoadClass, size_t, statistics::rclassHasher>>&
  get_country_int_edges() const;

  const std::unordered_map<uint64_t,
                           std::unordered_map<baldr::RoadClass, float, statistics::rclassHasher>>&
  get_tile_one_way() const;
  const std::unordered_map<std::string,
                           std::unordered_map<baldr::RoadClass, float, statistics::rclassHasher>>&
  get_country_one_way() const;

  const std::unordered_map<uint64_t,
                           std::unordered_map<baldr::RoadClass, float, statistics::rclassHasher>>&
  get_tile_speed_info() const;
  const std::unordered_map<std::string,
                           std::unordered_map<baldr::RoadClass, float, statistics::rclassHasher>>&
  get_country_speed_info() const;

  const std::unordered_map<uint64_t,
                           std::unordered_map<baldr::RoadClass, float, statistics::rclassHasher>>&
  get_tile_named() const;
  const std::unordered_map<std::string,
                           std::unordered_map<baldr::RoadClass, float, statistics::rclassHasher>>&
  get_country_named() const;

  const std::unordered_map<uint64_t,
                           std::unordered_map<baldr::RoadClass, float, statistics::rclassHasher>>&
  get_tile_hazmat() const;
  const std::unordered_map<std::string,
                           std::unordered_map<baldr::RoadClass, float, statistics::rclassHasher>>&
  get_country_hazmat() const;

  const std::unordered_map<uint64_t,
                           std::unordered_map<baldr::RoadClass, float, statistics::rclassHasher>>&
  get_tile_truck_route() const;
  const std::unordered_map<std::string,
                           std::unordered_map<baldr::RoadClass, float, statistics::rclassHasher>>&
  get_country_truck_route() const;

  const std::unordered_map<uint64_t,
                           std::unordered_map<baldr::RoadClass, size_t, statistics::rclassHasher>>&
  get_tile_height() const;
  const std::unordered_map<std::string,
                           std::unordered_map<baldr::RoadClass, size_t, statistics::rclassHasher>>&
  get_country_height() const;

  const std::unordered_map<uint64_t,
                           std::unordered_map<baldr::RoadClass, size_t, statistics::rclassHasher>>&
  get_tile_width() const;
  const std::unordered_map<std::string,
                           std::unordered_map<baldr::RoadClass, size_t, statistics::rclassHasher>>&
  get_country_width() const;

  const std::unordered_map<uint64_t,
                           std::unordered_map<baldr::RoadClass, size_t, statistics::rclassHasher>>&
  get_tile_length() const;
  const std::unordered_map<std::string,
                           std::unordered_map<baldr::RoadClass, size_t, statistics::rclassHasher>>&
  get_country_length() const;

  const std::unordered_map<uint64_t,
                           std::unordered_map<baldr::RoadClass, size_t, statistics::rclassHasher>>&
  get_tile_weight() const;
  const std::unordered_map<std::string,
                           std::unordered_map<baldr::RoadClass, size_t, statistics::rclassHasher>>&
  get_country_weight() const;

  const std::unordered_map<uint64_t,
                           std::unordered_map<baldr::RoadClass, size_t, statistics::rclassHasher>>&
  get_tile_axle_load() const;
  const std::unordered_map<std::string,
                           std::unordered_map<baldr::RoadClass, size_t, statistics::rclassHasher>>&
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

  const std::unordered_map<uint64_t, midgard::AABB2<midgard::PointLL>>& get_tile_geometries() const;

  void add(const statistics& stats);

  void build_db();

private:
  void create_tile_tables(Sqlite3& db);

  void create_country_tables(Sqlite3& db);

  void create_exit_tables(Sqlite3& db);

  void insert_tile_data(Sqlite3& db, sqlite3_stmt* stmt);

  void insert_country_data(Sqlite3& db, sqlite3_stmt* stmt);

  void insert_exit_data(Sqlite3& db, sqlite3_stmt* stmt);
};
} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_STATISTICS_H_
