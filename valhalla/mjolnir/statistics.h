#ifndef VALHALLA_MJOLNIR_STATISTICS_H_
#define VALHALLA_MJOLNIR_STATISTICS_H_

#include <set>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include "valhalla/midgard/aabb2.h"
#include <valhalla/baldr/graphconstants.h>
#include <boost/property_tree/ptree.hpp>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
namespace {
  const std::map<RoadClass, std::string> roadClassToString =
    { {RoadClass::kMotorway, "Motorway"}, {RoadClass::kTrunk, "Trunk"}, {RoadClass::kPrimary, "Primary"},
      {RoadClass::kSecondary, "Secondary"}, {RoadClass::kTertiary, "Tertiary"},
      {RoadClass::kUnclassified, "Unclassified"},{RoadClass::kResidential, "Residential"},
      {RoadClass::kServiceOther, "ServiceOther"}
    };
  const std::vector<RoadClass> rclasses =
    { RoadClass::kMotorway, RoadClass::kPrimary,
      RoadClass::kResidential, RoadClass::kSecondary,
      RoadClass::kServiceOther, RoadClass::kTertiary,
      RoadClass::kTrunk, RoadClass::kUnclassified
    };
}
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

class validator_stats {
  struct rclassHasher {
    std::size_t operator()(const RoadClass& r) const {
      return static_cast<size_t>(r);
    }
  };
  // Total Length
  std::unordered_map<uint32_t, std::unordered_map<RoadClass, float, rclassHasher> > tile_lengths;
  std::unordered_map<std::string, std::unordered_map<RoadClass, float, rclassHasher> > country_lengths;
  // Internal Edges Count
  std::unordered_map<uint32_t, std::unordered_map<RoadClass, uint32_t, rclassHasher> > tile_int_edges;
  std::unordered_map<std::string, std::unordered_map<RoadClass, uint32_t, rclassHasher> > country_int_edges;
  // Length of one way road
  std::unordered_map<uint32_t, std::unordered_map<RoadClass, float, rclassHasher> > tile_one_way;
  std::unordered_map<std::string, std::unordered_map<RoadClass, float, rclassHasher> > country_one_way;
  // Length of road with speed info
  std::unordered_map<uint32_t, std::unordered_map<RoadClass, float, rclassHasher> > tile_speed_info;
  std::unordered_map<std::string, std::unordered_map<RoadClass, float, rclassHasher> > country_speed_info;
  // Length of named road
  std::unordered_map<uint32_t, std::unordered_map<RoadClass, float, rclassHasher> > tile_named;
  std::unordered_map<std::string, std::unordered_map<RoadClass, float, rclassHasher> > country_named;
  std::unordered_set<uint32_t> tile_ids;
  std::unordered_set<std::string> iso_codes;
  std::unordered_map<uint32_t, float> tile_areas;
  std::unordered_map<uint32_t, AABB2<PointLL>> tile_geometries;
  std::vector<std::vector<uint32_t> > dupcounts;
  std::vector<std::vector<float> > densities;


public:

  validator_stats ();

  void add_tile_road (const uint32_t& tile_id, const RoadClass& rclass, float length);

  void add_country_road (const std::string& ctry_code, const RoadClass& rclass, float length);

  void add_tile_int_edge (const uint32_t& tile_id, const RoadClass& rclass, const uint32_t& count = 1);

  void add_country_int_edge (const std::string& ctry_code, const RoadClass& rclass, const uint32_t& count = 1);

  void add_tile_one_way (const uint32_t& tile_id, const RoadClass& rclass, float length);

  void add_country_one_way (const std::string& ctry_code, const RoadClass& rclass, float length);

  void add_tile_speed_info (const uint32_t& tile_id, const RoadClass& rclass, float length);

  void add_country_speed_info (const std::string& ctry_code, const RoadClass& rclass, float length);

  void add_tile_named (const uint32_t& tile_id, const RoadClass& rclass, float length);

  void add_country_named (const std::string& ctry_code, const RoadClass& rclass, float length);

  void add_tile_area (const uint32_t& tile_id, const float area);

  void add_tile_geom (const uint32_t& tile_id, const AABB2<PointLL> geom);

  void add_density (float density, int level);

  void add_dup (uint32_t newdup, int level);

  const std::unordered_set<uint32_t>& get_ids () const;

  const std::unordered_set<std::string>& get_isos () const;

  const std::unordered_map<uint32_t, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& get_tile_lengths () const;

  const std::unordered_map<std::string, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& get_country_lengths () const;

  const std::unordered_map<uint32_t, std::unordered_map<RoadClass, uint32_t, validator_stats::rclassHasher> >& get_tile_int_edges () const;

  const std::unordered_map<std::string, std::unordered_map<RoadClass, uint32_t, validator_stats::rclassHasher> >& get_country_int_edges () const;

  const std::unordered_map<uint32_t, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& get_tile_one_way () const;

  const std::unordered_map<std::string, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& get_country_one_way () const;

  const std::unordered_map<uint32_t, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& get_tile_speed_info () const;

  const std::unordered_map<std::string, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& get_country_speed_info () const;

  const std::unordered_map<uint32_t, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& get_tile_named () const;

  const std::unordered_map<std::string, std::unordered_map<RoadClass, float, validator_stats::rclassHasher> >& get_country_named () const;

  const std::unordered_map<uint32_t, float>& get_tile_areas() const;

  const std::unordered_map<uint32_t, AABB2<PointLL>>& get_tile_geometries() const;

  const std::vector<uint32_t> get_dups(int level) const;

  const std::vector<float> get_densities(int level) const;

  const std::vector<std::vector<uint32_t> > get_dups() const;

  const std::vector<std::vector<float> > get_densities() const;

  void add (const validator_stats& stats);

  void build_db(const boost::property_tree::ptree& pt);
};
}
}

#endif  // VALHALLA_MJOLNIR_STATISTICS_H_
