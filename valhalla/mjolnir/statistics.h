#ifndef VALHALLA_MJOLNIR_STATISTICS_H_
#define VALHALLA_MJOLNIR_STATISTICS_H_

#include <set>
#include <string>
#include <vector>
#include <map>

#include <valhalla/baldr/graphconstants.h>
#include <boost/property_tree/ptree.hpp>

using namespace valhalla::baldr;

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
  std::map<int32_t, std::map<RoadClass, float> > tile_maps;
  std::map<std::string, std::map<RoadClass, float> > country_maps;
  std::set<uint32_t> tile_ids;
  std::set<std::string> iso_codes;
  std::vector<std::vector<uint32_t> > dupcounts;
  std::vector<std::vector<float> > densities;
  std::map<RoadClass, std::string> roadClassToString =
    { {RoadClass::kMotorway, "Motorway"}, {RoadClass::kTrunk, "Trunk"}, {RoadClass::kPrimary, "Primary"},
      {RoadClass::kSecondary, "Secondary"}, {RoadClass::kTertiary, "Tertiary"},
      {RoadClass::kUnclassified, "Unclassified"},{RoadClass::kResidential, "Residential"},
      {RoadClass::kServiceOther, "ServiceOther"}
    };
  std::vector<RoadClass> rclasses =
    { RoadClass::kMotorway, RoadClass::kPrimary,
      RoadClass::kResidential, RoadClass::kSecondary,
      RoadClass::kServiceOther, RoadClass::kTertiary,
      RoadClass::kTrunk, RoadClass::kUnclassified
    };
public:

  validator_stats ();

  void add_tile_road (const uint32_t& tile_id, const RoadClass& rclass, float length);

  void add_country_road (const std::string& ctry_code, const RoadClass& rclass, float length);

  void add_density (float density, int level);

  void add_dup (uint32_t newdup, int level);

  const std::set<uint32_t>& get_ids () const;

  const std::set<std::string>& get_isos () const;

  const std::map<int32_t, std::map<RoadClass, float> >& get_tile_maps () const;

  const std::map<std::string, std::map<RoadClass, float> >& get_country_maps () const;

  const std::vector<uint32_t> get_dups(int level) const;

  const std::vector<float> get_densities(int level) const;

  const std::vector<std::vector<uint32_t> > get_dups() const;

  const std::vector<std::vector<float> > get_densities() const;

  void add (const validator_stats& stats);

  void log_tile_stats();

  void log_country_stats();

  void build_db(const boost::property_tree::ptree& pt);
};
}
}

#endif  // VALHALLA_MJOLNIR_STATISTICS_H_
