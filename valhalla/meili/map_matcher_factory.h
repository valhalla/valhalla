// -*- mode: c++ -*-
#ifndef MMP_MAP_MATCHER_FACTORY_H_
#define MMP_MAP_MATCHER_FACTORY_H_

#include <string>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/sif/costconstants.h>
#include <valhalla/baldr/graphreader.h>

#include <valhalla/meili/candidate_search.h>
#include <valhalla/meili/map_matcher.h>


namespace valhalla {
namespace meili {

class MapMatcherFactory final
{
public:
  MapMatcherFactory(const boost::property_tree::ptree&);

  ~MapMatcherFactory();

  baldr::GraphReader& graphreader()
  { return graphreader_; }

  CandidateQuery& candidatequery()
  { return candidatequery_; }

  sif::TravelMode NameToTravelMode(const std::string&);

  const std::string& TravelModeToName(sif::TravelMode);

  MapMatcher* Create(sif::TravelMode travelmode)
  { return Create(travelmode, boost::property_tree::ptree()); }

  MapMatcher* Create(const std::string& name)
  { return Create(NameToTravelMode(name), boost::property_tree::ptree()); }

  MapMatcher* Create(const std::string& name,
                     const boost::property_tree::ptree& preferences)
  { return Create(NameToTravelMode(name), preferences); }

  MapMatcher* Create(const boost::property_tree::ptree&);

  MapMatcher* Create(sif::TravelMode, const boost::property_tree::ptree&);

  boost::property_tree::ptree
  MergeConfig(const std::string&, const boost::property_tree::ptree&);

  boost::property_tree::ptree&
  MergeConfig(const std::string&, boost::property_tree::ptree&);

  void ClearFullCache();

  void ClearCache();

  static constexpr size_t kModeCostingCount = 8;

private:
  typedef sif::cost_ptr_t (*factory_function_t)(const boost::property_tree::ptree&);

  boost::property_tree::ptree config_;

  baldr::GraphReader graphreader_;

  sif::cost_ptr_t mode_costing_[kModeCostingCount];

  std::string mode_name_[kModeCostingCount];

  CandidateGridQuery candidatequery_;

  float max_grid_cache_size_;

  size_t register_costing(const std::string&, factory_function_t, const boost::property_tree::ptree&);

  sif::cost_ptr_t* init_costings(const boost::property_tree::ptree&);
};

}
}

#endif // MMP_MAP_MATCHER_FACTORY_H_
