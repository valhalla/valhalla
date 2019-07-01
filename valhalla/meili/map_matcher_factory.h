// -*- mode: c++ -*-
#ifndef MMP_MAP_MATCHER_FACTORY_H_
#define MMP_MAP_MATCHER_FACTORY_H_
#include <cstdint>

#include <string>

#include <boost/property_tree/ptree.hpp>
#include <rapidjson/rapidjson.h>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/sif/costconstants.h>
#include <valhalla/sif/costfactory.h>

#include <valhalla/meili/candidate_search.h>
#include <valhalla/meili/map_matcher.h>

namespace valhalla {
namespace meili {

class MapMatcherFactory final {
public:
  MapMatcherFactory(const boost::property_tree::ptree& root,
                    const std::shared_ptr<baldr::GraphReader>& graph_reader = {});

  ~MapMatcherFactory();

  std::shared_ptr<baldr::GraphReader>& graphreader() {
    return graphreader_;
  }

  CandidateQuery& candidatequery() {
    return *candidatequery_;
  }

  MapMatcher* Create(const Costing costing, const Options& options);

  MapMatcher* Create(const Costing costing) {
    return Create(costing, Options());
  }

  MapMatcher* Create(const Options& options);

  boost::property_tree::ptree MergeConfig(const Options& options);

  void ClearFullCache();

  void ClearCache();

  static constexpr size_t kModeCostingCount = 8;

private:
  typedef sif::cost_ptr_t (*factory_function_t)(const boost::property_tree::ptree&);

  boost::property_tree::ptree config_;

  std::shared_ptr<baldr::GraphReader> graphreader_;

  valhalla::sif::cost_ptr_t mode_costing_[kModeCostingCount];

  sif::CostFactory<sif::DynamicCost> cost_factory_;

  std::shared_ptr<CandidateGridQuery> candidatequery_;

  float max_grid_cache_size_;
};

} // namespace meili
} // namespace valhalla

#endif // MMP_MAP_MATCHER_FACTORY_H_
