// -*- mode: c++ -*-
#ifndef MMP_MAP_MATCHER_FACTORY_H_
#define MMP_MAP_MATCHER_FACTORY_H_

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/sif/costconstants.h>
#include <valhalla/sif/costfactory.h>

#include <valhalla/meili/candidate_search.h>
#include <valhalla/meili/config.h>
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

  MapMatcher* Create(const Options& options);

  MapMatcher* Create(const Costing::Type costing_type) {
    Options options;
    options.set_costing_type(costing_type);
    return Create(options);
  }

  Config MergeConfig(const Options& options) const;

  void ClearFullCache();

  void ClearCache();

private:
  typedef sif::cost_ptr_t (*factory_function_t)(const boost::property_tree::ptree&);

  Config config_;

  std::shared_ptr<baldr::GraphReader> graphreader_;

  valhalla::sif::mode_costing_t mode_costing_;

  sif::CostFactory cost_factory_;

  std::shared_ptr<CandidateGridQuery> candidatequery_;
};

} // namespace meili
} // namespace valhalla

#endif // MMP_MAP_MATCHER_FACTORY_H_
