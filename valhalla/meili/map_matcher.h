// -*- mode: c++ -*-
#ifndef MMP_MAP_MATCHER_H_
#define MMP_MAP_MATCHER_H_

#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/graphreader.h>

#include <valhalla/meili/candidate_search.h>
#include <valhalla/meili/measurement.h>
#include <valhalla/meili/map_matching.h>
#include <valhalla/meili/match_result.h>


namespace valhalla {
namespace meili {

// A facade that connects everything
class MapMatcher final
{
public:
  MapMatcher(const boost::property_tree::ptree& config,
             baldr::GraphReader& graphreader,
             CandidateQuery& candidatequery,
             const sif::cost_ptr_t* mode_costing,
             sif::TravelMode travelmode);

  ~MapMatcher();

  baldr::GraphReader& graphreader()
  { return graphreader_; }

  const CandidateQuery& candidatequery()
  { return candidatequery_; }

  sif::TravelMode travelmode() const
  { return travelmode_; }

  const boost::property_tree::ptree& config() const
  { return config_; }

  const MapMatching& mapmatching() const
  { return mapmatching_; }

  std::vector<MatchResult>
  OfflineMatch(const std::vector<Measurement>& measurements, uint32_t k = 1);

  /**
   * Set a callback that will throw when the map-matching should be aborted
   * @param interrupt_callback  the function to periodically call to see if we should abort
   */
  void set_interrupt(const std::function<void ()>* interrupt_callback) {
    interrupt_ = interrupt_callback;
  }

private:
  Time AppendMeasurement(const Measurement& measurement);

  boost::property_tree::ptree config_;

  baldr::GraphReader& graphreader_;

  CandidateQuery& candidatequery_;

  const sif::cost_ptr_t* mode_costing_;

  sif::TravelMode travelmode_;

  MapMatching mapmatching_;

  // Interrupt callback. Can be set to interrupt if connection is closed.
  const std::function<void ()>* interrupt_;
};

}
}
#endif // MMP_MAP_MATCHER_H_
