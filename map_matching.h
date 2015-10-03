// -*- mode: c++ -*-

#include <algorithm>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/thor/pathinfo.h>
#include <valhalla/thor/pathalgorithm.h>
#include <valhalla/thor/timedistancematrix.h>

#include "viterbi_search.h"
#include "edge_search.h"

using namespace valhalla;


using MeasurementId = uint32_t;


struct Measurement {
  MeasurementId id;
  PointLL lnglat;
};


using CandidateRoute = std::pair<CandidateId, std::vector<thor::PathInfo>>;


class MapMatching: public ViterbiSearch<Candidate>
{
 public:
  MapMatching(float sigma_z,
              float beta,
              baldr::GraphReader& graphreader,
              const std::shared_ptr<sif::DynamicCost>* mode_costing,
              const sif::TravelMode mode)
      : sigma_z_(sigma_z),
        beta_(beta),
        graphreader_(graphreader),
        mode_costing_(mode_costing),
        mode_(mode)
  {
    if (sigma_z_ < 0.f) {
      throw std::invalid_argument("sigma_z must be non-negative");
    }

    if (beta_ < 0.f) {
      throw std::invalid_argument("beta must be non-negative");
    }

    double_sq_sigma_z_ = sigma_z_ * sigma_z_ * 2.f;
  }

  ~MapMatching()
  {
    Clear();
  }

  Time AppendState(const Measurement& measurement,
                   const std::vector<Candidate>& candidates)
  {
    auto time = ViterbiSearch<Candidate>::AppendState(candidates.begin(), candidates.end());
    assert(time == measurements_.size());
    measurements_.push_back(measurement);
    return time;
  }

  const std::shared_ptr<sif::DynamicCost> costing() const
  {
    return mode_costing_[static_cast<uint32_t>(mode_)];
  }

  baldr::GraphReader& graphreader() const
  {
    return graphreader_;
  }

  const Measurement& measurement(Time time) const
  {
    return measurements_[time];
  }

 private:
  float sigma_z_;
  double double_sq_sigma_z_;  // equals to sigma_z_ * sigma_z_ * 2.f
  float beta_;
  std::vector<Measurement> measurements_;

  baldr::GraphReader& graphreader_;
  const std::shared_ptr<DynamicCost>* mode_costing_;
  const TravelMode mode_;
  mutable std::unordered_map<CandidatePairId, float> transition_cache_;

 protected:
  float GetBestPathDistance(const CandidateWrapper<Candidate>& left,
                            const CandidateWrapper<Candidate>& right) const
  {
    thor::PathAlgorithm pa;
    auto path = pa.GetBestPath(left.candidate().pathlocation(),
                               right.candidate().pathlocation(),
                               graphreader_,
                               mode_costing_,
                               mode_);
    if (path.empty()) {
      return -1.f;
    }
    return static_cast<float>(path.back().elapsed_time);
  }

  inline float GreatCircleDistance(const CandidateWrapper<Candidate>& left, const CandidateWrapper<Candidate>& right) const
  {
    const auto &left_mmt = measurements_[left.time()],
              &right_mmt = measurements_[right.time()];
    return left_mmt.lnglat.Distance(right_mmt.lnglat);
  }

  // It's slow. Don't use it
  float TransitionCost2(const CandidateWrapper<Candidate>& left,
                        const CandidateWrapper<Candidate>& right) const
  {
    auto route_distance = GetBestPathDistance(left, right);
    const auto &left_measurement = measurements_[left.time()],
              &right_measurement = measurements_[right.time()];
    const auto great_circle_distance = left_measurement.lnglat.Distance(right_measurement.lnglat);
    return std::abs(route_distance - great_circle_distance) / beta_;
  }

  float MaxRouteDistance(const CandidateWrapper<Candidate>& left,
                         const CandidateWrapper<Candidate>& right) const
  {
    return std::min(2000.f, GreatCircleDistance(left, right) * 2);
  }

  float TransitionCost(const CandidateWrapper<Candidate>& left,
                       const CandidateWrapper<Candidate>& right) const override
  {
    // Use cache
    auto pair = candidateid_make_pair(left.id(), right.id());
    auto cached = transition_cache_.find(pair);
    if (cached != transition_cache_.end()) {
      return cached->second;
    }

    // Prepare locations
    const auto& candidates = state(right.time());
    std::vector<PathLocation> locations;
    locations.reserve(1 + candidates.size());
    locations.push_back(left.candidate().pathlocation());
    for (const auto& candidate_ptr : candidates) {
      locations.push_back(candidate_ptr->candidate().pathlocation());
    }

    // Route
    auto max_route_distance = MaxRouteDistance(left, right);
    if (max_route_distance <= 0.f) {
      return 0.f;
    }
    thor::TimeDistanceMatrix pa(max_route_distance);
    const auto& timedistances = pa.OneToMany(0, locations, graphreader_, mode_costing_, mode_);
    assert(timedistances.size() == candidates.size() + 1);

    // Cache results
    auto great_circle_distance = GreatCircleDistance(left, right);
    auto candidate_itr = candidates.begin();
    for (auto td_itr = timedistances.begin() + 1; td_itr != timedistances.end(); td_itr++, candidate_itr++) {
      CandidatePairId pair = candidateid_make_pair(left.id(), (*candidate_itr)->id());
      // dist <= 0 means path not found
      float cost = td_itr->dist <= 0? -1.f : std::abs(td_itr->dist - great_circle_distance) / beta_;
      transition_cache_[pair] = cost;
    }

    // Must be there
    assert(transition_cache_.find(pair)!=transition_cache_.end());

    return transition_cache_[pair];
  }

  inline float EmissionCost(const CandidateWrapper<Candidate>& candidate) const override
  {
    return candidate.candidate().sq_distance() / double_sq_sigma_z_;
  }

  inline double CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const override
  {
    return prev_costsofar + transition_cost + emission_cost;
  }
};


std::vector<const CandidateWrapper<Candidate>*>
OfflineMatch(MapMatching& mm,
             const std::vector<Measurement>& measurements,
             float search_radius)
{
  mm.Clear();
  Time time;
  CandidateQuery cq(mm.graphreader());
  for (const auto& measurement : measurements) {
    auto candidates = cq.Query(measurement.lnglat,
                               search_radius,
                               mm.costing()->GetFilter());
    time = mm.AppendState(measurement, candidates);
  }
  auto path = mm.SearchPath(time);
  std::reverse(path.begin(), path.end());
  return path;
}
