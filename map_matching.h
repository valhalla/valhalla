// -*- mode: c++ -*-

#include <algorithm>

#include <valhalla/midgard/pointll.h>

#include "viterbi_search.h"
#include "edge_search.h"
#include "sp.h"

using namespace valhalla;


class Measurement {
 public:
  Measurement(const PointLL& lnglat)
      : lnglat_(lnglat) {
  }

  const PointLL& lnglat() const {
    return lnglat_;
  };

 private:
  PointLL lnglat_;
};


constexpr float kBreakageDistance = 2000.f;  // meters
constexpr float kClosestDistance = 0.f;  // meters


class MapMatching: public ViterbiSearch<Candidate>
{
 public:
  MapMatching(float sigma_z,
              float beta,
              baldr::GraphReader& graphreader,
              const std::shared_ptr<sif::DynamicCost>* mode_costing,
              const sif::TravelMode mode)
      : sigma_z_(sigma_z),
        inv_double_sq_sigma_z_(1.f / (sigma_z_ * sigma_z_ * 2.f)),
        beta_(beta),
        inv_beta_(1.f / beta_),
        measurements_(),
        graphreader_(graphreader),
        mode_costing_(mode_costing),
        mode_(mode),
        states_(),
        transition_cache_(),
        labelset_cache_(),
        label_idx_cache_()
  {
    if (sigma_z_ <= 0.f) {
      throw std::invalid_argument("expect sigma_z to be positive");
    }

    if (beta_ <= 0.f) {
      throw std::invalid_argument("expect beta to be positive");
    }
  }

  ~MapMatching()
  {
    Clear();
  }

  void Clear() {
    measurements_.clear();
    states_.clear();
    transition_cache_.clear();
    labelset_cache_.clear();
    label_idx_cache_.clear();
    ViterbiSearch<Candidate>::Clear();
  }

  Time AppendState(const Measurement& measurement,
                   const std::vector<Candidate>::const_iterator begin,
                   const std::vector<Candidate>::const_iterator end)
  {
    auto time = ViterbiSearch<Candidate>::AppendState(begin, end);
    assert(time == measurements_.size());
    measurements_.push_back(measurement);
    states_.emplace_back(unreached_states(time));
    return time;
  }

  const std::vector<const CandidateWrapper<Candidate>*>&
  states(Time time) const
  {
    return states_[time];
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

  std::vector<Measurement>::size_type size() const
  {
    return measurements_.size();
  }

  const LabelSet& labelset(CandidateId id) const
  {
    return labelset_cache_.at(id);
  }

  bool has_labelset(CandidateId id) const
  {
    return labelset_cache_.find(id) != labelset_cache_.end();
  }

  uint32_t label_idx(CandidateId left, CandidateId right) const
  {
    auto p = candidateid_make_pair(left, right);
    auto it = label_idx_cache_.find(p);
    if (it != label_idx_cache_.end()) {
      return it->second;
    }
    return kInvalidLabelIndex;
  }

 private:
  float sigma_z_;
  double inv_double_sq_sigma_z_;  // equals to 1.f / (sigma_z_ * sigma_z_ * 2.f)
  float beta_;
  float inv_beta_;  // equals to 1.f / beta_
  std::vector<Measurement> measurements_;
  baldr::GraphReader& graphreader_;
  const std::shared_ptr<DynamicCost>* mode_costing_;
  const TravelMode mode_;
  std::vector<std::vector<const CandidateWrapper<Candidate>*>> states_;

  // Caches
  mutable std::unordered_map<CandidatePairId, float> transition_cache_;
  mutable std::unordered_map<CandidateId, LabelSet> labelset_cache_;
  mutable std::unordered_map<CandidatePairId, uint32_t> label_idx_cache_;

 protected:
  inline float GreatCircleDistance(const CandidateWrapper<Candidate>& left,
                                   const CandidateWrapper<Candidate>& right) const
  {
    const auto &left_pt = left.candidate().pathlocation().vertex(),
              &right_pt = right.candidate().pathlocation().vertex();
    return left_pt.Distance(right_pt);
  }

  inline float GreatCircleDistance(const Measurement& left,
                                   const Measurement& right) const
  {
    return left.lnglat().Distance(right.lnglat());
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

    // Handle cases when two measurements are too far or too close
    const auto &left_mmt = measurements_[left.time()],
              &right_mmt = measurements_[right.time()];
    auto mmt_distance = GreatCircleDistance(left_mmt, right_mmt);
    if (mmt_distance > kBreakageDistance) {
      for (const auto& candidate_ptr : unreached_states(right.time())) {
        auto p = candidateid_make_pair(left.id(), candidate_ptr->id());
        transition_cache_[p] = -1.f;
      }
      return -1.f;
    } else if (mmt_distance <= kClosestDistance) {
      for (const auto& candidate_ptr : unreached_states(right.time())) {
        auto p = candidateid_make_pair(left.id(), candidate_ptr->id());
        transition_cache_[p] = 0.f;
      }
      return 0.f;
    }
    auto max_route_distance = std::min(mmt_distance * 3, kBreakageDistance);

    // Prepare locations
    const auto& candidates = unreached_states(right.time());
    std::vector<PathLocation> locations;
    locations.reserve(1 + candidates.size());
    locations.push_back(left.candidate().pathlocation());
    for (const auto& candidate_ptr : candidates) {
      locations.push_back(candidate_ptr->candidate().pathlocation());
    }

    // Route and cache results
    const auto& result = labelset_cache_.emplace(left.id(), std::ceil(max_route_distance));
    assert(result.second);  // Must be new insertion
    auto& labelset = result.first->second;
    const auto& results = find_shortest_path(graphreader_, locations, 0, labelset);
    auto candidate_itr = candidates.begin();
    for (uint16_t dest = 1; dest < locations.size(); dest++, candidate_itr++) {
      float cost = -1.f;
      auto p = candidateid_make_pair(left.id(), (*candidate_itr)->id());
      auto it = results.find(dest);
      if (it != results.end()) {
        auto route_distance = labelset.label(it->second).cost;
        cost = std::abs(route_distance - mmt_distance) * inv_beta_;
        label_idx_cache_[p] = it->second;
      }
      transition_cache_[p] = cost;
    }

    return transition_cache_[pair];
  }

  inline float EmissionCost(const CandidateWrapper<Candidate>& candidate) const override
  {
    return candidate.candidate().sq_distance() * inv_double_sq_sigma_z_;
  }

  inline double CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const override
  {
    return prev_costsofar + transition_cost + emission_cost;
  }
};


std::vector<const CandidateWrapper<Candidate>*>
OfflineMatch(MapMatching& mm,
             const CandidateQuery& cq,
             const std::vector<Measurement>& measurements,
             float sq_search_radius)
{
  mm.Clear();
  if (measurements.empty()) {
    return {};
  }
  Time time;
  for (const auto& measurement : measurements) {
    const auto& candidates = cq.Query(measurement.lnglat(),
                                      sq_search_radius,
                                      mm.costing()->GetFilter());
    time = mm.AppendState(measurement, candidates.begin(), candidates.end());
  }
  auto path = mm.SearchPath(time);
  std::reverse(path.begin(), path.end());
  return path;
}
