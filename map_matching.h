// -*- mode: c++ -*-

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/thor/pathinfo.h>
#include <valhalla/thor/pathalgorithm.h>

#include "viterbi_search.h"
#include "edge_search.h"

#include <iostream>  // TODO remove

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
    assert(double_sq_sigma_z_ >= 0.f);
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

  const Measurement measurement(Time time) const
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

 protected:
  float TransitionCost(const CandidateWrapper<Candidate>& left,
                       const CandidateWrapper<Candidate>& right) const override
  {
    auto pa = thor::PathAlgorithm();
    auto path = pa.GetBestPath(left.candidate().pathlocation(),
                               right.candidate().pathlocation(),
                               graphreader_,
                               mode_costing_,
                               mode_);
    if (path.empty()) {
      return -1.f;
    }
    auto route_distance = path.back().elapsed_time;
    const auto& left_measurement = measurements_[left.time()],
               right_measurement = measurements_[right.time()];
    const auto great_circle_distance = left_measurement.lnglat.Distance(right_measurement.lnglat);
    const auto delta = std::abs(route_distance - great_circle_distance);
    return delta / beta_;
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
