// -*- mode: c++ -*-
#ifndef MMP_CANDIDATE_SEARCH_H_
#define MMP_CANDIDATE_SEARCH_H_

#include <algorithm>
#include <cmath>

#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/linesegment2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/tiles.h>
#include <valhalla/sif/dynamiccost.h>

#include <valhalla/meili/grid_range_query.h>

namespace valhalla {
namespace meili {

class CandidateQuery {
public:
  virtual ~CandidateQuery() = default;

  virtual std::vector<baldr::PathLocation> Query(const midgard::PointLL& point,
                                                 baldr::Location::StopType stop_type,
                                                 float radius,
                                                 const sif::cost_ptr_t& costing = nullptr) const = 0;
};

class CandidateGridQuery final : public CandidateQuery {
public:
  using grid_t = GridRangeQuery<baldr::GraphId, midgard::PointLL>;

  CandidateGridQuery(baldr::GraphReader& reader, float cell_width, float cell_height);

  ~CandidateGridQuery() override;

  std::vector<baldr::PathLocation> Query(const midgard::PointLL& location,
                                         baldr::Location::StopType stop_type,
                                         float sq_search_radius,
                                         const sif::cost_ptr_t& costing) const override;

  template <typename Collector>
  auto Query(const midgard::PointLL& location,
             baldr::Location::StopType stop_type,
             float sq_search_radius,
             const sif::cost_ptr_t& costing,
             const Collector& collector) const {
    if (!location.IsValid()) {
      throw std::invalid_argument("Expect a valid location");
    }

    const auto range = midgard::ExpandMeters(location, std::sqrt(sq_search_radius));
    const auto edgeids = RangeQuery(range);

    return collector.WithinSquaredDistance(location, stop_type, sq_search_radius, edgeids.begin(),
                                           edgeids.end(), costing);
  }

  std::unordered_map<baldr::GraphId, grid_t>::size_type size() const {
    return grid_cache_.size();
  }

  void Clear() {
    grid_cache_.clear();
  }

private:
  // Get a grid for a specified bin within a tile. Tile support for
  // graph tiles and bins is provided to go between bin Ids and tile Ids.
  const grid_t* GetGrid(const int32_t bin_id,
                        const midgard::Tiles<midgard::PointLL>& tiles,
                        const midgard::Tiles<midgard::PointLL>& bins) const;

  std::unordered_set<baldr::GraphId> RangeQuery(const midgard::AABB2<midgard::PointLL>& range) const;

  uint32_t bin_level_;

  float cell_width_;
  float cell_height_;

  // Grid cache - cached per "bin" within a graph tile
  mutable std::unordered_map<int32_t, grid_t> grid_cache_;

  baldr::GraphReader& reader_;
};

} // namespace meili

} // namespace valhalla

#endif // MMP_CANDIDATE_SEARCH_H_
