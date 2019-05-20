// -*- mode: c++ -*-
#ifndef MMP_CANDIDATE_SEARCH_H_
#define MMP_CANDIDATE_SEARCH_H_

#include <algorithm>
#include <cmath>
#include <tuple>

#include <boost/property_tree/ptree.hpp>

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
  CandidateQuery(baldr::GraphReader& graphreader);

  virtual ~CandidateQuery() {
  }

  virtual std::vector<baldr::PathLocation>
  Query(const midgard::PointLL& point, float radius, sif::EdgeFilter filter = nullptr) const = 0;

  virtual std::vector<std::vector<baldr::PathLocation>>
  QueryBulk(const std::vector<midgard::PointLL>& points,
            float radius,
            sif::EdgeFilter filter = nullptr);

protected:
  template <typename edgeid_iterator_t>
  std::vector<baldr::PathLocation> WithinSquaredDistance(const midgard::PointLL& location,
                                                         float sq_search_radius,
                                                         edgeid_iterator_t edgeid_begin,
                                                         edgeid_iterator_t edgeid_end,
                                                         sif::EdgeFilter filter) const;

  baldr::GraphReader& reader_;
};

class CandidateGridQuery final : public CandidateQuery {
public:
  using grid_t = GridRangeQuery<baldr::GraphId, midgard::PointLL>;

  CandidateGridQuery(baldr::GraphReader& reader, float cell_width, float cell_height);

  ~CandidateGridQuery();

  std::vector<baldr::PathLocation> Query(const midgard::PointLL& location,
                                         float sq_search_radius,
                                         sif::EdgeFilter filter) const override;

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
};

} // namespace meili

} // namespace valhalla

#endif // MMP_CANDIDATE_SEARCH_H_
