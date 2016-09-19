// -*- mode: c++ -*-
#ifndef MMP_CANDIDATE_SEARCH_H_
#define MMP_CANDIDATE_SEARCH_H_

#include <cmath>
#include <tuple>
#include <algorithm>

#include <boost/property_tree/ptree.hpp>
#include <boost/iterator/counting_iterator.hpp>

#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/linesegment2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/sif/dynamiccost.h>

#include <valhalla/meili/grid_range_query.h>

namespace valhalla{

namespace meili {


class CandidateQuery
{
 public:
  CandidateQuery(baldr::GraphReader& graphreader);

  virtual ~CandidateQuery() {}

  virtual std::vector<baldr::PathLocation>
  Query(const midgard::PointLL& point, float radius, sif::EdgeFilter filter = nullptr) const = 0;

  virtual std::vector<std::vector<baldr::PathLocation>>
  QueryBulk(const std::vector<midgard::PointLL>& points, float radius, sif::EdgeFilter filter = nullptr);

 protected:
  template <typename edgeid_iterator_t> std::vector<baldr::PathLocation>
  WithinSquaredDistance(const midgard::PointLL& location,
                        float sq_search_radius,
                        edgeid_iterator_t edgeid_begin,
                        edgeid_iterator_t edgeid_end,
                        sif::EdgeFilter filter) const;

  baldr::GraphReader& reader_;
};


class CandidateGridQuery final: public CandidateQuery
{
 public:
  using grid_t = GridRangeQuery<baldr::GraphId, midgard::PointLL>;

  CandidateGridQuery(baldr::GraphReader& reader, float cell_width, float cell_height);

  ~CandidateGridQuery();

  std::vector<baldr::PathLocation>
  Query(const midgard::PointLL& location, float sq_search_radius, sif::EdgeFilter filter) const override;

  std::unordered_map<baldr::GraphId, grid_t>::size_type
  size() const
  { return grid_cache_.size(); }

  void Clear()
  { grid_cache_.clear(); }

 private:

  const grid_t* GetGrid(const baldr::GraphId& tile_id) const;

  const grid_t* GetGrid(const baldr::GraphTile* tile_ptr) const;

  std::unordered_set<baldr::GraphId>
  RangeQuery(const midgard::AABB2<midgard::PointLL>& range) const;

  const baldr::TileHierarchy& hierarchy_;

  float cell_width_;

  float cell_height_;

  mutable std::unordered_map<baldr::GraphId, grid_t> grid_cache_;
};

}

}


#endif // MMP_CANDIDATE_SEARCH_H_
