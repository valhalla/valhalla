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
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/sif/dynamiccost.h>

#include <meili/candidate.h>
#include <meili/grid_range_query.h>

namespace valhalla{

namespace meili {


class CandidateQuery
{
 public:
  CandidateQuery(baldr::GraphReader& reader) : reader_(reader) {}

  virtual ~CandidateQuery() {}

  virtual std::vector<Candidate>
  Query(const midgard::PointLL& point, float radius, sif::EdgeFilter filter = nullptr) const;

  virtual std::vector<std::vector<Candidate>>
  QueryBulk(const std::vector<midgard::PointLL>& points, float radius, sif::EdgeFilter filter = nullptr);

 protected:
  template <typename edgeid_iterator_t> std::vector<Candidate>
  WithinSquaredDistance(const midgard::PointLL& location,
                        float sq_search_radius,
                        edgeid_iterator_t edgeid_begin,
                        edgeid_iterator_t edgeid_end,
                        sif::EdgeFilter filter,
                        bool directed) const;

  baldr::GraphReader& reader_;
};


class CandidateGridQuery final: public CandidateQuery
{
 public:
  CandidateGridQuery(baldr::GraphReader& reader, float cell_width, float cell_height);

  ~CandidateGridQuery();

  const GridRangeQuery<baldr::GraphId>* GetGrid(baldr::GraphId tile_id) const;

  const GridRangeQuery<baldr::GraphId>* GetGrid(const baldr::GraphTile* tile_ptr) const;

  std::unordered_set<baldr::GraphId>
  RangeQuery(const midgard::AABB2<midgard::PointLL>& range) const;

  std::vector<Candidate>
  Query(const midgard::PointLL& location, float sq_search_radius, sif::EdgeFilter filter) const override;

  std::unordered_map<baldr::GraphId, GridRangeQuery<baldr::GraphId> >::size_type
  size() const
  { return grid_cache_.size(); }

  void Clear()
  { grid_cache_.clear(); }

 private:

  const baldr::TileHierarchy& hierarchy_;

  float cell_width_;

  float cell_height_;

  mutable std::unordered_map<baldr::GraphId, GridRangeQuery<baldr::GraphId>> grid_cache_;
};

}

}


#endif // MMP_CANDIDATE_SEARCH_H_
