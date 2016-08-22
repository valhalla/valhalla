// -*- mode: c++ -*-
#ifndef MMP_MATCH_RESULT_H_
#define MMP_MATCH_RESULT_H_

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphid.h>

#include <valhalla/meili/viterbi_search.h>


namespace valhalla {
namespace meili {

class MatchResult
{
 public:
  MatchResult(const midgard::PointLL& lnglat,
              float distance,
              const baldr::GraphId edgeid,
              StateId stateid)
      : lnglat_(lnglat),
        distance_(distance),
        edgeid_(edgeid),
        stateid_(stateid) {}

  MatchResult(const midgard::PointLL& lnglat)
      : lnglat_(lnglat),
        distance_(0.f),
        edgeid_(),
        stateid_(kInvalidStateId) {}

  // Coordinate of the matched point
  const midgard::PointLL& lnglat() const
  { return lnglat_; }

  // Distance from measurement to the matched point
  float distance() const
  { return distance_; }

  // Which edge/node this matched point stays
  const baldr::GraphId& edgeid() const
  { return edgeid_; }

  // Attach the state pointer for other information (e.g. reconstruct
  // the route path) and debugging
  StateId stateid() const
  { return stateid_; }

  bool HasState() const
  { return stateid_ != kInvalidStateId; }

 private:
  midgard::PointLL lnglat_;

  float distance_;

  baldr::GraphId edgeid_;

  StateId stateid_;
};

}
}
#endif // MMP_MATCH_RESULT_H_
