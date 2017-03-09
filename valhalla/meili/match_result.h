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
              float distance_along,
              float epoch_time = 0,
              StateId stateid = kInvalidStateId)
      : lnglat_(lnglat),
        distance_from_(distance),
        edgeid_(edgeid),
        stateid_(stateid),
        epoch_time_(epoch_time),
        distance_along_(distance_along){}

  MatchResult(const midgard::PointLL& lnglat, float epoch_time = 0)
      : MatchResult(lnglat, 0.f, {}, -1.f, epoch_time) {}

  // Coordinate of the match point
  const midgard::PointLL& lnglat() const
  { return lnglat_; }

  // Distance from measurement to the match point
  float distance_from() const
  { return distance_from_; }

  // Which edge this match point stays
  const baldr::GraphId& edgeid() const
  { return edgeid_; }

  StateId stateid() const
  { return stateid_; }

  bool HasState() const
  { return stateid_ != kInvalidStateId; }

  float distance_along() const
  { return distance_along_; }

 private:
  midgard::PointLL lnglat_;

  float distance_from_;

  baldr::GraphId edgeid_;

  StateId stateid_;

  float distance_along_;

  float epoch_time_;
};

}
}
#endif // MMP_MATCH_RESULT_H_
