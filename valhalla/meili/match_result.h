// -*- mode: c++ -*-
#ifndef MMP_MATCH_RESULT_H_
#define MMP_MATCH_RESULT_H_

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphid.h>

#include <valhalla/meili/viterbi_search.h>


namespace valhalla {
namespace meili {

enum class GraphType: uint8_t
{ kUnknown = 0, kEdge, kNode };


class MatchResult
{
 public:
  MatchResult(const midgard::PointLL& lnglat,
              float distance,
              const baldr::GraphId graphid,
              GraphType graphtype,
              StateId stateid)
      : lnglat_(lnglat),
        distance_(distance),
        graphid_(graphid),
        graphtype_(graphtype),
        stateid_(stateid) {}

  MatchResult(const midgard::PointLL& lnglat)
      : lnglat_(lnglat),
        distance_(0.f),
        graphid_(),
        graphtype_(GraphType::kUnknown),
        stateid_(kInvalidStateId) {}

  // Coordinate of the matched point
  const midgard::PointLL& lnglat() const
  { return lnglat_; }

  // Distance from measurement to the matched point
  float distance() const
  { return distance_; }

  // Which edge/node this matched point stays
  const baldr::GraphId graphid() const
  { return graphid_; }

  GraphType graphtype() const
  { return graphtype_; }

  // Attach the state pointer for other information (e.g. reconstruct
  // the route path) and debugging
  StateId stateid() const
  { return stateid_; }

  bool HasState() const
  { return stateid_ != kInvalidStateId; }

 private:
  midgard::PointLL lnglat_;

  float distance_;

  baldr::GraphId graphid_;

  GraphType graphtype_;

  StateId stateid_;
};

}
}
#endif // MMP_MATCH_RESULT_H_
