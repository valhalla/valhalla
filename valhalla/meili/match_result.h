// -*- mode: c++ -*-
#ifndef MMP_MATCH_RESULT_H_
#define MMP_MATCH_RESULT_H_

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphid.h>

#include <valhalla/meili/viterbi_search.h>


namespace valhalla {
namespace meili {

struct MatchResult
{
  // Coordinate of the match point
  midgard::PointLL lnglat;
  // Distance from measurement to the match point
  float distance_from;
  // Which edge this match point stays
  baldr::GraphId edgeid;
  // Percentage distance along the edge
  float distance_along;
  // Optional epoch time for this match point copied from original measurement
  double epoch_time;
  // Sequential state id
  StateId stateid;

  bool HasState() const { return stateid != kInvalidStateId; }
};

}
}
#endif // MMP_MATCH_RESULT_H_
