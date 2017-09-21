// -*- mode: c++ -*-
#ifndef MMP_MATCH_ROUTE_H_
#define MMP_MATCH_ROUTE_H_

#include <vector>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>

#include <valhalla/meili/map_matcher.h>
#include <valhalla/meili/routing.h>
#include <valhalla/meili/state.h>

namespace valhalla {
namespace meili {

struct EdgeSegment
{
  EdgeSegment(baldr::GraphId the_edgeid,
              float the_source = 0.f,
              float the_target = 1.f);

  std::vector<midgard::PointLL>
  Shape(baldr::GraphReader& graphreader) const;

  bool
  Adjoined(baldr::GraphReader& graphreader, const EdgeSegment& other) const;

  // TODO make them private
  baldr::GraphId edgeid;

  float source;

  float target;
};


std::vector<EdgeSegment>&
MergeRoute(std::vector<EdgeSegment>& route, const State& source, const State& target);


std::vector<EdgeSegment>
MergeRoute(const State& source, const State& target);


template <typename match_iterator_t>
std::vector<EdgeSegment>
ConstructRoute(const MapMatcher& mapmatcher,
               match_iterator_t begin,
               match_iterator_t end);


template <typename segment_iterator_t>
std::vector<std::vector<midgard::PointLL>>
ConstructRouteShapes(baldr::GraphReader& graphreader,
                     segment_iterator_t begin,
                     segment_iterator_t end);

}
}
#endif // MMP_MATCH_ROUTE_H_
