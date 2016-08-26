// -*- mode: c++ -*-
#ifndef MMP_MATCH_ROUTE_H_
#define MMP_MATCH_ROUTE_H_

#include <vector>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/graphid.h>

#include <valhalla/meili/map_matcher.h>


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


template <typename match_iterator_t>
std::vector<EdgeSegment>
ConstructRoute(baldr::GraphReader& graphreader,
               const MapMatcher& mapmatcher,
               match_iterator_t begin,
               match_iterator_t end);

}
}
#endif // MMP_MATCH_ROUTE_H_
