#ifndef VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H
#define VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H

#include <sstream>
#include <iostream>
#include <vector>
#include <map>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>

using namespace valhalla::baldr;

#include "osmnode.h"

namespace valhalla {
namespace mjolnir {

struct Edge {
  uint64_t sourcenode_;
  uint64_t targetnode_;
  std::vector<midgard::PointLL> latlngs_;
  uint32_t wayindex_;

  // Construct a new edge. Target node and additional lat,lngs will
  // be filled in later.
  Edge(const uint64_t sourcenode, const uint32_t wayindex,
       const midgard::PointLL& ll)
      : sourcenode_(sourcenode),
        targetnode_(0),
        wayindex_(wayindex) {
    latlngs_.emplace_back(ll);
  }

  void AddLL(const midgard::PointLL& ll) {
    latlngs_.emplace_back(ll);
  }

 private:
  Edge()
      : sourcenode_(0),
        targetnode_(0),
        wayindex_(0){
  }
};

}
}

#endif  // VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H

