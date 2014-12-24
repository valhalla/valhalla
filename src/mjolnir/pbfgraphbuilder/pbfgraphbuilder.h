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
  uint32_t wayindex_;

  std::vector<midgard::PointLL>* latlngs_;

  Edge()
      : sourcenode_(0),
        targetnode_(0),
        wayindex_(0),
        latlngs_(nullptr) {
  }

  void AddLL(const midgard::PointLL& ll) {
    if (latlngs_ == nullptr) {
      latlngs_ = new std::vector<midgard::PointLL>;
    }
    latlngs_->push_back(ll);
  }
};

}
}

#endif  // VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H

