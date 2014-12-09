#ifndef VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H
#define VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H

#include <sstream>
#include <iostream>
#include <vector>
#include <map>
#include "geo/pointll.h"

namespace valhalla{
namespace mjolnir{

// OSM node
struct OSMNode {
  geo::PointLL latlng_;
  unsigned char  uses_;
  std::vector<unsigned int> edges_;

  // TODO - add traffic light, bollard, other??

  OSMNode() {
    latlng_.Set(0.0f, 0.0f);
    uses_ = 0;
  }
  OSMNode(const float lat, const float lng) {
    latlng_.Set(lat, lng);
    uses_ = 0;
  }
};

// OSM Way
struct OSMWay {
  uint64_t osmwayid_;
  std::vector<uint64_t> nodelist_;

  // TODO - add essential routing parameters

  OSMWay(uint64_t id) {
    osmwayid_ = id;
  }
};

struct Edge {
  uint64_t osmwayid_;
  uint64_t sourcenode_;
  uint64_t targetnode_;

  std::vector<geo::PointLL> latlngs_;
};

}
}

#endif  // VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H


