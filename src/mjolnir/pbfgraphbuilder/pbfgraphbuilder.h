#ifndef VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H
#define VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H

#include <sstream>
#include <iostream>
#include <vector>
#include <map>
#include "geo/pointll.h"

namespace valhalla{
namespace mjolnir{

//#include "osmpbfreader.h"
//using namespace CanalTP;  // For OSM pbf reader
//using namespace std;
//constexpr static unsigned stxxl_memory = (
//    (sizeof(std::size_t) == 4) ?
//        std::numeric_limits<int>::max() : std::numeric_limits<unsigned>::max());

// OSM node
struct OSMNode {
  geo::PointLL latlng_;
  unsigned char uses;

  // TODO - add traffic light, bollard, other??

  OSMNode() {
    latlng_.Set(0.0f, 0.0f);
    uses = 0;
  }
  OSMNode(const float lat, const float lng) {
    latlng_.Set(lat, lng);
    uses = 0;
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


