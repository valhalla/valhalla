#ifndef VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H
#define VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H

#include <sstream>
#include <iostream>
#include <vector>
#include <map>
#include "geo/pointll.h"
#include "baldr/graphid.h"

namespace valhalla {
namespace mjolnir {

// OSM node
struct OSMNode {
  geo::PointLL latlng_;
  unsigned int uses_;
  std::vector<unsigned int>* edges_;

  // TODO - add traffic light, bollard, other??

  OSMNode() {
    latlng_.Set(0.0f, 0.0f);
    uses_  = 0;
    edges_ = nullptr;
  }
  OSMNode(const float lat, const float lng) {
    latlng_.Set(lat, lng);
    uses_  = 0;
    edges_ = nullptr;
  }
  ~OSMNode() {
    // TODO - get corruption if I leave this in??
  /*  if (edges_ != nullptr) {
      edges_->clear();
      delete edges_;
    }*/
  }

  void AddEdge(const unsigned int edgeindex) {
    if (edges_ == nullptr) {
      edges_ = new std::vector<unsigned int>;
    }
    edges_->push_back(edgeindex);
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
  std::vector<geo::PointLL>* latlngs_;

  Edge()
    : osmwayid_(0),
      sourcenode_(0),
      targetnode_(0),
      latlngs_(nullptr) {
  }

  void AddLL(const geo::PointLL& ll) {
    if (latlngs_ == nullptr) {
      latlngs_ = new std::vector<geo::PointLL>;
    }
    latlngs_->push_back(ll);
  }
};

}
}

#endif  // VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H


