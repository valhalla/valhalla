#ifndef VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H
#define VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H

#include <sstream>
#include <iostream>
#include <vector>
#include <map>

#include "midgard/pointll.h"
#include "baldr/graphid.h"

namespace valhalla {
namespace mjolnir {

// OSM node
struct OSMNode {
  midgard::PointLL latlng_;
  unsigned int uses_;
  std::vector<unsigned int>* edges_;

  std::string exit_to_;
  std::string ref_;
  bool gate_;
  bool bollard_;
  unsigned short modes_mask_;

  OSMNode() {
    latlng_.Set(0.0f, 0.0f);
    uses_ = 0;
    edges_ = nullptr;

    exit_to_ = "";
    ref_ = "";
    gate_ = false;
    bollard_ = false;
    modes_mask_ = 0;

  }
  OSMNode(const float lat, const float lng) {
    latlng_.Set(lat, lng);
    uses_ = 0;
    edges_ = nullptr;

    exit_to_ = "";
    ref_ = "";
    gate_ = false;
    bollard_ = false;
    modes_mask_ = 0;
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

  unsigned short road_class_;

  bool auto_forward_;
  bool bike_forward_;
  bool auto_backward_;
  bool bike_backward_;
  bool pedestrian_;

  bool oneway_;
  bool roundabout_;
  bool ferry_;
  bool rail_;

  std::string name_;
  std::string name_en_;

  unsigned short maxspeed_;
  unsigned short minspeed_;

  std::string ref_;
  std::string int_ref_;

  bool  surface_; //TODO:  Expand out?
  unsigned short lanes_;
  unsigned char  tunnel_;
  unsigned char  toll_;
  unsigned short bike_network_mask_;

  std::string bike_national_ref_;
  std::string  bike_regional_ref_;
  std::string  bike_local_ref_;

  OSMWay(uint64_t id) {
    osmwayid_ = id;

    road_class_ = 0;

    auto_forward_ = false;
    bike_forward_ = false;
    auto_backward_ = false;
    bike_backward_ = false;
    pedestrian_ = false;

    oneway_ = false;
    roundabout_ = false;
    ferry_ = false;
    rail_ = false;
    name_ = "";
    name_en_ = "";
    maxspeed_ = 0;
    minspeed_ = 0;
    ref_ = "";
    int_ref_ = "";
    surface_ = false;
    lanes_ = 0;
    tunnel_  = false;
    toll_  = false;

    bike_network_mask_ = 0;
    bike_national_ref_ = "";
    bike_regional_ref_ = "";
    bike_local_ref_ = "";
  }
};

struct Edge {
  uint64_t osmwayid_;
  uint64_t sourcenode_;
  uint64_t targetnode_;
  std::vector<midgard::PointLL>* latlngs_;

  Edge()
    : osmwayid_(0),
      sourcenode_(0),
      targetnode_(0),
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


