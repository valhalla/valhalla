#ifndef VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H
#define VALHALLA_MJOLNIR_PBFGRAPHBUILDER_H

#include <sstream>
#include <iostream>
#include <vector>
#include <map>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphid.h>

#include "osmnode.h"

namespace valhalla {
namespace mjolnir {

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

  bool private_;
  unsigned short use_;
  bool no_thru_traffic_;
  bool oneway_;
  bool roundabout_;
  bool link_;
  bool ferry_;
  bool rail_;

  std::string name_;
  std::string name_en_;
  std::string alt_name_;

  unsigned short speed;

  std::string ref_;
  std::string int_ref_;

  bool  surface_; //TODO:  Expand out?
  unsigned short lanes_;
  bool tunnel_;
  bool toll_;
  bool bridge_;

  std::string destination_;
  std::string destination_ref_;
  std::string destination_ref_to_;
  std::string junction_ref_;

  unsigned short bike_network_mask_;
  std::string bike_national_ref_;
  std::string bike_regional_ref_;
  std::string bike_local_ref_;

  OSMWay(uint64_t id) {
    osmwayid_ = id;

    road_class_ = 0;

    auto_forward_ = false;
    bike_forward_ = false;
    auto_backward_ = false;
    bike_backward_ = false;
    pedestrian_ = false;

    private_ = false;
    use_ = 0;
    no_thru_traffic_ = false;
    oneway_ = false;
    roundabout_ = false;
    link_ = false;
    ferry_ = false;
    rail_ = false;

    name_ = "";
    name_en_ = "";
    alt_name_ = "";

    speed = 0;

    ref_ = "";
    int_ref_ = "";
    surface_ = false;
    lanes_ = 0;
    tunnel_  = false;
    toll_  = false;
    bridge_ = false;

    destination_ = "";
    destination_ref_ = "";
    destination_ref_to_ = "";
    junction_ref_ = "";

    bike_network_mask_ = 0;
    bike_national_ref_ = "";
    bike_regional_ref_ = "";
    bike_local_ref_ = "";
  }

};

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


