#include <iostream>

#include <valhalla/proto/trippath.pb.h>
#include "odin/enhancedtrippath.h"

namespace valhalla {
namespace odin {

///////////////////////////////////////////////////////////////////////////////
// EnhancedTripPath

EnhancedTripPath::EnhancedTripPath() {
}

EnhancedTripPath_Edge* EnhancedTripPath::GetPrevEdge(const int nodeIndex,
                                                     int delta) {
  int index = nodeIndex - delta;
  if (IsValidNodeIndex(index))
    return static_cast<EnhancedTripPath_Edge*>(mutable_node(index)->mutable_edge(
        0));
  else
    return nullptr;
}

EnhancedTripPath_Edge* EnhancedTripPath::GetCurrEdge(const int nodeIndex) {
  return GetNextEdge(nodeIndex, 0);
}

EnhancedTripPath_Edge* EnhancedTripPath::GetNextEdge(const int nodeIndex,
                                                     int delta) {
  int index = nodeIndex + delta;
  if (IsValidNodeIndex(index) && !IsLastNodeIndex(index))
    return static_cast<EnhancedTripPath_Edge*>(mutable_node(index)->mutable_edge(
        0));
  else
    return nullptr;
}

bool EnhancedTripPath::IsValidNodeIndex(int nodeIndex) const {
  if ((nodeIndex >= 0) && (nodeIndex < node_size()))
    return true;
  return false;
}

bool EnhancedTripPath::IsFirstNodeIndex(int nodeIndex) const {
  if (nodeIndex == 0)
    return true;
  return false;
}

bool EnhancedTripPath::IsLastNodeIndex(int nodeIndex) const {
  if (IsValidNodeIndex(nodeIndex) && (nodeIndex == (node_size() - 1)))
    return true;
  return false;
}

int EnhancedTripPath::GetLastNodeIndex() const {
  return (node_size() - 1);
}

///////////////////////////////////////////////////////////////////////////////
// EnhancedTripPath_Edge

EnhancedTripPath_Edge::EnhancedTripPath_Edge() {
}

bool EnhancedTripPath_Edge::IsUnnamed() const {
  if (name_size() == 0)
    return true;
  return false;
}

bool EnhancedTripPath_Edge::IsHighway() const {
  if ((road_class() == TripPath_RoadClass_kMotorway) && (!ramp()))
    return true;
  return false;
}

std::string EnhancedTripPath_Edge::ToString() const {
  std::string str;
  str.reserve(128);

  str += "name=";
  if (name_size() == 0) {
    str += "unnamed";
  } else {
    bool is_first = true;
    for (const auto& name : this->name()) {
      if (is_first)
        is_first = false;
      else
        str += "/";
      str += name;
    }
  }

  str += " | length=";
  str += std::to_string(length());

  str += " | speed=";
  str += std::to_string(speed());

  str += " | road_class=";
  str += std::to_string(road_class());

  str += " | ramp=";
  str += std::to_string(ramp());

  str += " | ferry=";
  str += std::to_string(ferry());

  str += " | rail_ferry=";
  str += std::to_string(rail_ferry());

  str += " | roundabout=";
  str += std::to_string(roundabout());

  return str;
}

}
}

