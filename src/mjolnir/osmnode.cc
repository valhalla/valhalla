#include "mjolnir/osmnode.h"

#include <algorithm>

namespace valhalla {
namespace mjolnir {

OSMNode::OSMNode()
    : osmid_(0),
      latlng_{},
      attributes_{} {
}

OSMNode::OSMNode(const uint64_t osmid, const float lng, const float lat)
    : osmid_(osmid),
      latlng_{lng, lat},
      attributes_{} {
}

OSMNode::~OSMNode() {
}

// Sets the lat,lng.
void OSMNode::set_latlng(const std::pair<float, float>& ll) {
  latlng_ = ll;
}

// Gets the lat,lng.
const std::pair<float, float>& OSMNode::latlng() const {
  return latlng_;
}

// Set modes mask.
void OSMNode::set_modes_mask(const uint32_t modes_mask) {
  attributes_.modes_mask = modes_mask;
}

// Get the modes mask.
uint32_t OSMNode::modes_mask() const {
  return attributes_.modes_mask;
}

// Set the exit to flag
void OSMNode::set_exit_to(const bool exit_to) {
  attributes_.exit_to = exit_to;
}

// Get the exit to flag
bool OSMNode::exit_to() const {
  return attributes_.exit_to;
}

// Set the ref flag
void OSMNode::set_ref(const bool ref) {
  attributes_.ref = ref;
}

// Get the ref flag
bool OSMNode::ref() const {
  return attributes_.ref;
}

// Set the name flag
void OSMNode::set_name(const bool name) {
  attributes_.name = name;
}

// Get the name flag
bool OSMNode::name() const {
  return attributes_.name;
}

// Set gate flag.
void OSMNode::set_gate(const bool gate) {
  attributes_.gate = gate;
}

// Get the gate flag.
bool OSMNode::gate() const {
  return attributes_.gate;
}

// Set bollard flag.
void OSMNode::set_bollard(const bool bollard) {
  attributes_.bollard = bollard;
}

// Get the bollard flag.
bool OSMNode::bollard() const {
  return attributes_.bollard;
}

// Set the intersection flag.
void OSMNode::set_intersection(const bool intersection) {
  attributes_.intersection = intersection;
}

// Get the intersection flag
bool OSMNode::intersection() const {
  return attributes_.intersection;
}

// Set traffic_signal flag.
void OSMNode:: set_traffic_signal(const bool traffic_signal) {
  attributes_.traffic_signal = traffic_signal;
}

// Get the traffic_signal flag.
bool OSMNode::traffic_signal() const {
  return attributes_.traffic_signal;
}

// Get the attributes value.
const NodeAttributes& OSMNode::attributes() const {
  return attributes_;
}


}
}
