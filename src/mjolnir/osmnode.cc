#include "mjolnir/osmnode.h"

#include <algorithm>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

OSMNode::OSMNode() {
  latlng_.Set(0.0f, 0.0f);
  attributes_.v = 0;
}

OSMNode::OSMNode(const float lng, const float lat) {
  latlng_.Set(lng, lat);
  attributes_.v = 0;
}

OSMNode::~OSMNode() {
}

// Sets the lat,lng.
void OSMNode::set_latlng(const midgard::PointLL& ll) {
  latlng_ = ll;
}

// Gets the lat,lng.
const PointLL& OSMNode::latlng() const {
  return latlng_;
}

// Set modes mask.
void OSMNode::set_modes_mask(const uint32_t modes_mask) {
  attributes_.fields.modes_mask = modes_mask;
}

// Get the modes mask.
uint32_t OSMNode::modes_mask() const {
  return attributes_.fields.modes_mask;
}

// Set the exit to flag
void OSMNode::set_exit_to(const bool exit_to) {
  attributes_.fields.exit_to = exit_to;
}

// Get the exit to flag
bool OSMNode::exit_to() const {
  return attributes_.fields.exit_to;
}

// Set the ref flag
void OSMNode::set_ref(const bool ref) {
  attributes_.fields.ref = ref;
}

// Get the ref flag
bool OSMNode::ref() const {
  return attributes_.fields.ref;
}

// Set the name flag
void OSMNode::set_name(const bool name) {
  attributes_.fields.name = name;
}

// Get the name flag
bool OSMNode::name() const {
  return attributes_.fields.name;
}

// Set gate flag.
void OSMNode::set_gate(const bool gate) {
  attributes_.fields.gate = gate;
}

// Get the gate flag.
bool OSMNode::gate() const {
  return attributes_.fields.gate;
}

// Set bollard flag.
void OSMNode::set_bollard(const bool bollard) {
  attributes_.fields.bollard = bollard;
}

// Get the bollard flag.
bool OSMNode::bollard() const {
  return attributes_.fields.bollard;
}

// Set the intersection flag.
void OSMNode::set_intersection(const bool intersection) {
  attributes_.fields.intersection = intersection;
}

// Get the intersection flag
bool OSMNode::intersection() const {
  return attributes_.fields.intersection;
}

// Set traffic_signal flag.
void OSMNode:: set_traffic_signal(const bool traffic_signal) {
  attributes_.fields.traffic_signal = traffic_signal;
}

// Get the traffic_signal flag.
bool OSMNode::traffic_signal() const {
  return attributes_.fields.traffic_signal;
}

// Get the attributes value.
bool OSMNode::attributes() const {
  return attributes_.v;
}


}
}
