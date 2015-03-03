#include "mjolnir/osmnode.h"

#include <algorithm>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Sets the lat,lng.
void OSMNode::set_latlng(const std::pair<float, float>& ll) {
  lng = ll.first;
  lat = ll.second;
}

// Gets the lat,lng.
std::pair<float, float> OSMNode::latlng() const {
  return std::make_pair(lng, lat);
}

// Set access mask.
void OSMNode::set_access_mask(const uint32_t access_mask) {
  attributes_.access_mask = access_mask;
}

// Get the access mask.
uint32_t OSMNode::access_mask() const {
  return attributes_.access_mask;
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

// Set the node type.
void OSMNode::set_type(const NodeType type) {
  attributes_.type = static_cast<uint8_t>(type);
}

// Get the node type.
NodeType OSMNode::type() const {
  return static_cast<baldr::NodeType>(attributes_.type);
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
void OSMNode::set_traffic_signal(const bool traffic_signal) {
  attributes_.traffic_signal = traffic_signal;
}

// Get the traffic_signal flag.
bool OSMNode::traffic_signal() const {
  return attributes_.traffic_signal;
}

// Set forward_signal flag.
void OSMNode::set_forward_signal(const bool forward_signal) {
  attributes_.forward_signal = forward_signal;
}

// Get the forward_signal flag.
bool OSMNode::forward_signal() const {
  return attributes_.forward_signal;
}

// Set backward_signal flag.
void OSMNode::set_backward_signal(const bool backward_signal) {
  attributes_.backward_signal = backward_signal;
}

// Get the backward_signal flag.
bool OSMNode::backward_signal() const {
  return attributes_.backward_signal;
}

// Get the attributes value.
const NodeAttributes& OSMNode::attributes() const {
  return attributes_;
}


}
}
