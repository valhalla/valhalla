#include "osmnode.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

OSMNode::OSMNode() {
  latlng_.Set(0.0f, 0.0f);
  edges_ = new std::vector<uint32_t>;
  attributes_.v = 0;
}

OSMNode::OSMNode(const float lat, const float lng) {
  latlng_.Set(lat, lng);
  edges_ = new std::vector<uint32_t>;
  attributes_.v = 0;
}

OSMNode::~OSMNode() {
  // TODO - get corruption if I leave this in??
  /*  if (edges_ != nullptr) {
  edges_->clear();
  delete edges_;
  }*/
  }

// Sets the lat,lng.
void OSMNode::set_latlng(const midgard::PointLL& ll) {
  latlng_ = ll;
}

// Gets the lat,lng.
const PointLL& OSMNode::latlng() const {
  return latlng_;
}

// Set the graph Id
void OSMNode::set_graphid(const GraphId& graphid) {
  graphid_ = graphid;
}

// Get the graph Id of this node (after tiling).
const baldr::GraphId& OSMNode::graphid() const {
  return graphid_;
}

// Add an edge to the list of outbound edges
void OSMNode::AddEdge(const uint32_t edgeindex) {
  edges_->push_back(edgeindex);
}

// Get the list of edges.
const std::vector<uint32_t>& OSMNode::edges() const {
  return *edges_;
}

// Get the number of outbound edges.
uint32_t OSMNode::edge_count() const {
  return edges_->size();
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

// Increment uses (number of ways that use this node).
void OSMNode::IncrementUses() {
  attributes_.fields.uses++;
}

// Get the number of uses
uint32_t OSMNode::uses() const {
  return attributes_.fields.uses;
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

// Set modes mask.
void OSMNode::set_modes_mask(const uint32_t modes_mask) {
  attributes_.fields.modes_mask = modes_mask;
}

// Get the modes mask.
bool OSMNode::modes_mask() const {
  return attributes_.fields.modes_mask;
}

}
}
