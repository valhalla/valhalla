#include "osmnode.h"

using namespace valhalla::midgard;

namespace valhalla {
namespace mjolnir {

OSMNode::OSMNode() {
  latlng_.Set(0.0f, 0.0f);
  edges_ = nullptr;
  exit_to_ = "";
  ref_ = "";
  attributes_.v = 0;
}

OSMNode::OSMNode(const float lat, const float lng) {
  latlng_.Set(lat, lng);
  edges_ = nullptr;
  exit_to_ = "";
  ref_ = "";
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
PointLL OSMNode::latlng() const {
  return latlng_;
}

// Add an edge to the list of outbound edges
void OSMNode::AddEdge(const uint32_t edgeindex) {
  if (edges_ == nullptr) {
    edges_ = new std::vector<uint32_t>;
  }
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

// Set the exit to string
void OSMNode::set_exit_to(const std::string& exitto) {
  exit_to_ = exitto;
}

// Get the exit to string
const std::string& OSMNode::exit_to() const {
  return exit_to_;
}

// Set the ref string
void OSMNode::set_ref(const std::string& ref) {
  ref_ = ref;
}

// Get the ref string
const std::string& OSMNode::ref() const {
  return ref_;
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
