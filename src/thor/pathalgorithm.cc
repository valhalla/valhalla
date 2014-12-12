#include "thor/pathalgorithm.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

// Default constructor
PathAlgorithm::PathAlgorithm()
    : adjacencylist_(nullptr),
      edgestatus_(nullptr),
      edgecost_(nullptr) {
}

// Destructor
PathAlgorithm::~PathAlgorithm() {
  // Need to clean up all the allocated edge labels!
  // Is there a better way to manage?

  // Delete all edgelabels from the done set
  for (auto label : doneset_) {
    delete label;
  }

  if (adjacencylist_ != nullptr) {
    delete adjacencylist_;
  }
  if (edgestatus_ != nullptr) {
    delete edgestatus_;
  }
  if (edgecost_ != nullptr) {
    delete edgecost_;
  }
}


}
}
